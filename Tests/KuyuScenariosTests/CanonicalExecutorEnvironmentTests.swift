import Foundation
import KuyuCore
import KuyuPhysics
@testable import KuyuScenarios
import Synchronization
import Testing

@Suite("Canonical executor environment propagation")
struct CanonicalExecutorEnvironmentTests {
    @Test(.timeLimit(.minutes(1)))
    func environmentUsesOneInjectedExecutorForPlantAndSensor() throws {
        let executor = EnvironmentRecordingCanonicalExecutor()
        let definition = try scenario()
        var environment = ReferenceQuadrotorRLEnvironment(
            schedule: try SimulationSchedule.baseline(cutPeriodSteps: 1),
            determinism: .tier1Baseline,
            motorNerveRateLimitPerSecond: 100,
            motorNerveSmoothingTimeConstant: nil,
            canonicalExecutor: executor
        )
        _ = try environment.reset(
            seed: definition.config.seed,
            scenario: definition
        )

        _ = try environment.step(
            action: .driveIntents(
                try (0..<4).map { index in
                    try DriveIntent(
                        index: DriveIndex(UInt32(index)),
                        activation: 0.5
                    )
                },
                corrections: []
            )
        )

        #expect(executor.generalizedForceCount > 0)
        #expect(executor.derivativeCount > 0)
        #expect(executor.observablesCount > 0)
        #expect(
            environment.activeExecutionContract?.canonicalExecutorVersion
                == executor.executorVersion
        )
        #expect(
            environment.activeExecutionContract?.schemaVersion
                == ReferenceQuadrotorEnvironmentExecutionContract.schemaVersion
        )

        var scalarEnvironment = ReferenceQuadrotorRLEnvironment(
            schedule: try SimulationSchedule.baseline(cutPeriodSteps: 1),
            determinism: .tier1Baseline,
            motorNerveRateLimitPerSecond: 100,
            motorNerveSmoothingTimeConstant: nil
        )
        _ = try scalarEnvironment.reset(
            seed: definition.config.seed,
            scenario: definition
        )
        #expect(environment.configHash != scalarEnvironment.configHash)
    }

    @Test(.timeLimit(.minutes(1)))
    func environmentRejectsInvalidExecutorIdentityBeforeActivation() throws {
        let definition = try scenario()
        var environment = ReferenceQuadrotorRLEnvironment(
            schedule: try SimulationSchedule.baseline(cutPeriodSteps: 1),
            determinism: .tier1Baseline,
            canonicalExecutor: EnvironmentRecordingCanonicalExecutor(executorVersion: " ")
        )

        #expect(
            throws: ReferenceQuadrotorEnvironmentExecutionContract.ValidationError
                .invalidCanonicalExecutorVersion(" ")
        ) {
            _ = try environment.reset(
                seed: definition.config.seed,
                scenario: definition
            )
        }
        #expect(environment.activeExecutionContract == nil)
    }

    @Test(.timeLimit(.minutes(1)))
    func executionContractRoundTripsAndRejectsPriorSchema() throws {
        let definition = try scenario()
        var environment = ReferenceQuadrotorRLEnvironment(
            schedule: try SimulationSchedule.baseline(cutPeriodSteps: 1),
            determinism: .tier1Baseline
        )
        _ = try environment.reset(seed: definition.config.seed, scenario: definition)
        let contract = try #require(environment.activeExecutionContract)
        let encoded = try JSONEncoder().encode(contract)

        #expect(
            try JSONDecoder().decode(
                ReferenceQuadrotorEnvironmentExecutionContract.self,
                from: encoded
            ) == contract
        )

        let currentJSON = String(decoding: encoded, as: UTF8.self)
        let priorJSON = currentJSON.replacingOccurrences(
            of: "\"schemaVersion\":2",
            with: "\"schemaVersion\":1"
        )
        #expect(priorJSON != currentJSON)
        #expect(
            throws: ReferenceQuadrotorEnvironmentExecutionContract.ValidationError
                .unsupportedSchemaVersion(expected: 2, actual: 1)
        ) {
            _ = try JSONDecoder().decode(
                ReferenceQuadrotorEnvironmentExecutionContract.self,
                from: Data(priorJSON.utf8)
            )
        }
    }

    @Test(.timeLimit(.minutes(1)))
    func scenarioRunnerUsesInjectedExecutorForPlantAndSensor() async throws {
        let executor = EnvironmentRecordingCanonicalExecutor()
        let definition = try scenario()
        let parameters = ReferenceQuadrotorParameters.baseline
        let mixer = ReferenceQuadrotorMixer(
            armLength: parameters.armLength,
            yawCoefficient: parameters.yawCoefficient
        )
        let cut = try ImuRateDampingDriveCut(
            hoverThrust: parameters.mass * parameters.gravity / 4,
            kp: 2,
            kd: 0.25,
            yawDamping: 0.2,
            armLength: parameters.armLength,
            yawCoefficient: parameters.yawCoefficient,
            maxThrust: parameters.maxThrust,
            initialRoll: 0,
            initialPitch: 0,
            tiltCorrectionTimeConstant: 0.4
        )
        let motorNerve = FixedQuadMotorNerve(
            config: FixedQuadMotorNerve.Config(
                mixer: mixer,
                motorMaxThrusts: try MotorMaxThrusts.uniform(parameters.maxThrust)
            )
        )
        let runner = ReferenceQuadrotorScenarioRunner<
            ImuRateDampingDriveCut,
            FixedQuadMotorNerve
        >(
            parameters: parameters,
            mixer: mixer,
            schedule: try SimulationSchedule.baseline(cutPeriodSteps: 1),
            determinism: .tier1Baseline,
            canonicalExecutor: executor
        )

        let log = try await runner.runScenario(
            definition: definition,
            cut: cut,
            motorNerve: motorNerve
        )

        #expect(log.events.isEmpty == false)
        #expect(executor.generalizedForceCount > 0)
        #expect(executor.derivativeCount > 0)
        #expect(executor.observablesCount > 0)
    }

    private func scenario() throws -> ReferenceQuadrotorScenarioDefinition {
        let timeStep = try TimeStep(delta: 0.001)
        return ReferenceQuadrotorScenarioDefinition(
            config: try ScenarioConfig(
                id: ScenarioID("KUY-MOJO-DI/ATT"),
                seed: ScenarioSeed(42),
                duration: 0.02,
                timeStep: timeStep
            ),
            kind: .hoverStart,
            initialPosition: Axis3(x: 0, y: 0, z: 2),
            initialAttitude: EulerAngles.degrees(
                roll: 5,
                pitch: 0,
                yaw: 0
            ),
            initialAngularVelocity: Axis3(x: 0, y: 0, z: 0),
            safetyEnvelope: try SafetyEnvelope(
                omegaSafeMax: 20,
                tiltSafeMaxDegrees: 60,
                sustainedViolationSeconds: 0.2,
                groundZ: 0,
                fallDurationSeconds: 0.5,
                fallVelocityThreshold: 0.05
            ),
            torqueEvents: [],
            actuatorDegradation: nil,
            gyroDriftScale: 1,
            swapEvents: [],
            hfEvents: []
        )
    }
}

private final class EnvironmentRecordingCanonicalExecutor:
    ReferenceQuadrotorCanonicalExecuting,
    Sendable
{
    private struct InvocationCounts: Sendable {
        var generalizedForce = 0
        var derivative = 0
        var observables = 0
    }

    let executorVersion: String
    var generalizedForceCount: Int {
        counts.withLock { $0.generalizedForce }
    }
    var derivativeCount: Int {
        counts.withLock { $0.derivative }
    }
    var observablesCount: Int {
        counts.withLock { $0.observables }
    }

    private let base = ReferenceQuadrotorScalarDynamicsExecutor()
    private let counts = Mutex(InvocationCounts())

    init(executorVersion: String = "environment-recording-scalar-v1") {
        self.executorVersion = executorVersion
    }

    func generalizedForce(
        program: CanonicalDynamicsProgram,
        state: ReferenceQuadrotorState,
        parameters: ReferenceQuadrotorParameters,
        mixer: ReferenceQuadrotorMixer,
        motorThrusts: MotorThrusts,
        disturbances: DisturbanceState,
        environment: WorldEnvironment,
        activeTerms: Set<QuadrotorForceTermID>
    ) throws -> QuadrotorGeneralizedForce {
        counts.withLock { $0.generalizedForce += 1 }
        return try base.generalizedForce(
            program: program,
            state: state,
            parameters: parameters,
            mixer: mixer,
            motorThrusts: motorThrusts,
            disturbances: disturbances,
            environment: environment,
            activeTerms: activeTerms
        )
    }

    func derivative(
        program: CanonicalDynamicsProgram,
        state: ReferenceQuadrotorState,
        parameters: ReferenceQuadrotorParameters,
        force: QuadrotorGeneralizedForce
    ) throws -> ReferenceQuadrotorStateDerivative {
        counts.withLock { $0.derivative += 1 }
        return try base.derivative(
            program: program,
            state: state,
            parameters: parameters,
            force: force
        )
    }

    func observables(
        program: CanonicalDynamicsProgram,
        state: ReferenceQuadrotorState,
        parameters: ReferenceQuadrotorParameters,
        environment: WorldEnvironment,
        force: QuadrotorGeneralizedForce
    ) throws -> ReferenceQuadrotorCanonicalObservables {
        counts.withLock { $0.observables += 1 }
        return try base.observables(
            program: program,
            state: state,
            parameters: parameters,
            environment: environment,
            force: force
        )
    }
}
