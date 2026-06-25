import KuyuCore
import KuyuPhysics

public enum KuyAtt1BaselineMode: String, Sendable, Codable {
    case sensor
    case teacher
}

public struct KuyAtt1Runner {
    private enum RunnerError: Error, Sendable, Equatable {
        case emptyActiveTeacherScenario(String, UInt64)
    }

    public var parameters: ReferenceQuadrotorParameters
    public var mixer: ReferenceQuadrotorMixer
    public var schedule: SimulationSchedule
    public var determinism: DeterminismConfig
    public var noise: IMU6NoiseConfig
    public var environment: WorldEnvironment
    public var gains: ImuRateDampingCutGains
    public var baselineMode: KuyAtt1BaselineMode
    public var teacherConfig: PrivilegedAltitudeHoldTeacherConfig
    /// When enabled, every scenario is re-run with a fresh controller and the
    /// two logs are compared via `ReplayChecker`; the suite passes only if all
    /// replay checks pass. Disable only where the duplicated rollout cost is
    /// unjustified (interactive UI runs, training rollouts).
    public var replayVerification: Bool

    public init(
        parameters: ReferenceQuadrotorParameters = .baseline,
        mixer: ReferenceQuadrotorMixer? = nil,
        schedule: SimulationSchedule,
        determinism: DeterminismConfig,
        noise: IMU6NoiseConfig = .zero,
        environment: WorldEnvironment = .standard,
        gains: ImuRateDampingCutGains,
        baselineMode: KuyAtt1BaselineMode = .sensor,
        teacherConfig: PrivilegedAltitudeHoldTeacherConfig = .activeAltitudeHold,
        replayVerification: Bool = true
    ) {
        self.parameters = parameters
        self.mixer = mixer ?? ReferenceQuadrotorMixer(armLength: parameters.armLength, yawCoefficient: parameters.yawCoefficient)
        self.schedule = schedule
        self.determinism = determinism
        self.noise = noise
        self.environment = environment
        self.gains = gains
        self.baselineMode = baselineMode
        self.teacherConfig = teacherConfig
        self.replayVerification = replayVerification
    }

    public static func activeAltitudeHoldTeacher(
        gains: ImuRateDampingCutGains,
        noise: IMU6NoiseConfig = .zero,
        cutPeriodSteps: UInt64 = 2,
        teacherConfig: PrivilegedAltitudeHoldTeacherConfig = .activeAltitudeHold,
        replayVerification: Bool = true
    ) throws -> KuyAtt1Runner {
        let schedule = try SimulationSchedule.baseline(cutPeriodSteps: cutPeriodSteps)
        return KuyAtt1Runner(
            schedule: schedule,
            determinism: .tier1Baseline,
            noise: noise,
            gains: gains,
            baselineMode: .teacher,
            teacherConfig: teacherConfig,
            replayVerification: replayVerification
        )
    }

    public func run(
        referenceLogs: [ScenarioKey: SimulationLog] = [:],
        definitions overrideDefinitions: [ReferenceQuadrotorScenarioDefinition]? = nil,
        control: SimulationControl? = nil,
        telemetry: WorldStepTelemetry? = nil
    ) async throws -> SuiteRunResult {
        if baselineMode == .teacher {
            return try await runActiveTeacherWithLogs(
                referenceLogs: referenceLogs,
                definitions: overrideDefinitions,
                control: control,
                telemetry: telemetry
            ).result
        }
        if let overrideDefinitions {
            return try await runSensorWithLogs(
                referenceLogs: referenceLogs,
                definitions: overrideDefinitions,
                control: control,
                telemetry: telemetry
            ).result
        }

        let runner = ReferenceQuadrotorScenarioRunner<ImuRateDampingDriveCut, FixedQuadMotorNerve>(
            parameters: parameters,
            mixer: mixer,
            schedule: schedule,
            determinism: determinism,
            noise: noise,
            environment: environment,
            hoverThrustScale: gains.hoverThrustScale
        )

        let validation = KuyAtt1Validation(runner: runner)

        return try await validation.run(
            cutFactory: { definition in try makeDriveCut(definition: definition) },
            motorNerveFactory: { _ in try makeMotorNerve() },
            referenceLogs: referenceLogs,
            control: control,
            telemetry: telemetry
        )
    }

    public func runWithLogs(
        referenceLogs: [ScenarioKey: SimulationLog] = [:],
        definitions overrideDefinitions: [ReferenceQuadrotorScenarioDefinition]? = nil,
        control: SimulationControl? = nil,
        telemetry: WorldStepTelemetry? = nil
    ) async throws -> KuyAtt1RunOutput {
        if baselineMode == .teacher {
            return try await runActiveTeacherWithLogs(
                referenceLogs: referenceLogs,
                definitions: overrideDefinitions,
                control: control,
                telemetry: telemetry
            )
        }
        if let overrideDefinitions {
            return try await runSensorWithLogs(
                referenceLogs: referenceLogs,
                definitions: overrideDefinitions,
                control: control,
                telemetry: telemetry
            )
        }

        let runner = ReferenceQuadrotorScenarioRunner<ImuRateDampingDriveCut, FixedQuadMotorNerve>(
            parameters: parameters,
            mixer: mixer,
            schedule: schedule,
            determinism: determinism,
            noise: noise,
            environment: environment,
            hoverThrustScale: gains.hoverThrustScale
        )

        let validation = KuyAtt1Validation(runner: runner)
        let output = try await validation.runWithLogs(
            cutFactory: { definition in try makeDriveCut(definition: definition) },
            motorNerveFactory: { _ in try makeMotorNerve() },
            referenceLogs: referenceLogs,
            control: control,
            telemetry: telemetry
        )

        let aggregate = EvaluationAggregate.from(evaluations: output.result.evaluations)
        let summary = ValidationSummary(
            suitePassed: output.result.passed,
            evaluations: output.result.evaluations,
            replay: output.result.replay,
            manifest: output.manifest,
            aggregate: aggregate
        )

        return KuyAtt1RunOutput(result: output.result, summary: summary, logs: output.logs)
    }

    private func runActiveTeacherWithLogs(
        referenceLogs: [ScenarioKey: SimulationLog] = [:],
        definitions overrideDefinitions: [ReferenceQuadrotorScenarioDefinition]? = nil,
        control: SimulationControl? = nil,
        telemetry: WorldStepTelemetry? = nil
    ) async throws -> KuyAtt1RunOutput {
        let definitions = try overrideDefinitions ?? KuyAtt1Suite().scenarios()
        let manifest = ReferenceQuadrotorScenarioManifestBuilder().build(from: definitions)
        let replayChecker = ReplayChecker()
        var evaluations: [ScenarioEvaluation] = []
        var replayChecks: [ReplayCheckResult] = []
        var logs: [ScenarioLogEntry] = []
        evaluations.reserveCapacity(definitions.count)
        logs.reserveCapacity(definitions.count)

        for definition in definitions {
            if let control {
                try await control.checkpoint()
            }
            let log = try await runActiveTeacherScenario(
                definition: definition,
                control: control,
                telemetry: telemetry
            )
            let key = ScenarioKey(scenarioId: definition.config.id, seed: definition.config.seed)
            logs.append(ScenarioLogEntry(key: key, log: log))
            evaluations.append(ReferenceQuadrotorScenarioEvaluator().evaluate(definition: definition, log: log))

            if replayVerification {
                if let reference = referenceLogs[key] {
                    replayChecks.append(try replayChecker.check(reference: reference, candidate: log))
                } else {
                    // A fresh environment + policy is constructed per call, so a
                    // second rollout is a valid deterministic replay candidate.
                    let replayLog = try await runActiveTeacherScenario(
                        definition: definition,
                        control: control,
                        telemetry: nil
                    )
                    replayChecks.append(try replayChecker.check(reference: log, candidate: replayLog))
                }
            }
        }

        let result = SuiteRunResultFactory().makeConfigured(
            evaluations: evaluations,
            replayChecks: replayChecks,
            replayVerificationEnabled: replayVerification,
            replaySkippedReason: "Replay verification is disabled by runner configuration."
        )
        let aggregate = EvaluationAggregate.from(evaluations: result.evaluations)
        let summary = ValidationSummary(
            suitePassed: result.passed,
            evaluations: result.evaluations,
            replay: result.replay,
            manifest: manifest,
            aggregate: aggregate
        )
        return KuyAtt1RunOutput(result: result, summary: summary, logs: logs)
    }

    private func runSensorWithLogs(
        referenceLogs: [ScenarioKey: SimulationLog] = [:],
        definitions: [ReferenceQuadrotorScenarioDefinition],
        control: SimulationControl? = nil,
        telemetry: WorldStepTelemetry? = nil
    ) async throws -> KuyAtt1RunOutput {
        let runner = ReferenceQuadrotorScenarioRunner<ImuRateDampingDriveCut, FixedQuadMotorNerve>(
            parameters: parameters,
            mixer: mixer,
            schedule: schedule,
            determinism: determinism,
            noise: noise,
            environment: environment,
            hoverThrustScale: gains.hoverThrustScale
        )
        let manifest = ReferenceQuadrotorScenarioManifestBuilder().build(from: definitions)
        let replayChecker = ReplayChecker()
        var evaluations: [ScenarioEvaluation] = []
        var replayChecks: [ReplayCheckResult] = []
        var logs: [ScenarioLogEntry] = []
        evaluations.reserveCapacity(definitions.count)
        logs.reserveCapacity(definitions.count)

        for definition in definitions {
            if let control {
                try await control.checkpoint()
            }
            let log = try await runner.runScenario(
                definition: definition,
                cut: try makeDriveCut(definition: definition),
                motorNerve: try makeMotorNerve(),
                control: control,
                telemetry: telemetry
            )
            let key = ScenarioKey(scenarioId: definition.config.id, seed: definition.config.seed)
            logs.append(ScenarioLogEntry(key: key, log: log))
            evaluations.append(ReferenceQuadrotorScenarioEvaluator().evaluate(definition: definition, log: log))

            if replayVerification {
                replayChecks.append(try await validateScenarioReplay(
                    runner: runner,
                    replayChecker: replayChecker,
                    definition: definition,
                    primaryLog: log,
                    cutFactory: { definition in try makeDriveCut(definition: definition) },
                    motorNerveFactory: { _ in try makeMotorNerve() },
                    referenceLogs: referenceLogs
                ))
            }
        }

        let result = SuiteRunResultFactory().makeConfigured(
            evaluations: evaluations,
            replayChecks: replayChecks,
            replayVerificationEnabled: replayVerification,
            replaySkippedReason: "Replay verification is disabled by runner configuration."
        )
        let aggregate = EvaluationAggregate.from(evaluations: result.evaluations)
        let summary = ValidationSummary(
            suitePassed: result.passed,
            evaluations: result.evaluations,
            replay: result.replay,
            manifest: manifest,
            aggregate: aggregate
        )
        return KuyAtt1RunOutput(result: result, summary: summary, logs: logs)
    }

    private func runActiveTeacherScenario(
        definition: ReferenceQuadrotorScenarioDefinition,
        control: SimulationControl?,
        telemetry: WorldStepTelemetry?
    ) async throws -> SimulationLog {
        var rlEnvironment = ReferenceQuadrotorRLEnvironment(
            parameters: parameters,
            mixer: mixer,
            schedule: schedule,
            determinism: determinism,
            noise: noise,
            worldEnvironment: environment,
            hoverThrustScale: gains.hoverThrustScale,
            motorNerveRateLimitPerSecond: 100.0,
            motorNerveSmoothingTimeConstant: nil
        )
        var policy = try KuyAtt1BaselineEnvironmentPolicy(
            definition: definition,
            parameters: parameters,
            gains: gains,
            mode: .teacher,
            teacherConfig: teacherConfig
        )
        var observation = try rlEnvironment.reset(seed: definition.config.seed, scenario: definition)
        var steps: [EnvironmentStep] = []
        let plannedStepCount = Int((definition.config.duration / definition.config.timeStep.delta).rounded(.down))
        steps.reserveCapacity(plannedStepCount)

        while true {
            if let control {
                try await control.checkpoint()
            }
            let action = try await policy.action(for: observation)
            let step = try rlEnvironment.step(action: action)
            telemetry?(step.log)
            steps.append(step)
            observation = step.observation
            if step.done || step.truncated {
                break
            }
        }

        guard let final = steps.last else {
            throw RunnerError.emptyActiveTeacherScenario(
                definition.config.id.rawValue,
                definition.config.seed.rawValue
            )
        }
        return SimulationLog(
            scenarioId: definition.config.id,
            seed: definition.config.seed,
            timeStep: definition.config.timeStep,
            determinism: determinism,
            configHash: final.info.configHash,
            events: steps.map(\.log),
            failureReason: final.info.failureReason,
            failureTime: final.info.failureTime,
            eventSchedule: StressEventSchedule(
                swapEvents: definition.swapEvents,
                hfEvents: definition.hfEvents
            )
        )
    }

    private func makeDriveCut(definition _: ReferenceQuadrotorScenarioDefinition) throws -> ImuRateDampingDriveCut {
        let hoverThrust = parameters.mass * parameters.gravity / 4.0 * gains.hoverThrustScale

        return try ImuRateDampingDriveCut(
            hoverThrust: hoverThrust,
            kp: gains.kp,
            kd: gains.kd,
            yawDamping: gains.yawDamping,
            armLength: parameters.armLength,
            yawCoefficient: parameters.yawCoefficient,
            maxThrust: parameters.maxThrust,
            initialRoll: 0,
            initialPitch: 0,
            tiltCorrectionTimeConstant: 0.4
        )
    }

    private func makeMotorNerve() throws -> FixedQuadMotorNerve {
        let maxThrusts = try MotorMaxThrusts.uniform(parameters.maxThrust)
        return FixedQuadMotorNerve(config: FixedQuadMotorNerve.Config(
            mixer: mixer,
            motorMaxThrusts: maxThrusts
        ))
    }
}
