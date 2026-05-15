import KuyuCore
import KuyuPhysics

public enum KuyAtt1BaselineMode: String, Sendable, Codable {
    case sensor
    case teacher
}

public struct KuyAtt1Runner {
    public var parameters: ReferenceQuadrotorParameters
    public var mixer: ReferenceQuadrotorMixer
    public var schedule: SimulationSchedule
    public var determinism: DeterminismConfig
    public var noise: IMU6NoiseConfig
    public var environment: WorldEnvironment
    public var gains: ImuRateDampingCutGains
    public var baselineMode: KuyAtt1BaselineMode

    public init(
        parameters: ReferenceQuadrotorParameters = .baseline,
        mixer: ReferenceQuadrotorMixer? = nil,
        schedule: SimulationSchedule,
        determinism: DeterminismConfig,
        noise: IMU6NoiseConfig = .zero,
        environment: WorldEnvironment = .standard,
        gains: ImuRateDampingCutGains,
        baselineMode: KuyAtt1BaselineMode = .sensor
    ) {
        self.parameters = parameters
        self.mixer = mixer ?? ReferenceQuadrotorMixer(armLength: parameters.armLength, yawCoefficient: parameters.yawCoefficient)
        self.schedule = schedule
        self.determinism = determinism
        self.noise = noise
        self.environment = environment
        self.gains = gains
        self.baselineMode = baselineMode
    }

    public static func teacherBaseline(
        gains: ImuRateDampingCutGains,
        noise: IMU6NoiseConfig = .zero,
        cutPeriodSteps: UInt64 = 2
    ) throws -> KuyAtt1Runner {
        let schedule = try SimulationSchedule.baseline(cutPeriodSteps: cutPeriodSteps)
        return KuyAtt1Runner(
            schedule: schedule,
            determinism: .tier1Baseline,
            noise: noise,
            gains: gains,
            baselineMode: .teacher
        )
    }

    public static func baseline(
        gains: ImuRateDampingCutGains,
        noise: IMU6NoiseConfig = .zero,
        cutPeriodSteps: UInt64 = 2
    ) throws -> KuyAtt1Runner {
        try teacherBaseline(gains: gains, noise: noise, cutPeriodSteps: cutPeriodSteps)
    }

    public func run(
        referenceLogs: [ScenarioKey: SimulationLog] = [:],
        control: SimulationControl? = nil
    ) async throws -> SuiteRunResult {
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
            control: control
        )
    }

    public func runWithLogs(
        referenceLogs: [ScenarioKey: SimulationLog] = [:],
        control: SimulationControl? = nil
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

        let validation = KuyAtt1Validation(runner: runner)
        let output = try await validation.runWithLogs(
            cutFactory: { definition in try makeDriveCut(definition: definition) },
            motorNerveFactory: { _ in try makeMotorNerve() },
            referenceLogs: referenceLogs,
            control: control
        )

        let aggregate = EvaluationAggregate.from(evaluations: output.result.evaluations)
        let summary = ValidationSummary(
            suitePassed: output.result.passed,
            evaluations: output.result.evaluations,
            replayChecks: output.result.replayChecks,
            manifest: output.manifest,
            aggregate: aggregate
        )

        return KuyAtt1RunOutput(result: output.result, summary: summary, logs: output.logs)
    }

    private func makeDriveCut(definition: ReferenceQuadrotorScenarioDefinition) throws -> ImuRateDampingDriveCut {
        let hoverThrust = parameters.mass * parameters.gravity / 4.0 * gains.hoverThrustScale
        let initialAttitude: EulerAngles
        let tiltCorrectionTimeConstant: Double?

        switch baselineMode {
        case .teacher:
            initialAttitude = definition.initialAttitude
            tiltCorrectionTimeConstant = nil
        case .sensor:
            initialAttitude = EulerAngles(roll: 0, pitch: 0, yaw: 0)
            tiltCorrectionTimeConstant = 0.4
        }

        return try ImuRateDampingDriveCut(
            hoverThrust: hoverThrust,
            kp: gains.kp,
            kd: gains.kd,
            yawDamping: gains.yawDamping,
            armLength: parameters.armLength,
            yawCoefficient: parameters.yawCoefficient,
            maxThrust: parameters.maxThrust,
            initialRoll: initialAttitude.roll,
            initialPitch: initialAttitude.pitch,
            tiltCorrectionTimeConstant: tiltCorrectionTimeConstant
        )
    }

    private func makeMotorNerve() throws -> FixedQuadMotorNerve {
        let maxThrusts = try MotorMaxThrusts.uniform(parameters.maxThrust)
        let config: FixedQuadMotorNerve.Config

        switch baselineMode {
        case .teacher:
            config = FixedQuadMotorNerve.Config(
                mixer: mixer,
                motorMaxThrusts: maxThrusts,
                rateLimitPerSecond: 100.0,
                smoothingTimeConstant: nil
            )
        case .sensor:
            config = FixedQuadMotorNerve.Config(
                mixer: mixer,
                motorMaxThrusts: maxThrusts
            )
        }

        return FixedQuadMotorNerve(config: config)
    }
}
