import KuyuCore
import KuyuPhysics

public struct ReferenceQuadrotorRLEnvironment: KuyuEnvironment {
    public enum EnvironmentError: Error, Equatable {
        case unsupportedScenarioKind(ReferenceQuadrotorScenarioKind)
        case unsupportedRobotShape(robotID: String, driveCount: Int, actuatorCount: Int)
        case seedMismatch(expected: UInt64, actual: UInt64)
        case notReset
        case episodeTerminated
        case invalidStepLimit
        case unsupportedSingleLiftStress(String)
        case unsupportedControlSchedule(
            cutPeriodSteps: UInt64,
            motorNervePeriodSteps: UInt64?,
            sensorPeriodSteps: UInt64
        )
        case actionContractMismatch(
            expected: ReferenceQuadrotorActionKind,
            received: ReferenceQuadrotorActionKind
        )
        case unsupportedActionRealization(
            actionKind: ReferenceQuadrotorActionKind,
            scenarioKind: ReferenceQuadrotorScenarioKind
        )
        case invalidMotorNerveRateLimit(Double)
    }

    typealias QuadActuator = SwappableActuatorEngine<ActuatorDegradationEngine>
    typealias QuadSensor = SwappableSensorField<IMU6SensorField>
    typealias SingleActuator = SwappableActuatorEngine<SinglePropActuatorEngine>
    typealias SingleSensor = SwappableSensorField<SinglePropIMU6SensorField>
    typealias QuadSimulator = WorldSimulator<
        TorqueDisturbanceField,
        QuadActuator,
        ReferenceQuadrotorPlantEngine,
        QuadSensor,
        EnvironmentActionCut,
        ReferenceQuadrotorEnvironmentMotorNerve
    >
    typealias LiftSimulator = WorldSimulator<
        TorqueDisturbanceField,
        QuadActuator,
        ReferenceQuadrotorPlantEngine,
        QuadSensor,
        EnvironmentActionCut,
        LiftMotorNerve
    >
    typealias SingleSimulator = WorldSimulator<
        TorqueDisturbanceField,
        SingleActuator,
        SinglePropPlantEngine,
        SingleSensor,
        EnvironmentActionCut,
        FixedSinglePropMotorNerve
    >

    enum SimulatorStorage {
        case quad(QuadSimulator)
        case lift(LiftSimulator)
        case single(SingleSimulator)
    }

    public var parameters: ReferenceQuadrotorParameters
    public var mixer: ReferenceQuadrotorMixer
    public var schedule: SimulationSchedule
    public var determinism: DeterminismConfig
    public var noise: IMU6NoiseConfig
    public var worldEnvironment: WorldEnvironment
    public var hoverThrustScale: Double
    public var robotManifestID: String?
    public var driveCount: Int
    public var actuatorCount: Int
    public var actionRealization: ReferenceQuadrotorActionRealization
    public var motorNerveRateLimitPerSecond: Double
    public var motorNerveSmoothingTimeConstant: Double?
    public var worldModelAdapter: (any WorldModelEnvironmentAdapter)?
    public var worldModelAdapterConfiguration: WorldModelAdapterConfiguration

    let rewardFunction: ReferenceQuadrotorDenseReward
    let safetyCostFunction: ReferenceQuadrotorSafetyCost?
    var simulator: SimulatorStorage?
    var monitor: SafetyFailurePolicy?
    var definition: ReferenceQuadrotorScenarioDefinition?
    var configHash: String?
    var maxSteps: Int = 0
    var physicsStepsRemaining: Int = 0
    var stepCount: Int = 0
    var rewardSum: Double = 0.0
    var terminalFailure: FailureEvent?
    var terminalReason: String?
    var terminated: Bool = false
    var currentObservation: EnvironmentObservation?
    public internal(set) var activeExecutionContract: ReferenceQuadrotorEnvironmentExecutionContract?

    public init(
        parameters: ReferenceQuadrotorParameters = .baseline,
        mixer: ReferenceQuadrotorMixer? = nil,
        schedule: SimulationSchedule,
        determinism: DeterminismConfig,
        noise: IMU6NoiseConfig = .zero,
        worldEnvironment: WorldEnvironment = .standard,
        hoverThrustScale: Double = 1.0,
        robotManifestID: String? = nil,
        driveCount: Int = 4,
        actuatorCount: Int = 4,
        actionRealization: ReferenceQuadrotorActionRealization = .driveMixer,
        motorNerveRateLimitPerSecond: Double = 2.0,
        motorNerveSmoothingTimeConstant: Double? = 0.08,
        rewardFunction: ReferenceQuadrotorDenseReward = ReferenceQuadrotorDenseReward(),
        safetyCostFunction: ReferenceQuadrotorSafetyCost? = nil,
        worldModelAdapter: (any WorldModelEnvironmentAdapter)? = nil,
        worldModelAdapterConfiguration: WorldModelAdapterConfiguration = WorldModelAdapterConfiguration()
    ) {
        self.parameters = parameters
        self.mixer = mixer ?? ReferenceQuadrotorMixer(armLength: parameters.armLength, yawCoefficient: parameters.yawCoefficient)
        self.schedule = schedule
        self.determinism = determinism
        self.noise = noise
        self.worldEnvironment = worldEnvironment
        self.hoverThrustScale = hoverThrustScale
        self.robotManifestID = robotManifestID
        self.driveCount = driveCount
        self.actuatorCount = actuatorCount
        self.actionRealization = actionRealization
        self.motorNerveRateLimitPerSecond = motorNerveRateLimitPerSecond
        self.motorNerveSmoothingTimeConstant = motorNerveSmoothingTimeConstant
        self.worldModelAdapter = worldModelAdapter
        self.worldModelAdapterConfiguration = worldModelAdapterConfiguration
        self.rewardFunction = rewardFunction
        self.safetyCostFunction = safetyCostFunction
        self.activeExecutionContract = nil
    }

    public init(
        loadedRobot: LoadedKuyuRobot,
        schedule: SimulationSchedule,
        determinism: DeterminismConfig,
        noise: IMU6NoiseConfig = .zero,
        worldEnvironment: WorldEnvironment = .standard,
        hoverThrustScale: Double = 1.0,
        actionRealization: ReferenceQuadrotorActionRealization = .driveMixer,
        motorNerveRateLimitPerSecond: Double = 2.0,
        motorNerveSmoothingTimeConstant: Double? = 0.08,
        rewardFunction: ReferenceQuadrotorDenseReward = ReferenceQuadrotorDenseReward(),
        safetyCostFunction: ReferenceQuadrotorSafetyCost? = nil,
        worldModelAdapter: (any WorldModelEnvironmentAdapter)? = nil,
        worldModelAdapterConfiguration: WorldModelAdapterConfiguration = WorldModelAdapterConfiguration()
    ) throws {
        let embodiment = loadedRobot.embodiment
        let driveCount = embodiment.control.driveChannels.count
        let actuatorCount = embodiment.signals.actuator.count
        guard (driveCount == 4 && actuatorCount == 4) || (driveCount == 1 && actuatorCount == 1) else {
            throw EnvironmentError.unsupportedRobotShape(
                robotID: loadedRobot.manifest.robotID,
                driveCount: driveCount,
                actuatorCount: actuatorCount
            )
        }

        guard motorNerveRateLimitPerSecond.isFinite, motorNerveRateLimitPerSecond > 0 else {
            throw EnvironmentError.invalidMotorNerveRateLimit(motorNerveRateLimitPerSecond)
        }
        let embodimentResolution = try ReferenceQuadrotorEmbodimentResolver().resolution(for: loadedRobot)
        let effectiveRateLimit = min(
            motorNerveRateLimitPerSecond,
            embodimentResolution.normalizedActuatorRateLimitPerSecond
        )

        self.init(
            parameters: embodimentResolution.parameters,
            schedule: schedule,
            determinism: determinism,
            noise: noise,
            worldEnvironment: worldEnvironment,
            hoverThrustScale: hoverThrustScale,
            robotManifestID: loadedRobot.manifest.robotID,
            driveCount: driveCount,
            actuatorCount: actuatorCount,
            actionRealization: actionRealization,
            motorNerveRateLimitPerSecond: effectiveRateLimit,
            motorNerveSmoothingTimeConstant: motorNerveSmoothingTimeConstant,
            rewardFunction: rewardFunction,
            safetyCostFunction: safetyCostFunction,
            worldModelAdapter: worldModelAdapter,
            worldModelAdapterConfiguration: worldModelAdapterConfiguration
        )
    }
}
