import Foundation
import simd
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
    }

    private typealias QuadActuator = SwappableActuatorEngine<ActuatorDegradationEngine>
    private typealias QuadSensor = SwappableSensorField<IMU6SensorField>
    private typealias SingleActuator = SwappableActuatorEngine<SinglePropActuatorEngine>
    private typealias SingleSensor = SwappableSensorField<SinglePropIMU6SensorField>
    private typealias QuadSimulator = WorldSimulator<
        TorqueDisturbanceField,
        QuadActuator,
        ReferenceQuadrotorPlantEngine,
        QuadSensor,
        EnvironmentActionCut,
        FixedQuadMotorNerve
    >
    private typealias LiftSimulator = WorldSimulator<
        TorqueDisturbanceField,
        QuadActuator,
        ReferenceQuadrotorPlantEngine,
        QuadSensor,
        EnvironmentActionCut,
        LiftMotorNerve
    >
    private typealias SingleSimulator = WorldSimulator<
        TorqueDisturbanceField,
        SingleActuator,
        SinglePropPlantEngine,
        SingleSensor,
        EnvironmentActionCut,
        FixedSinglePropMotorNerve
    >

    private enum SimulatorStorage {
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
    public var motorNerveRateLimitPerSecond: Double
    public var motorNerveSmoothingTimeConstant: Double?
    public var worldModelAdapter: (any WorldModelEnvironmentAdapter)?
    public var worldModelAdapterConfiguration: WorldModelAdapterConfiguration

    private let rewardFunction: ReferenceQuadrotorDenseReward
    private var simulator: SimulatorStorage?
    private var monitor: SafetyFailurePolicy?
    private var definition: ReferenceQuadrotorScenarioDefinition?
    private var configHash: String?
    private var maxSteps: Int = 0
    private var stepCount: Int = 0
    private var rewardSum: Double = 0.0
    private var terminalFailure: FailureEvent?
    private var terminalReason: String?
    private var terminated: Bool = false

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
        motorNerveRateLimitPerSecond: Double = 2.0,
        motorNerveSmoothingTimeConstant: Double? = 0.08,
        rewardFunction: ReferenceQuadrotorDenseReward = ReferenceQuadrotorDenseReward(),
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
        self.motorNerveRateLimitPerSecond = motorNerveRateLimitPerSecond
        self.motorNerveSmoothingTimeConstant = motorNerveSmoothingTimeConstant
        self.worldModelAdapter = worldModelAdapter
        self.worldModelAdapterConfiguration = worldModelAdapterConfiguration
        self.rewardFunction = rewardFunction
    }

    public init(
        loadedRobot: LoadedKuyuRobot,
        schedule: SimulationSchedule,
        determinism: DeterminismConfig,
        noise: IMU6NoiseConfig = .zero,
        worldEnvironment: WorldEnvironment = .standard,
        hoverThrustScale: Double = 1.0,
        motorNerveRateLimitPerSecond: Double = 2.0,
        motorNerveSmoothingTimeConstant: Double? = 0.08,
        rewardFunction: ReferenceQuadrotorDenseReward = ReferenceQuadrotorDenseReward(),
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

        let loader = KuyuModelLoader()
        let inertial = try loader.loadPlantInertialProperties(robot: loadedRobot)
        let parameters = try ReferenceQuadrotorParameters.reference(
            from: inertial,
            robotID: loadedRobot.manifest.robotID
        )

        self.init(
            parameters: parameters,
            schedule: schedule,
            determinism: determinism,
            noise: noise,
            worldEnvironment: worldEnvironment,
            hoverThrustScale: hoverThrustScale,
            robotManifestID: loadedRobot.manifest.robotID,
            driveCount: driveCount,
            actuatorCount: actuatorCount,
            motorNerveRateLimitPerSecond: motorNerveRateLimitPerSecond,
            motorNerveSmoothingTimeConstant: motorNerveSmoothingTimeConstant,
            rewardFunction: rewardFunction,
            worldModelAdapter: worldModelAdapter,
            worldModelAdapterConfiguration: worldModelAdapterConfiguration
        )
    }

    @discardableResult
    public mutating func reset(
        seed: ScenarioSeed,
        scenario: ReferenceQuadrotorScenarioDefinition
    ) throws -> EnvironmentObservation {
        guard seed == scenario.config.seed else {
            throw EnvironmentError.seedMismatch(expected: scenario.config.seed.rawValue, actual: seed.rawValue)
        }
        let timeStep = scenario.config.timeStep
        let steps = Int((scenario.config.duration / timeStep.delta).rounded(.down))
        guard steps > 0 else { throw EnvironmentError.invalidStepLimit }

        let simulationConfig = SimulationConfig(
            scenario: scenario.config,
            schedule: schedule,
            determinism: determinism,
            environment: worldEnvironment
        )

        switch scenario.kind {
        case .singleLiftHover:
            simulator = .single(try makeSingleSimulator(definition: scenario, config: simulationConfig))
        case .liftHover:
            simulator = .lift(try makeLiftSimulator(definition: scenario, config: simulationConfig))
        case .hoverStart, .impulseTorqueShock, .sustainedWindTorque, .sensorDriftStress, .actuatorDegradation:
            simulator = .quad(try makeQuadSimulator(definition: scenario, config: simulationConfig))
        }
        monitor = SafetyFailurePolicy(
            envelope: scenario.safetyEnvelope,
            timeStep: scenario.config.timeStep.delta
        )
        definition = scenario
        configHash = try ConfigHash.hash(simulationConfig)
        maxSteps = steps
        stepCount = 0
        rewardSum = 0.0
        terminalFailure = nil
        terminalReason = nil
        terminated = false

        return try initialObservation()
    }

    public mutating func step(action: EnvironmentAction) throws -> EnvironmentStep {
        guard !terminated else { throw EnvironmentError.episodeTerminated }
        guard var simulator, var monitor, let definition, let configHash else {
            throw EnvironmentError.notReset
        }

        let log: WorldStepLog
        switch simulator {
        case .quad(var quad):
            quad.cut.action = action
            log = try quad.step(deltaTime: definition.config.timeStep.delta)
            simulator = .quad(quad)
        case .lift(var lift):
            lift.cut.action = action
            log = try lift.step(deltaTime: definition.config.timeStep.delta)
            simulator = .lift(lift)
        case .single(var single):
            single.cut.action = action
            log = try single.step(deltaTime: definition.config.timeStep.delta)
            simulator = .single(single)
        }
        let failure = monitor.update(log: log)
        let nextStepCount = stepCount + 1
        let truncated = failure == nil && nextStepCount >= maxSteps
        let done = failure != nil
        var nextTerminalFailure = terminalFailure
        var nextTerminalReason = terminalReason
        if let failure {
            nextTerminalFailure = failure
            nextTerminalReason = failure.reason.rawValue
        } else if truncated {
            nextTerminalReason = "time-limit"
        }
        let nextTerminated = done || truncated

        let reward = try rewardFunction.reward(
            scenario: definition,
            log: log,
            failure: failure,
            truncated: truncated
        )
        let nextRewardSum = rewardSum + reward

        let info = EpisodeInfo(
            scenarioId: definition.config.id,
            seed: definition.config.seed,
            configHash: configHash,
            robotManifestID: robotManifestID,
            policyId: nil,
            rewardDescriptor: rewardFunction.descriptor,
            stepCount: nextStepCount,
            rewardSum: nextRewardSum,
            failureReason: nextTerminalFailure?.reason,
            failureTime: nextTerminalFailure?.time,
            terminalReason: nextTerminalReason
        )
        let output = try EnvironmentStep(
            observation: EnvironmentObservation(log: log),
            reward: reward,
            done: done,
            truncated: truncated,
            info: info,
            log: log
        )

        try validateWorldModelPrediction(reference: output)

        self.simulator = simulator
        self.monitor = monitor
        self.stepCount = nextStepCount
        self.rewardSum = nextRewardSum
        self.terminalFailure = nextTerminalFailure
        self.terminalReason = nextTerminalReason
        self.terminated = nextTerminated
        return output
    }

    private func validateWorldModelPrediction(reference output: EnvironmentStep) throws {
        guard let worldModelAdapter else { return }
        let prediction = try worldModelAdapter.predict(reference: output)
        _ = try worldModelAdapter.validate(
            prediction: prediction,
            reference: output,
            configuration: worldModelAdapterConfiguration
        )
    }

    private mutating func initialObservation() throws -> EnvironmentObservation {
        guard var simulator else { throw EnvironmentError.notReset }
        let time = try WorldTime(stepIndex: 0, time: 0.0)
        let observation: EnvironmentObservation
        switch simulator {
        case .quad(var quad):
            let samples = try quad.sensor.sample(time: time)
            observation = EnvironmentObservation(
                time: time,
                sensorSamples: samples,
                plantState: quad.plant.snapshot(),
                safetyTrace: try quad.plant.safetyTrace(),
                actuatorTelemetry: quad.actuator.telemetrySnapshot(),
                disturbances: quad.disturbance.snapshot()
            )
            simulator = .quad(quad)
        case .lift(var lift):
            let samples = try lift.sensor.sample(time: time)
            observation = EnvironmentObservation(
                time: time,
                sensorSamples: samples,
                plantState: lift.plant.snapshot(),
                safetyTrace: try lift.plant.safetyTrace(),
                actuatorTelemetry: lift.actuator.telemetrySnapshot(),
                disturbances: lift.disturbance.snapshot()
            )
            simulator = .lift(lift)
        case .single(var single):
            let samples = try single.sensor.sample(time: time)
            observation = EnvironmentObservation(
                time: time,
                sensorSamples: samples,
                plantState: single.plant.snapshot(),
                safetyTrace: try single.plant.safetyTrace(),
                actuatorTelemetry: single.actuator.telemetrySnapshot(),
                disturbances: single.disturbance.snapshot()
            )
            simulator = .single(single)
        }
        self.simulator = simulator
        return observation
    }

    private func makeQuadSimulator(
        definition: ReferenceQuadrotorScenarioDefinition,
        config simulationConfig: SimulationConfig
    ) throws -> QuadSimulator {
        let timeStep = definition.config.timeStep
        let store = try buildStore(definition: definition)
        let scaledNoise = try scaledNoise(for: definition)
        let baseMaxThrusts = try MotorMaxThrusts.uniform(parameters.maxThrust)
        let actuatorBase = ReferenceQuadrotorActuatorEngine(
            parameters: parameters,
            store: store,
            timeStep: timeStep,
            motorMaxThrusts: baseMaxThrusts
        )
        let degraded = ActuatorDegradationEngine(
            engine: actuatorBase,
            degradation: definition.actuatorDegradation
        )
        let actuator = SwappableActuatorEngine(
            engine: degraded,
            baseMaxThrusts: baseMaxThrusts,
            swapEvents: definition.swapEvents,
            hfEvents: definition.hfEvents
        )
        let disturbance = TorqueDisturbanceField(
            events: definition.torqueEvents,
            hfEvents: definition.hfEvents,
            store: store
        )
        let plant = ReferenceQuadrotorPlantEngine(
            parameters: parameters,
            mixer: mixer,
            store: store,
            timeStep: timeStep,
            environment: worldEnvironment
        )
        let baseSensor = try IMU6SensorField(
            parameters: parameters,
            mixer: mixer,
            store: store,
            timeStep: timeStep,
            environment: worldEnvironment,
            noiseSeed: definition.config.seed.rawValue,
            gyroNoiseStdDev: scaledNoise.gyroNoiseStdDev,
            gyroBias: scaledNoise.gyroBias,
            gyroRandomWalkSigma: scaledNoise.gyroRandomWalkSigma,
            accelNoiseStdDev: scaledNoise.accelNoiseStdDev,
            accelBias: scaledNoise.accelBias,
            accelRandomWalkSigma: scaledNoise.accelRandomWalkSigma,
            delaySteps: scaledNoise.delaySteps
        )
        let sensor = SwappableSensorField(
            base: baseSensor,
            swapEvents: definition.swapEvents,
            hfEvents: definition.hfEvents,
            baseNoise: scaledNoise,
            seed: definition.config.seed.rawValue,
            stateChannelStore: definition.kind == .liftHover ? store : nil
        )
        let motorNerve = FixedQuadMotorNerve(
            config: FixedQuadMotorNerve.Config(
                mixer: mixer,
                motorMaxThrusts: baseMaxThrusts,
                rateLimitPerSecond: motorNerveRateLimitPerSecond,
                smoothingTimeConstant: motorNerveSmoothingTimeConstant
            )
        )
        return try WorldSimulator(
            config: simulationConfig,
            disturbance: disturbance,
            actuator: actuator,
            plant: plant,
            sensor: sensor,
            cut: EnvironmentActionCut(),
            motorNerve: motorNerve
        )
    }

    private func makeLiftSimulator(
        definition: ReferenceQuadrotorScenarioDefinition,
        config simulationConfig: SimulationConfig
    ) throws -> LiftSimulator {
        let timeStep = definition.config.timeStep
        let store = try buildStore(definition: definition)
        let scaledNoise = try scaledNoise(for: definition)
        let baseMaxThrusts = try MotorMaxThrusts.uniform(parameters.maxThrust)
        let actuatorBase = ReferenceQuadrotorActuatorEngine(
            parameters: parameters,
            store: store,
            timeStep: timeStep,
            motorMaxThrusts: baseMaxThrusts
        )
        let degraded = ActuatorDegradationEngine(
            engine: actuatorBase,
            degradation: definition.actuatorDegradation
        )
        let actuator = SwappableActuatorEngine(
            engine: degraded,
            baseMaxThrusts: baseMaxThrusts,
            swapEvents: definition.swapEvents,
            hfEvents: definition.hfEvents
        )
        let disturbance = TorqueDisturbanceField(
            events: definition.torqueEvents,
            hfEvents: definition.hfEvents,
            store: store
        )
        let plant = ReferenceQuadrotorPlantEngine(
            parameters: parameters,
            mixer: mixer,
            store: store,
            timeStep: timeStep,
            environment: worldEnvironment
        )
        let baseSensor = try IMU6SensorField(
            parameters: parameters,
            mixer: mixer,
            store: store,
            timeStep: timeStep,
            environment: worldEnvironment,
            noiseSeed: definition.config.seed.rawValue,
            gyroNoiseStdDev: scaledNoise.gyroNoiseStdDev,
            gyroBias: scaledNoise.gyroBias,
            gyroRandomWalkSigma: scaledNoise.gyroRandomWalkSigma,
            accelNoiseStdDev: scaledNoise.accelNoiseStdDev,
            accelBias: scaledNoise.accelBias,
            accelRandomWalkSigma: scaledNoise.accelRandomWalkSigma,
            delaySteps: scaledNoise.delaySteps
        )
        let sensor = SwappableSensorField(
            base: baseSensor,
            swapEvents: definition.swapEvents,
            hfEvents: definition.hfEvents,
            baseNoise: scaledNoise,
            seed: definition.config.seed.rawValue,
            stateChannelStore: store
        )
        let motorNerve = LiftMotorNerve(motorMaxThrusts: baseMaxThrusts)
        return try WorldSimulator(
            config: simulationConfig,
            disturbance: disturbance,
            actuator: actuator,
            plant: plant,
            sensor: sensor,
            cut: EnvironmentActionCut(),
            motorNerve: motorNerve
        )
    }

    private func makeSingleSimulator(
        definition: ReferenceQuadrotorScenarioDefinition,
        config simulationConfig: SimulationConfig
    ) throws -> SingleSimulator {
        try validateSingleLiftStress(definition: definition)
        let timeStep = definition.config.timeStep
        let store = try buildStore(definition: definition)
        let scaledNoise = try scaledNoise(for: definition)
        let baseMaxThrusts = try MotorMaxThrusts.uniform(parameters.maxThrust)
        let actuatorBase = SinglePropActuatorEngine(
            maxThrust: parameters.maxThrust,
            motorTimeConstant: parameters.motorTimeConstant,
            store: store,
            timeStep: timeStep
        )
        let actuator = SwappableActuatorEngine(
            engine: actuatorBase,
            baseMaxThrusts: baseMaxThrusts,
            swapEvents: definition.swapEvents,
            hfEvents: definition.hfEvents
        )
        let disturbance = TorqueDisturbanceField(
            events: [],
            hfEvents: definition.hfEvents,
            store: store
        )
        let plant = SinglePropPlantEngine(
            parameters: parameters,
            store: store,
            timeStep: timeStep,
            environment: worldEnvironment
        )
        let baseSensor = try SinglePropIMU6SensorField(
            parameters: parameters,
            store: store,
            timeStep: timeStep,
            environment: worldEnvironment,
            noiseSeed: definition.config.seed.rawValue,
            gyroNoiseStdDev: scaledNoise.gyroNoiseStdDev,
            gyroBias: scaledNoise.gyroBias,
            gyroRandomWalkSigma: scaledNoise.gyroRandomWalkSigma,
            accelNoiseStdDev: scaledNoise.accelNoiseStdDev,
            accelBias: scaledNoise.accelBias,
            accelRandomWalkSigma: scaledNoise.accelRandomWalkSigma,
            delaySteps: scaledNoise.delaySteps
        )
        let sensor = SwappableSensorField(
            base: baseSensor,
            swapEvents: definition.swapEvents,
            hfEvents: definition.hfEvents,
            baseNoise: scaledNoise,
            seed: definition.config.seed.rawValue
        )
        let motorNerve = FixedSinglePropMotorNerve(
            config: FixedSinglePropMotorNerve.Config(
                maxThrust: parameters.maxThrust,
                rateLimitPerSecond: motorNerveRateLimitPerSecond,
                smoothingTimeConstant: motorNerveSmoothingTimeConstant
            )
        )
        return try WorldSimulator(
            config: simulationConfig,
            disturbance: disturbance,
            actuator: actuator,
            plant: plant,
            sensor: sensor,
            cut: EnvironmentActionCut(),
            motorNerve: motorNerve
        )
    }

    private func validateSingleLiftStress(definition: ReferenceQuadrotorScenarioDefinition) throws {
        guard definition.torqueEvents.isEmpty else {
            throw EnvironmentError.unsupportedSingleLiftStress("torqueEvents")
        }
        guard definition.actuatorDegradation == nil else {
            throw EnvironmentError.unsupportedSingleLiftStress("actuatorDegradation")
        }
        for event in definition.hfEvents {
            switch event.kind {
            case .impulse, .vibration:
                throw EnvironmentError.unsupportedSingleLiftStress("hfEvent.\(event.kind.rawValue)")
            default:
                break
            }
        }
        for event in definition.swapEvents {
            switch event {
            case .actuator(let actuator) where actuator.motorIndex != 0:
                throw EnvironmentError.unsupportedSingleLiftStress("actuatorSwap.motorIndex.\(actuator.motorIndex)")
            case .sensor(let sensor) where sensor.targetChannels.contains(where: { $0 > 7 }):
                throw EnvironmentError.unsupportedSingleLiftStress("sensorSwap.targetChannel")
            default:
                break
            }
        }
    }

    private func scaledNoise(for definition: ReferenceQuadrotorScenarioDefinition) throws -> IMU6NoiseConfig {
        try IMU6NoiseConfig(
            gyroNoiseStdDev: noise.gyroNoiseStdDev,
            gyroBias: noise.gyroBias,
            gyroRandomWalkSigma: noise.gyroRandomWalkSigma * definition.gyroDriftScale,
            accelNoiseStdDev: noise.accelNoiseStdDev,
            accelBias: noise.accelBias,
            accelRandomWalkSigma: noise.accelRandomWalkSigma,
            delaySteps: noise.delaySteps
        )
    }

    private func buildStore(definition: ReferenceQuadrotorScenarioDefinition) throws -> ReferenceQuadrotorWorldStore {
        let isSingleLift = definition.kind == .singleLiftHover
        let orientation = isSingleLift
            ? simd_quatd(angle: 0.0, axis: SIMD3<Double>(0, 0, 1))
            : definition.initialAttitude.toQuaternion()
        let angularVelocity = isSingleLift
            ? SIMD3<Double>(repeating: 0.0)
            : SIMD3<Double>(
                definition.initialAngularVelocity.x,
                definition.initialAngularVelocity.y,
                definition.initialAngularVelocity.z
            )
        let initialPosition = SIMD3<Double>(
            definition.initialPosition.x,
            definition.initialPosition.y,
            definition.initialPosition.z
        )

        let state = try ReferenceQuadrotorState(
            position: initialPosition,
            velocity: SIMD3<Double>(repeating: 0),
            orientation: orientation,
            angularVelocity: angularVelocity
        )
        let hoverThrust = isSingleLift
            ? parameters.mass * parameters.gravity * hoverThrustScale
            : parameters.mass * parameters.gravity / 4.0 * hoverThrustScale
        let thrusts = isSingleLift
            ? try MotorThrusts(f1: hoverThrust, f2: 0, f3: 0, f4: 0)
            : try MotorThrusts.uniform(hoverThrust)
        return ReferenceQuadrotorWorldStore(state: state, motorThrusts: thrusts)
    }
}
