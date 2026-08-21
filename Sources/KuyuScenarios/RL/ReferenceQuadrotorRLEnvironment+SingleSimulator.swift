import KuyuCore
import KuyuPhysics

extension ReferenceQuadrotorRLEnvironment {
    func makeSingleSimulator(
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
        let plant = try SinglePropPlantEngine(
            parameters: parameters,
            store: store,
            timeStep: timeStep,
            environment: worldEnvironment
        )
        return try WorldSimulator(
            config: simulationConfig,
            disturbance: TorqueDisturbanceField(events: [], hfEvents: definition.hfEvents, store: store),
            actuator: actuator,
            plant: plant,
            sensor: try singleSensor(definition: definition, store: store, scaledNoise: scaledNoise),
            cut: EnvironmentActionCut(),
            motorNerve: fixedSinglePropMotorNerve()
        )
    }

    func singleSensor(
        definition: ReferenceQuadrotorScenarioDefinition,
        store: ReferenceQuadrotorWorldStore,
        scaledNoise: IMU6NoiseConfig
    ) throws -> SingleSensor {
        let baseSensor = try SinglePropIMU6SensorField(
            parameters: parameters,
            store: store,
            timeStep: definition.config.timeStep,
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
        return SwappableSensorField(
            base: baseSensor,
            swapEvents: definition.swapEvents,
            hfEvents: definition.hfEvents,
            baseNoise: scaledNoise,
            seed: definition.config.seed.rawValue
        )
    }

    func fixedSinglePropMotorNerve() -> FixedSinglePropMotorNerve {
        FixedSinglePropMotorNerve(
            config: FixedSinglePropMotorNerve.Config(
                maxThrust: parameters.maxThrust,
                rateLimitPerSecond: motorNerveRateLimitPerSecond,
                smoothingTimeConstant: motorNerveSmoothingTimeConstant
            )
        )
    }
}
