import KuyuCore
import KuyuPhysics

extension ReferenceQuadrotorRLEnvironment {
    func makeLiftSimulator(
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
        let plant = try ReferenceQuadrotorPlantEngine(
            parameters: parameters,
            mixer: mixer,
            store: store,
            timeStep: timeStep,
            environment: worldEnvironment
        )
        let sensor = SwappableSensorField(
            base: try imuSensor(definition: definition, store: store, scaledNoise: scaledNoise),
            swapEvents: definition.swapEvents,
            hfEvents: definition.hfEvents,
            baseNoise: scaledNoise,
            seed: definition.config.seed.rawValue,
            stateChannelStore: store
        )
        return try WorldSimulator(
            config: simulationConfig,
            disturbance: disturbance(definition: definition, store: store),
            actuator: actuator,
            plant: plant,
            sensor: sensor,
            cut: EnvironmentActionCut(),
            motorNerve: LiftMotorNerve(motorMaxThrusts: baseMaxThrusts)
        )
    }
}
