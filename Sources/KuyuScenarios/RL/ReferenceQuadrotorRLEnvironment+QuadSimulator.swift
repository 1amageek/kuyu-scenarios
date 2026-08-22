import KuyuCore
import KuyuPhysics

extension ReferenceQuadrotorRLEnvironment {
    func makeQuadSimulator(
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
        let plant = try ReferenceQuadrotorPlantEngine(
            parameters: parameters,
            mixer: mixer,
            store: store,
            timeStep: timeStep,
            environment: worldEnvironment,
            canonicalExecutor: canonicalExecutor
        )
        let sensor = try quadSensor(definition: definition, store: store, scaledNoise: scaledNoise)
        return try WorldSimulator(
            config: simulationConfig,
            disturbance: disturbance(definition: definition, store: store),
            actuator: actuator,
            plant: plant,
            sensor: sensor,
            cut: EnvironmentActionCut(),
            motorNerve: quadMotorNerve(baseMaxThrusts: baseMaxThrusts)
        )
    }

    func quadSensor(
        definition: ReferenceQuadrotorScenarioDefinition,
        store: ReferenceQuadrotorWorldStore,
        scaledNoise: IMU6NoiseConfig
    ) throws -> QuadSensor {
        let baseSensor = try imuSensor(definition: definition, store: store, scaledNoise: scaledNoise)
        return SwappableSensorField(
            base: baseSensor,
            swapEvents: definition.swapEvents,
            hfEvents: definition.hfEvents,
            baseNoise: scaledNoise,
            seed: definition.config.seed.rawValue,
            stateChannelStore: definition.kind == .liftHover ? store : nil
        )
    }

    func quadMotorNerve(baseMaxThrusts: MotorMaxThrusts) -> ReferenceQuadrotorEnvironmentMotorNerve {
        switch actionRealization {
        case .driveMixer:
            .driveMixer(
                FixedQuadMotorNerve(
                    config: FixedQuadMotorNerve.Config(
                        mixer: mixer,
                        motorMaxThrusts: baseMaxThrusts,
                        rateLimitPerSecond: motorNerveRateLimitPerSecond,
                        smoothingTimeConstant: motorNerveSmoothingTimeConstant
                    )
                )
            )
        case .temporalCTBR:
            .normalizedMotor(
                FixedQuadNormalizedMotorNerve(
                    config: FixedQuadNormalizedMotorNerve.Config(
                        motorMaxThrusts: baseMaxThrusts,
                        rateLimitPerSecond: motorNerveRateLimitPerSecond,
                        smoothingTimeConstant: motorNerveSmoothingTimeConstant
                    )
                )
            )
        }
    }
}
