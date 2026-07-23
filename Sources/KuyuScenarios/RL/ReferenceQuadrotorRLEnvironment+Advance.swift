import KuyuCore

extension ReferenceQuadrotorRLEnvironment {
    mutating func applyControl(
        simulator: inout SimulatorStorage,
        action: EnvironmentAction,
        observation: EnvironmentObservation
    ) throws -> WorldControlApplication {
        switch simulator {
        case .quad(var quad):
            quad.cut.action = action
            let application = try quad.applyControl(samples: observation.sensorSamples)
            simulator = .quad(quad)
            return application
        case .lift(var lift):
            lift.cut.action = action
            let application = try lift.applyControl(samples: observation.sensorSamples)
            simulator = .lift(lift)
            return application
        case .single(var single):
            single.cut.action = action
            let application = try single.applyControl(samples: observation.sensorSamples)
            simulator = .single(single)
            return application
        }
    }

    mutating func advanceHoldingControl(
        simulator: inout SimulatorStorage,
        definition: ReferenceQuadrotorScenarioDefinition
    ) throws -> WorldStepLog {
        switch simulator {
        case .quad(var quad):
            let log = try quad.stepHoldingControl(deltaTime: definition.config.timeStep.delta)
            simulator = .quad(quad)
            return log
        case .lift(var lift):
            let log = try lift.stepHoldingControl(deltaTime: definition.config.timeStep.delta)
            simulator = .lift(lift)
            return log
        case .single(var single):
            let log = try single.stepHoldingControl(deltaTime: definition.config.timeStep.delta)
            simulator = .single(single)
            return log
        }
    }
}
