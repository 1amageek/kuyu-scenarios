import KuyuCore

public extension ReferenceQuadrotorRLEnvironment {
    @discardableResult
    mutating func reset(
        seed: ScenarioSeed,
        scenario: ReferenceQuadrotorScenarioDefinition
    ) throws -> EnvironmentObservation {
        let plan = try validatedResetPlan(seed: seed, scenario: scenario)
        let simulationConfig = plan.simulationConfig
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
        let executionContract = ReferenceQuadrotorEnvironmentExecutionContract(
            simulation: simulationConfig,
            actionRealization: actionRealization,
            parameters: parameters,
            robotManifestID: robotManifestID,
            motorNerveRateLimitPerSecond: motorNerveRateLimitPerSecond,
            motorNerveSmoothingTimeConstant: motorNerveSmoothingTimeConstant
        )
        activeExecutionContract = executionContract
        configHash = try ConfigHash.hash(executionContract)
        maxSteps = plan.controlStepCount
        physicsStepsRemaining = plan.physicsStepCount
        stepCount = 0
        rewardSum = 0.0
        terminalFailure = nil
        terminalReason = nil
        terminated = false
        let observation = try initialObservation()
        currentObservation = observation
        return observation
    }
}
