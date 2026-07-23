import KuyuCore
import KuyuPhysics

extension ReferenceQuadrotorRLEnvironment {
    struct ControlIntervalResult {
        let reward: Double
        let failure: FailureEvent?
        let finalLog: WorldStepLog
        let advancedPhysicsStepCount: Int
        let costMeasurement: EnvironmentCostMeasurement?
    }

    mutating func advanceControlInterval(
        simulator: inout SimulatorStorage,
        monitor: inout SafetyFailurePolicy,
        definition: ReferenceQuadrotorScenarioDefinition,
        physicsStepsThisControl: Int,
        remainingPhysicsSteps: Int
    ) throws -> ControlIntervalResult {
        var intervalReward = 0.0
        var failure: FailureEvent?
        var finalLog: WorldStepLog?
        var advancedPhysicsStepCount = 0
        var integratedSafetyCost = 0.0
        let lastPhysicsStep = physicsStepsThisControl - 1
        for physicsStepIndex in 0..<physicsStepsThisControl {
            let physicsLog = try advanceHoldingControl(
                simulator: &simulator,
                definition: definition
            )
            let stepFailure = monitor.update(log: physicsLog)
            advancedPhysicsStepCount += 1
            let stepTruncated = stepFailure == nil
                && physicsStepIndex == lastPhysicsStep
                && physicsStepsThisControl == remainingPhysicsSteps
            intervalReward += try rewardFunction.reward(
                scenario: definition,
                log: physicsLog,
                failure: stepFailure,
                truncated: stepTruncated
            )
            if let safetyCostFunction {
                integratedSafetyCost += try safetyCostFunction.cost(
                    scenario: definition,
                    log: physicsLog,
                    duration: definition.config.timeStep.delta,
                    failure: stepFailure,
                    truncated: stepTruncated
                )
            }
            finalLog = physicsLog
            if let stepFailure {
                failure = stepFailure
                break
            }
        }
        guard let finalLog else { throw EnvironmentError.invalidStepLimit }
        let costMeasurement = try safetyCostFunction.map {
            try EnvironmentCostMeasurement(
                descriptor: $0.descriptor,
                value: integratedSafetyCost,
                duration: definition.config.timeStep.delta * Double(advancedPhysicsStepCount),
                physicsTickCount: advancedPhysicsStepCount
            )
        }
        return ControlIntervalResult(
            reward: intervalReward,
            failure: failure,
            finalLog: finalLog,
            advancedPhysicsStepCount: advancedPhysicsStepCount,
            costMeasurement: costMeasurement
        )
    }
}
