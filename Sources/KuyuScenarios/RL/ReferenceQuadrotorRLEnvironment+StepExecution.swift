import KuyuPhysics
import KuyuCore

extension ReferenceQuadrotorRLEnvironment {
    /// Advances one control interval for an action that has already been
    /// realized into drive intents. The action-contract entry points own the
    /// realization; this owns the simulate-evaluate-commit sequence.
    mutating func performStep(realizedAction action: EnvironmentAction) throws -> EnvironmentStep {
        guard !terminated else { throw EnvironmentError.episodeTerminated }
        guard var simulator,
              var monitor,
              let definition,
              let configHash,
              let actionObservation = currentObservation else {
            throw EnvironmentError.notReset
        }
        let application = try applyControl(
            simulator: &simulator,
            action: action,
            observation: actionObservation
        )
        let nextStepCount = stepCount + 1
        let controlPeriodSteps = Int(schedule.cut.periodSteps)
        let physicsStepsThisControl = min(controlPeriodSteps, physicsStepsRemaining)
        guard physicsStepsThisControl > 0 else {
            throw EnvironmentError.episodeTerminated
        }
        let interval = try advanceControlInterval(
            simulator: &simulator,
            monitor: &monitor,
            definition: definition,
            physicsStepsThisControl: physicsStepsThisControl,
            remainingPhysicsSteps: physicsStepsRemaining
        )
        let intervalReward = interval.reward
        let failure = interval.failure
        let finalPhysicsLog = interval.finalLog
        let advancedPhysicsStepCount = interval.advancedPhysicsStepCount
        let outputLog = log(finalPhysicsLog, applying: application)
        let nextPhysicsStepsRemaining = physicsStepsRemaining - advancedPhysicsStepCount
        let truncated = failure == nil && nextPhysicsStepsRemaining == 0
        let done = failure != nil
        var nextTerminalFailure = terminalFailure
        var nextTerminalReason = terminalReason
        if let failure {
            nextTerminalFailure = failure
            nextTerminalReason = failure.reason.rawValue
        } else if truncated {
            nextTerminalReason = "time-limit"
        }
        let nextRewardSum = rewardSum + intervalReward
        let output = try EnvironmentStep(
            observation: EnvironmentObservation(log: outputLog),
            reward: intervalReward,
            done: done,
            truncated: truncated,
            info: episodeInfo(
                definition: definition,
                configHash: configHash,
                stepCount: nextStepCount,
                rewardSum: nextRewardSum,
                terminalFailure: nextTerminalFailure,
                terminalReason: nextTerminalReason
            ),
            log: outputLog,
            costMeasurement: interval.costMeasurement
        )
        try validateWorldModelPrediction(reference: output)

        self.simulator = simulator
        self.monitor = monitor
        self.stepCount = nextStepCount
        self.physicsStepsRemaining = nextPhysicsStepsRemaining
        self.rewardSum = nextRewardSum
        self.terminalFailure = nextTerminalFailure
        self.terminalReason = nextTerminalReason
        self.terminated = done || truncated
        self.currentObservation = output.observation
        return output
    }
}
