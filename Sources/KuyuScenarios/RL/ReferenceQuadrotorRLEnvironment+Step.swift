import KuyuPhysics
import KuyuCore

public extension ReferenceQuadrotorRLEnvironment {
    mutating func step(action: EnvironmentAction) throws -> EnvironmentStep {
        guard actionRealization.kind == .driveMixer else {
            throw EnvironmentError.actionContractMismatch(
                expected: actionRealization.kind,
                received: .driveMixer
            )
        }
        return try step(realizedAction: action)
    }
    mutating func step(command: ReferenceQuadrotorCTBRCommand) throws -> EnvironmentStep {
        guard case .temporalCTBR(let controlConfig) = actionRealization else {
            throw EnvironmentError.actionContractMismatch(
                expected: actionRealization.kind,
                received: .temporalCTBR
            )
        }
        guard let actionObservation = currentObservation else {
            throw EnvironmentError.notReset
        }
        let motorCommand = try ReferenceQuadrotorCTBRControlLaw(config: controlConfig).motorCommand(
            for: command,
            currentAngularVelocity: actionObservation.plantState.root.angularVelocity,
            parameters: parameters,
            mixer: mixer
        )
        let realizedAction = EnvironmentAction.driveIntents(
            try motorCommand.values.enumerated().map { index, value in
                try DriveIntent(index: DriveIndex(UInt32(index)), activation: value)
            },
            corrections: []
        )
        return try step(realizedAction: realizedAction)
    }
    private mutating func step(realizedAction action: EnvironmentAction) throws -> EnvironmentStep {
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
