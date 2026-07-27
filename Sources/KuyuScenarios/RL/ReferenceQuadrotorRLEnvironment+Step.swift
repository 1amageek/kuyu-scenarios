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
        return try performStep(realizedAction: action)
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
        return try performStep(realizedAction: realizedAction)
    }
}
