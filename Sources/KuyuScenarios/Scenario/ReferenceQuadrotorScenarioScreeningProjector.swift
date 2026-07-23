import KuyuCore

public struct ReferenceQuadrotorScenarioScreeningProjector: Sendable {
    public enum ProjectionError: Error, Sendable, Equatable {
        case invalidMaximumControlSteps(Int)
        case invalidControlPeriodSteps(UInt64)
    }

    public init() {}

    public func projected(
        _ definition: ReferenceQuadrotorScenarioDefinition,
        maximumControlStepsPerEpisode: Int,
        controlPeriodSteps: UInt64
    ) throws -> ReferenceQuadrotorScenarioDefinition {
        guard maximumControlStepsPerEpisode > 0 else {
            throw ProjectionError.invalidMaximumControlSteps(maximumControlStepsPerEpisode)
        }
        guard controlPeriodSteps > 0 else {
            throw ProjectionError.invalidControlPeriodSteps(controlPeriodSteps)
        }
        let maximumDuration = Double(maximumControlStepsPerEpisode)
            * Double(controlPeriodSteps)
            * definition.config.timeStep.delta
        let projectedDuration = min(definition.config.duration, maximumDuration)
        guard projectedDuration < definition.config.duration else {
            return definition
        }
        return ReferenceQuadrotorScenarioDefinition(
            config: try ScenarioConfig(
                id: try ScenarioID(
                    "\(definition.config.id.rawValue)-SCREEN-\(maximumControlStepsPerEpisode)"
                ),
                seed: definition.config.seed,
                duration: projectedDuration,
                timeStep: definition.config.timeStep
            ),
            kind: definition.kind,
            initialPosition: definition.initialPosition,
            initialAttitude: definition.initialAttitude,
            initialAngularVelocity: definition.initialAngularVelocity,
            safetyEnvelope: definition.safetyEnvelope,
            liftEnvelope: definition.liftEnvelope,
            torqueEvents: definition.torqueEvents,
            actuatorDegradation: definition.actuatorDegradation,
            gyroDriftScale: definition.gyroDriftScale,
            swapEvents: definition.swapEvents,
            hfEvents: definition.hfEvents
        )
    }
}
