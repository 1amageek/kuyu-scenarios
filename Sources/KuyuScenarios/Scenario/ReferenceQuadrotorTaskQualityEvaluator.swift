import Foundation
import KuyuCore

public struct ReferenceQuadrotorTaskQualityEvaluator: Sendable {
    public static let evaluatorID = "ReferenceQuadrotorTaskQualityEvaluator"

    private let scenarioEvaluator: ReferenceQuadrotorScenarioEvaluator

    public init(scenarioEvaluator: ReferenceQuadrotorScenarioEvaluator = ReferenceQuadrotorScenarioEvaluator()) {
        self.scenarioEvaluator = scenarioEvaluator
    }

    public func evaluate(
        definition: ReferenceQuadrotorScenarioDefinition,
        log: SimulationLog
    ) -> ReferenceQuadrotorTaskQualitySummary {
        let base = scenarioEvaluator.evaluate(definition: definition, log: log)
        guard (definition.kind == .liftHover || definition.kind == .singleLiftHover),
              let lift = definition.liftEnvelope else {
            return ReferenceQuadrotorTaskQualitySummary(
                task: taskName(for: definition.kind),
                scenarioID: definition.config.id.rawValue,
                seed: definition.config.seed.rawValue,
                passed: base.passed,
                failureReasons: base.failures,
                evaluatorID: Self.evaluatorID,
                targetZ: nil,
                tolerance: nil,
                warmupTime: nil,
                requiredHoldTime: nil,
                achievedHoldTime: nil,
                maxAltitudeErrorAfterWarmup: nil,
                maxVerticalVelocityAfterWarmup: nil
            )
        }

        let metrics = liftMetrics(lift: lift, log: log)
        var failureReasons = base.failures
        if metrics.achievedHoldTime < lift.requiredHoldTime {
            failureReasons.append("hold-time-below-min")
        }
        if metrics.maxAltitudeErrorAfterWarmup > lift.tolerance {
            failureReasons.append("altitude-error-above-tolerance")
        }
        let passed = base.passed
            && metrics.achievedHoldTime >= lift.requiredHoldTime
            && metrics.maxAltitudeErrorAfterWarmup <= lift.tolerance

        return ReferenceQuadrotorTaskQualitySummary(
            task: taskName(for: definition.kind),
            scenarioID: definition.config.id.rawValue,
            seed: definition.config.seed.rawValue,
            passed: passed,
            failureReasons: Array(Set(failureReasons)).sorted(),
            evaluatorID: Self.evaluatorID,
            targetZ: lift.targetZ,
            tolerance: lift.tolerance,
            warmupTime: lift.warmupTime,
            requiredHoldTime: lift.requiredHoldTime,
            achievedHoldTime: metrics.achievedHoldTime,
            maxAltitudeErrorAfterWarmup: metrics.maxAltitudeErrorAfterWarmup,
            maxVerticalVelocityAfterWarmup: metrics.maxVerticalVelocityAfterWarmup
        )
    }

    private func liftMetrics(
        lift: LiftEnvelope,
        log: SimulationLog
    ) -> LiftQualityMetrics {
        var maxAltitudeErrorAfterWarmup = 0.0
        var maxVerticalVelocityAfterWarmup = 0.0
        var stableWindowStart: Double?
        var bestHoldTime = 0.0

        for step in log.events where step.time.time >= lift.warmupTime {
            let altitude = step.plantState.root.position.z
            let verticalVelocity = step.plantState.root.velocity.z
            let altitudeError = abs(altitude - lift.targetZ)
            let verticalVelocityMagnitude = abs(verticalVelocity)

            maxAltitudeErrorAfterWarmup = max(maxAltitudeErrorAfterWarmup, altitudeError)
            maxVerticalVelocityAfterWarmup = max(maxVerticalVelocityAfterWarmup, verticalVelocityMagnitude)

            if altitudeError <= lift.tolerance && verticalVelocityMagnitude <= lift.maxVelocity {
                stableWindowStart = stableWindowStart ?? step.time.time
                if let start = stableWindowStart {
                    bestHoldTime = max(bestHoldTime, step.time.time - start)
                }
            } else {
                stableWindowStart = nil
            }
        }

        return LiftQualityMetrics(
            achievedHoldTime: bestHoldTime,
            maxAltitudeErrorAfterWarmup: maxAltitudeErrorAfterWarmup,
            maxVerticalVelocityAfterWarmup: maxVerticalVelocityAfterWarmup
        )
    }

    private func taskName(for kind: ReferenceQuadrotorScenarioKind) -> String {
        switch kind {
        case .liftHover:
            return "lift"
        case .singleLiftHover:
            return "singleLift"
        default:
            return "attitude"
        }
    }
}

private struct LiftQualityMetrics: Sendable, Equatable {
    let achievedHoldTime: Double
    let maxAltitudeErrorAfterWarmup: Double
    let maxVerticalVelocityAfterWarmup: Double
}
