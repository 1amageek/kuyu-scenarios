import KuyuCore
import KuyuPhysics

public struct ArticulatedDynamicScenarioOutputFactory: Sendable {
    private static let replaySkippedReason = "Articulated dynamic simulation does not execute replay verification."

    public init() {}

    public func makeOutput(log: SimulationLog) -> KuyAtt1RunOutput {
        let evaluation = makeEvaluation(log: log)
        let result = SuiteRunResultFactory().makeEvaluationOnly(
            evaluations: [evaluation],
            replaySkippedReason: Self.replaySkippedReason
        )
        let key = ScenarioKey(scenarioId: log.scenarioId, seed: log.seed)
        return KuyAtt1RunOutputFactory().makeOutput(
            result: result,
            logs: [ScenarioLogEntry(key: key, log: log)],
            manifest: [makeManifest(log: log)]
        )
    }

    public func makeEvaluation(log: SimulationLog) -> ScenarioEvaluation {
        let failures = failureReasons(log: log)
        let maxOmega = log.events.map(\.safetyTrace.omegaMagnitude).max() ?? 0.0
        let maxTiltRadians = log.events.map(\.safetyTrace.tiltRadians).max() ?? 0.0
        return ScenarioEvaluation(
            scenarioId: log.scenarioId,
            seed: log.seed,
            passed: failures.isEmpty,
            maxOmega: maxOmega,
            maxTiltDegrees: maxTiltRadians * 180.0 / Double.pi,
            sustainedViolationSeconds: 0.0,
            recoveryTimeSeconds: nil,
            overshootDegrees: nil,
            hfStabilityScore: nil,
            failures: failures,
            failureReason: log.failureReason,
            failureTime: log.failureTime
        )
    }

    public func makeManifest(log: SimulationLog) -> ReferenceQuadrotorScenarioManifest {
        ReferenceQuadrotorScenarioManifest(
            scenarioId: log.scenarioId,
            seed: log.seed,
            kind: .liftHover,
            duration: Double(log.events.count) * log.timeStep.delta,
            timeStep: log.timeStep,
            torqueEvents: [],
            actuatorDegradation: nil,
            gyroDriftScale: 0.0,
            swapEvents: [],
            hfEvents: []
        )
    }

    private func failureReasons(log: SimulationLog) -> [String] {
        var failures: [String] = []
        if log.events.isEmpty {
            failures.append("simulation-log-empty")
        }
        if let failureReason = log.failureReason {
            failures.append(failureReason.rawValue)
        }
        return failures
    }
}
