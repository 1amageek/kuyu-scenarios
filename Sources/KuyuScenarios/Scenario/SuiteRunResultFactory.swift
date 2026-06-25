import KuyuPhysics

public struct SuiteRunResultFactory: Sendable {
    public init() {}

    public func makeEvaluationOnly(
        evaluations: [ScenarioEvaluation],
        replaySkippedReason: String
    ) -> SuiteRunResult {
        SuiteRunResult(
            evaluations: evaluations,
            replay: .notPerformed(reason: replaySkippedReason),
            passed: evaluationsPassed(evaluations)
        )
    }

    public func makeReplayVerified(
        evaluations: [ScenarioEvaluation],
        replayChecks: [ReplayCheckResult]
    ) -> SuiteRunResult {
        SuiteRunResult(
            evaluations: evaluations,
            replay: .performed(replayChecks),
            passed: evaluationsPassed(evaluations) && replayChecksPassed(replayChecks)
        )
    }

    public func makeConfigured(
        evaluations: [ScenarioEvaluation],
        replayChecks: [ReplayCheckResult],
        replayVerificationEnabled: Bool,
        replaySkippedReason: String
    ) -> SuiteRunResult {
        if replayVerificationEnabled {
            return makeReplayVerified(evaluations: evaluations, replayChecks: replayChecks)
        }
        return makeEvaluationOnly(
            evaluations: evaluations,
            replaySkippedReason: replaySkippedReason
        )
    }

    public func evaluationsPassed(_ evaluations: [ScenarioEvaluation]) -> Bool {
        evaluations.allSatisfy(\.passed)
    }

    public func replayChecksPassed(_ replayChecks: [ReplayCheckResult]) -> Bool {
        replayChecks.allSatisfy(\.passed)
    }
}
