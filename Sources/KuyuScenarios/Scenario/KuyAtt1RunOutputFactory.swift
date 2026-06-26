public struct KuyAtt1RunOutputFactory: Sendable {
    public static let interactiveReplaySkippedReason = "Interactive simulation runs do not execute replay verification."

    public init() {}

    public func makeOutput(
        result: SuiteRunResult,
        logs: [ScenarioLogEntry],
        manifest: [ReferenceQuadrotorScenarioManifest]
    ) -> KuyAtt1RunOutput {
        let checkedResult = failClosedResult(result)
        let summary = ValidationSummary(
            suitePassed: checkedResult.passed,
            evaluations: checkedResult.evaluations,
            replay: checkedResult.replay,
            manifest: manifest,
            aggregate: EvaluationAggregate.from(evaluations: checkedResult.evaluations)
        )
        return KuyAtt1RunOutput(
            result: checkedResult,
            summary: summary,
            logs: logs
        )
    }

    public func makeEvaluationOnly(
        evaluations: [ScenarioEvaluation],
        logs: [ScenarioLogEntry],
        manifest: [ReferenceQuadrotorScenarioManifest],
        replaySkippedReason: String = Self.interactiveReplaySkippedReason
    ) -> KuyAtt1RunOutput {
        makeOutput(
            result: SuiteRunResultFactory().makeEvaluationOnly(
                evaluations: evaluations,
                replaySkippedReason: replaySkippedReason
            ),
            logs: logs,
            manifest: manifest
        )
    }

    private func failClosedResult(_ result: SuiteRunResult) -> SuiteRunResult {
        guard !result.evaluations.isEmpty else {
            return SuiteRunResult(
                evaluations: result.evaluations,
                replay: result.replay,
                passed: false
            )
        }
        return result
    }
}
