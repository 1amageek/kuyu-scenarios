import KuyuCore
import KuyuPhysics
import Testing
@testable import KuyuScenarios

@Test func kuyAtt1RunOutputFactoryBuildsSummaryFromResult() throws {
    let evaluation = try makeKuyAtt1FactoryEvaluation(passed: true)
    let replay = ReplayVerification.performed([
        try makeKuyAtt1FactoryReplayCheck(passed: true),
    ])
    let result = SuiteRunResult(
        evaluations: [evaluation],
        replay: replay,
        passed: true
    )

    let output = KuyAtt1RunOutputFactory().makeOutput(
        result: result,
        logs: [],
        manifest: []
    )

    #expect(output.result.passed)
    #expect(output.summary.suitePassed)
    #expect(output.summary.replay == replay)
    #expect(output.summary.evaluations == [evaluation])
}

@Test func kuyAtt1RunOutputFactoryRejectsEmptyPassedResult() {
    let replay = ReplayVerification.notPerformed(reason: "fixture")
    let result = SuiteRunResult(evaluations: [], replay: replay, passed: true)

    let output = KuyAtt1RunOutputFactory().makeOutput(
        result: result,
        logs: [],
        manifest: []
    )

    #expect(!output.result.passed)
    #expect(!output.summary.suitePassed)
    #expect(output.summary.evaluations.isEmpty)
}

@Test func kuyAtt1RunOutputFactoryBuildsEvaluationOnlyOutput() throws {
    let accepted = KuyAtt1RunOutputFactory().makeEvaluationOnly(
        evaluations: [try makeKuyAtt1FactoryEvaluation(passed: true)],
        logs: [],
        manifest: [],
        replaySkippedReason: "interactive"
    )
    let rejected = KuyAtt1RunOutputFactory().makeEvaluationOnly(
        evaluations: [try makeKuyAtt1FactoryEvaluation(passed: false)],
        logs: [],
        manifest: [],
        replaySkippedReason: "interactive"
    )

    #expect(accepted.result.passed)
    #expect(accepted.summary.replay.notPerformedReason == "interactive")
    #expect(!rejected.result.passed)
}

private func makeKuyAtt1FactoryEvaluation(passed: Bool) throws -> ScenarioEvaluation {
    ScenarioEvaluation(
        scenarioId: try ScenarioID("kuy-att1-output-factory"),
        seed: ScenarioSeed(1),
        passed: passed,
        maxOmega: 0,
        maxTiltDegrees: 0,
        sustainedViolationSeconds: 0,
        recoveryTimeSeconds: nil,
        overshootDegrees: nil,
        hfStabilityScore: nil,
        failures: passed ? [] : ["failed"]
    )
}

private func makeKuyAtt1FactoryReplayCheck(passed: Bool) throws -> ReplayCheckResult {
    ReplayCheckResult(
        scenarioId: try ScenarioID("kuy-att1-output-factory"),
        seed: ScenarioSeed(1),
        tier: .tier1,
        passed: passed,
        issues: passed ? [] : ["replay-failed"],
        residuals: .zero
    )
}
