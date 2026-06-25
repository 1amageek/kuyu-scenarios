import KuyuCore
import KuyuPhysics
@testable import KuyuScenarios
import Testing

@Test func suiteRunResultFactoryPassesEvaluationOnlyWhenAllEvaluationsPass() throws {
    let factory = SuiteRunResultFactory()

    let result = try factory.makeEvaluationOnly(
        evaluations: [
            makeSuiteFactoryEvaluation(passed: true),
            makeSuiteFactoryEvaluation(passed: true)
        ],
        replaySkippedReason: "interactive"
    )

    #expect(result.passed)
    #expect(result.replay.notPerformedReason == "interactive")
}

@Test func suiteRunResultFactoryRejectsEvaluationOnlyWhenAnyEvaluationFails() throws {
    let factory = SuiteRunResultFactory()

    let result = try factory.makeEvaluationOnly(
        evaluations: [
            makeSuiteFactoryEvaluation(passed: true),
            makeSuiteFactoryEvaluation(passed: false)
        ],
        replaySkippedReason: "interactive"
    )

    #expect(!result.passed)
}

@Test func suiteRunResultFactoryRequiresEvaluationsAndReplayChecksForVerifiedResult() throws {
    let factory = SuiteRunResultFactory()

    let accepted = try factory.makeReplayVerified(
        evaluations: [makeSuiteFactoryEvaluation(passed: true)],
        replayChecks: [makeSuiteFactoryReplayCheck(passed: true)]
    )
    let rejectedByReplay = try factory.makeReplayVerified(
        evaluations: [makeSuiteFactoryEvaluation(passed: true)],
        replayChecks: [makeSuiteFactoryReplayCheck(passed: false)]
    )
    let rejectedByEvaluation = try factory.makeReplayVerified(
        evaluations: [makeSuiteFactoryEvaluation(passed: false)],
        replayChecks: [makeSuiteFactoryReplayCheck(passed: true)]
    )

    #expect(accepted.passed)
    #expect(!rejectedByReplay.passed)
    #expect(!rejectedByEvaluation.passed)
}

private func makeSuiteFactoryEvaluation(passed: Bool) throws -> ScenarioEvaluation {
    ScenarioEvaluation(
        scenarioId: try ScenarioID("suite-factory"),
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

private func makeSuiteFactoryReplayCheck(passed: Bool) throws -> ReplayCheckResult {
    ReplayCheckResult(
        scenarioId: try ScenarioID("suite-factory"),
        seed: ScenarioSeed(1),
        tier: .tier1,
        passed: passed,
        issues: passed ? [] : ["replay-failed"],
        residuals: .zero
    )
}
