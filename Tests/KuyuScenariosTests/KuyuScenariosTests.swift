import Testing
@testable import KuyuScenarios

@Test func scenarioEvaluationExists() {
    // Verify core scenario types are accessible
    let evaluation = ScenarioEvaluation(passed: true, metrics: [:])
    #expect(evaluation.passed == true)
}
