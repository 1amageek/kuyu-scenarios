import Testing
@testable import KuyuScenarios
import KuyuCore

@Test func scenarioEvaluationCanBeCreated() throws {
    let evaluation = ScenarioEvaluation(
        scenarioId: try ScenarioID("test-scenario"),
        seed: ScenarioSeed(42),
        passed: true,
        maxOmega: 1.0,
        maxTiltDegrees: 5.0,
        sustainedViolationSeconds: 0,
        recoveryTimeSeconds: nil,
        overshootDegrees: nil,
        hfStabilityScore: nil,
        failures: []
    )
    #expect(evaluation.passed)
    #expect(evaluation.failures.isEmpty)
}
