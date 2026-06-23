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

@Test func scenarioTerminalFactsValidateCompletedAndFailedStates() throws {
    try ScenarioTerminalFacts(
        done: false,
        truncated: true,
        terminalReason: ScenarioTerminalFacts.completedTerminalReason,
        failureReason: nil,
        failureTime: nil
    ).validate()

    try ScenarioTerminalFacts(
        done: true,
        truncated: false,
        terminalReason: FailureReason.sustainedFall.rawValue,
        failureReason: .sustainedFall,
        failureTime: 1.25
    ).validate()
}

@Test func scenarioTerminalFactsRejectInvalidStateCombinations() throws {
    #expect(throws: ScenarioTerminalFacts.ValidationError.conflictingTerminalState) {
        try ScenarioTerminalFacts(
            done: true,
            truncated: true,
            terminalReason: FailureReason.sustainedFall.rawValue,
            failureReason: .sustainedFall,
            failureTime: 1
        ).validate()
    }

    #expect(throws: ScenarioTerminalFacts.ValidationError.failureRequiresDone(reason: .sustainedFall)) {
        try ScenarioTerminalFacts(
            done: false,
            truncated: true,
            terminalReason: FailureReason.sustainedFall.rawValue,
            failureReason: .sustainedFall,
            failureTime: 1
        ).validate()
    }

    #expect(throws: ScenarioTerminalFacts.ValidationError.missingFailureTime(reason: .sustainedFall)) {
        try ScenarioTerminalFacts(
            done: true,
            truncated: false,
            terminalReason: FailureReason.sustainedFall.rawValue,
            failureReason: .sustainedFall,
            failureTime: nil
        ).validate()
    }

    #expect(throws: ScenarioTerminalFacts.ValidationError.failureTimeWithoutFailure(1)) {
        try ScenarioTerminalFacts(
            done: false,
            truncated: true,
            terminalReason: ScenarioTerminalFacts.timeLimitTerminalReason,
            failureReason: nil,
            failureTime: 1
        ).validate()
    }
}
