import Foundation
import KuyuCore
import KuyuPhysics
import Testing
@testable import KuyuScenarios

@Test func a1ConformanceReportFactoryRejectsEmptySuiteList() {
    let report = A1ConformanceReportFactory().makeReport(
        controller: "baseline",
        determinismTier: .tier1,
        learning: A1ConformanceReport.LearningFlags(core: false, reflex: false),
        suites: [],
        generatedAt: Date(timeIntervalSince1970: 0)
    )

    #expect(!report.passed)
}

@Test func a1ConformanceReportFactoryRequiresPerformedReplay() throws {
    let report = A1ConformanceReportFactory().makeReport(
        controller: "baseline",
        determinismTier: .tier1,
        learning: A1ConformanceReport.LearningFlags(core: false, reflex: false),
        suites: [try makeSuiteEntry(suitePassed: true, replay: .notPerformed(reason: "skipped"))],
        generatedAt: Date(timeIntervalSince1970: 0)
    )

    #expect(!report.passed)
}

@Test func a1ConformanceReportFactoryRejectsEmptyReplayChecks() throws {
    let report = A1ConformanceReportFactory().makeReport(
        controller: "baseline",
        determinismTier: .tier1,
        learning: A1ConformanceReport.LearningFlags(core: false, reflex: false),
        suites: [try makeSuiteEntry(suitePassed: true, replay: .performed([]))],
        generatedAt: Date(timeIntervalSince1970: 0)
    )

    #expect(!report.passed)
}

@Test func a1ConformanceReportFactoryAcceptsOnlyPassingSuitesWithPassingReplay() throws {
    let factory = A1ConformanceReportFactory()
    let accepted = factory.makeReport(
        controller: "baseline",
        determinismTier: .tier1,
        learning: A1ConformanceReport.LearningFlags(core: false, reflex: false),
        suites: [try makeSuiteEntry(suitePassed: true, replay: .performed([try makeReplayCheck(passed: true)]))],
        generatedAt: Date(timeIntervalSince1970: 0)
    )
    let failedSuite = factory.makeReport(
        controller: "baseline",
        determinismTier: .tier1,
        learning: A1ConformanceReport.LearningFlags(core: false, reflex: false),
        suites: [try makeSuiteEntry(suitePassed: false, replay: .performed([try makeReplayCheck(passed: true)]))],
        generatedAt: Date(timeIntervalSince1970: 0)
    )
    let failedReplay = factory.makeReport(
        controller: "baseline",
        determinismTier: .tier1,
        learning: A1ConformanceReport.LearningFlags(core: false, reflex: false),
        suites: [try makeSuiteEntry(suitePassed: true, replay: .performed([try makeReplayCheck(passed: false)]))],
        generatedAt: Date(timeIntervalSince1970: 0)
    )

    #expect(accepted.passed)
    #expect(!failedSuite.passed)
    #expect(!failedReplay.passed)
}

private func makeSuiteEntry(
    suitePassed: Bool,
    replay: ReplayVerification
) throws -> A1ConformanceReport.SuiteEntry {
    A1ConformanceReport.SuiteEntry(
        suiteID: "Suite-0",
        level: 0,
        title: "Warmup",
        seeds: [1],
        summary: ValidationSummary(
            suitePassed: suitePassed,
            evaluations: [
                ScenarioEvaluation(
                    scenarioId: try ScenarioID("KUY-A1/Suite-0/SCN-1"),
                    seed: ScenarioSeed(1),
                    passed: suitePassed,
                    maxOmega: 0,
                    maxTiltDegrees: 0,
                    sustainedViolationSeconds: 0,
                    recoveryTimeSeconds: 0,
                    overshootDegrees: 0,
                    hfStabilityScore: 1,
                    failures: []
                )
            ],
            replay: replay,
            manifest: [],
            aggregate: EvaluationAggregate(
                averageRecoveryTime: 0,
                worstOvershootDegrees: 0,
                averageHfStabilityScore: 1
            )
        )
    )
}

private func makeReplayCheck(passed: Bool) throws -> ReplayCheckResult {
    ReplayCheckResult(
        scenarioId: try ScenarioID("KUY-A1/Suite-0/SCN-1"),
        seed: ScenarioSeed(1),
        tier: .tier1,
        passed: passed,
        issues: passed ? [] : ["residual"],
        residuals: .zero
    )
}
