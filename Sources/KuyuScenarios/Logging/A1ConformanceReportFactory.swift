import Foundation
import KuyuCore

public struct A1ConformanceReportFactory: Sendable {
    public init() {}

    public func makeReport(
        controller: String,
        determinismTier: DeterminismTier,
        learning: A1ConformanceReport.LearningFlags,
        suites: [A1ConformanceReport.SuiteEntry],
        generatedAt: Date = Date()
    ) -> A1ConformanceReport {
        A1ConformanceReport(
            controller: controller,
            determinismTier: determinismTier,
            learning: learning,
            suites: suites,
            passed: overallPassed(suites: suites),
            generatedAt: generatedAt
        )
    }

    public func overallPassed(suites: [A1ConformanceReport.SuiteEntry]) -> Bool {
        !suites.isEmpty && suites.allSatisfy { entry in
            entry.summary.suitePassed && suiteReplayVerified(entry.summary.replay)
        }
    }

    public func suiteReplayVerified(_ replay: ReplayVerification) -> Bool {
        switch replay {
        case .performed(let checks):
            return !checks.isEmpty && checks.allSatisfy(\.passed)
        case .notPerformed:
            return false
        }
    }
}
