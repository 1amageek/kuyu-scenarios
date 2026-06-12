import Foundation
import KuyuCore
import KuyuPhysics

/// A1 conformance execution report covering the Reporting requirements of
/// `A1_MANAS_CONFORMANCE_SUITE.md`: suite + seed list, per-scenario metric
/// summary, aggregate summary, and learning flags.
public struct A1ConformanceReport: Sendable, Codable, Equatable {
    public struct LearningFlags: Sendable, Codable, Equatable {
        public let core: Bool
        public let reflex: Bool

        public init(core: Bool, reflex: Bool) {
            self.core = core
            self.reflex = reflex
        }
    }

    public struct SuiteEntry: Sendable, Codable, Equatable {
        public let suiteID: String
        public let level: Int
        public let title: String
        public let seeds: [UInt64]
        public let summary: ValidationSummary

        public init(
            suiteID: String,
            level: Int,
            title: String,
            seeds: [UInt64],
            summary: ValidationSummary
        ) {
            self.suiteID = suiteID
            self.level = level
            self.title = title
            self.seeds = seeds
            self.summary = summary
        }
    }

    public let controller: String
    public let determinismTier: DeterminismTier
    public let learning: LearningFlags
    public let suites: [SuiteEntry]
    /// Overall verdict. Callers must derive this from a non-empty suite list;
    /// it is an explicit parameter so an empty report can never read as a pass.
    public let passed: Bool
    public let generatedAt: Date

    public init(
        controller: String,
        determinismTier: DeterminismTier,
        learning: LearningFlags,
        suites: [SuiteEntry],
        passed: Bool,
        generatedAt: Date
    ) {
        self.controller = controller
        self.determinismTier = determinismTier
        self.learning = learning
        self.suites = suites
        self.passed = passed
        self.generatedAt = generatedAt
    }
}
