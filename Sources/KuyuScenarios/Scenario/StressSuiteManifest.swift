import Foundation

public struct StressSuiteManifest: Sendable, Codable, Equatable {
    public let suiteID: String
    public let profile: Profile
    public let records: [ScenarioRecord]
    public let coverageTargets: [CoverageTarget]
    public let coverageCounts: [StressDimension: Int]
    public let replayRequirement: ReplayRequirement
    public let replayEvidence: ReplayEvidence
    public let referenceM2BenchmarkEvidence: ReferenceM2BenchmarkEvidence?

    public static let requiredReferenceQuadrotorM2Dimensions: [StressDimension] = [
        .longHorizon,
        .plannerDegradation,
        .morphologyTransfer,
        .torqueDisturbance,
        .hfLatencySpike,
        .partialObservability,
    ]

    public init(
        suiteID: String,
        profile: Profile,
        records: [ScenarioRecord],
        coverageTargets: [CoverageTarget],
        replayRequirement: ReplayRequirement,
        replay: ReplayVerification,
        referenceM2BenchmarkEvidence: ReferenceM2BenchmarkEvidence? = nil
    ) throws {
        let trimmedSuiteID = suiteID.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmedSuiteID.isEmpty else { throw ValidationError.emptySuiteID }
        guard !records.isEmpty else { throw ValidationError.emptyRecords }
        guard !coverageTargets.isEmpty else { throw ValidationError.emptyCoverageTargets }

        try Self.validateUniqueRecords(records)
        try Self.validateUniqueTargets(coverageTargets)
        let coverageCounts = Self.makeCoverageCounts(records: records)
        try Self.validateCoverage(targets: coverageTargets, coverageCounts: coverageCounts)
        let replayEvidence = try Self.validateReplay(
            replay,
            requirement: replayRequirement,
            records: records
        )
        try Self.validateReferenceM2BenchmarkEvidence(
            referenceM2BenchmarkEvidence,
            profile: profile,
            records: records,
            coverageTargets: coverageTargets
        )

        self.suiteID = trimmedSuiteID
        self.profile = profile
        self.records = records
        self.coverageTargets = coverageTargets
        self.coverageCounts = coverageCounts
        self.replayRequirement = replayRequirement
        self.replayEvidence = replayEvidence
        self.referenceM2BenchmarkEvidence = referenceM2BenchmarkEvidence
    }
}
