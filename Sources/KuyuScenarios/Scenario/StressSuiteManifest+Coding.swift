import Foundation

private enum StressSuiteManifestCodingKeys: String, CodingKey {
    case suiteID
    case profile
    case records
    case coverageTargets
    case coverageCounts
    case replayRequirement
    case replayEvidence
    case referenceM2BenchmarkEvidence
}

public extension StressSuiteManifest {
    init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: StressSuiteManifestCodingKeys.self)
        let suiteID = try container.decode(String.self, forKey: .suiteID)
        let profile = try container.decode(Profile.self, forKey: .profile)
        let records = try container.decode([ScenarioRecord].self, forKey: .records)
        let coverageTargets = try container.decode([CoverageTarget].self, forKey: .coverageTargets)
        let decodedCoverageCounts = try container.decode([StressDimension: Int].self, forKey: .coverageCounts)
        let replayRequirement = try container.decode(ReplayRequirement.self, forKey: .replayRequirement)
        let replayEvidence = try container.decode(ReplayEvidence.self, forKey: .replayEvidence)
        let referenceM2BenchmarkEvidence = try container.decodeIfPresent(
            ReferenceM2BenchmarkEvidence.self,
            forKey: .referenceM2BenchmarkEvidence
        )

        let trimmedSuiteID = suiteID.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmedSuiteID.isEmpty else { throw ValidationError.emptySuiteID }
        guard !records.isEmpty else { throw ValidationError.emptyRecords }
        guard !coverageTargets.isEmpty else { throw ValidationError.emptyCoverageTargets }
        try Self.validateUniqueRecords(records)
        try Self.validateUniqueTargets(coverageTargets)
        let coverageCounts = Self.makeCoverageCounts(records: records)
        guard decodedCoverageCounts == coverageCounts else {
            throw ValidationError.decodedCoverageCountMismatch
        }
        try Self.validateCoverage(targets: coverageTargets, coverageCounts: coverageCounts)
        try Self.validateDecodedReplayEvidence(replayEvidence, requirement: replayRequirement)
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

    func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: StressSuiteManifestCodingKeys.self)
        try container.encode(suiteID, forKey: .suiteID)
        try container.encode(profile, forKey: .profile)
        try container.encode(records, forKey: .records)
        try container.encode(coverageTargets, forKey: .coverageTargets)
        try container.encode(coverageCounts, forKey: .coverageCounts)
        try container.encode(replayRequirement, forKey: .replayRequirement)
        try container.encode(replayEvidence, forKey: .replayEvidence)
        try container.encodeIfPresent(referenceM2BenchmarkEvidence, forKey: .referenceM2BenchmarkEvidence)
    }
}
