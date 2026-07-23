import Foundation

extension StressSuiteManifest.ReferenceM2BenchmarkEvidence {
    func validateShape() throws {
        let trackCounts = countByTrack
        guard trackCounts.count == LongHorizonBenchmarkTrack.allCases.count else {
            throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                "missing-track-evidence"
            )
        }
        let expectedCount = try expectedBalancedTrackCount(from: trackCounts)
        guard morphologyTransfers.count == expectedCount else {
            throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                "morphology-transfer-count-mismatch"
            )
        }
        guard plannerDegradationScenarioIDs.count == expectedCount else {
            throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                "planner-degradation-count-mismatch"
            )
        }
        try Self.validateUniqueScenarioIDs(
            plannerDegradationScenarioIDs,
            field: "planner-degradation"
        )
        try Self.validateUniqueScenarioIDs(
            morphologyTransfers.map(\.scenarioID),
            field: "morphology-transfer"
        )
        guard disturbanceScenarioIDs.count == expectedCount else {
            throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                "disturbance-count-mismatch"
            )
        }
        guard latencyScenarioIDs.count == expectedCount else {
            throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                "latency-count-mismatch"
            )
        }
        guard partialObservabilityScenarioIDs.count == expectedCount else {
            throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                "partial-observability-count-mismatch"
            )
        }
    }

    static func validatedScenarioIDs(
        _ scenarioIDs: [String],
        field: String
    ) throws -> [String] {
        let trimmed = scenarioIDs.map { $0.trimmingCharacters(in: .whitespacesAndNewlines) }
        guard trimmed.allSatisfy({ !$0.isEmpty }) else {
            throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                "empty-\(field)-scenario-id"
            )
        }
        try validateUniqueScenarioIDs(trimmed, field: field)
        return trimmed
    }

    private func expectedBalancedTrackCount(
        from trackCounts: [LongHorizonBenchmarkTrack: Int]
    ) throws -> Int {
        let counts = try LongHorizonBenchmarkTrack.allCases.map { track in
            guard let count = trackCounts[track] else {
                throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "missing-track-evidence:\(track.rawValue)"
                )
            }
            return count
        }
        guard let expected = counts.first,
              counts.allSatisfy({ $0 == expected }) else {
            throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                "imbalanced-track-evidence"
            )
        }
        return expected
    }

    private static func validateUniqueScenarioIDs(
        _ scenarioIDs: [String],
        field: String
    ) throws {
        guard Set(scenarioIDs).count == scenarioIDs.count else {
            throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                "duplicate-\(field)-scenario-id"
            )
        }
    }
}
