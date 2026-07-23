import Foundation

extension StressSuiteManifest {
    static func validateReferenceM2BenchmarkEvidence(
        _ evidence: ReferenceM2BenchmarkEvidence?,
        profile: Profile,
        records: [ScenarioRecord],
        coverageTargets: [CoverageTarget]
    ) throws {
        guard let evidence else { return }
        guard profile == .referenceQuadrotor else {
            throw ValidationError.unexpectedReferenceM2BenchmarkEvidence(profile)
        }
        guard evidence.isComplete else {
            throw ValidationError.invalidReferenceM2BenchmarkEvidence("incomplete-evidence")
        }
        let targetDimensions = Set(coverageTargets.map(\.dimension))
        guard requiredReferenceQuadrotorM2Dimensions.allSatisfy({ targetDimensions.contains($0) }) else {
            throw ValidationError.invalidReferenceM2BenchmarkEvidence("missing-required-coverage-target")
        }

        let recordByID = Dictionary(uniqueKeysWithValues: records.map { ($0.scenarioID, $0) })
        let recordsByTrack = Self.recordsByReferenceM2Track(records)
        let trackCounts = evidence.countByTrack
        for track in LongHorizonBenchmarkTrack.allCases {
            guard recordsByTrack[track, default: 0] == trackCounts[track, default: 0] else {
                throw ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "record-track-count-mismatch:\(track.rawValue)"
                )
            }
        }
        try validateMorphologyTransferEvidence(
            evidence.morphologyTransfers,
            recordByID: recordByID
        )
        try validateScenarioIDs(
            evidence.plannerDegradationScenarioIDs,
            recordByID: recordByID,
            requiredDimension: .plannerDegradation,
            field: "planner-degradation"
        )
        try validateScenarioIDs(
            evidence.disturbanceScenarioIDs,
            recordByID: recordByID,
            requiredDimension: .torqueDisturbance,
            field: "disturbance"
        )
        try validateScenarioIDs(
            evidence.latencyScenarioIDs,
            recordByID: recordByID,
            requiredDimension: .hfLatencySpike,
            field: "latency"
        )
        try validateScenarioIDs(
            evidence.partialObservabilityScenarioIDs,
            recordByID: recordByID,
            requiredDimension: .partialObservability,
            field: "partial-observability"
        )
    }

    private static func recordsByReferenceM2Track(
        _ records: [ScenarioRecord]
    ) -> [LongHorizonBenchmarkTrack: Int] {
        var counts: [LongHorizonBenchmarkTrack: Int] = [:]
        for record in records {
            let dimensions = Set(record.dimensions)
            if dimensions.contains(.morphologyTransfer) {
                counts[.morphologyTransfer, default: 0] += 1
            } else if dimensions.contains(.partialObservability) {
                counts[.disturbanceDelayPartialObservability, default: 0] += 1
            } else if dimensions.contains(.plannerDegradation) {
                counts[.longHorizonTask, default: 0] += 1
            }
        }
        return counts
    }

    private static func validateMorphologyTransferEvidence(
        _ transfers: [ReferenceM2BenchmarkEvidence.MorphologyTransferEvidence],
        recordByID: [String: ScenarioRecord]
    ) throws {
        for transfer in transfers {
            guard let record = recordByID[transfer.scenarioID] else {
                throw ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "missing-morphology-record:\(transfer.scenarioID)"
                )
            }
            guard record.dimensions.contains(.morphologyTransfer) else {
                throw ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "morphology-record-missing-dimension:\(transfer.scenarioID)"
                )
            }
        }
    }

    private static func validateScenarioIDs(
        _ scenarioIDs: [String],
        recordByID: [String: ScenarioRecord],
        requiredDimension: StressDimension,
        field: String
    ) throws {
        for scenarioID in scenarioIDs {
            guard let record = recordByID[scenarioID] else {
                throw ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "missing-\(field)-record:\(scenarioID)"
                )
            }
            guard record.dimensions.contains(requiredDimension) else {
                throw ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "\(field)-record-missing-dimension:\(scenarioID)"
                )
            }
        }
    }
}
