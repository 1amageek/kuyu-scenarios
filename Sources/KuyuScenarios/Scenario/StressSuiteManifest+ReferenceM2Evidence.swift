import Foundation

public extension StressSuiteManifest {
    struct ReferenceM2BenchmarkEvidence: Sendable, Codable, Equatable {
        public let tracks: [TrackEvidence]
        public let plannerDegradationScenarioIDs: [String]
        public let morphologyTransfers: [MorphologyTransferEvidence]
        public let disturbanceScenarioIDs: [String]
        public let latencyScenarioIDs: [String]
        public let partialObservabilityScenarioIDs: [String]

        private enum CodingKeys: String, CodingKey {
            case tracks
            case plannerDegradationScenarioIDs
            case morphologyTransfers
            case disturbanceScenarioIDs
            case latencyScenarioIDs
            case partialObservabilityScenarioIDs
        }

        public init(
            tracks: [TrackEvidence],
            plannerDegradationScenarioIDs: [String],
            morphologyTransfers: [MorphologyTransferEvidence],
            disturbanceScenarioIDs: [String],
            latencyScenarioIDs: [String],
            partialObservabilityScenarioIDs: [String]
        ) throws {
            self.tracks = tracks
            self.plannerDegradationScenarioIDs = try Self.validatedScenarioIDs(
                plannerDegradationScenarioIDs,
                field: "planner-degradation"
            )
            self.morphologyTransfers = morphologyTransfers
            self.disturbanceScenarioIDs = try Self.validatedScenarioIDs(
                disturbanceScenarioIDs,
                field: "disturbance"
            )
            self.latencyScenarioIDs = try Self.validatedScenarioIDs(
                latencyScenarioIDs,
                field: "latency"
            )
            self.partialObservabilityScenarioIDs = try Self.validatedScenarioIDs(
                partialObservabilityScenarioIDs,
                field: "partial-observability"
            )
            try validateShape()
        }

        public init(benchmark: LongHorizonBenchmarkSuite) throws {
            let trackCounts = try benchmark.validatedReferenceM2TrackCounts()
            let tracks = try LongHorizonBenchmarkTrack.allCases.map { track in
                try TrackEvidence(track: track, count: trackCounts[track, default: 0])
            }
            let plannerCases = benchmark.cases.filter {
                $0.track == .longHorizonTask
            }
            var morphologyTransfers: [MorphologyTransferEvidence] = []
            for benchmarkCase in benchmark.cases where benchmarkCase.track == .morphologyTransfer {
                guard let contract = benchmarkCase.morphologyTransfer else {
                    throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                        "missing-morphology-contract:\(benchmarkCase.definition.config.id.rawValue)"
                    )
                }
                let evidence = try MorphologyTransferEvidence(
                    scenarioID: benchmarkCase.definition.config.id.rawValue,
                    contract: contract
                )
                morphologyTransfers.append(evidence)
            }
            let disturbanceCases = benchmark.cases.filter {
                $0.track == .disturbanceDelayPartialObservability
            }
            try self.init(
                tracks: tracks,
                plannerDegradationScenarioIDs: plannerCases.map(\.definition.config.id.rawValue),
                morphologyTransfers: morphologyTransfers,
                disturbanceScenarioIDs: disturbanceCases.map(\.definition.config.id.rawValue),
                latencyScenarioIDs: disturbanceCases.map(\.definition.config.id.rawValue),
                partialObservabilityScenarioIDs: disturbanceCases.map(\.definition.config.id.rawValue)
            )
        }

        public init(from decoder: Decoder) throws {
            let container = try decoder.container(keyedBy: CodingKeys.self)
            try self.init(
                tracks: try container.decode([TrackEvidence].self, forKey: .tracks),
                plannerDegradationScenarioIDs: try container.decodeIfPresent(
                    [String].self,
                    forKey: .plannerDegradationScenarioIDs
                ) ?? [],
                morphologyTransfers: try container.decode(
                    [MorphologyTransferEvidence].self,
                    forKey: .morphologyTransfers
                ),
                disturbanceScenarioIDs: try container.decode([String].self, forKey: .disturbanceScenarioIDs),
                latencyScenarioIDs: try container.decode([String].self, forKey: .latencyScenarioIDs),
                partialObservabilityScenarioIDs: try container.decode(
                    [String].self,
                    forKey: .partialObservabilityScenarioIDs
                )
            )
        }

        public var countByTrack: [LongHorizonBenchmarkTrack: Int] {
            Dictionary(uniqueKeysWithValues: tracks.map { ($0.track, $0.count) })
        }

        public var isComplete: Bool {
            guard let expectedCount = tracks.first?.count,
                  LongHorizonBenchmarkTrack.allCases.allSatisfy({ countByTrack[$0] == expectedCount }) else {
                return false
            }
            return morphologyTransfers.count == expectedCount
                && plannerDegradationScenarioIDs.count == expectedCount
                && disturbanceScenarioIDs.count == expectedCount
                && latencyScenarioIDs.count == expectedCount
                && partialObservabilityScenarioIDs.count == expectedCount
        }
    }
}
