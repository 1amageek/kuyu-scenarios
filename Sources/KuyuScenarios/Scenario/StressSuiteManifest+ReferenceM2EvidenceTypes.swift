import Foundation
import KuyuPhysics

public extension StressSuiteManifest.ReferenceM2BenchmarkEvidence {
    struct TrackEvidence: Sendable, Codable, Equatable {
        public let track: LongHorizonBenchmarkTrack
        public let count: Int

        private enum CodingKeys: String, CodingKey {
            case track
            case count
        }

        public init(track: LongHorizonBenchmarkTrack, count: Int) throws {
            guard count > 0 else {
                throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "non-positive-track-count:\(track.rawValue)"
                )
            }
            self.track = track
            self.count = count
        }

        public init(from decoder: Decoder) throws {
            let container = try decoder.container(keyedBy: CodingKeys.self)
            try self.init(
                track: try container.decode(LongHorizonBenchmarkTrack.self, forKey: .track),
                count: try container.decode(Int.self, forKey: .count)
            )
        }
    }

    struct ParameterDeltaEvidence: Sendable, Codable, Equatable {
        public let name: String
        public let sourceValue: Double
        public let targetValue: Double

        private enum CodingKeys: String, CodingKey {
            case name
            case sourceValue
            case targetValue
        }

        public init(
            name: String,
            sourceValue: Double,
            targetValue: Double
        ) throws {
            let trimmed = name.trimmingCharacters(in: .whitespacesAndNewlines)
            guard !trimmed.isEmpty else {
                throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "empty-parameter-delta-name"
                )
            }
            guard sourceValue.isFinite, targetValue.isFinite else {
                throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "non-finite-parameter-delta:\(trimmed)"
                )
            }
            guard sourceValue != targetValue else {
                throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "unchanged-parameter-delta:\(trimmed)"
                )
            }
            self.name = trimmed
            self.sourceValue = sourceValue
            self.targetValue = targetValue
        }

        public init(
            delta: LongHorizonMorphologyTransferContract.ParameterDelta
        ) throws {
            try self.init(
                name: delta.name,
                sourceValue: delta.sourceValue,
                targetValue: delta.targetValue
            )
        }

        public init(from decoder: Decoder) throws {
            let container = try decoder.container(keyedBy: CodingKeys.self)
            try self.init(
                name: try container.decode(String.self, forKey: .name),
                sourceValue: try container.decode(Double.self, forKey: .sourceValue),
                targetValue: try container.decode(Double.self, forKey: .targetValue)
            )
        }
    }

    struct MorphologyTransferEvidence: Sendable, Codable, Equatable {
        public let scenarioID: String
        public let sourceRobotID: String
        public let targetRobotID: String
        public let sourceReadiness: ReadinessLevel
        public let targetReadiness: ReadinessLevel
        public let parameterDeltas: [ParameterDeltaEvidence]

        private enum CodingKeys: String, CodingKey {
            case scenarioID
            case sourceRobotID
            case targetRobotID
            case sourceReadiness
            case targetReadiness
            case parameterDeltas
        }

        public init(
            scenarioID: String,
            sourceRobotID: String,
            targetRobotID: String,
            sourceReadiness: ReadinessLevel,
            targetReadiness: ReadinessLevel,
            parameterDeltas: [ParameterDeltaEvidence]
        ) throws {
            let scenarioID = scenarioID.trimmingCharacters(in: .whitespacesAndNewlines)
            let sourceRobotID = sourceRobotID.trimmingCharacters(in: .whitespacesAndNewlines)
            let targetRobotID = targetRobotID.trimmingCharacters(in: .whitespacesAndNewlines)
            guard !scenarioID.isEmpty else {
                throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "empty-morphology-scenario-id"
                )
            }
            guard !sourceRobotID.isEmpty, !targetRobotID.isEmpty else {
                throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "empty-morphology-robot-id:\(scenarioID)"
                )
            }
            guard sourceRobotID != targetRobotID else {
                throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "identical-morphology-robot-id:\(scenarioID)"
                )
            }
            guard sourceReadiness >= .dynamicSimulation,
                  targetReadiness >= .dynamicSimulation else {
                throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "morphology-readiness-below-dynamic-simulation:\(scenarioID)"
                )
            }
            guard !parameterDeltas.isEmpty else {
                throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "empty-morphology-parameter-deltas:\(scenarioID)"
                )
            }
            let names = Set(parameterDeltas.map(\.name))
            guard names.count == parameterDeltas.count else {
                throw StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
                    "duplicate-morphology-parameter-delta:\(scenarioID)"
                )
            }
            self.scenarioID = scenarioID
            self.sourceRobotID = sourceRobotID
            self.targetRobotID = targetRobotID
            self.sourceReadiness = sourceReadiness
            self.targetReadiness = targetReadiness
            self.parameterDeltas = parameterDeltas
        }

        public init(
            scenarioID: String,
            contract: LongHorizonMorphologyTransferContract
        ) throws {
            try self.init(
                scenarioID: scenarioID,
                sourceRobotID: contract.sourceRobotID,
                targetRobotID: contract.targetRobotID,
                sourceReadiness: contract.sourceReadiness,
                targetReadiness: contract.targetReadiness,
                parameterDeltas: try contract.parameterDeltas.map(ParameterDeltaEvidence.init)
            )
        }

        public init(from decoder: Decoder) throws {
            let container = try decoder.container(keyedBy: CodingKeys.self)
            try self.init(
                scenarioID: try container.decode(String.self, forKey: .scenarioID),
                sourceRobotID: try container.decode(String.self, forKey: .sourceRobotID),
                targetRobotID: try container.decode(String.self, forKey: .targetRobotID),
                sourceReadiness: try container.decode(ReadinessLevel.self, forKey: .sourceReadiness),
                targetReadiness: try container.decode(ReadinessLevel.self, forKey: .targetReadiness),
                parameterDeltas: try container.decode([ParameterDeltaEvidence].self, forKey: .parameterDeltas)
            )
        }
    }
}
