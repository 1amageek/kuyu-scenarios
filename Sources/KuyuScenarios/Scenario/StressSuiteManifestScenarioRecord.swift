import Foundation
import KuyuCore
import KuyuPhysics

public extension StressSuiteManifest {
    struct ScenarioRecord: Sendable, Codable, Equatable {
        public enum ValidationError: Error, Equatable {
            case emptyScenarioID
            case nonFiniteDuration
            case nonPositiveDuration
        }

        public let scenarioID: String
        public let seed: UInt64
        public let duration: Double
        public let timeStep: TimeStep
        public let configHash: String?
        public let dimensions: [StressDimension]
        public let failureReason: FailureReason?

        private enum CodingKeys: String, CodingKey {
            case scenarioID
            case seed
            case duration
            case timeStep
            case configHash
            case dimensions
            case failureReason
        }

        public init(
            scenarioID: String,
            seed: UInt64,
            duration: Double,
            timeStep: TimeStep,
            configHash: String?,
            dimensions: [StressDimension],
            failureReason: FailureReason? = nil
        ) throws {
            let trimmed = scenarioID.trimmingCharacters(in: .whitespacesAndNewlines)
            guard !trimmed.isEmpty else { throw ValidationError.emptyScenarioID }
            guard duration.isFinite else { throw ValidationError.nonFiniteDuration }
            guard duration > 0 else { throw ValidationError.nonPositiveDuration }

            self.scenarioID = trimmed
            self.seed = seed
            self.duration = duration
            self.timeStep = timeStep
            self.configHash = configHash
            self.dimensions = Self.canonicalDimensions(dimensions)
            self.failureReason = failureReason
        }

        public init(from decoder: Decoder) throws {
            let container = try decoder.container(keyedBy: CodingKeys.self)
            try self.init(
                scenarioID: try container.decode(String.self, forKey: .scenarioID),
                seed: try container.decode(UInt64.self, forKey: .seed),
                duration: try container.decode(Double.self, forKey: .duration),
                timeStep: try container.decode(TimeStep.self, forKey: .timeStep),
                configHash: try container.decodeIfPresent(String.self, forKey: .configHash),
                dimensions: try container.decode([StressDimension].self, forKey: .dimensions),
                failureReason: try container.decodeIfPresent(FailureReason.self, forKey: .failureReason)
            )
        }

        public var key: String {
            "\(scenarioID):\(seed)"
        }

        private static func canonicalDimensions(_ dimensions: [StressDimension]) -> [StressDimension] {
            Array(Set(dimensions)).sorted { $0.rawValue < $1.rawValue }
        }
    }
}
