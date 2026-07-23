import Foundation
import KuyuPhysics

public struct LongHorizonMorphologyTransferContract: Sendable, Codable, Equatable {
    public struct ParameterDelta: Sendable, Codable, Equatable {
        public enum ValidationError: Error, Equatable {
            case emptyName
            case nonFiniteSourceValue(String)
            case nonFiniteTargetValue(String)
            case unchangedValue(String)
        }

        public let name: String
        public let sourceValue: Double
        public let targetValue: Double

        private enum CodingKeys: String, CodingKey {
            case name
            case sourceValue
            case targetValue
        }

        public init(name: String, sourceValue: Double, targetValue: Double) throws {
            let trimmed = name.trimmingCharacters(in: .whitespacesAndNewlines)
            guard !trimmed.isEmpty else { throw ValidationError.emptyName }
            guard sourceValue.isFinite else { throw ValidationError.nonFiniteSourceValue(trimmed) }
            guard targetValue.isFinite else { throw ValidationError.nonFiniteTargetValue(trimmed) }
            guard abs(sourceValue - targetValue) > 1e-12 else {
                throw ValidationError.unchangedValue(trimmed)
            }

            self.name = trimmed
            self.sourceValue = sourceValue
            self.targetValue = targetValue
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

    public enum ValidationError: Error, Equatable {
        case emptySourceRobotID
        case emptyTargetRobotID
        case identicalRobotIDs(String)
        case sourceReadinessBelowDynamicSimulation(ReadinessLevel)
        case targetReadinessBelowDynamicSimulation(ReadinessLevel)
        case emptyParameterDeltas
        case duplicateParameterDelta(String)
    }

    public let sourceRobotID: String
    public let targetRobotID: String
    public let sourceReadiness: ReadinessLevel
    public let targetReadiness: ReadinessLevel
    public let parameterDeltas: [ParameterDelta]

    private enum CodingKeys: String, CodingKey {
        case sourceRobotID
        case targetRobotID
        case sourceReadiness
        case targetReadiness
        case parameterDeltas
    }

    public init(
        sourceRobotID: String,
        targetRobotID: String,
        sourceReadiness: ReadinessLevel,
        targetReadiness: ReadinessLevel,
        parameterDeltas: [ParameterDelta]
    ) throws {
        let trimmedSource = sourceRobotID.trimmingCharacters(in: .whitespacesAndNewlines)
        let trimmedTarget = targetRobotID.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmedSource.isEmpty else { throw ValidationError.emptySourceRobotID }
        guard !trimmedTarget.isEmpty else { throw ValidationError.emptyTargetRobotID }
        guard trimmedSource != trimmedTarget else {
            throw ValidationError.identicalRobotIDs(trimmedSource)
        }
        guard sourceReadiness >= .dynamicSimulation else {
            throw ValidationError.sourceReadinessBelowDynamicSimulation(sourceReadiness)
        }
        guard targetReadiness >= .dynamicSimulation else {
            throw ValidationError.targetReadinessBelowDynamicSimulation(targetReadiness)
        }
        guard !parameterDeltas.isEmpty else { throw ValidationError.emptyParameterDeltas }

        var seen: Set<String> = []
        for delta in parameterDeltas {
            guard seen.insert(delta.name).inserted else {
                throw ValidationError.duplicateParameterDelta(delta.name)
            }
        }

        self.sourceRobotID = trimmedSource
        self.targetRobotID = trimmedTarget
        self.sourceReadiness = sourceReadiness
        self.targetReadiness = targetReadiness
        self.parameterDeltas = parameterDeltas
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            sourceRobotID: try container.decode(String.self, forKey: .sourceRobotID),
            targetRobotID: try container.decode(String.self, forKey: .targetRobotID),
            sourceReadiness: try container.decode(ReadinessLevel.self, forKey: .sourceReadiness),
            targetReadiness: try container.decode(ReadinessLevel.self, forKey: .targetReadiness),
            parameterDeltas: try container.decode([ParameterDelta].self, forKey: .parameterDeltas)
        )
    }
}
