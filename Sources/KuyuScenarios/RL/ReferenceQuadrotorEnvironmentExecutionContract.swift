import KuyuCore
import KuyuPhysics

public struct ReferenceQuadrotorEnvironmentExecutionContract: Sendable, Codable, Equatable {
    public enum ValidationError: Error, Equatable {
        case invalidCanonicalExecutorVersion(String)
        case unsupportedSchemaVersion(expected: Int, actual: Int)
    }

    public static let schemaVersion = 2

    public let schemaVersion: Int
    public let canonicalExecutorVersion: String
    public let simulation: SimulationConfig
    public let actionRealization: ReferenceQuadrotorActionRealization
    public let parameters: ReferenceQuadrotorParameters
    public let robotManifestID: String?
    public let motorNerveRateLimitPerSecond: Double
    public let motorNerveSmoothingTimeConstant: Double?

    public init(
        canonicalExecutorVersion: String,
        simulation: SimulationConfig,
        actionRealization: ReferenceQuadrotorActionRealization,
        parameters: ReferenceQuadrotorParameters,
        robotManifestID: String?,
        motorNerveRateLimitPerSecond: Double,
        motorNerveSmoothingTimeConstant: Double?
    ) throws {
        guard !canonicalExecutorVersion.isEmpty,
              canonicalExecutorVersion.utf8.allSatisfy({ $0 > 32 && $0 < 127 }) else {
            throw ValidationError.invalidCanonicalExecutorVersion(canonicalExecutorVersion)
        }
        self.schemaVersion = Self.schemaVersion
        self.canonicalExecutorVersion = canonicalExecutorVersion
        self.simulation = simulation
        self.actionRealization = actionRealization
        self.parameters = parameters
        self.robotManifestID = robotManifestID
        self.motorNerveRateLimitPerSecond = motorNerveRateLimitPerSecond
        self.motorNerveSmoothingTimeConstant = motorNerveSmoothingTimeConstant
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        let decodedSchemaVersion = try container.decode(Int.self, forKey: .schemaVersion)
        guard decodedSchemaVersion == Self.schemaVersion else {
            throw ValidationError.unsupportedSchemaVersion(
                expected: Self.schemaVersion,
                actual: decodedSchemaVersion
            )
        }
        try self.init(
            canonicalExecutorVersion: container.decode(String.self, forKey: .canonicalExecutorVersion),
            simulation: container.decode(SimulationConfig.self, forKey: .simulation),
            actionRealization: container.decode(
                ReferenceQuadrotorActionRealization.self,
                forKey: .actionRealization
            ),
            parameters: container.decode(ReferenceQuadrotorParameters.self, forKey: .parameters),
            robotManifestID: container.decodeIfPresent(String.self, forKey: .robotManifestID),
            motorNerveRateLimitPerSecond: container.decode(
                Double.self,
                forKey: .motorNerveRateLimitPerSecond
            ),
            motorNerveSmoothingTimeConstant: container.decodeIfPresent(
                Double.self,
                forKey: .motorNerveSmoothingTimeConstant
            )
        )
    }
}
