import KuyuCore
import KuyuPhysics

public struct ReferenceQuadrotorEnvironmentExecutionContract: Sendable, Codable, Equatable {
    public static let schemaVersion = 1

    public let schemaVersion: Int
    public let simulation: SimulationConfig
    public let actionRealization: ReferenceQuadrotorActionRealization
    public let parameters: ReferenceQuadrotorParameters
    public let robotManifestID: String?
    public let motorNerveRateLimitPerSecond: Double
    public let motorNerveSmoothingTimeConstant: Double?

    public init(
        simulation: SimulationConfig,
        actionRealization: ReferenceQuadrotorActionRealization,
        parameters: ReferenceQuadrotorParameters,
        robotManifestID: String?,
        motorNerveRateLimitPerSecond: Double,
        motorNerveSmoothingTimeConstant: Double?
    ) {
        self.schemaVersion = Self.schemaVersion
        self.simulation = simulation
        self.actionRealization = actionRealization
        self.parameters = parameters
        self.robotManifestID = robotManifestID
        self.motorNerveRateLimitPerSecond = motorNerveRateLimitPerSecond
        self.motorNerveSmoothingTimeConstant = motorNerveSmoothingTimeConstant
    }
}
