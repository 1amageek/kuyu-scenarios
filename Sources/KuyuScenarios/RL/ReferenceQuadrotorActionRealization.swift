import KuyuPhysics

public enum ReferenceQuadrotorActionKind: String, Sendable, Codable, Equatable {
    case driveMixer
    case temporalCTBR
}

public enum ReferenceQuadrotorActionRealization: Sendable, Codable, Equatable {
    case driveMixer
    case temporalCTBR(ReferenceQuadrotorCTBRControlConfig)

    public var kind: ReferenceQuadrotorActionKind {
        switch self {
        case .driveMixer:
            .driveMixer
        case .temporalCTBR:
            .temporalCTBR
        }
    }
}
