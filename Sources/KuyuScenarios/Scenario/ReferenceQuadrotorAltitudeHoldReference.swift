import KuyuCore
import KuyuPhysics

public enum ReferenceQuadrotorAltitudeHoldReferenceError: Error, Sendable, Equatable {
    case nonFiniteTargetPosition(Axis3)
    case invalidTolerance(Double)
    case invalidReferenceVerticalVelocity(Double)
}

public struct ReferenceQuadrotorAltitudeHoldReference: Sendable, Codable, Equatable {
    public static let attitudeTolerance = 0.2
    public static let attitudeReferenceVerticalVelocity = 0.5

    public let targetPosition: Axis3
    public let tolerance: Double
    public let referenceVerticalVelocity: Double

    public init(
        targetPosition: Axis3,
        tolerance: Double,
        referenceVerticalVelocity: Double
    ) throws {
        guard targetPosition.x.isFinite, targetPosition.y.isFinite, targetPosition.z.isFinite else {
            throw ReferenceQuadrotorAltitudeHoldReferenceError.nonFiniteTargetPosition(targetPosition)
        }
        guard tolerance.isFinite, tolerance > 0 else {
            throw ReferenceQuadrotorAltitudeHoldReferenceError.invalidTolerance(tolerance)
        }
        guard referenceVerticalVelocity.isFinite, referenceVerticalVelocity > 0 else {
            throw ReferenceQuadrotorAltitudeHoldReferenceError
                .invalidReferenceVerticalVelocity(referenceVerticalVelocity)
        }
        self.targetPosition = targetPosition
        self.tolerance = tolerance
        self.referenceVerticalVelocity = referenceVerticalVelocity
    }

    public init(definition: ReferenceQuadrotorScenarioDefinition) throws {
        if let liftEnvelope = definition.liftEnvelope {
            try self.init(
                targetPosition: Axis3(
                    x: definition.initialPosition.x,
                    y: definition.initialPosition.y,
                    z: liftEnvelope.targetZ
                ),
                tolerance: liftEnvelope.tolerance,
                referenceVerticalVelocity: liftEnvelope.maxVelocity
            )
        } else {
            try self.init(
                targetPosition: definition.initialPosition,
                tolerance: Self.attitudeTolerance,
                referenceVerticalVelocity: Self.attitudeReferenceVerticalVelocity
            )
        }
    }
}
