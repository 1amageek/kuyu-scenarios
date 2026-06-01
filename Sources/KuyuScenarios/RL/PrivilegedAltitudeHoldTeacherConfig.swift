public struct PrivilegedAltitudeHoldTeacherConfig: Sendable, Codable, Equatable {
    public static let activeAltitudeHold = PrivilegedAltitudeHoldTeacherConfig(
        validatedAltitudeKp: 3.0,
        verticalVelocityKd: 2.0,
        maxThrottleCorrection: 0.20
    )

    public enum ValidationError: Error, Sendable, Equatable {
        case invalidAltitudeKp(Double)
        case invalidVerticalVelocityKd(Double)
        case invalidMaxThrottleCorrection(Double)
    }

    public let altitudeKp: Double
    public let verticalVelocityKd: Double
    public let maxThrottleCorrection: Double

    public init(
        altitudeKp: Double = 3.0,
        verticalVelocityKd: Double = 2.0,
        maxThrottleCorrection: Double = 0.20
    ) throws {
        guard altitudeKp.isFinite, altitudeKp >= 0 else {
            throw ValidationError.invalidAltitudeKp(altitudeKp)
        }
        guard verticalVelocityKd.isFinite, verticalVelocityKd >= 0 else {
            throw ValidationError.invalidVerticalVelocityKd(verticalVelocityKd)
        }
        guard maxThrottleCorrection.isFinite, maxThrottleCorrection >= 0, maxThrottleCorrection <= 1 else {
            throw ValidationError.invalidMaxThrottleCorrection(maxThrottleCorrection)
        }
        self.altitudeKp = altitudeKp
        self.verticalVelocityKd = verticalVelocityKd
        self.maxThrottleCorrection = maxThrottleCorrection
    }

    private init(
        validatedAltitudeKp altitudeKp: Double,
        verticalVelocityKd: Double,
        maxThrottleCorrection: Double
    ) {
        self.altitudeKp = altitudeKp
        self.verticalVelocityKd = verticalVelocityKd
        self.maxThrottleCorrection = maxThrottleCorrection
    }

    public var cacheComponent: String {
        "active-altitude-v1;altKp=\(altitudeKp);altKd=\(verticalVelocityKd);altLimit=\(maxThrottleCorrection)"
    }
}
