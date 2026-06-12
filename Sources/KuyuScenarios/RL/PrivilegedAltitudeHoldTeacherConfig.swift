public struct PrivilegedAltitudeHoldTeacherConfig: Sendable, Codable, Equatable {
    /// Gains must reject the worst sustained A1 thrust deficit (Suite-2 motor
    /// gainScale 0.8 ≈ 0.49 N on the 1 kg baseline plant) while keeping
    /// |vz| below the 0.05 m/s sustained-fall envelope. The loop closes on
    /// privileged (true) altitude state, so the only lag is the 0.030 s motor
    /// time constant; Kd = 12 keeps the crossover (~12 rad/s) well below the
    /// motor pole (~33 rad/s) and the response overdamped (ζ ≈ 1.7).
    public static let activeAltitudeHold = PrivilegedAltitudeHoldTeacherConfig(
        validatedAltitudeKp: 12.0,
        verticalVelocityKd: 12.0,
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
        altitudeKp: Double = 12.0,
        verticalVelocityKd: Double = 12.0,
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
