import KuyuCore
import KuyuPhysics

/// Control quality metrics analogous to Micro-World's PSNR/FVD.
///
/// Measures tracking accuracy, steady-state performance, control effort,
/// and smoothness — providing a complete picture of control quality
/// beyond safety envelope compliance.
public struct ControlQualityMetrics: Sendable, Codable, Equatable {

    private enum CodingKeys: String, CodingKey {
        case rmsTrackingError
        case maxTrackingError
        case steadyStateError
        case settlingTime
        case riseTime
        case maxOvershootDegrees = "percentOvershoot"
        case controlEffort
        case smoothness
    }

    /// Root-mean-square tracking error over the entire run.
    public let rmsTrackingError: Double

    /// Peak instantaneous tracking error.
    public let maxTrackingError: Double

    /// Mean error in the last 20% of the run (steady-state accuracy).
    public let steadyStateError: Double

    /// Time to first enter and remain within the settling band (seconds).
    public let settlingTime: Double?

    /// Time from 10% to 90% of a step response (seconds).
    public let riseTime: Double?

    /// Peak overshoot from the target (degrees).
    public let maxOvershootDegrees: Double?

    /// Integral of squared actuator output: ∫|u|²dt (energy consumption).
    public let controlEffort: Double

    /// Integral of squared actuator rate-of-change: ∫|du/dt|²dt (smoothness penalty).
    public let smoothness: Double

    public init(
        rmsTrackingError: Double,
        maxTrackingError: Double,
        steadyStateError: Double,
        settlingTime: Double?,
        riseTime: Double?,
        maxOvershootDegrees: Double?,
        controlEffort: Double,
        smoothness: Double
    ) {
        self.rmsTrackingError = rmsTrackingError
        self.maxTrackingError = maxTrackingError
        self.steadyStateError = steadyStateError
        self.settlingTime = settlingTime
        self.riseTime = riseTime
        self.maxOvershootDegrees = maxOvershootDegrees
        self.controlEffort = controlEffort
        self.smoothness = smoothness
    }
}
