/// Bounded normalization for error magnitudes that have no physical upper
/// bound, such as altitude error and vertical speed.
///
/// # Why not `clamp(error / scale)`
///
/// The survival-normalized reward contract requires every shaping activation to
/// stay in `[0, 1]` so the weighted penalty sum is bounded by
/// `penaltyWeightSum`. Clamping a ratio satisfies that bound, but it also makes
/// the activation exactly constant for every `error >= scale`, which removes the
/// shaping gradient in precisely the regime where the state is worst.
///
/// That flat region was not hypothetical. Measured over the 280,541 on-policy
/// KUY-ATT-1 transitions of `corridor-v5v4-seg2c-20260726` iteration 0, with
/// `attitudeTolerance` 0.2 m and `attitudeReferenceVerticalVelocity` 0.05 m/s:
/// the vertical-velocity activation saturated at t = 0.014 s and the altitude
/// activation at t = 0.156 s in every single episode, leaving 99.85% and 98.32%
/// of all transitions on the flat side. The reward therefore carried no gradient
/// along the whole vertical axis for effectively the entire training
/// distribution, and the policy settled on a collective thrust of 0.75 -- 3.67x
/// hover for the 12 N reference motor -- climbing to 165 m/s and 720 m above the
/// hover target without ever paying more than the saturated 0.5 of the 1.77
/// penalty budget.
///
/// # Contract
///
/// `saturating(error:scale:)` returns `error / (scale + error)`, which is:
///
/// - in `[0, 1)`, so the survival-normalized bound still holds;
/// - strictly increasing in `error`, so the gradient never vanishes;
/// - equal to `error / scale` to first order as `error` approaches zero, so
///   near-target shaping keeps the slope the clamped form had;
/// - equal to `0.5` at `error == scale`, so `scale` keeps its meaning as the
///   characteristic error at which the term becomes significant.
public enum ReferenceQuadrotorErrorNormalization {
    /// Maps a non-negative error magnitude into `[0, 1)` without saturating.
    ///
    /// Non-finite or negative input maps to `1.0` and `0.0` respectively, so a
    /// caller can never turn a broken state into an unpenalized one.
    public static func saturating(error: Double, scale: Double) -> Double {
        guard error.isFinite else { return 1.0 }
        guard error > 0 else { return 0.0 }
        let denominator = max(scale, 1e-6) + error
        guard denominator > 0 else { return 1.0 }
        return error / denominator
    }
}
