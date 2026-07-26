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

    /// Tilt at which the reward's tilt activation reaches its full penalty:
    /// the scenario's own termination limit, but never weaker than the fixed
    /// `pi / 2` the reward used before.
    ///
    /// # Why the envelope limit and not `pi / 2`
    ///
    /// The safety envelope ends the episode at `tiltSafeMaxDegrees`, and
    /// `ReferenceQuadrotorSafetyCost` normalizes by that same limit. Dividing
    /// the reward's tilt error by a fixed `pi / 2` decoupled the two. On a
    /// 60-degree envelope it put a third of the reward's tilt range past
    /// termination, in states the episode can never occupy, and left the
    /// marginal penalty per degree identical at 5 degrees and at 59 - so the
    /// reward applied no rising pressure as the boundary approached and a
    /// policy could spend its entire tilt margin without the reward noticing.
    /// Measured on `corridor-v6v4-samplerfix-seg1` and
    /// `corridor-v6v4-wide23-seg1`, the incumbent held all six surviving
    /// scenarios within 0.3 to 10.6 degrees of the 60-degree limit.
    ///
    /// # Why the minimum
    ///
    /// Lift and single-lift declare `tiltSafeMaxDegrees` 180, meaning tilt is
    /// unconstrained for those tasks. Normalizing by the envelope alone would
    /// halve their tilt slope rather than raise it. Taking the stricter of the
    /// envelope limit and `pi / 2` tightens the term exactly where the envelope
    /// binds tighter than the reward already assumed and leaves every other
    /// task at its previous scale. `ReferenceQuadrotorSafetyCost` resolves its
    /// scenario and task limits the same way.
    ///
    /// `SafetyEnvelope.init` rejects a non-finite or non-positive
    /// `tiltSafeMaxDegrees`, but its `Codable` conformance is synthesized, so a
    /// decoded scenario can still carry one. Such a limit propagates here
    /// instead of being replaced by a plausible scale, and every caller rejects
    /// it rather than dividing by it: `ReferenceQuadrotorDenseReward` throws
    /// `RewardError.degenerateTiltNormalization`, the tensor world throws at
    /// construction, and the differentiable SHAC reward throws from
    /// `Config.init`. Flooring it would be worse than useless here - a negative
    /// divisor clamps the tilt activation to zero, which removes the tilt
    /// penalty altogether for exactly the scenarios that are already corrupt.
    public static func tiltNormalizationRadians(tiltSafeMaxDegrees: Double) -> Double {
        let limit = tiltSafeMaxDegrees * Double.pi / 180.0
        guard limit.isFinite else { return limit }
        return min(limit, Double.pi / 2.0)
    }
}
