import Foundation
import KuyuCore

/// Dense per-physics-step reward for the reference quadrotor tasks.
///
/// # Survival-normalized contract
///
/// Every shaping term is a penalty normalized to `[0, 1]`, so their weighted
/// sum is bounded by `Config.penaltyWeightSum`. The step reward is the survival
/// budget scaled by how much of that penalty budget the state left unspent:
///
/// ```text
/// reward = survivalReward * (1 - penalty / penaltyWeightSum)   in [0, survivalReward]
/// reward -= failurePenalty                                     on the failing step
/// ```
///
/// This is a positive affine transform of the previous
/// `aliveReward - penalty` form, so it preserves the ordering and the relative
/// magnitude of the shaping gradient. What it changes is the sign of the
/// continuation value. A terminated episode bootstraps to zero, so with an
/// all-negative step reward the absorbing terminal state was the highest-value
/// reachable state and the discounted advantage of crashing was strongly
/// positive; a one-time `failurePenalty` cannot offset a whole discounted
/// remaining life. With a non-negative step reward, continuing is worth at
/// least as much as terminating for every reachable state, and
/// `failurePenalty` only widens that margin.
public struct ReferenceQuadrotorDenseReward: RewardFunction {
    public enum RewardError: Error, Equatable {
        case nonFinite
        /// A weight is negative, so the penalty sum can leave `[0, penaltyWeightSum]`
        /// and the reward can leave `[0, survivalReward]`.
        case negativeWeight
        /// The penalty weights sum to zero (or worse), so the survival scale is
        /// undefined and no shaping gradient exists.
        case degeneratePenaltyWeights
        /// The scenario's `tiltSafeMaxDegrees` yields a non-finite or
        /// non-positive tilt normalization, so the tilt activation has no
        /// defined scale. Reported rather than clamped: a negative divisor
        /// would silently drive the activation to zero and remove the tilt
        /// penalty entirely for a scenario that is already corrupt.
        case degenerateTiltNormalization(Double)
    }

    public struct Config: Sendable, Codable, Equatable {
        public let survivalReward: Double
        public let tiltPenalty: Double
        public let omegaPenalty: Double
        public let altitudePenalty: Double
        public let verticalVelocityPenalty: Double
        public let controlPenalty: Double
        public let failurePenalty: Double

        public init(
            survivalReward: Double = 1.0,
            tiltPenalty: Double = 1.0,
            omegaPenalty: Double = 0.25,
            altitudePenalty: Double = 0.4,
            verticalVelocityPenalty: Double = 0.1,
            controlPenalty: Double = 0.02,
            failurePenalty: Double = 10.0
        ) {
            self.survivalReward = survivalReward
            self.tiltPenalty = tiltPenalty
            self.omegaPenalty = omegaPenalty
            self.altitudePenalty = altitudePenalty
            self.verticalVelocityPenalty = verticalVelocityPenalty
            self.controlPenalty = controlPenalty
            self.failurePenalty = failurePenalty
        }

        /// Normalization denominator for the shaping terms. Equal to the maximum
        /// attainable penalty because every normalized activation is in `[0, 1]`.
        public var penaltyWeightSum: Double {
            tiltPenalty + omegaPenalty + altitudePenalty + verticalVelocityPenalty + controlPenalty
        }
    }

    public let config: Config
    public let descriptor: RewardDescriptor
    /// Validation deferred from `init` to `reward(_:)`: the initializer is used
    /// as a default argument and as a stored-property initializer, neither of
    /// which may throw. The failure is reported as a typed error on first use
    /// rather than silently clamped.
    private let configError: RewardError?
    private let penaltyWeightSum: Double

    public init(config: Config = Config()) {
        self.config = config
        self.penaltyWeightSum = config.penaltyWeightSum
        self.configError = Self.validate(config)
        // version 5: survival-normalized reward. The shaped term is now
        // `survivalReward * (1 - penalty / penaltyWeightSum)` in [0, survivalReward]
        // instead of `aliveReward - penalty` in [-penaltyWeightSum, aliveReward],
        // which removes the early-termination incentive that an all-negative step
        // reward creates under bootstrap-to-zero termination.
        //
        // version 6: the altitude and vertical-velocity activations are
        // normalized by `ReferenceQuadrotorErrorNormalization` instead of a
        // clamped ratio. Both quantities are physically unbounded, so clamping
        // them made the reward exactly flat along the vertical axis for 98.32%
        // and 99.85% of the measured training distribution; the policy could
        // hold 3.67x hover thrust indefinitely at no marginal cost. The bound
        // and the weights are unchanged, so v6 is a strictly monotone
        // re-shaping of v5 in the vertical terms only.
        //
        // version 7: the tilt activation is normalized by
        // `ReferenceQuadrotorErrorNormalization.tiltNormalizationRadians`, the
        // scenario's own termination limit, instead of a fixed `pi / 2`. Under
        // v6 a 60-degree envelope left a third of the tilt range past
        // termination and charged the same marginal penalty per degree at 5
        // degrees as at 59, so the reward paid nothing extra for the last 30
        // degrees before the cliff and the measured policy spent its whole tilt
        // margin. The normalization is scenario-derived and therefore not part
        // of `configHash`, matching how the altitude tolerance and the
        // reference vertical velocity already come from the scenario.
        self.descriptor = RewardDescriptor(
            id: "reference-quadrotor-dense",
            version: "7",
            configHash: Self.configHash(config)
        )
    }

    private static func validate(_ config: Config) -> RewardError? {
        let weights = [
            config.survivalReward,
            config.tiltPenalty,
            config.omegaPenalty,
            config.altitudePenalty,
            config.verticalVelocityPenalty,
            config.controlPenalty,
            config.failurePenalty,
        ]
        guard weights.allSatisfy({ $0.isFinite }) else { return .nonFinite }
        guard weights.allSatisfy({ $0 >= 0 }) else { return .negativeWeight }
        guard config.penaltyWeightSum > 0 else { return .degeneratePenaltyWeights }
        return nil
    }

    public func reward(
        scenario: ReferenceQuadrotorScenarioDefinition,
        log: WorldStepLog,
        failure: FailureEvent?,
        truncated: Bool
    ) throws -> Double {
        if let configError { throw configError }

        // Full tilt penalty at the scenario's termination limit, so the reward
        // and `ReferenceQuadrotorSafetyCost` agree on where the cliff is.
        let tiltNormalization = ReferenceQuadrotorErrorNormalization.tiltNormalizationRadians(
            tiltSafeMaxDegrees: scenario.safetyEnvelope.tiltSafeMaxDegrees
        )
        guard tiltNormalization.isFinite, tiltNormalization > 0 else {
            throw RewardError.degenerateTiltNormalization(scenario.safetyEnvelope.tiltSafeMaxDegrees)
        }
        let normalizedTilt = clamp(log.safetyTrace.tiltRadians / tiltNormalization)
        let normalizedOmega = clamp(log.safetyTrace.omegaMagnitude / 20.0)
        let controlMagnitude = meanControlMagnitude(log: log)
        var penalty = (config.tiltPenalty * normalizedTilt)
            + (config.omegaPenalty * normalizedOmega)
            + (config.controlPenalty * controlMagnitude)

        let altitudeReference = try ReferenceQuadrotorAltitudeHoldReference(definition: scenario)
        // Dense vertical-velocity penalty for all tasks. Scenario semantics own
        // the reference velocity so reward, observation, and tensor worlds agree.
        // Normalized by `ReferenceQuadrotorErrorNormalization`, not by a clamped
        // ratio: vertical speed is unbounded and the clamped form was flat for
        // 99.85% of the measured training distribution.
        let normalizedVerticalVelocity = ReferenceQuadrotorErrorNormalization.saturating(
            error: abs(log.plantState.root.velocity.z),
            scale: altitudeReference.referenceVerticalVelocity
        )
        penalty += config.verticalVelocityPenalty * normalizedVerticalVelocity

        // Dense altitude-position penalty for all tasks. Lift/single-lift use their
        // explicit lift envelope; attitude has no lift envelope, so the scenario's
        // initial altitude is the hover target. Without this P-like position term,
        // attitude PPO only sees a D-like |vz| penalty and can settle into slow
        // descent with little immediate reward pressure to climb back.
        let altitudeError = abs(log.plantState.root.position.z - altitudeReference.targetPosition.z)
        let normalizedAltitudeError = ReferenceQuadrotorErrorNormalization.saturating(
            error: altitudeError,
            scale: altitudeReference.tolerance
        )
        penalty += config.altitudePenalty * normalizedAltitudeError

        var value = config.survivalReward * (1.0 - (penalty / penaltyWeightSum))

        if failure != nil {
            value -= config.failurePenalty
        }

        guard value.isFinite else { throw RewardError.nonFinite }
        return value
    }

    private func meanControlMagnitude(log: WorldStepLog) -> Double {
        if !log.driveIntents.isEmpty {
            let sum = log.driveIntents.reduce(0.0) { partial, drive in
                partial + abs(drive.activation)
            }
            return clamp(sum / Double(log.driveIntents.count))
        }
        if !log.actuatorValues.isEmpty {
            let sum = log.actuatorValues.reduce(0.0) { partial, actuator in
                partial + abs(actuator.value)
            }
            return clamp(sum / max(1.0, Double(log.actuatorValues.count)))
        }
        return 0.0
    }

    private func clamp(_ value: Double) -> Double {
        guard value.isFinite else { return 1.0 }
        return min(max(value, 0.0), 1.0)
    }

    private static func configHash(_ config: Config) -> String {
        let components = [
            config.survivalReward,
            config.tiltPenalty,
            config.omegaPenalty,
            config.altitudePenalty,
            config.verticalVelocityPenalty,
            config.controlPenalty,
            config.failurePenalty,
        ].map { String(format: "%.17g", $0) }
        let payload = components.joined(separator: "|")
        let digest = FNV1a64.hash(data: Array(payload.utf8))
        return String(format: "%016llx", digest)
    }
}
