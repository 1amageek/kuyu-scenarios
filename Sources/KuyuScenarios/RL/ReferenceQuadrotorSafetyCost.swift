import Foundation
import KuyuCore

/// Envelope-relative safety cost for constrained (Lagrangian) PPO.
///
/// Unlike `ReferenceQuadrotorDenseReward`, which folds safety penalties into
/// the scalar reward with hand-tuned weights, this cost isolates the safety
/// signal so a constrained optimizer can bound its expectation directly. The
/// cost is zero while the state stays inside a margin of the scenario's
/// `SafetyEnvelope` and grows linearly as tilt or angular velocity approach
/// and exceed the envelope limits; a hard failure adds a fixed violation cost.
public struct ReferenceQuadrotorSafetyCost: CostFunction {
    public enum CostError: Error, Equatable {
        case nonFinite
    }

    public struct Config: Sendable, Codable, Equatable {
        public enum ValidationError: Error, Equatable {
            case nonFinite(String)
            case outOfRange(String)
        }

        /// Fraction of the envelope limit below which the cost is zero.
        /// Above it the cost rises linearly, reaching 1 at the limit.
        public let marginFraction: Double
        public let tiltWeight: Double
        public let omegaWeight: Double
        /// Added once per step whenever the world reports a failure event.
        public let violationCost: Double

        public init(
            marginFraction: Double = 0.8,
            tiltWeight: Double = 1.0,
            omegaWeight: Double = 1.0,
            violationCost: Double = 1.0
        ) throws {
            guard marginFraction.isFinite else { throw ValidationError.nonFinite("marginFraction") }
            guard tiltWeight.isFinite else { throw ValidationError.nonFinite("tiltWeight") }
            guard omegaWeight.isFinite else { throw ValidationError.nonFinite("omegaWeight") }
            guard violationCost.isFinite else { throw ValidationError.nonFinite("violationCost") }
            guard marginFraction >= 0, marginFraction < 1 else {
                throw ValidationError.outOfRange("marginFraction")
            }
            guard tiltWeight >= 0 else { throw ValidationError.outOfRange("tiltWeight") }
            guard omegaWeight >= 0 else { throw ValidationError.outOfRange("omegaWeight") }
            guard violationCost >= 0 else { throw ValidationError.outOfRange("violationCost") }
            self.marginFraction = marginFraction
            self.tiltWeight = tiltWeight
            self.omegaWeight = omegaWeight
            self.violationCost = violationCost
        }
    }

    public let config: Config
    public let descriptor: CostDescriptor

    public init(config: Config) {
        self.config = config
        self.descriptor = CostDescriptor(
            id: "reference-quadrotor-safety-cost",
            version: "1",
            configHash: Self.configHash(config)
        )
    }

    public func cost(
        scenario: ReferenceQuadrotorScenarioDefinition,
        log: WorldStepLog,
        failure: FailureEvent?,
        truncated: Bool
    ) throws -> Double {
        try cost(
            tiltRadians: log.safetyTrace.tiltRadians,
            omegaMagnitude: log.safetyTrace.omegaMagnitude,
            safetyEnvelope: scenario.safetyEnvelope,
            failed: failure != nil
        )
    }

    /// Scalar entry point for pipelines that carry state outside `WorldStepLog`
    /// (e.g. recorded training datasets). The log-based overload delegates here
    /// so both paths share one cost definition.
    public func cost(
        tiltRadians: Double,
        omegaMagnitude: Double,
        safetyEnvelope: SafetyEnvelope,
        failed: Bool
    ) throws -> Double {
        let tiltLimitRadians = safetyEnvelope.tiltSafeMaxDegrees * Double.pi / 180.0
        let normalizedTilt = normalizedMagnitude(
            tiltRadians,
            limit: tiltLimitRadians
        )
        let normalizedOmega = normalizedMagnitude(
            omegaMagnitude,
            limit: safetyEnvelope.omegaSafeMax
        )
        var value = config.tiltWeight * hinge(normalizedTilt)
            + config.omegaWeight * hinge(normalizedOmega)
        if failed {
            value += config.violationCost
        }
        guard value.isFinite else { throw CostError.nonFinite }
        return value
    }

    /// Linear hinge in the normalized magnitude: 0 below the margin, 1 at the
    /// envelope limit, capped at 2 so post-violation states stay bounded.
    private func hinge(_ normalized: Double) -> Double {
        let span = 1.0 - config.marginFraction
        let raw = (normalized - config.marginFraction) / span
        return min(max(raw, 0.0), 2.0)
    }

    /// Non-finite traces map to the capped maximum so a diverging plant is
    /// maximally costly instead of silently cheap.
    private func normalizedMagnitude(_ value: Double, limit: Double) -> Double {
        guard value.isFinite, limit > 0 else { return 2.0 }
        return abs(value) / limit
    }

    private static func configHash(_ config: Config) -> String {
        let components = [
            config.marginFraction,
            config.tiltWeight,
            config.omegaWeight,
            config.violationCost,
        ].map { String(format: "%.17g", $0) }
        let payload = components.joined(separator: "|")
        let digest = FNV1a64.hash(data: Array(payload.utf8))
        return String(format: "%016llx", digest)
    }
}
