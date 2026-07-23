import Foundation
import KuyuCore

/// Envelope-relative safety cost for constrained (Lagrangian) PPO.
///
/// Unlike `ReferenceQuadrotorDenseReward`, which folds safety penalties into
/// the scalar reward with hand-tuned weights, this cost isolates the safety
/// signal so a constrained optimizer can bound its expectation directly. The
/// cost is zero while the state stays inside a margin of the scenario's
/// `SafetyEnvelope` and grows linearly as tilt or angular velocity approach
/// and exceed the envelope limits. The continuous risk is integrated over the
/// transition duration, while a hard failure adds a duration-independent
/// impulse expressed in equivalent cost-seconds.
public struct ReferenceQuadrotorSafetyCost: CostFunction {
    public enum CostError: Error, Equatable {
        case nonFinite
        case invalidDuration(Double)
        case missingMeasurement
        case descriptorMismatch(expected: CostDescriptor, actual: CostDescriptor)
    }

    public struct Config: Sendable, Codable, Equatable {
        public struct ConstraintLimits: Sendable, Codable, Equatable {
            public enum ValidationError: Error, Equatable {
                case nonFinite(String)
                case outOfRange(String)
                case empty
            }

            public let maximumAngularRate: Double?
            public let maximumAttitudeDeviationRadians: Double?
            public let minimumRootAltitude: Double?
            public let altitudeWeight: Double

            public init(
                maximumAngularRate: Double? = nil,
                maximumAttitudeDeviationRadians: Double? = nil,
                minimumRootAltitude: Double? = nil,
                altitudeWeight: Double = 1.0
            ) throws {
                guard maximumAngularRate != nil
                    || maximumAttitudeDeviationRadians != nil
                    || minimumRootAltitude != nil else {
                    throw ValidationError.empty
                }
                try Self.validatePositive(maximumAngularRate, field: "maximumAngularRate")
                try Self.validatePositive(
                    maximumAttitudeDeviationRadians,
                    field: "maximumAttitudeDeviationRadians"
                )
                if let maximumAttitudeDeviationRadians,
                   maximumAttitudeDeviationRadians > .pi {
                    throw ValidationError.outOfRange("maximumAttitudeDeviationRadians")
                }
                if let minimumRootAltitude, !minimumRootAltitude.isFinite {
                    throw ValidationError.nonFinite("minimumRootAltitude")
                }
                guard altitudeWeight.isFinite else {
                    throw ValidationError.nonFinite("altitudeWeight")
                }
                guard altitudeWeight >= 0 else {
                    throw ValidationError.outOfRange("altitudeWeight")
                }
                self.maximumAngularRate = maximumAngularRate
                self.maximumAttitudeDeviationRadians = maximumAttitudeDeviationRadians
                self.minimumRootAltitude = minimumRootAltitude
                self.altitudeWeight = altitudeWeight
            }

            private static func validatePositive(_ value: Double?, field: String) throws {
                guard let value else { return }
                guard value.isFinite else { throw ValidationError.nonFinite(field) }
                guard value > 0 else { throw ValidationError.outOfRange(field) }
            }
        }

        public enum ValidationError: Error, Equatable {
            case nonFinite(String)
            case outOfRange(String)
        }

        /// Fraction of the envelope limit below which the cost is zero.
        /// Above it the cost rises linearly, reaching 1 at the limit.
        public let marginFraction: Double
        public let tiltWeight: Double
        public let omegaWeight: Double
        /// Optional task-level hard limits shared with checkpoint acceptance.
        /// Scenario limits remain active; the stricter bound wins.
        public let constraintLimits: ConstraintLimits?
        /// Added once when the transition reports a hard failure. The value is
        /// expressed in equivalent cost-seconds and is not scaled by duration.
        public let failureImpulseCost: Double

        public init(
            marginFraction: Double = 0.8,
            tiltWeight: Double = 1.0,
            omegaWeight: Double = 1.0,
            constraintLimits: ConstraintLimits? = nil,
            failureImpulseCost: Double = 1.0
        ) throws {
            guard marginFraction.isFinite else { throw ValidationError.nonFinite("marginFraction") }
            guard tiltWeight.isFinite else { throw ValidationError.nonFinite("tiltWeight") }
            guard omegaWeight.isFinite else { throw ValidationError.nonFinite("omegaWeight") }
            guard failureImpulseCost.isFinite else {
                throw ValidationError.nonFinite("failureImpulseCost")
            }
            guard marginFraction >= 0, marginFraction < 1 else {
                throw ValidationError.outOfRange("marginFraction")
            }
            guard tiltWeight >= 0 else { throw ValidationError.outOfRange("tiltWeight") }
            guard omegaWeight >= 0 else { throw ValidationError.outOfRange("omegaWeight") }
            guard failureImpulseCost >= 0 else {
                throw ValidationError.outOfRange("failureImpulseCost")
            }
            self.marginFraction = marginFraction
            self.tiltWeight = tiltWeight
            self.omegaWeight = omegaWeight
            self.constraintLimits = constraintLimits
            self.failureImpulseCost = failureImpulseCost
        }
    }

    public let config: Config
    public let descriptor: CostDescriptor

    public init(config: Config) {
        self.config = config
        self.descriptor = CostDescriptor(
            id: "reference-quadrotor-safety-cost",
            version: "3",
            configHash: Self.configHash(config)
        )
    }

    public func cost(
        scenario: ReferenceQuadrotorScenarioDefinition,
        log: WorldStepLog,
        duration: Double,
        failure: FailureEvent?,
        truncated: Bool
    ) throws -> Double {
        try cost(
            tiltRadians: log.safetyTrace.tiltRadians,
            omegaMagnitude: log.safetyTrace.omegaMagnitude,
            rootAltitude: log.plantState.root.position.z,
            targetAltitude: try ReferenceQuadrotorAltitudeHoldReference(
                definition: scenario
            ).targetPosition.z,
            safetyEnvelope: scenario.safetyEnvelope,
            duration: duration,
            failed: failure != nil
        )
    }

    /// Scalar entry point for pipelines that carry state outside `WorldStepLog`
    /// (e.g. recorded training datasets). The log-based overload delegates here
    /// so both paths share one cost definition.
    public func cost(
        tiltRadians: Double,
        omegaMagnitude: Double,
        rootAltitude: Double? = nil,
        targetAltitude: Double? = nil,
        safetyEnvelope: SafetyEnvelope,
        duration: Double,
        failed: Bool
    ) throws -> Double {
        guard duration.isFinite, duration > 0 else {
            throw CostError.invalidDuration(duration)
        }
        let scenarioTiltLimit = safetyEnvelope.tiltSafeMaxDegrees * Double.pi / 180.0
        let tiltLimitRadians = stricterUpperBound(
            scenarioTiltLimit,
            config.constraintLimits?.maximumAttitudeDeviationRadians
        )
        let omegaLimit = stricterUpperBound(
            safetyEnvelope.omegaSafeMax,
            config.constraintLimits?.maximumAngularRate
        )
        let normalizedTilt = normalizedMagnitude(
            tiltRadians,
            limit: tiltLimitRadians
        )
        let normalizedOmega = normalizedMagnitude(
            omegaMagnitude,
            limit: omegaLimit
        )
        var riskRate = config.tiltWeight * hinge(normalizedTilt)
            + config.omegaWeight * hinge(normalizedOmega)
        if let constraintLimits = config.constraintLimits,
           let minimumRootAltitude = constraintLimits.minimumRootAltitude {
            riskRate += constraintLimits.altitudeWeight * hinge(
                normalizedLowerBoundRisk(
                    value: rootAltitude,
                    reference: targetAltitude,
                    limit: max(safetyEnvelope.groundZ, minimumRootAltitude)
                )
            )
        }
        let value = riskRate * duration + (failed ? config.failureImpulseCost : 0)
        guard value.isFinite else { throw CostError.nonFinite }
        return value
    }

    public func measuredCost(in step: EnvironmentStep) throws -> Double {
        guard let measurement = step.costMeasurement else {
            throw CostError.missingMeasurement
        }
        guard measurement.descriptor == descriptor else {
            throw CostError.descriptorMismatch(
                expected: descriptor,
                actual: measurement.descriptor
            )
        }
        return measurement.value
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

    private func normalizedLowerBoundRisk(
        value: Double?,
        reference: Double?,
        limit: Double
    ) -> Double {
        guard let value,
              let reference,
              value.isFinite,
              reference.isFinite,
              limit.isFinite,
              reference > limit else {
            return 2.0
        }
        return (reference - value) / (reference - limit)
    }

    private func stricterUpperBound(_ scenarioLimit: Double, _ taskLimit: Double?) -> Double {
        guard let taskLimit else { return scenarioLimit }
        return min(scenarioLimit, taskLimit)
    }

    private static func configHash(_ config: Config) -> String {
        let components = [
            config.marginFraction,
            config.tiltWeight,
            config.omegaWeight,
            config.failureImpulseCost,
        ].map { String(format: "%.17g", $0) }
        let limits = config.constraintLimits
        let constraintComponents = [
            limits?.maximumAngularRate,
            limits?.maximumAttitudeDeviationRadians,
            limits?.minimumRootAltitude,
            limits?.altitudeWeight,
        ].map { value in
            value.map { String(format: "%.17g", $0) } ?? "nil"
        }
        let payload = (components + constraintComponents).joined(separator: "|")
        let digest = FNV1a64.hash(data: Array(payload.utf8))
        return String(format: "%016llx", digest)
    }
}
