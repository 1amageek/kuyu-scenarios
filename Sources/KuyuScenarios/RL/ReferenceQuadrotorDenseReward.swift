import Foundation
import KuyuCore

public struct ReferenceQuadrotorDenseReward: RewardFunction {
    public enum RewardError: Error, Equatable {
        case nonFinite
    }

    public struct Config: Sendable, Codable, Equatable {
        public let aliveReward: Double
        public let tiltPenalty: Double
        public let omegaPenalty: Double
        public let altitudePenalty: Double
        public let verticalVelocityPenalty: Double
        public let controlPenalty: Double
        public let failurePenalty: Double

        public init(
            aliveReward: Double = 0.02,
            tiltPenalty: Double = 1.0,
            omegaPenalty: Double = 0.25,
            altitudePenalty: Double = 0.4,
            verticalVelocityPenalty: Double = 0.1,
            controlPenalty: Double = 0.02,
            failurePenalty: Double = 10.0
        ) {
            self.aliveReward = aliveReward
            self.tiltPenalty = tiltPenalty
            self.omegaPenalty = omegaPenalty
            self.altitudePenalty = altitudePenalty
            self.verticalVelocityPenalty = verticalVelocityPenalty
            self.controlPenalty = controlPenalty
            self.failurePenalty = failurePenalty
        }
    }

    public let config: Config
    public let descriptor: RewardDescriptor

    public init(config: Config = Config()) {
        self.config = config
        // version 3: attitude tasks without a lift envelope now receive dense
        // altitude-hold reward around their initial height. The configHash is over
        // the Config weights, so the version records this semantic change.
        self.descriptor = RewardDescriptor(
            id: "reference-quadrotor-dense",
            version: "3",
            configHash: Self.configHash(config)
        )
    }

    public func reward(
        scenario: ReferenceQuadrotorScenarioDefinition,
        log: WorldStepLog,
        failure: FailureEvent?,
        truncated: Bool
    ) throws -> Double {
        let normalizedTilt = clamp(log.safetyTrace.tiltRadians / (.pi / 2.0))
        let normalizedOmega = clamp(log.safetyTrace.omegaMagnitude / 20.0)
        let controlMagnitude = meanControlMagnitude(log: log)
        var value = config.aliveReward
            - (config.tiltPenalty * normalizedTilt)
            - (config.omegaPenalty * normalizedOmega)
            - (config.controlPenalty * controlMagnitude)

        let altitudeReference = try ReferenceQuadrotorAltitudeHoldReference(definition: scenario)
        // Dense vertical-velocity penalty for all tasks. Scenario semantics own
        // the reference velocity so reward, observation, and tensor worlds agree.
        let normalizedVerticalVelocity = clamp(
            abs(log.plantState.root.velocity.z) / max(altitudeReference.referenceVerticalVelocity, 1e-6)
        )
        value -= config.verticalVelocityPenalty * normalizedVerticalVelocity

        // Dense altitude-position penalty for all tasks. Lift/single-lift use their
        // explicit lift envelope; attitude has no lift envelope, so the scenario's
        // initial altitude is the hover target. Without this P-like position term,
        // attitude PPO only sees a D-like |vz| penalty and can settle into slow
        // descent with little immediate reward pressure to climb back.
        let altitudeError = abs(log.plantState.root.position.z - altitudeReference.targetPosition.z)
        let normalizedAltitudeError = clamp(altitudeError / max(altitudeReference.tolerance, 1e-6))
        value -= config.altitudePenalty * normalizedAltitudeError

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
            config.aliveReward,
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
