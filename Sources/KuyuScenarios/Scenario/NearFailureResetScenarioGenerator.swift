import Foundation
import KuyuCore
import KuyuPhysics

/// Generates reset-to-state curriculum scenarios whose initial attitude and
/// angular velocity lie close to the safety envelope.
///
/// Terminal-frontier replay only revisits failure neighborhoods the policy
/// happens to reach; this generator places episode starts there directly, so
/// recovery behavior is learned as a first-class objective. States are sampled
/// strictly inside the envelope (fractions < 1) so an episode never begins in
/// violation; difficulty is controlled by widening the sampled fraction ranges
/// toward the envelope across curriculum levels.
public struct NearFailureResetScenarioGenerator: Sendable {
    public enum GeneratorError: Error, Equatable {
        case invalidCount
        case sampledStateOutsideEnvelope(scenarioIndex: Int)
    }

    /// Fractions of the template's safety envelope from which initial states
    /// are sampled. Both ranges must stay below 1 so sampled states start
    /// inside the envelope rather than in immediate violation.
    public struct ResetStateSpace: Sendable, Codable, Equatable {
        public enum ValidationError: Error, Equatable {
            case nonFinite(String)
            case outOfBounds(String)
        }

        public let tiltFractionRange: ClosedRange<Double>
        public let omegaFractionRange: ClosedRange<Double>

        public init(
            tiltFractionRange: ClosedRange<Double> = 0.5...0.9,
            omegaFractionRange: ClosedRange<Double> = 0.2...0.7
        ) throws {
            guard tiltFractionRange.lowerBound.isFinite, tiltFractionRange.upperBound.isFinite else {
                throw ValidationError.nonFinite("tiltFractionRange")
            }
            guard omegaFractionRange.lowerBound.isFinite, omegaFractionRange.upperBound.isFinite else {
                throw ValidationError.nonFinite("omegaFractionRange")
            }
            guard tiltFractionRange.lowerBound >= 0, tiltFractionRange.upperBound < 1 else {
                throw ValidationError.outOfBounds("tiltFractionRange")
            }
            guard omegaFractionRange.lowerBound >= 0, omegaFractionRange.upperBound < 1 else {
                throw ValidationError.outOfBounds("omegaFractionRange")
            }
            self.tiltFractionRange = tiltFractionRange
            self.omegaFractionRange = omegaFractionRange
        }
    }

    public let template: ReferenceQuadrotorScenarioDefinition
    public let space: ResetStateSpace

    public init(
        template: ReferenceQuadrotorScenarioDefinition,
        space: ResetStateSpace
    ) {
        self.template = template
        self.space = space
    }

    /// Generate `count` scenarios with deterministic near-envelope initial states.
    public func generate(count: Int, baseSeed: UInt64) throws -> [ReferenceQuadrotorScenarioDefinition] {
        guard count > 0 else { throw GeneratorError.invalidCount }

        var rng = SplitMix64(seed: baseSeed)
        return try (0..<count).map { index in
            try generateOne(index: index, baseSeed: baseSeed, rng: &rng)
        }
    }

    /// Generate scenarios organized by difficulty level.
    ///
    /// Level 0 samples near the lower fraction bounds; higher levels widen the
    /// sampled ranges toward the configured upper bounds (closer to the envelope),
    /// mirroring `ParametricScenarioGenerator.generateCurriculum`.
    public func generateCurriculum(
        levels: Int,
        scenariosPerLevel: Int,
        baseSeed: UInt64
    ) throws -> [[ReferenceQuadrotorScenarioDefinition]] {
        guard levels > 0, scenariosPerLevel > 0 else { throw GeneratorError.invalidCount }

        return try (0..<levels).map { level in
            let fraction = Double(level + 1) / Double(levels)
            let levelSpace = try scaledSpace(fraction: fraction)
            let levelGenerator = NearFailureResetScenarioGenerator(template: template, space: levelSpace)
            let levelSeed = baseSeed &+ UInt64(level) &* 7919
            return try levelGenerator.generate(count: scenariosPerLevel, baseSeed: levelSeed)
        }
    }

    // MARK: - Private

    private func generateOne(
        index: Int,
        baseSeed: UInt64,
        rng: inout SplitMix64
    ) throws -> ReferenceQuadrotorScenarioDefinition {
        let envelope = template.safetyEnvelope
        let tiltLimitRadians = envelope.tiltSafeMaxDegrees * Double.pi / 180.0

        let tiltFraction = sample(range: space.tiltFractionRange, rng: &rng)
        let tiltRadians = tiltFraction * tiltLimitRadians
        let attitude = sampleAttitude(tiltRadians: tiltRadians, rng: &rng)

        let omegaFraction = sample(range: space.omegaFractionRange, rng: &rng)
        let omegaMagnitude = omegaFraction * envelope.omegaSafeMax
        let angularVelocity = sampleAngularVelocity(magnitude: omegaMagnitude, rng: &rng)

        // The construction above is exact, but the initial state is the one
        // contract this generator owns: starting in violation would convert the
        // curriculum into guaranteed sustained-failure episodes.
        let achievedTilt = acos(min(max(cos(attitude.roll) * cos(attitude.pitch), -1.0), 1.0))
        let achievedOmega = (
            angularVelocity.x * angularVelocity.x
                + angularVelocity.y * angularVelocity.y
                + angularVelocity.z * angularVelocity.z
        ).squareRoot()
        guard achievedTilt < tiltLimitRadians, achievedOmega < envelope.omegaSafeMax else {
            throw GeneratorError.sampledStateOutsideEnvelope(scenarioIndex: index)
        }

        let scenarioId = try ScenarioID("NFR-\(baseSeed)/SCN-\(index)")
        let config = try ScenarioConfig(
            id: scenarioId,
            seed: ScenarioSeed(baseSeed &+ UInt64(index) &* 31),
            duration: template.config.duration,
            timeStep: template.config.timeStep
        )

        return ReferenceQuadrotorScenarioDefinition(
            config: config,
            kind: template.kind,
            initialPosition: template.initialPosition,
            initialAttitude: attitude,
            initialAngularVelocity: angularVelocity,
            safetyEnvelope: envelope,
            liftEnvelope: template.liftEnvelope,
            torqueEvents: template.torqueEvents,
            actuatorDegradation: template.actuatorDegradation,
            gyroDriftScale: template.gyroDriftScale,
            swapEvents: template.swapEvents,
            hfEvents: template.hfEvents
        )
    }

    /// Build Euler angles whose composite tilt (angle between body z and world z)
    /// equals `tiltRadians` exactly, with a randomized roll/pitch split and signs.
    ///
    /// For the yaw-pitch-roll composition used by `EulerAngles.toQuaternion()`,
    /// tilt satisfies cos(tilt) = cos(roll) * cos(pitch) and is independent of yaw.
    /// Sampling roll = u * tilt and solving pitch from that identity keeps the
    /// tilt magnitude exact for any split u in [0, 1].
    private func sampleAttitude(tiltRadians: Double, rng: inout SplitMix64) -> EulerAngles {
        let split = rng.nextDouble()
        let rollMagnitude = tiltRadians * split
        let pitchCosine = min(max(cos(tiltRadians) / cos(rollMagnitude), -1.0), 1.0)
        let pitchMagnitude = acos(pitchCosine)
        let rollSign: Double = rng.nextDouble() < 0.5 ? -1 : 1
        let pitchSign: Double = rng.nextDouble() < 0.5 ? -1 : 1
        return EulerAngles(
            roll: rollSign * rollMagnitude,
            pitch: pitchSign * pitchMagnitude,
            yaw: 0
        )
    }

    /// Sample an angular velocity of the given magnitude with a direction drawn
    /// uniformly on the unit sphere.
    private func sampleAngularVelocity(magnitude: Double, rng: inout SplitMix64) -> Axis3 {
        let z = 2.0 * rng.nextDouble() - 1.0
        let azimuth = 2.0 * Double.pi * rng.nextDouble()
        let planar = max(0.0, 1.0 - z * z).squareRoot()
        return Axis3(
            x: magnitude * planar * cos(azimuth),
            y: magnitude * planar * sin(azimuth),
            z: magnitude * z
        )
    }

    private func scaledSpace(fraction: Double) throws -> ResetStateSpace {
        try ResetStateSpace(
            tiltFractionRange: scale(space.tiltFractionRange, fraction: fraction),
            omegaFractionRange: scale(space.omegaFractionRange, fraction: fraction)
        )
    }

    private func scale(_ range: ClosedRange<Double>, fraction: Double) -> ClosedRange<Double> {
        let width = range.upperBound - range.lowerBound
        let scaledUpper = range.lowerBound + width * fraction
        return range.lowerBound...max(range.lowerBound, scaledUpper)
    }

    private func sample(range: ClosedRange<Double>, rng: inout SplitMix64) -> Double {
        range.lowerBound + rng.nextDouble() * (range.upperBound - range.lowerBound)
    }
}
