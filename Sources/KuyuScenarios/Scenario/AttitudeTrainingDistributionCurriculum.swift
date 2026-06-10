import Foundation
import KuyuCore
import KuyuPhysics

/// Staged training-distribution expansion for the attitude task.
///
/// The fixed KUY-ATT-1 training suite starts every episode at 0-10 degrees of
/// initial tilt, while the M2 long-horizon benchmark samples onsets up to 30
/// degrees combined with stress events. Policies trained only on the fixed
/// suite die within the first second of the benchmark tracks because the onset
/// distribution never appears in training. This curriculum closes that gap:
/// each stage generates parametric scenarios whose initial tilt and stress
/// magnitudes expand toward the configured upper bounds, so training rollouts
/// progressively cover the benchmark onset distribution.
///
/// Stage selection is a pure function of a progress signal in [0, 1] (for
/// example mastered-horizon ratio), and scenario generation is deterministic
/// per stage, so two runs with the same configuration and progress sequence
/// see identical curriculum scenarios.
public struct AttitudeTrainingDistributionCurriculum: Sendable, Equatable {
    public enum ConfigurationError: Error, Equatable {
        case invalidStageCount(Int)
        case invalidScenariosPerStage(Int)
        case invalidTiltBounds(lowerDegrees: Double, upperDegrees: Double)
        case invalidTorqueMagnitudeUpperBound(Double)
        case invalidEpisodeDuration(Double)
    }

    public enum StageError: Error, Equatable {
        case stageOutOfRange(stage: Int, stageCount: Int)
        case nonFiniteProgress
    }

    public struct Config: Sendable, Codable, Equatable {
        public let stageCount: Int
        public let scenariosPerStage: Int
        public let baseSeed: UInt64
        public let tiltLowerBoundDegrees: Double
        public let tiltUpperBoundDegrees: Double
        public let torqueMagnitudeUpperBound: Double
        public let episodeDuration: Double

        /// - Parameters:
        ///   - tiltLowerBoundDegrees: Easy bound of the initial-tilt range. The
        ///     default starts where the fixed training suite ends (10 degrees)
        ///     so the curriculum spends its budget on onsets the base suite
        ///     never produces.
        ///   - tiltUpperBoundDegrees: Hard bound reached by the final stage.
        ///     The default overshoots the benchmark's 30-degree onset so the
        ///     trained distribution contains the evaluated one.
        ///   - episodeDuration: Fixed scenario duration in seconds. Pinned (not
        ///     a range) so curriculum scenarios share the base suite's horizon
        ///     semantics instead of perturbing the full-horizon step count.
        public init(
            stageCount: Int = 3,
            scenariosPerStage: Int = 5,
            baseSeed: UInt64 = 41_000,
            tiltLowerBoundDegrees: Double = 10,
            tiltUpperBoundDegrees: Double = 35,
            torqueMagnitudeUpperBound: Double = 0.8,
            episodeDuration: Double = 20.0
        ) throws {
            guard stageCount > 0 else {
                throw ConfigurationError.invalidStageCount(stageCount)
            }
            guard scenariosPerStage > 0 else {
                throw ConfigurationError.invalidScenariosPerStage(scenariosPerStage)
            }
            guard tiltLowerBoundDegrees.isFinite,
                  tiltUpperBoundDegrees.isFinite,
                  tiltLowerBoundDegrees >= 0,
                  tiltLowerBoundDegrees <= tiltUpperBoundDegrees,
                  tiltUpperBoundDegrees < 90 else {
                throw ConfigurationError.invalidTiltBounds(
                    lowerDegrees: tiltLowerBoundDegrees,
                    upperDegrees: tiltUpperBoundDegrees
                )
            }
            guard torqueMagnitudeUpperBound.isFinite, torqueMagnitudeUpperBound >= 0 else {
                throw ConfigurationError.invalidTorqueMagnitudeUpperBound(torqueMagnitudeUpperBound)
            }
            guard episodeDuration.isFinite, episodeDuration > 0 else {
                throw ConfigurationError.invalidEpisodeDuration(episodeDuration)
            }
            self.stageCount = stageCount
            self.scenariosPerStage = scenariosPerStage
            self.baseSeed = baseSeed
            self.tiltLowerBoundDegrees = tiltLowerBoundDegrees
            self.tiltUpperBoundDegrees = tiltUpperBoundDegrees
            self.torqueMagnitudeUpperBound = torqueMagnitudeUpperBound
            self.episodeDuration = episodeDuration
        }
    }

    public let config: Config

    public init(config: Config) {
        self.config = config
    }

    /// Maps a progress signal in [0, 1] to a stage index. Finite values outside
    /// [0, 1] clamp to the nearest stage (progress below the curriculum start
    /// selects stage 0, progress past mastery stays on the final stage); a
    /// non-finite signal is a caller bug and is rejected explicitly.
    public func stage(forProgress progress: Double) throws -> Int {
        guard progress.isFinite else {
            throw StageError.nonFiniteProgress
        }
        let clamped = min(max(progress, 0), 1)
        return min(config.stageCount - 1, Int(clamped * Double(config.stageCount)))
    }

    /// Deterministic scenario definitions for one stage. Scenario IDs use the
    /// generator's `GEN-<seed>/SCN-<index>` namespace, so they never collide
    /// with the fixed `KUY-ATT-1/...` suite.
    public func definitions(stage: Int) throws -> [ReferenceQuadrotorScenarioDefinition] {
        guard stage >= 0, stage < config.stageCount else {
            throw StageError.stageOutOfRange(stage: stage, stageCount: config.stageCount)
        }
        return try makeGenerator().generateCurriculum(
            levels: config.stageCount,
            scenariosPerLevel: config.scenariosPerStage,
            baseSeed: config.baseSeed
        )[stage]
    }

    /// All stages flattened, easiest first. Intended for building lookup tables
    /// (scenario key -> definition) that must cover every stage a run can reach.
    public func allDefinitions() throws -> [ReferenceQuadrotorScenarioDefinition] {
        try makeGenerator().generateCurriculum(
            levels: config.stageCount,
            scenariosPerLevel: config.scenariosPerStage,
            baseSeed: config.baseSeed
        ).flatMap { $0 }
    }

    /// The initial-tilt upper bound a stage samples up to, in degrees. Derived
    /// from the generator's scaling rule so reported bounds always match the
    /// generated scenarios.
    public func tiltUpperBoundDegrees(stage: Int) throws -> Double {
        guard stage >= 0, stage < config.stageCount else {
            throw StageError.stageOutOfRange(stage: stage, stageCount: config.stageCount)
        }
        let fraction = Double(stage + 1) / Double(config.stageCount)
        return makeGenerator().parameterSpace(scaledBy: fraction).tiltRange.upperBound
    }

    // MARK: - Private

    private func makeGenerator() -> ParametricScenarioGenerator {
        ParametricScenarioGenerator(parameterSpace: ParametricScenarioGenerator.ParameterSpace(
            tiltRange: config.tiltLowerBoundDegrees...config.tiltUpperBoundDegrees,
            torqueMagnitudeRange: 0...config.torqueMagnitudeUpperBound,
            durationRange: config.episodeDuration...config.episodeDuration
        ))
    }
}
