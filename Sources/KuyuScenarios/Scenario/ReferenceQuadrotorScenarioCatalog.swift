import KuyuCore
import KuyuPhysics

/// Canonical source of truth for reference-quadrotor scenario selection by task
/// mode and suite identifier. Training and backend packages may request
/// scenarios through this catalog, but suite semantics live here.
public enum ReferenceQuadrotorScenarioCatalog {
    public enum ResolutionError: Error, Sendable, Equatable {
        case invalidEpisodeCount(Int)
        case unsupportedSuite(task: String, suite: Int)
    }

    public static func scenarios(
        for task: SimulationTaskMode,
        suite: Int,
        episodeCount: Int
    ) throws -> [ReferenceQuadrotorScenarioDefinition] {
        guard episodeCount > 0 else {
            throw ResolutionError.invalidEpisodeCount(episodeCount)
        }
        switch task {
        case .lift, .singleLift:
            return try liftSuiteScenarios(for: task, suite: suite, episodeCount: episodeCount)
        case .attitude:
            if let level = A1ConformanceSuite.Level(rawValue: suite) {
                return try A1ConformanceSuite(
                    level: level,
                    seeds: a1ConformanceSeeds(suite: suite, episodeCount: episodeCount)
                ).scenarios()
            }
            return try longHorizonScenarios(suite: suite, episodeCount: episodeCount)
        }
    }

    public static func trackName(forSuite suite: Int) -> String {
        if let level = A1ConformanceSuite.Level(rawValue: suite) {
            return level.suiteID
        }
        switch suite {
        case 6: return LongHorizonBenchmarkTrack.longHorizonTask.rawValue
        case 7: return LongHorizonBenchmarkTrack.morphologyTransfer.rawValue
        case 8: return LongHorizonBenchmarkTrack.disturbanceDelayPartialObservability.rawValue
        default: return "unknown"
        }
    }

    public static func liftSuiteVariation(
        of definition: ReferenceQuadrotorScenarioDefinition,
        task: SimulationTaskMode,
        suite: Int,
        index: Int
    ) throws -> ReferenceQuadrotorScenarioDefinition {
        guard let liftEnvelope = definition.liftEnvelope else {
            return definition
        }
        let targetOffset: Double
        let initialOffset: Double
        let actuatorDegradation: ActuatorDegradation?
        let torqueEvents: [TorqueDisturbanceEvent]
        let hfEvents: [HFStressEvent]
        switch suite {
        case 6:
            targetOffset = 0
            initialOffset = 0
            actuatorDegradation = definition.actuatorDegradation
            torqueEvents = definition.torqueEvents
            hfEvents = definition.hfEvents
        case 7:
            targetOffset = task == .singleLift ? 0.02 : 0.05
            initialOffset = 0
            actuatorDegradation = definition.actuatorDegradation
            torqueEvents = definition.torqueEvents
            hfEvents = definition.hfEvents
        case 8:
            targetOffset = task == .singleLift ? -0.01 : -0.02
            initialOffset = 0
            actuatorDegradation = definition.actuatorDegradation
            if task == .singleLift {
                torqueEvents = definition.torqueEvents
            } else {
                torqueEvents = definition.torqueEvents + [
                    try TorqueDisturbanceEvent(
                        startTime: max(0.75, definition.config.duration * 0.35),
                        duration: 0.05,
                        torqueBody: Axis3(x: 0.0005, y: 0, z: 0)
                    ),
                ]
            }
            hfEvents = definition.hfEvents + [
                try HFStressEvent(
                    kind: .latencySpike,
                    startTime: max(1.0, definition.config.duration * 0.50),
                    duration: 0.01,
                    magnitude: 0.01
                ),
            ]
        default:
            throw ResolutionError.unsupportedSuite(task: task.rawValue, suite: suite)
        }
        let targetZ = max(0.05, liftEnvelope.targetZ + targetOffset)
        let adjustedLiftEnvelope = LiftEnvelope(
            targetZ: targetZ,
            tolerance: liftEnvelope.tolerance,
            maxVelocity: liftEnvelope.maxVelocity,
            warmupTime: liftEnvelope.warmupTime,
            requiredHoldTime: liftEnvelope.requiredHoldTime
        )
        let prefix: String = switch task {
        case .attitude: "KUY-ATT-M2-S\(suite)"
        case .lift: "KUY-LIFT-M2-S\(suite)"
        case .singleLift: "KUY-SLIFT-M2-S\(suite)"
        }
        return ReferenceQuadrotorScenarioDefinition(
            config: try ScenarioConfig(
                id: try ScenarioID("\(prefix)/SCN-\(index + 1)"),
                seed: ScenarioSeed(definition.config.seed.rawValue &+ UInt64(suite * 10_000 + index)),
                duration: definition.config.duration,
                timeStep: definition.config.timeStep
            ),
            kind: definition.kind,
            initialPosition: Axis3(
                x: definition.initialPosition.x,
                y: definition.initialPosition.y,
                z: max(0.05, targetZ + initialOffset)
            ),
            initialAttitude: definition.initialAttitude,
            initialAngularVelocity: definition.initialAngularVelocity,
            safetyEnvelope: definition.safetyEnvelope,
            liftEnvelope: adjustedLiftEnvelope,
            torqueEvents: torqueEvents,
            actuatorDegradation: actuatorDegradation,
            gyroDriftScale: suite == 8 ? max(definition.gyroDriftScale, 1.5) : definition.gyroDriftScale,
            swapEvents: definition.swapEvents,
            hfEvents: hfEvents
        )
    }

    private static func a1ConformanceSeeds(suite: Int, episodeCount: Int) -> [UInt64] {
        (0..<episodeCount).map { UInt64(40_000 + suite * 1_000 + $0) }
    }

    private static func longHorizonScenarios(
        suite: Int,
        episodeCount: Int
    ) throws -> [ReferenceQuadrotorScenarioDefinition] {
        let track: LongHorizonBenchmarkTrack
        switch suite {
        case 6: track = .longHorizonTask
        case 7: track = .morphologyTransfer
        case 8: track = .disturbanceDelayPartialObservability
        default: throw ResolutionError.unsupportedSuite(task: SimulationTaskMode.attitude.rawValue, suite: suite)
        }
        let benchmark = try LongHorizonBenchmarkSuite.makeDefault(
            scenariosPerTrack: max(episodeCount, 1),
            baseSeed: 60_000
        )
        return benchmark.cases.filter { $0.track == track }.map(\.definition)
    }

    private static func liftSuiteScenarios(
        for task: SimulationTaskMode,
        suite: Int,
        episodeCount: Int
    ) throws -> [ReferenceQuadrotorScenarioDefinition] {
        let baseDefinitions = try baseScenarios(for: task)
        return try (0..<episodeCount).map { index in
            try liftSuiteVariation(
                of: baseDefinitions[index % baseDefinitions.count],
                task: task,
                suite: suite,
                index: index
            )
        }
    }

    private static func baseScenarios(
        for task: SimulationTaskMode
    ) throws -> [ReferenceQuadrotorScenarioDefinition] {
        switch task {
        case .attitude: try KuyAtt1Suite().scenarios()
        case .lift: try KuyLiftSuite().scenarios()
        case .singleLift: try KuySingleLiftSuite().scenarios()
        }
    }
}
