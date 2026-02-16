import KuyuCore
import KuyuPhysics

public enum LongHorizonBenchmarkTrack: String, Sendable, Codable, Equatable, CaseIterable {
    case longHorizonTask
    case morphologyTransfer
    case disturbanceDelayPartialObservability
}

public struct LongHorizonBenchmarkCase: Sendable, Codable, Equatable {
    public let track: LongHorizonBenchmarkTrack
    public let definition: ReferenceQuadrotorScenarioDefinition

    public init(track: LongHorizonBenchmarkTrack, definition: ReferenceQuadrotorScenarioDefinition) {
        self.track = track
        self.definition = definition
    }
}

public struct LongHorizonBenchmarkSuite: Sendable, Codable, Equatable {
    public let cases: [LongHorizonBenchmarkCase]

    public init(cases: [LongHorizonBenchmarkCase]) {
        self.cases = cases
    }

    public static func makeDefault(
        scenariosPerTrack: Int,
        baseSeed: UInt64
    ) throws -> LongHorizonBenchmarkSuite {
        let taskGenerator = ParametricScenarioGenerator(
            parameterSpace: .init(
                durationRange: 60.0...120.0
            )
        )
        let transferGenerator = ParametricScenarioGenerator(
            parameterSpace: .init(
                tiltRange: 0...20,
                durationRange: 30.0...90.0
            )
        )
        let disturbanceGenerator = ParametricScenarioGenerator(
            parameterSpace: .init(
                torqueMagnitudeRange: 0.2...0.8,
                torqueDurationRange: 0.05...2.0,
                durationRange: 45.0...90.0
            )
        )

        let task = try taskGenerator.generate(count: scenariosPerTrack, baseSeed: baseSeed)
        let transfer = try transferGenerator.generate(count: scenariosPerTrack, baseSeed: baseSeed &+ 10_000)
        let disturbance = try disturbanceGenerator.generate(count: scenariosPerTrack, baseSeed: baseSeed &+ 20_000)

        let taskCases = try task.enumerated().map { index, definition in
            try LongHorizonBenchmarkCase(
                track: .longHorizonTask,
                definition: retag(definition, prefix: "LH-TASK", index: index)
            )
        }
        let transferCases = try transfer.enumerated().map { index, definition in
            try LongHorizonBenchmarkCase(
                track: .morphologyTransfer,
                definition: retag(definition, prefix: "LH-TRANSFER", index: index)
            )
        }
        let disturbanceCases = try disturbance.enumerated().map { index, definition in
            var hfEvents = definition.hfEvents
            hfEvents.append(
                try HFStressEvent(
                    kind: .latencySpike,
                    startTime: max(0.0, definition.config.duration * 0.5),
                    duration: 0.1,
                    magnitude: 1.0
                )
            )
            let disturbanceDefinition = ReferenceQuadrotorScenarioDefinition(
                config: try ScenarioConfig(
                    id: try ScenarioID("LH-DISTURB-\(index)"),
                    seed: definition.config.seed,
                    duration: definition.config.duration,
                    timeStep: definition.config.timeStep
                ),
                kind: definition.kind,
                initialPosition: definition.initialPosition,
                initialAttitude: definition.initialAttitude,
                initialAngularVelocity: definition.initialAngularVelocity,
                safetyEnvelope: definition.safetyEnvelope,
                liftEnvelope: definition.liftEnvelope,
                torqueEvents: definition.torqueEvents,
                actuatorDegradation: definition.actuatorDegradation,
                gyroDriftScale: definition.gyroDriftScale,
                swapEvents: definition.swapEvents,
                hfEvents: hfEvents
            )
            return LongHorizonBenchmarkCase(
                track: .disturbanceDelayPartialObservability,
                definition: disturbanceDefinition
            )
        }

        return LongHorizonBenchmarkSuite(
            cases: taskCases + transferCases + disturbanceCases
        )
    }

    private static func retag(
        _ definition: ReferenceQuadrotorScenarioDefinition,
        prefix: String,
        index: Int
    ) throws -> ReferenceQuadrotorScenarioDefinition {
        ReferenceQuadrotorScenarioDefinition(
            config: try ScenarioConfig(
                id: try ScenarioID("\(prefix)-\(index)"),
                seed: definition.config.seed,
                duration: definition.config.duration,
                timeStep: definition.config.timeStep
            ),
            kind: definition.kind,
            initialPosition: definition.initialPosition,
            initialAttitude: definition.initialAttitude,
            initialAngularVelocity: definition.initialAngularVelocity,
            safetyEnvelope: definition.safetyEnvelope,
            liftEnvelope: definition.liftEnvelope,
            torqueEvents: definition.torqueEvents,
            actuatorDegradation: definition.actuatorDegradation,
            gyroDriftScale: definition.gyroDriftScale,
            swapEvents: definition.swapEvents,
            hfEvents: definition.hfEvents
        )
    }
}

public struct DeterministicReplayBundle: Sendable, Codable, Equatable {
    public struct Entry: Sendable, Codable, Equatable {
        public let scenarioId: String
        public let seed: UInt64
        public let configHash: String
        public let eventCount: Int
        public let failureReason: String?

        public init(
            scenarioId: String,
            seed: UInt64,
            configHash: String,
            eventCount: Int,
            failureReason: String?
        ) {
            self.scenarioId = scenarioId
            self.seed = seed
            self.configHash = configHash
            self.eventCount = eventCount
            self.failureReason = failureReason
        }
    }

    public let suiteVersion: String
    public let entries: [Entry]

    public init(suiteVersion: String, entries: [Entry]) {
        self.suiteVersion = suiteVersion
        self.entries = entries
    }

    public static func fromLogs(
        suiteVersion: String,
        logs: [ScenarioLogEntry]
    ) -> DeterministicReplayBundle {
        let entries = logs.map { entry in
            Entry(
                scenarioId: entry.key.scenarioId.rawValue,
                seed: entry.key.seed.rawValue,
                configHash: entry.log.configHash,
                eventCount: entry.log.events.count,
                failureReason: entry.log.failureReason?.rawValue
            )
        }
        return DeterministicReplayBundle(suiteVersion: suiteVersion, entries: entries)
    }
}

public struct LongHorizonTaskOutcome: Sendable, Codable, Equatable {
    public let scenarioId: String
    public let seed: UInt64
    public let completed: Bool
    public let completionTimeSeconds: Double?

    public init(
        scenarioId: String,
        seed: UInt64,
        completed: Bool,
        completionTimeSeconds: Double?
    ) {
        self.scenarioId = scenarioId
        self.seed = seed
        self.completed = completed
        self.completionTimeSeconds = completionTimeSeconds
    }
}

public struct LongHorizonControlMetrics: Sendable, Codable, Equatable {
    public let passRate: Double
    public let averageMaxTiltDegrees: Double
    public let averageRecoveryTimeSeconds: Double?

    public init(
        passRate: Double,
        averageMaxTiltDegrees: Double,
        averageRecoveryTimeSeconds: Double?
    ) {
        self.passRate = passRate
        self.averageMaxTiltDegrees = averageMaxTiltDegrees
        self.averageRecoveryTimeSeconds = averageRecoveryTimeSeconds
    }
}

public struct LongHorizonTaskMetrics: Sendable, Codable, Equatable {
    public let completionRate: Double
    public let averageCompletionTimeSeconds: Double?
    public let incompleteRate: Double

    public init(
        completionRate: Double,
        averageCompletionTimeSeconds: Double?,
        incompleteRate: Double
    ) {
        self.completionRate = completionRate
        self.averageCompletionTimeSeconds = averageCompletionTimeSeconds
        self.incompleteRate = incompleteRate
    }
}

public struct LongHorizonBenchmarkResultEntry: Sendable, Codable, Equatable {
    public let scenarioId: String
    public let seed: UInt64
    public let passed: Bool
    public let maxTiltDegrees: Double
    public let recoveryTimeSeconds: Double?
    public let taskCompleted: Bool
    public let taskCompletionTimeSeconds: Double?

    public init(
        scenarioId: String,
        seed: UInt64,
        passed: Bool,
        maxTiltDegrees: Double,
        recoveryTimeSeconds: Double?,
        taskCompleted: Bool,
        taskCompletionTimeSeconds: Double?
    ) {
        self.scenarioId = scenarioId
        self.seed = seed
        self.passed = passed
        self.maxTiltDegrees = maxTiltDegrees
        self.recoveryTimeSeconds = recoveryTimeSeconds
        self.taskCompleted = taskCompleted
        self.taskCompletionTimeSeconds = taskCompletionTimeSeconds
    }
}

public struct LongHorizonBenchmarkReport: Sendable, Codable, Equatable {
    public let track: LongHorizonBenchmarkTrack
    public let control: LongHorizonControlMetrics
    public let task: LongHorizonTaskMetrics
    public let entries: [LongHorizonBenchmarkResultEntry]

    public init(
        track: LongHorizonBenchmarkTrack,
        control: LongHorizonControlMetrics,
        task: LongHorizonTaskMetrics,
        entries: [LongHorizonBenchmarkResultEntry]
    ) {
        self.track = track
        self.control = control
        self.task = task
        self.entries = entries
    }

    public static func fromEvaluations(
        track: LongHorizonBenchmarkTrack,
        evaluations: [ScenarioEvaluation],
        taskOutcomes: [LongHorizonTaskOutcome]
    ) -> LongHorizonBenchmarkReport {
        let totalCount = evaluations.count
        if totalCount == 0 {
            return LongHorizonBenchmarkReport(
                track: track,
                control: LongHorizonControlMetrics(
                    passRate: 0.0,
                    averageMaxTiltDegrees: 0.0,
                    averageRecoveryTimeSeconds: nil
                ),
                task: LongHorizonTaskMetrics(
                    completionRate: 0.0,
                    averageCompletionTimeSeconds: nil,
                    incompleteRate: 0.0
                ),
                entries: []
            )
        }

        var outcomeByKey: [String: LongHorizonTaskOutcome] = [:]
        for outcome in taskOutcomes {
            outcomeByKey["\(outcome.scenarioId)#\(outcome.seed)"] = outcome
        }

        let entries = evaluations.map { evaluation in
            let key = "\(evaluation.scenarioId.rawValue)#\(evaluation.seed.rawValue)"
            let outcome = outcomeByKey[key]
            let completed = outcome?.completed ?? evaluation.passed
            let completionTime = outcome?.completionTimeSeconds ?? evaluation.recoveryTimeSeconds
            return LongHorizonBenchmarkResultEntry(
                scenarioId: evaluation.scenarioId.rawValue,
                seed: evaluation.seed.rawValue,
                passed: evaluation.passed,
                maxTiltDegrees: evaluation.maxTiltDegrees,
                recoveryTimeSeconds: evaluation.recoveryTimeSeconds,
                taskCompleted: completed,
                taskCompletionTimeSeconds: completionTime
            )
        }

        let passCount = entries.filter(\.passed).count
        let completedCount = entries.filter(\.taskCompleted).count
        let averageTilt = entries.map(\.maxTiltDegrees).reduce(0.0, +) / Double(entries.count)
        let averageRecovery = average(entries.compactMap(\.recoveryTimeSeconds))
        let completionTimes = entries.compactMap { entry -> Double? in
            guard entry.taskCompleted else { return nil }
            return entry.taskCompletionTimeSeconds
        }

        return LongHorizonBenchmarkReport(
            track: track,
            control: LongHorizonControlMetrics(
                passRate: Double(passCount) / Double(entries.count),
                averageMaxTiltDegrees: averageTilt,
                averageRecoveryTimeSeconds: averageRecovery
            ),
            task: LongHorizonTaskMetrics(
                completionRate: Double(completedCount) / Double(entries.count),
                averageCompletionTimeSeconds: average(completionTimes),
                incompleteRate: Double(entries.count - completedCount) / Double(entries.count)
            ),
            entries: entries
        )
    }

    private static func average(_ values: [Double]) -> Double? {
        guard !values.isEmpty else { return nil }
        return values.reduce(0.0, +) / Double(values.count)
    }
}
