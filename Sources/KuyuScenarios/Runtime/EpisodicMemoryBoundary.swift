public struct EpisodicMemoryKey: Sendable, Codable, Equatable, Hashable {
    public let task: String
    public let morphology: String
    public let scenarioId: String
    public let seed: UInt64

    public init(task: String, morphology: String, scenarioId: String, seed: UInt64) {
        self.task = task
        self.morphology = morphology
        self.scenarioId = scenarioId
        self.seed = seed
    }
}

public struct EpisodicMemoryRecord: Sendable, Codable, Equatable {
    public let key: EpisodicMemoryKey
    public let descendingContext: [Double]
    public let failureReason: String?
    public let recoveryTime: Double?

    public init(
        key: EpisodicMemoryKey,
        descendingContext: [Double],
        failureReason: String? = nil,
        recoveryTime: Double? = nil
    ) {
        self.key = key
        self.descendingContext = descendingContext
        self.failureReason = failureReason
        self.recoveryTime = recoveryTime
    }
}

public struct EpisodicMemoryStore: Sendable {
    private var records: [EpisodicMemoryRecord]

    public init(records: [EpisodicMemoryRecord] = []) {
        self.records = records
    }

    public mutating func append(_ record: EpisodicMemoryRecord) {
        records.append(record)
    }

    public func query(
        task: String? = nil,
        morphology: String? = nil,
        scenarioId: String? = nil,
        seed: UInt64? = nil
    ) -> [EpisodicMemoryRecord] {
        records.filter { record in
            if let task, record.key.task != task { return false }
            if let morphology, record.key.morphology != morphology { return false }
            if let scenarioId, record.key.scenarioId != scenarioId { return false }
            if let seed, record.key.seed != seed { return false }
            return true
        }
    }
}

public enum EpisodicMemoryBoundary {
    /// Projects recalled memory into descending/context channels only.
    /// This function intentionally does not emit actuator commands.
    public static func descendingVector(
        from records: [EpisodicMemoryRecord],
        channelCount: Int
    ) -> [Double] {
        guard channelCount > 0 else { return [] }
        guard let latest = records.last else {
            return [Double](repeating: 0.0, count: channelCount)
        }

        var projected = latest.descendingContext.prefix(channelCount).map { value -> Double in
            guard value.isFinite else { return 0.0 }
            return value
        }
        if projected.count < channelCount {
            projected.append(contentsOf: [Double](repeating: 0.0, count: channelCount - projected.count))
        }
        return projected
    }
}
