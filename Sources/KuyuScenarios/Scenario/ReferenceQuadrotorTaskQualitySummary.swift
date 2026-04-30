import Foundation

public struct ReferenceQuadrotorTaskQualitySummary: Sendable, Codable, Equatable {
    public let task: String
    public let scenarioID: String
    public let seed: UInt64
    public let passed: Bool
    public let failureReasons: [String]
    public let evaluatorID: String
    public let targetZ: Double?
    public let tolerance: Double?
    public let warmupTime: Double?
    public let requiredHoldTime: Double?
    public let achievedHoldTime: Double?
    public let maxAltitudeErrorAfterWarmup: Double?
    public let maxVerticalVelocityAfterWarmup: Double?

    public init(
        task: String,
        scenarioID: String,
        seed: UInt64,
        passed: Bool,
        failureReasons: [String],
        evaluatorID: String,
        targetZ: Double?,
        tolerance: Double?,
        warmupTime: Double?,
        requiredHoldTime: Double?,
        achievedHoldTime: Double?,
        maxAltitudeErrorAfterWarmup: Double?,
        maxVerticalVelocityAfterWarmup: Double?
    ) {
        self.task = task
        self.scenarioID = scenarioID
        self.seed = seed
        self.passed = passed
        self.failureReasons = failureReasons
        self.evaluatorID = evaluatorID
        self.targetZ = targetZ
        self.tolerance = tolerance
        self.warmupTime = warmupTime
        self.requiredHoldTime = requiredHoldTime
        self.achievedHoldTime = achievedHoldTime
        self.maxAltitudeErrorAfterWarmup = maxAltitudeErrorAfterWarmup
        self.maxVerticalVelocityAfterWarmup = maxVerticalVelocityAfterWarmup
    }
}
