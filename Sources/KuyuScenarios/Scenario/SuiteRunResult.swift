import KuyuCore
import KuyuPhysics

public struct SuiteRunResult: Sendable, Codable, Equatable {
    public let evaluations: [ScenarioEvaluation]
    public let replay: ReplayVerification
    public let passed: Bool

    public init(
        evaluations: [ScenarioEvaluation],
        replay: ReplayVerification,
        passed: Bool
    ) {
        self.evaluations = evaluations
        self.replay = replay
        self.passed = passed
    }
}
