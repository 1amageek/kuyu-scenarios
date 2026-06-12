import KuyuCore
import KuyuPhysics

public struct ScenarioLogBundle: Sendable, Codable, Equatable {
    public let summary: ValidationSummary
    public let manifest: [ReferenceQuadrotorScenarioManifest]
    public let evaluations: [ScenarioEvaluation]
    public let replay: ReplayVerification
    public let logs: [ScenarioLogIndex]

    public init(
        summary: ValidationSummary,
        manifest: [ReferenceQuadrotorScenarioManifest],
        evaluations: [ScenarioEvaluation],
        replay: ReplayVerification,
        logs: [ScenarioLogIndex]
    ) {
        self.summary = summary
        self.manifest = manifest
        self.evaluations = evaluations
        self.replay = replay
        self.logs = logs
    }
}
