import KuyuCore

public struct EnvironmentActionCut: CutInterface, Sendable {
    public var action: EnvironmentAction

    public init(action: EnvironmentAction = .driveIntents([], corrections: [])) {
        self.action = action
    }

    public mutating func update(samples: [ChannelSample], time: WorldTime) throws -> CutOutput {
        action.cutOutput()
    }
}
