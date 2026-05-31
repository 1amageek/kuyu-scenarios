import KuyuCore
import KuyuPhysics

public struct ScenarioTerminalFacts: Sendable, Codable, Equatable {
    public static let completedTerminalReason = "scenario-complete"

    public let done: Bool
    public let truncated: Bool
    public let terminalReason: String?
    public let failureReason: FailureReason?
    public let failureTime: Double?

    public init(
        done: Bool,
        truncated: Bool,
        terminalReason: String?,
        failureReason: FailureReason?,
        failureTime: Double?
    ) {
        self.done = done
        self.truncated = truncated
        self.terminalReason = terminalReason
        self.failureReason = failureReason
        self.failureTime = failureTime
    }

    public init(log: SimulationLog) {
        self = Self.derive(failureReason: log.failureReason, failureTime: log.failureTime)
    }

    public init(evaluation: ScenarioEvaluation) {
        self = Self.derive(failureReason: evaluation.failureReason, failureTime: evaluation.failureTime)
    }

    private static func derive(failureReason: FailureReason?, failureTime: Double?) -> ScenarioTerminalFacts {
        if let failureReason {
            return ScenarioTerminalFacts(
                done: true,
                truncated: false,
                terminalReason: failureReason.rawValue,
                failureReason: failureReason,
                failureTime: failureTime
            )
        }
        return ScenarioTerminalFacts(
            done: false,
            truncated: true,
            terminalReason: completedTerminalReason,
            failureReason: nil,
            failureTime: nil
        )
    }
}
