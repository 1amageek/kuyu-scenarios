import KuyuCore
import KuyuPhysics

public struct ScenarioTerminalFacts: Sendable, Codable, Equatable {
    public static let completedTerminalReason = "scenario-complete"
    public static let timeLimitTerminalReason = "time-limit"

    public enum ValidationError: Error, Sendable, Equatable {
        case missingTerminalState
        case conflictingTerminalState
        case missingTerminalReason
        case failureRequiresDone(reason: FailureReason)
        case failureReasonMismatch(expected: String, actual: String?)
        case missingFailureTime(reason: FailureReason)
        case nonFiniteFailureTime(Double)
        case failureTimeWithoutFailure(Double)
    }

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

    public func validate() throws {
        guard done || truncated else {
            throw ValidationError.missingTerminalState
        }
        guard !(done && truncated) else {
            throw ValidationError.conflictingTerminalState
        }
        guard let terminalReason, !terminalReason.isEmpty else {
            throw ValidationError.missingTerminalReason
        }

        if let failureReason {
            guard done else {
                throw ValidationError.failureRequiresDone(reason: failureReason)
            }
            guard terminalReason == failureReason.rawValue else {
                throw ValidationError.failureReasonMismatch(
                    expected: failureReason.rawValue,
                    actual: terminalReason
                )
            }
            guard let failureTime else {
                throw ValidationError.missingFailureTime(reason: failureReason)
            }
            guard failureTime.isFinite else {
                throw ValidationError.nonFiniteFailureTime(failureTime)
            }
        } else if let failureTime {
            throw ValidationError.failureTimeWithoutFailure(failureTime)
        }
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
