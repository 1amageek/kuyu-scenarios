import KuyuPhysics

/// Explicit record of whether deterministic replay verification ran for a
/// suite, so that "no replay checks" can never be mistaken for "all replay
/// checks passed". Paths that skip replay must state why.
public enum ReplayVerification: Sendable, Equatable {
    case performed([ReplayCheckResult])
    case notPerformed(reason: String)

    /// Replay check results when verification was performed; empty otherwise.
    public var checks: [ReplayCheckResult] {
        switch self {
        case .performed(let checks):
            return checks
        case .notPerformed:
            return []
        }
    }

    /// Reason replay verification was skipped, when it was not performed.
    public var notPerformedReason: String? {
        switch self {
        case .performed:
            return nil
        case .notPerformed(let reason):
            return reason
        }
    }

    /// True when verification was performed and at least one check failed.
    /// A verification that was not performed has no failing checks; its
    /// absence is recorded in `notPerformedReason` instead of being treated
    /// as evidence of correctness.
    public var hasFailures: Bool {
        switch self {
        case .performed(let checks):
            return checks.contains { !$0.passed }
        case .notPerformed:
            return false
        }
    }
}

extension ReplayVerification: Codable {
    private enum CodingKeys: String, CodingKey {
        case status
        case checks
        case reason
    }

    private enum Status: String, Codable {
        case performed
        case notPerformed
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        switch try container.decode(Status.self, forKey: .status) {
        case .performed:
            self = .performed(try container.decode([ReplayCheckResult].self, forKey: .checks))
        case .notPerformed:
            self = .notPerformed(reason: try container.decode(String.self, forKey: .reason))
        }
    }

    public func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: CodingKeys.self)
        switch self {
        case .performed(let checks):
            try container.encode(Status.performed, forKey: .status)
            try container.encode(checks, forKey: .checks)
        case .notPerformed(let reason):
            try container.encode(Status.notPerformed, forKey: .status)
            try container.encode(reason, forKey: .reason)
        }
    }
}
