import Foundation

extension StressSuiteManifest {
    static func validateReplay(
        _ replay: ReplayVerification,
        requirement: ReplayRequirement,
        records: [ScenarioRecord]
    ) throws -> ReplayEvidence {
        switch replay {
        case .performed(let checks):
            guard !checks.isEmpty else { throw ValidationError.replayChecksEmpty }
            for check in checks where !check.passed {
                throw ValidationError.replayCheckFailed("\(check.scenarioId.rawValue):\(check.seed.rawValue)")
            }
            let expected = Set(records.map(\.key))
            let actual = Set(checks.map { "\($0.scenarioId.rawValue):\($0.seed.rawValue)" })
            let missing = expected.subtracting(actual).sorted()
            if !missing.isEmpty {
                throw ValidationError.missingReplayChecks(missing)
            }
            let unexpected = actual.subtracting(expected).sorted()
            if !unexpected.isEmpty {
                throw ValidationError.unexpectedReplayChecks(unexpected)
            }
            return ReplayEvidence(
                status: .performed,
                checkCount: checks.count,
                skippedReason: nil,
                passed: true
            )
        case .notPerformed(let reason):
            let trimmed = reason.trimmingCharacters(in: .whitespacesAndNewlines)
            guard !trimmed.isEmpty else { throw ValidationError.replaySkipReasonEmpty }
            guard requirement == .explicitSkipAllowed else {
                throw ValidationError.replayNotPerformed(trimmed)
            }
            return ReplayEvidence(
                status: .notPerformed,
                checkCount: 0,
                skippedReason: trimmed,
                passed: true
            )
        }
    }

    static func validateDecodedReplayEvidence(
        _ evidence: ReplayEvidence,
        requirement: ReplayRequirement
    ) throws {
        guard evidence.passed else {
            throw ValidationError.decodedReplayEvidenceInvalid("replay-evidence-not-passed")
        }
        switch evidence.status {
        case .performed:
            guard evidence.checkCount > 0 else {
                throw ValidationError.decodedReplayEvidenceInvalid("performed-replay-without-checks")
            }
            if let reason = evidence.skippedReason,
               !reason.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                throw ValidationError.decodedReplayEvidenceInvalid("performed-replay-has-skip-reason")
            }
        case .notPerformed:
            guard requirement == .explicitSkipAllowed else {
                throw ValidationError.decodedReplayEvidenceInvalid("required-replay-not-performed")
            }
            let reason = evidence.skippedReason?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
            guard !reason.isEmpty else {
                throw ValidationError.replaySkipReasonEmpty
            }
            guard evidence.checkCount == 0 else {
                throw ValidationError.decodedReplayEvidenceInvalid("skipped-replay-has-checks")
            }
        }
    }
}
