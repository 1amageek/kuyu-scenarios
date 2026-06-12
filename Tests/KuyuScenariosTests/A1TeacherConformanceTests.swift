import Testing
import KuyuCore
import KuyuPhysics
@testable import KuyuScenarios

// The privileged altitude-hold teacher is the reference baseline for A1
// conformance, so it must clear every stress suite under the strict
// sustained-fall envelope (|vz| >= 0.05 m/s for 0.5 s fails the run).
// Suite-2 and Suite-4 are the binding cases: a single-motor gain deficit
// (~0.49 N sustained on the 1 kg plant) and an all-channel sensor shock
// that costs cos(tilt) thrust. Suite-5 combines both mechanisms.
@Test(
    .timeLimit(.minutes(5)),
    arguments: [
        A1ConformanceSuite.Level.actuatorSwappability,
        A1ConformanceSuite.Level.bundleGatingStress,
        A1ConformanceSuite.Level.combined,
    ]
)
func activeTeacherPassesA1StressSuite(level: A1ConformanceSuite.Level) async throws {
    let definitions = try A1ConformanceSuite(level: level, seeds: [2001]).scenarios()
    let gains = try ImuRateDampingCutGains(kp: 2.0, kd: 0.25, yawDamping: 0.2)
    let runner = try KuyAtt1Runner.activeAltitudeHoldTeacher(gains: gains)

    let output = try await runner.runWithLogs(definitions: definitions)

    #expect(output.summary.suitePassed)
    for evaluation in output.summary.evaluations {
        #expect(
            evaluation.passed,
            "\(evaluation.scenarioId.rawValue) failed: \(evaluation.failures.joined(separator: ", "))"
        )
    }
    #expect(output.result.replay.notPerformedReason == nil)
    #expect(output.result.replay.checks.count == definitions.count)
    #expect(output.result.replay.checks.allSatisfy { $0.passed })
}
