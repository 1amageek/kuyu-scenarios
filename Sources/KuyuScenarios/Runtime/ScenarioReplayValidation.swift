import KuyuCore
import KuyuPhysics

public func validateScenarioReplay<Runner: PlantScenarioRunner>(
    runner: Runner,
    replayChecker: ReplayChecker,
    definition: Runner.Scenario,
    primaryLog: SimulationLog,
    cutFactory: (Runner.Scenario) throws -> Runner.Cut,
    motorNerveFactory: ((Runner.Scenario) throws -> Runner.Nerve?)?,
    referenceLogs: [ScenarioKey: SimulationLog]
) async throws -> ReplayCheckResult {
    let key = ScenarioKey(scenarioId: primaryLog.scenarioId, seed: primaryLog.seed)
    if let reference = referenceLogs[key] {
        return try replayChecker.check(reference: reference, candidate: primaryLog)
    }

    let replayCut = try cutFactory(definition)
    let replayMotorNerve = try motorNerveFactory?(definition) ?? nil
    let replayLog = try await runner.runScenario(
        definition: definition,
        cut: replayCut,
        motorNerve: replayMotorNerve,
        control: nil,
        telemetry: nil
    )
    return try replayChecker.check(reference: primaryLog, candidate: replayLog)
}
