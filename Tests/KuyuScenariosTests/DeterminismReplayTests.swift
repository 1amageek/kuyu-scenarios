import Testing
import KuyuCore
import KuyuPhysics
@testable import KuyuScenarios

/// Tier-0 bit-exact replay: the same seed must yield byte-identical observations and
/// rewards, including across mid-run swap/HF stress injection.
@Test(.timeLimit(.minutes(1))) func sameSeedReplayIsBitExactAcrossStress() async throws {
    let definition = try makeStressReplayScenario(seed: 7777)

    let first = try await runEpisode(definition: definition, steps: 30)
    let second = try await runEpisode(definition: definition, steps: 30)

    #expect(first.observations.count == second.observations.count)
    #expect(first.observations == second.observations)
    #expect(first.rewards == second.rewards)
    #expect(first.terminalReasons == second.terminalReasons)
}

/// A different seed must change the trajectory (the seed actually drives noise),
/// guarding against a degenerate "always identical" determinism.
@Test(.timeLimit(.minutes(1))) func differentSeedChangesTrajectory() async throws {
    let a = try await runEpisode(definition: try makeStressReplayScenario(seed: 1), steps: 30)
    let b = try await runEpisode(definition: try makeStressReplayScenario(seed: 2), steps: 30)
    #expect(a.observations != b.observations)
}

private struct EpisodeTrace: Equatable {
    var observations: [EnvironmentObservation]
    var rewards: [Double]
    var terminalReasons: [String?]
}

private func runEpisode(definition: ReferenceQuadrotorScenarioDefinition, steps: Int) async throws -> EpisodeTrace {
    let schedule = try SimulationSchedule.baseline(cutPeriodSteps: 1)
    let gains = try ImuRateDampingCutGains(kp: 2.0, kd: 0.25, yawDamping: 0.2)
    var policy = try KuyAtt1BaselineEnvironmentPolicy(definition: definition, gains: gains, mode: .teacher)
    var environment = ReferenceQuadrotorRLEnvironment(
        schedule: schedule,
        determinism: .tier0Strict,
        motorNerveRateLimitPerSecond: 100.0,
        motorNerveSmoothingTimeConstant: nil
    )

    var observation = try environment.reset(seed: definition.config.seed, scenario: definition)
    var trace = EpisodeTrace(observations: [observation], rewards: [], terminalReasons: [])
    for _ in 0..<steps {
        let action = try await policy.action(for: observation)
        let step = try environment.step(action: action)
        observation = step.observation
        trace.observations.append(step.observation)
        trace.rewards.append(step.reward)
        trace.terminalReasons.append(step.info.terminalReason)
        if step.done || step.truncated { break }
    }
    return trace
}

private func makeStressReplayScenario(seed: UInt64) throws -> ReferenceQuadrotorScenarioDefinition {
    let timeStep = try TimeStep(delta: 0.001)
    let envelope = try SafetyEnvelope(
        omegaSafeMax: 20.0,
        tiltSafeMaxDegrees: 60.0,
        sustainedViolationSeconds: 0.200,
        groundZ: 0.0,
        fallDurationSeconds: 0.5,
        fallVelocityThreshold: 0.05
    )
    let config = try ScenarioConfig(
        id: ScenarioID("KUY-DET-REPLAY"),
        seed: ScenarioSeed(seed),
        duration: 0.05,
        timeStep: timeStep
    )
    // A sensor swap and an HF glitch inside the short run, so replay must reproduce
    // the swap/HF code paths bit-for-bit too.
    let swap = try SensorSwapEvent(
        kind: .calibShift,
        startTime: 0.005,
        duration: 0.02,
        targetChannels: [0, 1, 2],
        gainScale: 1.4,
        biasShift: 0.05,
        noiseScale: 2.0,
        dropoutProbability: 0.0,
        delayShiftSteps: 0,
        bandwidthScale: 0.5
    )
    let glitch = try HFStressEvent(kind: .sensorGlitch, startTime: 0.01, duration: 0.01, magnitude: 0.1)
    return ReferenceQuadrotorScenarioDefinition(
        config: config,
        kind: .hoverStart,
        initialPosition: Axis3(x: 0, y: 0, z: 2.0),
        initialAttitude: EulerAngles.degrees(roll: 5, pitch: 0, yaw: 0),
        initialAngularVelocity: Axis3(x: 0, y: 0, z: 0),
        safetyEnvelope: envelope,
        torqueEvents: [],
        actuatorDegradation: nil,
        gyroDriftScale: 1.0,
        swapEvents: [.sensor(swap)],
        hfEvents: [glitch]
    )
}
