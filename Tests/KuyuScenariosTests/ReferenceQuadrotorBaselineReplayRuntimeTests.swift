import Testing
import KuyuCore
import KuyuPhysics
@testable import KuyuScenarios

@Test(.timeLimit(.minutes(1))) func baselineReplayRuntimePerformsLiftReplay() async throws {
    let definition = try shortLiftDefinition(
        kind: .liftHover,
        id: "KUY-BASELINE-REPLAY/LIFT",
        seed: 6101,
        initialZ: 2.0,
        targetZ: 2.0
    )
    let output = try await ReferenceQuadrotorBaselineReplayRuntime().run(
        request: try baselineRequest(taskMode: .lift),
        parameters: .baseline,
        schedule: try SimulationSchedule.baseline(cutPeriodSteps: 1),
        definitions: [definition]
    )

    #expect(output.summary.replay.notPerformedReason == nil)
    #expect(output.summary.replay.checks.count == 1)
    #expect(output.summary.replay.checks.allSatisfy { $0.passed })
    #expect(output.summary.evaluations.map {
        ScenarioKey(scenarioId: $0.scenarioId, seed: $0.seed)
    } == output.summary.replay.checks.map {
        ScenarioKey(scenarioId: $0.scenarioId, seed: $0.seed)
    })
}

@Test(.timeLimit(.minutes(1))) func baselineReplayRuntimePerformsSingleLiftReplay() async throws {
    let definition = try shortLiftDefinition(
        kind: .singleLiftHover,
        id: "KUY-BASELINE-REPLAY/SLIFT",
        seed: 6201,
        initialZ: 0.5,
        targetZ: 0.5
    )
    let output = try await ReferenceQuadrotorBaselineReplayRuntime().run(
        request: try baselineRequest(taskMode: .singleLift),
        parameters: .baseline,
        schedule: try SimulationSchedule.baseline(cutPeriodSteps: 1),
        definitions: [definition]
    )

    #expect(output.summary.replay.notPerformedReason == nil)
    #expect(output.summary.replay.checks.count == 1)
    #expect(output.summary.replay.checks.allSatisfy { $0.passed })
    #expect(output.summary.evaluations.map {
        ScenarioKey(scenarioId: $0.scenarioId, seed: $0.seed)
    } == output.summary.replay.checks.map {
        ScenarioKey(scenarioId: $0.scenarioId, seed: $0.seed)
    })
}

@Test func baselineReplayRuntimeRejectsNonBaselineController() async throws {
    do {
        _ = try await ReferenceQuadrotorBaselineReplayRuntime().run(
            request: try baselineRequest(taskMode: .lift, controller: .manasMLX),
            parameters: .baseline,
            schedule: try SimulationSchedule.baseline(cutPeriodSteps: 1),
            definitions: []
        )
        Issue.record("Expected non-baseline controller to be rejected.")
    } catch ReferenceQuadrotorBaselineReplayRuntimeError.unsupportedController(let controller) {
        #expect(controller == "ManasMLX")
    }
}

private func baselineRequest(
    taskMode: SimulationTaskMode,
    controller: ControllerSelection = .teacherActiveAltitudeHold
) throws -> SimulationRunRequest {
    SimulationRunRequest(
        controller: controller,
        taskMode: taskMode,
        gains: try ImuRateDampingCutGains(kp: 6.0, kd: 4.0, yawDamping: 0.4, hoverThrustScale: 1.0),
        cutPeriodSteps: 1,
        noise: .zero,
        determinism: .tier1Baseline,
        robotManifestPath: "reference-quadrotor",
        overrideParameters: nil,
        useAux: false,
        useQualityGating: true
    )
}

private func shortLiftDefinition(
    kind: ReferenceQuadrotorScenarioKind,
    id: String,
    seed: UInt64,
    initialZ: Double,
    targetZ: Double
) throws -> ReferenceQuadrotorScenarioDefinition {
    let envelope = try SafetyEnvelope(
        omegaSafeMax: 1000.0,
        tiltSafeMaxDegrees: 180.0,
        sustainedViolationSeconds: 999.0,
        groundZ: 0.0,
        fallDurationSeconds: 0.5,
        fallVelocityThreshold: 0.5
    )
    let liftEnvelope = LiftEnvelope(
        targetZ: targetZ,
        tolerance: 0.2,
        maxVelocity: 0.5,
        warmupTime: 0.0,
        requiredHoldTime: 0.02
    )
    return ReferenceQuadrotorScenarioDefinition(
        config: try ScenarioConfig(
            id: ScenarioID(id),
            seed: ScenarioSeed(seed),
            duration: 0.02,
            timeStep: try TimeStep(delta: 0.002)
        ),
        kind: kind,
        initialPosition: Axis3(x: 0, y: 0, z: initialZ),
        initialAttitude: EulerAngles(roll: 0, pitch: 0, yaw: 0),
        initialAngularVelocity: Axis3(x: 0, y: 0, z: 0),
        safetyEnvelope: envelope,
        liftEnvelope: liftEnvelope,
        torqueEvents: [],
        actuatorDegradation: nil,
        gyroDriftScale: 1.0,
        swapEvents: [],
        hfEvents: []
    )
}
