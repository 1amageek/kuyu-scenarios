import Testing
import KuyuCore
import KuyuPhysics
@testable import KuyuScenarios

@Test func taskQualityEvaluatorAcceptsSettledLiftLog() throws {
    let definition = try makeTaskQualityLiftScenario()
    let log = try makeTaskQualityLog(
        definition: definition,
        samples: [
            (0.0, 0.0, 0.0),
            (0.1, 0.7, 0.1),
            (0.2, 1.0, 0.0),
            (0.3, 1.0, 0.0),
            (0.4, 1.0, 0.0),
            (0.5, 1.0, 0.0),
        ]
    )

    let summary = ReferenceQuadrotorTaskQualityEvaluator().evaluate(definition: definition, log: log)

    #expect(summary.task == "lift")
    #expect(summary.passed)
    #expect((summary.achievedHoldTime ?? 0) >= (summary.requiredHoldTime ?? 0))
    #expect((summary.maxAltitudeErrorAfterWarmup ?? 1) <= (summary.tolerance ?? 0))
}

@Test func taskQualityEvaluatorRejectsUnsettledLiftLog() throws {
    let definition = try makeTaskQualityLiftScenario()
    let log = try makeTaskQualityLog(
        definition: definition,
        samples: [
            (0.0, 0.0, 0.0),
            (0.1, 0.4, 0.5),
            (0.2, 1.5, 0.5),
            (0.3, 1.6, 0.5),
            (0.4, 1.5, 0.5),
        ]
    )

    let summary = ReferenceQuadrotorTaskQualityEvaluator().evaluate(definition: definition, log: log)

    #expect(!summary.passed)
    #expect(summary.failureReasons.contains("lift-unsettled"))
    #expect(summary.failureReasons.contains("hold-time-below-min"))
    #expect(summary.failureReasons.contains("altitude-error-above-tolerance"))
}

private func makeTaskQualityLiftScenario() throws -> ReferenceQuadrotorScenarioDefinition {
    let timeStep = try TimeStep(delta: 0.1)
    let config = try ScenarioConfig(
        id: ScenarioID("KUY-TASK-QUALITY/LIFT"),
        seed: ScenarioSeed(7),
        duration: 0.6,
        timeStep: timeStep
    )
    let safetyEnvelope = try SafetyEnvelope(
        omegaSafeMax: 1000,
        tiltSafeMaxDegrees: 180,
        sustainedViolationSeconds: 999,
        groundZ: -1,
        fallDurationSeconds: 1,
        fallVelocityThreshold: 10
    )
    let liftEnvelope = LiftEnvelope(
        targetZ: 1,
        tolerance: 0.1,
        maxVelocity: 0.2,
        warmupTime: 0.2,
        requiredHoldTime: 0.2
    )
    return ReferenceQuadrotorScenarioDefinition(
        config: config,
        kind: .liftHover,
        initialPosition: Axis3(x: 0, y: 0, z: 0),
        initialAttitude: EulerAngles(roll: 0, pitch: 0, yaw: 0),
        initialAngularVelocity: Axis3(x: 0, y: 0, z: 0),
        safetyEnvelope: safetyEnvelope,
        liftEnvelope: liftEnvelope,
        torqueEvents: [],
        actuatorDegradation: nil,
        gyroDriftScale: 1,
        swapEvents: [],
        hfEvents: []
    )
}

private func makeTaskQualityLog(
    definition: ReferenceQuadrotorScenarioDefinition,
    samples: [(time: Double, altitude: Double, verticalVelocity: Double)]
) throws -> SimulationLog {
    let events = try samples.enumerated().map { index, sample in
        try makeTaskQualityStep(
            index: UInt64(index),
            time: sample.time,
            altitude: sample.altitude,
            verticalVelocity: sample.verticalVelocity
        )
    }
    return SimulationLog(
        scenarioId: definition.config.id,
        seed: definition.config.seed,
        timeStep: definition.config.timeStep,
        determinism: .tier1Baseline,
        configHash: "task-quality-test",
        events: events
    )
}

private func makeTaskQualityStep(
    index: UInt64,
    time: Double,
    altitude: Double,
    verticalVelocity: Double
) throws -> WorldStepLog {
    let root = RigidBodySnapshot(
        id: "root",
        position: Axis3(x: 0, y: 0, z: altitude),
        velocity: Axis3(x: 0, y: 0, z: verticalVelocity),
        orientation: QuaternionSnapshot(w: 1, x: 0, y: 0, z: 0),
        angularVelocity: Axis3(x: 0, y: 0, z: 0)
    )
    return WorldStepLog(
        time: try WorldTime(stepIndex: index, time: time),
        events: [],
        sensorSamples: [],
        driveIntents: [],
        reflexCorrections: [],
        actuatorValues: [],
        actuatorTelemetry: ActuatorTelemetrySnapshot(channels: []),
        safetyTrace: try SafetyTrace(omegaMagnitude: 0, tiltRadians: 0),
        plantState: PlantStateSnapshot(root: root),
        disturbances: DisturbanceSnapshot(
            forceWorld: Axis3(x: 0, y: 0, z: 0),
            torqueBody: Axis3(x: 0, y: 0, z: 0)
        )
    )
}
