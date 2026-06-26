import KuyuCore
import KuyuPhysics
import Testing
@testable import KuyuScenarios

@Test func articulatedDynamicScenarioOutputFactoryProjectsPassingLog() throws {
    let log = try makeArticulatedDynamicLog(
        events: [makeArticulatedStep(index: 0, omega: 0.4, tiltRadians: Double.pi / 6)]
    )

    let output = ArticulatedDynamicScenarioOutputFactory().makeOutput(log: log)

    #expect(output.result.passed)
    #expect(output.summary.suitePassed)
    #expect(output.result.evaluations.count == 1)
    #expect(output.summary.evaluations.count == 1)
    #expect(output.logs.count == 1)
    #expect(output.summary.manifest.count == 1)
    #expect(output.summary.replay.notPerformedReason == "Articulated dynamic simulation does not execute replay verification.")
    #expect(abs(output.summary.evaluations[0].maxTiltDegrees - 30.0) < 1e-12)
}

@Test func articulatedDynamicScenarioOutputFactoryRejectsFailureReason() throws {
    let log = try makeArticulatedDynamicLog(
        events: [makeArticulatedStep(index: 0, omega: 0.0, tiltRadians: 0.0)],
        failureReason: .groundViolation,
        failureTime: 0.25
    )

    let output = ArticulatedDynamicScenarioOutputFactory().makeOutput(log: log)

    #expect(!output.result.passed)
    #expect(!output.summary.suitePassed)
    #expect(output.summary.evaluations[0].failureReason == .groundViolation)
    #expect(output.summary.evaluations[0].failureTime == 0.25)
    #expect(output.summary.evaluations[0].failures == ["ground-violation"])
}

@Test func articulatedDynamicScenarioOutputFactoryRejectsEmptyLog() throws {
    let log = try makeArticulatedDynamicLog(events: [])

    let output = ArticulatedDynamicScenarioOutputFactory().makeOutput(log: log)

    #expect(!output.result.passed)
    #expect(output.summary.evaluations[0].failures == ["simulation-log-empty"])
    #expect(output.summary.manifest[0].duration == 0)
}

private func makeArticulatedDynamicLog(
    events: [WorldStepLog],
    failureReason: FailureReason? = nil,
    failureTime: Double? = nil
) throws -> SimulationLog {
    SimulationLog(
        scenarioId: try ScenarioID("ROARM-DYN-1"),
        seed: ScenarioSeed(11),
        timeStep: try TimeStep(delta: 0.01),
        determinism: .tier1Baseline,
        configHash: "articulated-test",
        events: events,
        failureReason: failureReason,
        failureTime: failureTime
    )
}

private func makeArticulatedStep(
    index: UInt64,
    omega: Double,
    tiltRadians: Double
) throws -> WorldStepLog {
    let root = RigidBodySnapshot(
        id: "root",
        position: Axis3(x: 0, y: 0, z: 0),
        velocity: Axis3(x: 0, y: 0, z: 0),
        orientation: QuaternionSnapshot(w: 1, x: 0, y: 0, z: 0),
        angularVelocity: Axis3(x: 0, y: 0, z: 0)
    )
    return WorldStepLog(
        time: try WorldTime(stepIndex: index, time: Double(index) * 0.01),
        events: [.timeAdvance, .logging],
        sensorSamples: [],
        driveIntents: [],
        reflexCorrections: [],
        actuatorValues: [],
        actuatorTelemetry: ActuatorTelemetrySnapshot(channels: []),
        safetyTrace: try SafetyTrace(omegaMagnitude: omega, tiltRadians: tiltRadians),
        plantState: PlantStateSnapshot(root: root),
        disturbances: DisturbanceSnapshot(
            forceWorld: Axis3(x: 0, y: 0, z: 0),
            torqueBody: Axis3(x: 0, y: 0, z: 0)
        )
    )
}
