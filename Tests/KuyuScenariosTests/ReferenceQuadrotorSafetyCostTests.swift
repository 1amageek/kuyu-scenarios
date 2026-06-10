import Foundation
import Testing
import KuyuCore
import KuyuPhysics
@testable import KuyuScenarios

private func makeScenario() throws -> ReferenceQuadrotorScenarioDefinition {
    let config = try ScenarioConfig(
        id: ScenarioID("SAFETY-COST/SCN-0"),
        seed: ScenarioSeed(1),
        duration: 8.0,
        timeStep: TimeStep(delta: 0.001)
    )
    return ReferenceQuadrotorScenarioDefinition(
        config: config,
        kind: .hoverStart,
        initialPosition: Axis3(x: 0, y: 0, z: 2),
        initialAttitude: EulerAngles(roll: 0, pitch: 0, yaw: 0),
        initialAngularVelocity: Axis3(x: 0, y: 0, z: 0),
        safetyEnvelope: try SafetyEnvelope(
            omegaSafeMax: 20.0,
            tiltSafeMaxDegrees: 60.0,
            sustainedViolationSeconds: 0.2,
            groundZ: 0.0,
            fallDurationSeconds: 0.5,
            fallVelocityThreshold: 0.05
        ),
        torqueEvents: [],
        actuatorDegradation: nil,
        gyroDriftScale: 1.0,
        swapEvents: [],
        hfEvents: []
    )
}

private func makeLog(tiltRadians: Double, omegaMagnitude: Double) throws -> WorldStepLog {
    let root = RigidBodySnapshot(
        id: "root",
        position: Axis3(x: 0, y: 0, z: 2),
        velocity: Axis3(x: 0, y: 0, z: 0),
        orientation: QuaternionSnapshot(w: 1, x: 0, y: 0, z: 0),
        angularVelocity: Axis3(x: 0, y: 0, z: 0)
    )
    return WorldStepLog(
        time: try WorldTime(stepIndex: 1, time: 0.001),
        events: [],
        sensorSamples: [],
        driveIntents: [],
        reflexCorrections: [],
        actuatorValues: [],
        actuatorTelemetry: ActuatorTelemetrySnapshot(channels: []),
        safetyTrace: try SafetyTrace(omegaMagnitude: omegaMagnitude, tiltRadians: tiltRadians),
        plantState: PlantStateSnapshot(root: root),
        disturbances: DisturbanceSnapshot(
            forceWorld: Axis3(x: 0, y: 0, z: 0),
            torqueBody: Axis3(x: 0, y: 0, z: 0)
        )
    )
}

@Test func safetyCostIsZeroInsideMargin() throws {
    let scenario = try makeScenario()
    let cost = ReferenceQuadrotorSafetyCost(config: try ReferenceQuadrotorSafetyCost.Config())
    let tiltLimit = scenario.safetyEnvelope.tiltSafeMaxDegrees * Double.pi / 180.0

    // 50% of both limits, well below the default 80% margin.
    let log = try makeLog(tiltRadians: 0.5 * tiltLimit, omegaMagnitude: 0.5 * 20.0)
    let value = try cost.cost(scenario: scenario, log: log, failure: nil, truncated: false)

    #expect(value == 0)
}

@Test func safetyCostRisesLinearlyFromMarginToLimit() throws {
    let scenario = try makeScenario()
    let config = try ReferenceQuadrotorSafetyCost.Config(
        marginFraction: 0.8,
        tiltWeight: 1.0,
        omegaWeight: 0.0,
        violationCost: 0.0
    )
    let cost = ReferenceQuadrotorSafetyCost(config: config)
    let tiltLimit = scenario.safetyEnvelope.tiltSafeMaxDegrees * Double.pi / 180.0

    // 90% of the limit sits halfway through the 80%...100% margin band.
    let halfway = try cost.cost(
        scenario: scenario,
        log: try makeLog(tiltRadians: 0.9 * tiltLimit, omegaMagnitude: 0),
        failure: nil,
        truncated: false
    )
    let atLimit = try cost.cost(
        scenario: scenario,
        log: try makeLog(tiltRadians: tiltLimit, omegaMagnitude: 0),
        failure: nil,
        truncated: false
    )

    #expect(abs(halfway - 0.5) < 1e-9)
    #expect(abs(atLimit - 1.0) < 1e-9)
}

@Test func safetyCostCapsBeyondEnvelopeAndAddsViolationCost() throws {
    let scenario = try makeScenario()
    let config = try ReferenceQuadrotorSafetyCost.Config(
        marginFraction: 0.8,
        tiltWeight: 1.0,
        omegaWeight: 1.0,
        violationCost: 5.0
    )
    let cost = ReferenceQuadrotorSafetyCost(config: config)

    // Far beyond both limits (tilt at pi = 3x the 60-degree limit, the
    // largest value SafetyTrace accepts): each hinge caps at 2, plus the
    // violation cost.
    let value = try cost.cost(
        scenario: scenario,
        log: try makeLog(tiltRadians: Double.pi, omegaMagnitude: 400.0),
        failure: FailureEvent(reason: .safetyEnvelope, time: 0.001),
        truncated: false
    )

    #expect(abs(value - (2.0 + 2.0 + 5.0)) < 1e-9)
}

@Test func safetyCostConfigRejectsInvalidValues() throws {
    #expect(throws: ReferenceQuadrotorSafetyCost.Config.ValidationError.outOfRange("marginFraction")) {
        _ = try ReferenceQuadrotorSafetyCost.Config(marginFraction: 1.0)
    }
    #expect(throws: ReferenceQuadrotorSafetyCost.Config.ValidationError.outOfRange("tiltWeight")) {
        _ = try ReferenceQuadrotorSafetyCost.Config(tiltWeight: -0.1)
    }
    #expect(throws: ReferenceQuadrotorSafetyCost.Config.ValidationError.nonFinite("violationCost")) {
        _ = try ReferenceQuadrotorSafetyCost.Config(violationCost: .nan)
    }
}

@Test func safetyCostDescriptorTracksConfig() throws {
    let first = ReferenceQuadrotorSafetyCost(config: try ReferenceQuadrotorSafetyCost.Config())
    let second = ReferenceQuadrotorSafetyCost(
        config: try ReferenceQuadrotorSafetyCost.Config(marginFraction: 0.5)
    )

    #expect(first.descriptor.id == "reference-quadrotor-safety-cost")
    #expect(first.descriptor.configHash != second.descriptor.configHash)
}
