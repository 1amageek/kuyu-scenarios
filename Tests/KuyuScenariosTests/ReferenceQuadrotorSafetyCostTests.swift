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

private func makeLog(
    tiltRadians: Double,
    omegaMagnitude: Double,
    time: TimeInterval = 0.001,
    stepIndex: UInt64 = 1
) throws -> WorldStepLog {
    let root = RigidBodySnapshot(
        id: "root",
        position: Axis3(x: 0, y: 0, z: 2),
        velocity: Axis3(x: 0, y: 0, z: 0),
        orientation: QuaternionSnapshot(w: 1, x: 0, y: 0, z: 0),
        angularVelocity: Axis3(x: 0, y: 0, z: 0)
    )
    return WorldStepLog(
        time: try WorldTime(stepIndex: stepIndex, time: time),
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
    let value = try cost.cost(
        scenario: scenario,
        log: log,
        duration: 0.002,
        failure: nil,
        truncated: false
    )

    #expect(value == 0)
}

@Test func safetyCostRisesLinearlyFromMarginToLimit() throws {
    let scenario = try makeScenario()
    let config = try ReferenceQuadrotorSafetyCost.Config(
        marginFraction: 0.8,
        tiltWeight: 1.0,
        omegaWeight: 0.0,
        failureImpulseCost: 0.0
    )
    let cost = ReferenceQuadrotorSafetyCost(config: config)
    let tiltLimit = scenario.safetyEnvelope.tiltSafeMaxDegrees * Double.pi / 180.0

    // 90% of the limit sits halfway through the 80%...100% margin band.
    let halfway = try cost.cost(
        scenario: scenario,
        log: try makeLog(tiltRadians: 0.9 * tiltLimit, omegaMagnitude: 0),
        duration: 0.2,
        failure: nil,
        truncated: false
    )
    let atLimit = try cost.cost(
        scenario: scenario,
        log: try makeLog(tiltRadians: tiltLimit, omegaMagnitude: 0),
        duration: 0.2,
        failure: nil,
        truncated: false
    )

    #expect(abs(halfway - 0.1) < 1e-9)
    #expect(abs(atLimit - 0.2) < 1e-9)
}

@Test func safetyCostCapsBeyondEnvelopeAndAddsViolationCost() throws {
    let scenario = try makeScenario()
    let config = try ReferenceQuadrotorSafetyCost.Config(
        marginFraction: 0.8,
        tiltWeight: 1.0,
        omegaWeight: 1.0,
        failureImpulseCost: 5.0
    )
    let cost = ReferenceQuadrotorSafetyCost(config: config)

    // Far beyond both limits (tilt at pi = 3x the 60-degree limit, the
    // largest value SafetyTrace accepts): each hinge caps at 2, so the risk
    // rate saturates at maximumRiskRate.
    let log = try makeLog(tiltRadians: Double.pi, omegaMagnitude: 400.0)
    let survived = try cost.cost(
        scenario: scenario,
        log: log,
        duration: 0.25,
        failure: nil,
        truncated: false
    )

    #expect(abs(survived - config.maximumRiskRate * 0.25) < 1e-9)

    // The same transition ending in a hard failure additionally pays the fixed
    // impulse and the unflown remainder of the scenario at maximumRiskRate.
    let failed = try cost.cost(
        scenario: scenario,
        log: log,
        duration: 0.25,
        failure: FailureEvent(reason: .safetyEnvelope, time: 0.001),
        truncated: false
    )
    let remaining = scenario.config.duration - log.time.time
    let expectedTerminal = config.failureImpulseCost + config.maximumRiskRate * remaining

    #expect(abs(failed - (survived + expectedTerminal)) < 1e-9)
}

@Test func failureCostsMoreThanAnySurvivableContinuation() throws {
    let scenario = try makeScenario()
    let config = try ReferenceQuadrotorSafetyCost.Config(
        marginFraction: 0.8,
        tiltWeight: 1.0,
        omegaWeight: 1.0,
        failureImpulseCost: 1.0
    )
    let cost = ReferenceQuadrotorSafetyCost(config: config)
    let step = 0.1
    let stepCount = Int((scenario.config.duration / step).rounded())
    let failureStep = stepCount / 2

    // Both episodes fly the same prefix and share the transition at
    // `failureStep`. The survivor then flies the whole remaining horizon past
    // both envelope limits, where the hinges cap and the risk rate saturates at
    // maximumRiskRate — the most expensive way to survive. The other episode
    // ends in a hard failure on the shared transition.
    var survivedTotal = 0.0
    var failedTotal = 0.0
    for index in 0 ..< stepCount {
        let time = Double(index + 1) * step
        let isPrefix = index < failureStep
        let log = try makeLog(
            tiltRadians: isPrefix ? 0 : Double.pi,
            omegaMagnitude: isPrefix ? 0 : 400.0,
            time: time,
            stepIndex: UInt64(index + 1)
        )
        survivedTotal += try cost.cost(
            scenario: scenario,
            log: log,
            duration: step,
            failure: nil,
            truncated: index == stepCount - 1
        )
        guard index <= failureStep else { continue }
        failedTotal += try cost.cost(
            scenario: scenario,
            log: log,
            duration: step,
            failure: index == failureStep
                ? FailureEvent(reason: .safetyEnvelope, time: time)
                : nil,
            truncated: false
        )
    }

    #expect(failedTotal > survivedTotal)
    // The strict margin is exactly the fixed impulse: the failure pays the
    // unflown remainder at the same rate the survivor could at most accrue.
    #expect(abs(failedTotal - survivedTotal - config.failureImpulseCost) < 1e-9)
}

@Test func failureRemainderShrinksAsTheHorizonIsExhausted() throws {
    let scenario = try makeScenario()
    let config = try ReferenceQuadrotorSafetyCost.Config(
        tiltWeight: 1.0,
        omegaWeight: 1.0,
        failureImpulseCost: 1.0
    )
    let cost = ReferenceQuadrotorSafetyCost(config: config)
    let failure = FailureEvent(reason: .safetyEnvelope, time: 0)

    let early = try cost.cost(
        scenario: scenario,
        log: try makeLog(tiltRadians: 0, omegaMagnitude: 0, time: 1.0, stepIndex: 1_000),
        duration: 0.001,
        failure: failure,
        truncated: false
    )
    let late = try cost.cost(
        scenario: scenario,
        log: try makeLog(tiltRadians: 0, omegaMagnitude: 0, time: 7.0, stepIndex: 7_000),
        duration: 0.001,
        failure: failure,
        truncated: false
    )
    let atHorizon = try cost.cost(
        scenario: scenario,
        log: try makeLog(
            tiltRadians: 0,
            omegaMagnitude: 0,
            time: scenario.config.duration,
            stepIndex: 8_000
        ),
        duration: 0.001,
        failure: failure,
        truncated: false
    )

    #expect(early > late)
    #expect(abs(early - late - config.maximumRiskRate * 6.0) < 1e-9)
    // Failing on the last tick leaves nothing unflown, so only the fixed
    // impulse remains.
    #expect(abs(atHorizon - config.failureImpulseCost) < 1e-9)
}

@Test func safetyCostRejectsNegativeRemainingDuration() throws {
    let scenario = try makeScenario()
    let cost = ReferenceQuadrotorSafetyCost(config: try ReferenceQuadrotorSafetyCost.Config())

    #expect(throws: ReferenceQuadrotorSafetyCost.CostError.invalidRemainingDuration(-1.0)) {
        _ = try cost.cost(
            tiltRadians: 0,
            omegaMagnitude: 0,
            safetyEnvelope: scenario.safetyEnvelope,
            duration: 0.001,
            termination: .failed(remainingDuration: -1.0)
        )
    }
}

@Test func logBasedCostFloorsTheRemainderAtZeroPastTheHorizon() throws {
    let scenario = try makeScenario()
    let config = try ReferenceQuadrotorSafetyCost.Config(
        tiltWeight: 0,
        omegaWeight: 0,
        failureImpulseCost: 1.0
    )
    let cost = ReferenceQuadrotorSafetyCost(config: config)

    // The physics clock can pass the nominal duration by a sub-tick rounding
    // amount on the final step; that must not produce a negative remainder.
    let value = try cost.cost(
        scenario: scenario,
        log: try makeLog(
            tiltRadians: 0,
            omegaMagnitude: 0,
            time: scenario.config.duration + 0.0004,
            stepIndex: 8_001
        ),
        duration: 0.001,
        failure: FailureEvent(reason: .safetyEnvelope, time: 0),
        truncated: false
    )

    #expect(value == config.failureImpulseCost)
}

@Test func safetyCostConfigRejectsInvalidValues() throws {
    #expect(throws: ReferenceQuadrotorSafetyCost.Config.ValidationError.outOfRange("marginFraction")) {
        _ = try ReferenceQuadrotorSafetyCost.Config(marginFraction: 1.0)
    }
    #expect(throws: ReferenceQuadrotorSafetyCost.Config.ValidationError.outOfRange("tiltWeight")) {
        _ = try ReferenceQuadrotorSafetyCost.Config(tiltWeight: -0.1)
    }
    #expect(
        throws: ReferenceQuadrotorSafetyCost.Config.ValidationError.nonFinite(
            "failureImpulseCost"
        )
    ) {
        _ = try ReferenceQuadrotorSafetyCost.Config(failureImpulseCost: .nan)
    }
}

@Test func integratedSafetyCostIsInvariantToControlPeriod() throws {
    let scenario = try makeScenario()
    let cost = ReferenceQuadrotorSafetyCost(
        config: try ReferenceQuadrotorSafetyCost.Config(
            tiltWeight: 1,
            omegaWeight: 0,
            failureImpulseCost: 0
        )
    )
    let tiltLimit = scenario.safetyEnvelope.tiltSafeMaxDegrees * Double.pi / 180.0
    let log = try makeLog(tiltRadians: tiltLimit, omegaMagnitude: 0)

    let fine = try (0..<100).reduce(into: 0.0) { total, _ in
        total += try cost.cost(
            scenario: scenario,
            log: log,
            duration: 0.01,
            failure: nil,
            truncated: false
        )
    }
    let coarse = try (0..<10).reduce(into: 0.0) { total, _ in
        total += try cost.cost(
            scenario: scenario,
            log: log,
            duration: 0.1,
            failure: nil,
            truncated: false
        )
    }

    #expect(abs(fine - coarse) < 1e-12)
    #expect(abs(fine - 1) < 1e-12)
}

@Test func failureImpulseDoesNotScaleWithControlPeriod() throws {
    let scenario = try makeScenario()
    // Zeroed weights make maximumRiskRate zero, so the unflown remainder costs
    // nothing and the fixed impulse is isolated.
    let cost = ReferenceQuadrotorSafetyCost(
        config: try ReferenceQuadrotorSafetyCost.Config(
            tiltWeight: 0,
            omegaWeight: 0,
            failureImpulseCost: 1
        )
    )
    let log = try makeLog(tiltRadians: 0, omegaMagnitude: 0)
    let failure = FailureEvent(reason: .safetyEnvelope, time: 0.001)

    let fine = try cost.cost(
        scenario: scenario,
        log: log,
        duration: 0.001,
        failure: failure,
        truncated: false
    )
    let coarse = try cost.cost(
        scenario: scenario,
        log: log,
        duration: 0.1,
        failure: failure,
        truncated: false
    )

    #expect(fine == 1)
    #expect(coarse == 1)
}

@Test func safetyCostDescriptorTracksConfig() throws {
    let first = ReferenceQuadrotorSafetyCost(config: try ReferenceQuadrotorSafetyCost.Config())
    let second = ReferenceQuadrotorSafetyCost(
        config: try ReferenceQuadrotorSafetyCost.Config(marginFraction: 0.5)
    )

    #expect(first.descriptor.id == "reference-quadrotor-safety-cost")
    #expect(first.descriptor.configHash != second.descriptor.configHash)
}
