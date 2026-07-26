import Foundation
import Testing
import KuyuCore
import KuyuPhysics
@testable import KuyuScenarios

@Test func referenceQuadrotorEnvironmentRunsToTimeLimitWithFiniteRewards() async throws {
    let definition = try makeShortAttitudeScenario()
    let schedule = try SimulationSchedule.baseline(cutPeriodSteps: 1)
    let gains = try ImuRateDampingCutGains(kp: 2.0, kd: 0.25, yawDamping: 0.2)
    var policy = try KuyAtt1BaselineEnvironmentPolicy(
        definition: definition,
        gains: gains,
        mode: .teacher
    )
    var environment = ReferenceQuadrotorRLEnvironment(
        schedule: schedule,
        determinism: .tier1Baseline,
        motorNerveRateLimitPerSecond: 100.0,
        motorNerveSmoothingTimeConstant: nil
    )

    var observation = try environment.reset(seed: definition.config.seed, scenario: definition)
    var final: EnvironmentStep?
    for _ in 0..<100 {
        let action = try await policy.action(for: observation)
        let step = try environment.step(action: action)
        #expect(step.reward.isFinite)
        observation = step.observation
        final = step
        if step.done || step.truncated { break }
    }

    #expect(final?.done == false)
    #expect(final?.truncated == true)
    #expect(final?.info.failureReason == nil)
}

@Test func referenceQuadrotorEnvironmentStepsOneCompleteControlPeriod() throws {
    for controlPeriodSteps in [UInt64(2), UInt64(3)] {
        let definition = try makeShortAttitudeScenario(duration: 0.018)
        let action = try makeDriveAction([0.2, 0.3, 0.4, 0.5])
        var environment = ReferenceQuadrotorRLEnvironment(
            schedule: try SimulationSchedule.baseline(cutPeriodSteps: controlPeriodSteps),
            determinism: .tier1Baseline,
            motorNerveRateLimitPerSecond: 100.0,
            motorNerveSmoothingTimeConstant: nil
        )
        var singleTickEnvironment = ReferenceQuadrotorRLEnvironment(
            schedule: try SimulationSchedule.baseline(cutPeriodSteps: 1),
            determinism: .tier1Baseline,
            motorNerveRateLimitPerSecond: 100.0,
            motorNerveSmoothingTimeConstant: nil
        )

        let actionObservation = try environment.reset(seed: definition.config.seed, scenario: definition)
        _ = try singleTickEnvironment.reset(seed: definition.config.seed, scenario: definition)
        let controlStep = try environment.step(action: action)
        var singleTickReward = 0.0
        var singleTickStep: EnvironmentStep?
        for _ in 0..<controlPeriodSteps {
            let step = try singleTickEnvironment.step(action: action)
            singleTickReward += step.reward
            singleTickStep = step
        }

        let expectedDuration = definition.config.timeStep.delta * Double(controlPeriodSteps)
        #expect(actionObservation.time.time == 0.0)
        #expect(abs(controlStep.observation.time.time - expectedDuration) <= 1.0e-12)
        #expect(controlStep.observation.time.stepIndex == controlPeriodSteps)
        #expect(controlStep.info.stepCount == 1)
        #expect(controlStep.log.driveIntents == action.driveIntents)
        #expect(controlStep.observation.plantState == singleTickStep?.observation.plantState)
        #expect(abs(controlStep.reward - singleTickReward) <= 1.0e-12)

        let nextAction = try makeDriveAction([0.6, 0.5, 0.4, 0.3])
        let nextStep = try environment.step(action: nextAction)
        #expect(nextStep.log.driveIntents == nextAction.driveIntents)
        #expect(nextStep.info.stepCount == 2)
        #expect(abs(nextStep.observation.time.time - (2.0 * expectedDuration)) <= 1.0e-12)
    }
}

@Test func referenceQuadrotorEnvironmentIntegratesSafetyCostAtPhysicsTickRate() throws {
    let definition = try makeShortAttitudeScenario(duration: 0.018)
    let safetyCost = ReferenceQuadrotorSafetyCost(
        config: try ReferenceQuadrotorSafetyCost.Config(
            marginFraction: 0,
            constraintLimits: try .init(
                maximumAngularRate: 1.0e-9,
                maximumAttitudeDeviationRadians: 1.0e-9
            ),
            failureImpulseCost: 0
        )
    )
    let action = try makeDriveAction([0.2, 0.3, 0.4, 0.5])
    var controlEnvironment = ReferenceQuadrotorRLEnvironment(
        schedule: try SimulationSchedule.baseline(cutPeriodSteps: 3),
        determinism: .tier1Baseline,
        motorNerveRateLimitPerSecond: 100,
        motorNerveSmoothingTimeConstant: nil,
        safetyCostFunction: safetyCost
    )
    var tickEnvironment = ReferenceQuadrotorRLEnvironment(
        schedule: try SimulationSchedule.baseline(cutPeriodSteps: 1),
        determinism: .tier1Baseline,
        motorNerveRateLimitPerSecond: 100,
        motorNerveSmoothingTimeConstant: nil,
        safetyCostFunction: safetyCost
    )
    _ = try controlEnvironment.reset(seed: definition.config.seed, scenario: definition)
    _ = try tickEnvironment.reset(seed: definition.config.seed, scenario: definition)

    let controlStep = try controlEnvironment.step(action: action)
    let controlMeasurement = try #require(controlStep.costMeasurement)
    var tickCost = 0.0
    for _ in 0..<3 {
        let tickStep = try tickEnvironment.step(action: action)
        tickCost += try #require(tickStep.costMeasurement).value
    }

    #expect(controlMeasurement.descriptor == safetyCost.descriptor)
    #expect(controlMeasurement.physicsTickCount == 3)
    #expect(abs(controlMeasurement.duration - 0.003) < 1.0e-12)
    #expect(abs(controlMeasurement.value - tickCost) < 1.0e-12)
}

@Test func referenceQuadrotorEnvironmentFailsWithinControlPeriodWithoutAdvancingPastFailure() throws {
    let definition = try makeShortAttitudeScenario(duration: 0.018, groundZ: 3.0)
    let safetyCost = ReferenceQuadrotorSafetyCost(
        config: try ReferenceQuadrotorSafetyCost.Config(
            tiltWeight: 0,
            omegaWeight: 0,
            failureImpulseCost: 5
        )
    )
    var environment = ReferenceQuadrotorRLEnvironment(
        schedule: try SimulationSchedule.baseline(cutPeriodSteps: 3),
        determinism: .tier1Baseline,
        motorNerveRateLimitPerSecond: 100.0,
        motorNerveSmoothingTimeConstant: nil,
        safetyCostFunction: safetyCost
    )
    _ = try environment.reset(seed: definition.config.seed, scenario: definition)

    let step = try environment.step(action: makeDriveAction([0.5, 0.5, 0.5, 0.5]))

    #expect(step.done)
    #expect(!step.truncated)
    #expect(step.info.failureReason == .groundViolation)
    #expect(step.observation.time.stepIndex == 1)
    #expect(abs(step.observation.time.time - definition.config.timeStep.delta) <= 1.0e-12)
    let measurement = try #require(step.costMeasurement)
    #expect(measurement.physicsTickCount == 1)
    #expect(measurement.value == 5)
}

@Test func referenceQuadrotorEnvironmentTruncatesAtPartialFinalControlPeriod() throws {
    let definition = try makeShortAttitudeScenario(duration: 0.02)
    var environment = ReferenceQuadrotorRLEnvironment(
        schedule: try SimulationSchedule.baseline(cutPeriodSteps: 3),
        determinism: .tier1Baseline,
        motorNerveRateLimitPerSecond: 100.0,
        motorNerveSmoothingTimeConstant: nil
    )
    var observation = try environment.reset(seed: definition.config.seed, scenario: definition)
    var transitions: [(source: Double, outcome: EnvironmentStep)] = []

    while true {
        let step = try environment.step(action: makeDriveAction([0.5, 0.5, 0.5, 0.5]))
        transitions.append((source: observation.time.time, outcome: step))
        observation = step.observation
        if step.done || step.truncated { break }
    }

    let final = try #require(transitions.last)
    #expect(transitions.count == 7)
    #expect(!final.outcome.done)
    #expect(final.outcome.truncated)
    #expect(abs(final.source - 0.018) <= 1.0e-12)
    #expect(abs(final.outcome.observation.time.time - 0.020) <= 1.0e-12)
    #expect(final.outcome.observation.time.stepIndex == 20)
}

@Test func referenceQuadrotorEnvironmentRejectsMismatchedControlSchedule() throws {
    let definition = try makeShortAttitudeScenario()
    let schedule = try SimulationSchedule(
        sensor: SubsystemSchedule(periodSteps: 1),
        actuator: SubsystemSchedule(periodSteps: 1),
        cut: SubsystemSchedule(periodSteps: 2),
        motorNerve: SubsystemSchedule(periodSteps: 1)
    )
    var environment = ReferenceQuadrotorRLEnvironment(
        schedule: schedule,
        determinism: .tier1Baseline
    )

    #expect(
        throws: ReferenceQuadrotorRLEnvironment.EnvironmentError.unsupportedControlSchedule(
            cutPeriodSteps: 2,
            motorNervePeriodSteps: 1,
            sensorPeriodSteps: 1
        )
    ) {
        _ = try environment.reset(seed: definition.config.seed, scenario: definition)
    }
}

@Test func referenceQuadrotorEnvironmentRealizesTemporalCTBRBeforeMotorNerve() throws {
    let definition = try makeShortAttitudeScenario()
    var environment = ReferenceQuadrotorRLEnvironment(
        schedule: try SimulationSchedule.baseline(cutPeriodSteps: 1),
        determinism: .tier1Baseline,
        actionRealization: .temporalCTBR(.canonical),
        motorNerveRateLimitPerSecond: 1000,
        motorNerveSmoothingTimeConstant: nil
    )
    _ = try environment.reset(seed: definition.config.seed, scenario: definition)
    let command = try ReferenceQuadrotorCTBRCommand(
        collectiveThrust: 0.5,
        normalizedBodyRate: Axis3(x: 1.0, y: 0.0, z: 0.0)
    )

    let step = try environment.step(command: command)
    let trace = try #require(step.log.motorNerveTrace)

    #expect(abs(trace.uRaw[0] - 0.5) < 1e-12)
    #expect(abs(trace.uRaw[1] - 0.6458333333333334) < 1e-12)
    #expect(abs(trace.uRaw[2] - 0.5) < 1e-12)
    #expect(abs(trace.uRaw[3] - 0.3541666666666667) < 1e-12)
    #expect(step.log.driveIntents.map(\.activation) == trace.uRaw)
    #expect(environment.activeExecutionContract?.actionRealization == .temporalCTBR(.canonical))
}

@Test func referenceQuadrotorEnvironmentRejectsDriveMixerActionInTemporalCTBRMode() throws {
    let definition = try makeShortAttitudeScenario()
    var environment = ReferenceQuadrotorRLEnvironment(
        schedule: try SimulationSchedule.baseline(cutPeriodSteps: 1),
        determinism: .tier1Baseline,
        actionRealization: .temporalCTBR(.canonical)
    )
    _ = try environment.reset(seed: definition.config.seed, scenario: definition)

    #expect(
        throws: ReferenceQuadrotorRLEnvironment.EnvironmentError.actionContractMismatch(
            expected: .temporalCTBR,
            received: .driveMixer
        )
    ) {
        _ = try environment.step(action: makeDriveAction([0.5, 0, 0, 0]))
    }
}

@Test func referenceQuadrotorEnvironmentRejectsTemporalCTBRCommandInDriveMixerMode() throws {
    let definition = try makeShortAttitudeScenario()
    var environment = ReferenceQuadrotorRLEnvironment(
        schedule: try SimulationSchedule.baseline(cutPeriodSteps: 1),
        determinism: .tier1Baseline
    )
    _ = try environment.reset(seed: definition.config.seed, scenario: definition)
    let command = try ReferenceQuadrotorCTBRCommand(
        collectiveThrust: 0.5,
        normalizedBodyRate: Axis3(x: 0, y: 0, z: 0)
    )

    #expect(
        throws: ReferenceQuadrotorRLEnvironment.EnvironmentError.actionContractMismatch(
            expected: .driveMixer,
            received: .temporalCTBR
        )
    ) {
        _ = try environment.step(command: command)
    }
}

@Test func referenceQuadrotorEnvironmentConfigHashBindsActionRealization() throws {
    let definition = try makeShortAttitudeScenario()
    let schedule = try SimulationSchedule.baseline(cutPeriodSteps: 1)
    var driveEnvironment = ReferenceQuadrotorRLEnvironment(
        schedule: schedule,
        determinism: .tier1Baseline
    )
    var ctbrEnvironment = ReferenceQuadrotorRLEnvironment(
        schedule: schedule,
        determinism: .tier1Baseline,
        actionRealization: .temporalCTBR(.canonical)
    )

    _ = try driveEnvironment.reset(seed: definition.config.seed, scenario: definition)
    _ = try ctbrEnvironment.reset(seed: definition.config.seed, scenario: definition)

    #expect(driveEnvironment.configHash != ctbrEnvironment.configHash)
}

@Test func privilegedAltitudeTeacherAdjustsOnlyCollectiveThrottle() async throws {
    let definition = try makeShortAttitudeScenario()
    let gains = try ImuRateDampingCutGains(kp: 2.0, kd: 0.25, yawDamping: 0.2)
    let lowObservation = EnvironmentObservation(
        log: try makeRewardLog(altitude: definition.initialPosition.z - 0.25, verticalVelocity: 0)
    )
    let highObservation = EnvironmentObservation(
        log: try makeRewardLog(altitude: definition.initialPosition.z + 0.25, verticalVelocity: 0)
    )
    let referenceObservation = EnvironmentObservation(
        log: try makeRewardLog(altitude: definition.initialPosition.z, verticalVelocity: 0)
    )

    var activeLow = try KuyAtt1BaselineEnvironmentPolicy(
        definition: definition,
        gains: gains,
        mode: .teacher
    )
    let activeLowDrives = try driveIntents(from: try await activeLow.action(for: lowObservation))

    var activeHigh = try KuyAtt1BaselineEnvironmentPolicy(
        definition: definition,
        gains: gains,
        mode: .teacher
    )
    let activeHighDrives = try driveIntents(from: try await activeHigh.action(for: highObservation))

    var activeReference = try KuyAtt1BaselineEnvironmentPolicy(
        definition: definition,
        gains: gains,
        mode: .teacher
    )
    let activeReferenceDrives = try driveIntents(from: try await activeReference.action(for: referenceObservation))

    #expect(activeLowDrives[0].activation > activeReferenceDrives[0].activation)
    #expect(activeHighDrives[0].activation < activeReferenceDrives[0].activation)
    #expect(Array(activeLowDrives.dropFirst()) == Array(activeReferenceDrives.dropFirst()))
    #expect(Array(activeHighDrives.dropFirst()) == Array(activeReferenceDrives.dropFirst()))
}

@Test func kuyAtt1TeacherRunnerUsesOverrideDefinitions() async throws {
    let definition = try makeShortAttitudeScenario()
    let gains = try ImuRateDampingCutGains(kp: 2.0, kd: 0.25, yawDamping: 0.2)
    let runner = try KuyAtt1Runner.activeAltitudeHoldTeacher(gains: gains)

    let output = try await runner.runWithLogs(definitions: [definition])

    #expect(output.logs.count == 1)
    #expect(output.logs.first?.key == ScenarioKey(scenarioId: definition.config.id, seed: definition.config.seed))
    #expect(output.summary.manifest.count == 1)
    #expect(output.result.replay.notPerformedReason == nil)
    #expect(output.result.replay.checks.count == 1)
    #expect(output.result.replay.checks.allSatisfy { $0.passed })
}

@Test func kuyAtt1TeacherRunnerRecordsDisabledReplayExplicitly() async throws {
    let definition = try makeShortAttitudeScenario()
    let gains = try ImuRateDampingCutGains(kp: 2.0, kd: 0.25, yawDamping: 0.2)
    let runner = try KuyAtt1Runner.activeAltitudeHoldTeacher(gains: gains, replayVerification: false)

    let output = try await runner.runWithLogs(definitions: [definition])

    #expect(output.result.replay.notPerformedReason != nil)
    #expect(output.result.replay.checks.isEmpty)
}

@Test func referenceQuadrotorRLEnvironmentResponsibilitiesLiveInSplitFiles() throws {
    let rlRoot = packageRootURL()
        .appendingPathComponent("Sources", isDirectory: true)
        .appendingPathComponent("KuyuScenarios", isDirectory: true)
        .appendingPathComponent("RL", isDirectory: true)
    let environment = try sourceContents(root: rlRoot, fileName: "ReferenceQuadrotorRLEnvironment.swift")
    let reset = try sourceContents(root: rlRoot, fileName: "ReferenceQuadrotorRLEnvironment+Reset.swift")
    let step = try sourceContents(root: rlRoot, fileName: "ReferenceQuadrotorRLEnvironment+Step.swift")
    let advance = try sourceContents(root: rlRoot, fileName: "ReferenceQuadrotorRLEnvironment+Advance.swift")
    let controlLog = try sourceContents(root: rlRoot, fileName: "ReferenceQuadrotorRLEnvironment+ControlLog.swift")
    let episodeInfo = try sourceContents(root: rlRoot, fileName: "ReferenceQuadrotorRLEnvironment+EpisodeInfo.swift")
    let observation = try sourceContents(root: rlRoot, fileName: "ReferenceQuadrotorRLEnvironment+Observation.swift")
    let quad = try sourceContents(root: rlRoot, fileName: "ReferenceQuadrotorRLEnvironment+QuadSimulator.swift")
    let lift = try sourceContents(root: rlRoot, fileName: "ReferenceQuadrotorRLEnvironment+LiftSimulator.swift")
    let single = try sourceContents(root: rlRoot, fileName: "ReferenceQuadrotorRLEnvironment+SingleSimulator.swift")
    let stress = try sourceContents(root: rlRoot, fileName: "ReferenceQuadrotorRLEnvironment+StressValidation.swift")
    let support = try sourceContents(root: rlRoot, fileName: "ReferenceQuadrotorRLEnvironment+Support.swift")
    let worldModel = try sourceContents(root: rlRoot, fileName: "ReferenceQuadrotorRLEnvironment+WorldModel.swift")

    #expect(environment.contains("public struct ReferenceQuadrotorRLEnvironment"))
    #expect(environment.contains("public enum EnvironmentError"))
    #expect(!environment.contains("mutating func reset"))
    #expect(!environment.contains("mutating func step"))
    #expect(!environment.contains("func makeQuadSimulator"))
    #expect(reset.contains("mutating func reset"))
    #expect(step.contains("mutating func step"))
    #expect(advance.contains("func advance"))
    #expect(controlLog.contains("func log"))
    #expect(controlLog.contains("applying application: WorldControlApplication"))
    #expect(episodeInfo.contains("func episodeInfo"))
    #expect(observation.contains("func initialObservation"))
    #expect(quad.contains("func makeQuadSimulator"))
    #expect(quad.contains("func quadMotorNerve"))
    #expect(lift.contains("func makeLiftSimulator"))
    #expect(single.contains("func makeSingleSimulator"))
    #expect(single.contains("func fixedSinglePropMotorNerve"))
    #expect(stress.contains("func validateSingleLiftStress"))
    #expect(support.contains("func buildStore"))
    #expect(support.contains("func scaledNoise"))
    #expect(worldModel.contains("func validateWorldModelPrediction"))
}

private func makeShortAttitudeScenario(
    duration: Double = 0.02,
    groundZ: Double = 0.0
) throws -> ReferenceQuadrotorScenarioDefinition {
    let timeStep = try TimeStep(delta: 0.001)
    let envelope = try SafetyEnvelope(
        omegaSafeMax: 20.0,
        tiltSafeMaxDegrees: 60.0,
        sustainedViolationSeconds: 0.200,
        groundZ: groundZ,
        fallDurationSeconds: 0.5,
        fallVelocityThreshold: 0.05
    )
    let config = try ScenarioConfig(
        id: ScenarioID("KUY-RL-TEST/ATT"),
        seed: ScenarioSeed(42),
        duration: duration,
        timeStep: timeStep
    )
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
        swapEvents: [],
        hfEvents: []
    )
}

private func makeDriveAction(_ activations: [Double]) throws -> EnvironmentAction {
    .driveIntents(
        try activations.enumerated().map { index, activation in
            try DriveIntent(index: DriveIndex(UInt32(index)), activation: activation)
        },
        corrections: []
    )
}

private func packageRootURL(filePath: String = #filePath) -> URL {
    URL(fileURLWithPath: filePath)
        .deletingLastPathComponent()
        .deletingLastPathComponent()
        .deletingLastPathComponent()
}

private func sourceContents(root: URL, fileName: String) throws -> String {
    try String(contentsOf: root.appendingPathComponent(fileName), encoding: .utf8)
}

@Test func referenceQuadrotorEnvironmentSupportsLiftAndSingleLiftTasks() async throws {
    let schedule = try SimulationSchedule.baseline(cutPeriodSteps: 1)

    for definition in [
        try makeShortLiftScenario(kind: .liftHover, id: "KUY-RL-TEST/LIFT", seed: 43, initialZ: 2.0, targetZ: 2.0),
        try makeShortLiftScenario(kind: .singleLiftHover, id: "KUY-RL-TEST/SLIFT", seed: 44, initialZ: 0.0, targetZ: 0.5),
    ] {
        let parameters = try parameters(for: definition.kind)
        var policy = try KuyLiftBaselineEnvironmentPolicy(
            definition: definition,
            parameters: parameters,
            mode: .teacher
        )
        var environment = ReferenceQuadrotorRLEnvironment(
            parameters: parameters,
            schedule: schedule,
            determinism: .tier1Baseline,
            motorNerveRateLimitPerSecond: 100.0,
            motorNerveSmoothingTimeConstant: nil
        )

        var observation = try environment.reset(seed: definition.config.seed, scenario: definition)
        var final: EnvironmentStep?
        for _ in 0..<100 {
            let action = try await policy.action(for: observation)
            switch action {
            case .driveIntents(let drives, _):
                #expect(drives.count == (definition.kind == .singleLiftHover ? 1 : 4))
            case .actuatorValues(let values):
                #expect(values.count == (definition.kind == .singleLiftHover ? 1 : 4))
            }
            let step = try environment.step(action: action)
            #expect(step.reward.isFinite)
            observation = step.observation
            final = step
            if step.done || step.truncated { break }
        }

        #expect(final?.done == false)
        #expect(final?.truncated == true)
        #expect(final?.info.failureReason == nil)
    }
}

@Test func referenceQuadrotorEnvironmentResetIncludesLiftObservationChannels() async throws {
    let schedule = try SimulationSchedule.baseline(cutPeriodSteps: 1)
    let cases: [(ReferenceQuadrotorScenarioDefinition, Double)] = [
        (
            try makeShortLiftScenario(
                kind: .liftHover,
                id: "KUY-RL-TEST/LIFT-RESET",
                seed: 45,
                initialZ: 2.25,
                targetZ: 2.0
            ),
            2.25
        ),
        (
            try makeShortLiftScenario(
                kind: .singleLiftHover,
                id: "KUY-RL-TEST/SLIFT-RESET",
                seed: 46,
                initialZ: 0.1,
                targetZ: 0.5
            ),
            0.1
        ),
    ]

    for (definition, expectedAltitude) in cases {
        var environment = ReferenceQuadrotorRLEnvironment(
            parameters: try parameters(for: definition.kind),
            schedule: schedule,
            determinism: .tier1Baseline,
            motorNerveRateLimitPerSecond: 100.0,
            motorNerveSmoothingTimeConstant: nil
        )
        let observation = try environment.reset(seed: definition.config.seed, scenario: definition)

        #expect(observation.time.stepIndex == 0)
        #expect(observation.sensorSamples.count == 8)
        #expect(observation.sensorSamples.first { $0.channelIndex == 6 }?.value == expectedAltitude)
        #expect(observation.sensorSamples.first { $0.channelIndex == 7 }?.value == 0.0)
    }
}

@Test func referenceQuadrotorEnvironmentAppliesSingleLiftLatencyStress() async throws {
    let schedule = try SimulationSchedule.baseline(cutPeriodSteps: 1)
    let base = try makeShortLiftScenario(
        kind: .singleLiftHover,
        id: "KUY-RL-TEST/SLIFT-LATENCY",
        seed: 48,
        initialZ: 0.1,
        targetZ: 0.5
    )
    let definition = try scenario(
        from: base,
        hfEvents: [
            try HFStressEvent(kind: .latencySpike, startTime: 0.0, duration: 0.01, magnitude: 1.0),
        ]
    )
    var environment = ReferenceQuadrotorRLEnvironment(
        parameters: try parameters(for: definition.kind),
        schedule: schedule,
        determinism: .tier1Baseline,
        motorNerveRateLimitPerSecond: 100.0,
        motorNerveSmoothingTimeConstant: nil
    )

    let observation = try environment.reset(seed: definition.config.seed, scenario: definition)
    #expect(observation.sensorSamples.isEmpty)
}

@Test func referenceQuadrotorEnvironmentRejectsSingleLiftTorqueStress() async throws {
    let schedule = try SimulationSchedule.baseline(cutPeriodSteps: 1)
    let base = try makeShortLiftScenario(
        kind: .singleLiftHover,
        id: "KUY-RL-TEST/SLIFT-TORQUE",
        seed: 49,
        initialZ: 0.1,
        targetZ: 0.5
    )
    let definition = try scenario(
        from: base,
        torqueEvents: [
            try TorqueDisturbanceEvent(
                startTime: 0.0,
                duration: 0.01,
                torqueBody: Axis3(x: 0.0002, y: 0, z: 0)
            ),
        ]
    )
    var environment = ReferenceQuadrotorRLEnvironment(
        parameters: try parameters(for: definition.kind),
        schedule: schedule,
        determinism: .tier1Baseline,
        motorNerveRateLimitPerSecond: 100.0,
        motorNerveSmoothingTimeConstant: nil
    )

    #expect(throws: ReferenceQuadrotorRLEnvironment.EnvironmentError.self) {
        _ = try environment.reset(seed: definition.config.seed, scenario: definition)
    }
}

@Test func referenceQuadrotorEnvironmentResetAndStepUseSameLiftObservationSchema() async throws {
    let schedule = try SimulationSchedule.baseline(cutPeriodSteps: 1)
    let definition = try makeShortLiftScenario(
        kind: .liftHover,
        id: "KUY-RL-TEST/LIFT-SCHEMA",
        seed: 47,
        initialZ: 2.25,
        targetZ: 2.0
    )
    var environment = ReferenceQuadrotorRLEnvironment(
        parameters: try parameters(for: definition.kind),
        schedule: schedule,
        determinism: .tier1Baseline,
        motorNerveRateLimitPerSecond: 100.0,
        motorNerveSmoothingTimeConstant: nil
    )

    let resetObservation = try environment.reset(seed: definition.config.seed, scenario: definition)
    let action = try EnvironmentAction.driveIntents([
        DriveIntent(index: DriveIndex(0), activation: 0.5),
        DriveIntent(index: DriveIndex(1), activation: 0.5),
        DriveIntent(index: DriveIndex(2), activation: 0.5),
        DriveIntent(index: DriveIndex(3), activation: 0.5),
    ], corrections: [])
    let step = try environment.step(action: action)
    let stepObservation = step.observation

    #expect(resetObservation.sensorSamples.count == 8)
    #expect(stepObservation.sensorSamples.count == 8)
    #expect(resetObservation.sensorSamples.map(\.channelIndex) == stepObservation.sensorSamples.map(\.channelIndex))
    #expect(resetObservation.sensorSamples.first { $0.channelIndex == 6 }?.value == resetObservation.plantState.root.position.z)
    #expect(resetObservation.sensorSamples.first { $0.channelIndex == 7 }?.value == resetObservation.plantState.root.velocity.z)
    #expect(stepObservation.sensorSamples.first { $0.channelIndex == 6 }?.value == step.log.plantState.root.position.z)
    #expect(stepObservation.sensorSamples.first { $0.channelIndex == 7 }?.value == step.log.plantState.root.velocity.z)
}

@Test func referenceQuadrotorEnvironmentCanBeConstructedFromBundledRobots() throws {
    let schedule = try SimulationSchedule.baseline(cutPeriodSteps: 1)
    let quad = try loadBundledRobot("QuadRef/quadref.kuyurobot.json")
    let single = try loadBundledRobot("SingleProp/singleprop.kuyurobot.json")

    let quadEnvironment = try ReferenceQuadrotorRLEnvironment(
        loadedRobot: quad,
        schedule: schedule,
        determinism: .tier1Baseline
    )
    let singleEnvironment = try ReferenceQuadrotorRLEnvironment(
        loadedRobot: single,
        schedule: schedule,
        determinism: .tier1Baseline
    )

    #expect(quadEnvironment.robotManifestID == "quadref-v0")
    #expect(quadEnvironment.driveCount == 4)
    #expect(quadEnvironment.actuatorCount == 4)
    #expect(singleEnvironment.robotManifestID == "singleprop-v0")
    #expect(singleEnvironment.driveCount == 1)
    #expect(singleEnvironment.actuatorCount == 1)
    #expect(singleEnvironment.parameters.maxThrust == 12.0)
}

@Test func environmentStepCarriesRewardDescriptor() async throws {
    let definition = try makeShortAttitudeScenario()
    let schedule = try SimulationSchedule.baseline(cutPeriodSteps: 1)
    let reward = ReferenceQuadrotorDenseReward(
        config: ReferenceQuadrotorDenseReward.Config(controlPenalty: 0.123)
    )
    var environment = ReferenceQuadrotorRLEnvironment(
        schedule: schedule,
        determinism: .tier1Baseline,
        motorNerveRateLimitPerSecond: 100.0,
        motorNerveSmoothingTimeConstant: nil,
        rewardFunction: reward
    )
    let observation = try environment.reset(seed: definition.config.seed, scenario: definition)
    let drive = try DriveIntent(index: DriveIndex(0), activation: 0.5)
    let step = try environment.step(action: .driveIntents([drive], corrections: []))

    #expect(observation.time.stepIndex == 0)
    #expect(step.info.rewardDescriptor == reward.descriptor)
}

@Test func denseRewardPenalizesAttitudeAltitudeErrorWithoutLiftEnvelope() throws {
    let definition = try makeShortAttitudeScenario()
    let reward = ReferenceQuadrotorDenseReward(
        config: ReferenceQuadrotorDenseReward.Config(verticalVelocityPenalty: 0)
    )
    let targetLog = try makeRewardLog(altitude: definition.initialPosition.z, verticalVelocity: 0)
    let lowLog = try makeRewardLog(altitude: definition.initialPosition.z - 0.1, verticalVelocity: 0)

    let targetReward = try reward.reward(
        scenario: definition,
        log: targetLog,
        failure: nil,
        truncated: false
    )
    let lowReward = try reward.reward(
        scenario: definition,
        log: lowLog,
        failure: nil,
        truncated: false
    )

    #expect(definition.liftEnvelope == nil)
    #expect(reward.descriptor.version == "5")
    #expect(lowReward < targetReward)
}

@Test func denseRewardStaysWithinSurvivalBudgetSoContinuingBeatsTerminating() throws {
    let definition = try makeShortAttitudeScenario()
    let reward = ReferenceQuadrotorDenseReward()
    let hoverLog = try makeRewardLog(altitude: definition.initialPosition.z, verticalVelocity: 0)
    // Saturates every normalized penalty term: 90-degree tilt is the tilt
    // normalization boundary, and the altitude/velocity errors are far past
    // their references.
    let worstLog = try makeRewardLog(
        altitude: definition.initialPosition.z - 100.0,
        verticalVelocity: -100.0,
        tiltRadians: .pi / 2.0,
        omegaMagnitude: 100.0,
        driveActivation: 1.0
    )

    let hoverReward = try reward.reward(
        scenario: definition,
        log: hoverLog,
        failure: nil,
        truncated: false
    )
    let worstReward = try reward.reward(
        scenario: definition,
        log: worstLog,
        failure: nil,
        truncated: false
    )

    // A terminated episode bootstraps to zero. Every non-failing step must be
    // worth at least that, or the optimal policy is to terminate immediately.
    #expect(worstReward >= 0)
    #expect(hoverReward <= reward.config.survivalReward)
    #expect(worstReward < hoverReward)
    #expect(abs(hoverReward - reward.config.survivalReward) < 1e-9)
    #expect(abs(worstReward) < 1e-9)
}

@Test func denseRewardChargesFailurePenaltyOnTheFailingStep() throws {
    let definition = try makeShortAttitudeScenario()
    let reward = ReferenceQuadrotorDenseReward()
    let log = try makeRewardLog(altitude: definition.initialPosition.z, verticalVelocity: 0)
    let failure = FailureEvent(reason: .groundViolation, time: log.time.time)

    let survived = try reward.reward(scenario: definition, log: log, failure: nil, truncated: false)
    let failed = try reward.reward(scenario: definition, log: log, failure: failure, truncated: false)

    #expect(abs((survived - failed) - reward.config.failurePenalty) < 1e-9)
    #expect(failed < 0)
}

@Test func denseRewardRejectsDegeneratePenaltyWeights() throws {
    let definition = try makeShortAttitudeScenario()
    let reward = ReferenceQuadrotorDenseReward(
        config: ReferenceQuadrotorDenseReward.Config(
            tiltPenalty: 0,
            omegaPenalty: 0,
            altitudePenalty: 0,
            verticalVelocityPenalty: 0,
            controlPenalty: 0
        )
    )
    let log = try makeRewardLog(altitude: definition.initialPosition.z, verticalVelocity: 0)

    #expect(throws: ReferenceQuadrotorDenseReward.RewardError.degeneratePenaltyWeights) {
        _ = try reward.reward(scenario: definition, log: log, failure: nil, truncated: false)
    }
}

@Test func denseRewardRejectsNegativeWeights() throws {
    let definition = try makeShortAttitudeScenario()
    let reward = ReferenceQuadrotorDenseReward(
        config: ReferenceQuadrotorDenseReward.Config(tiltPenalty: -1.0)
    )
    let log = try makeRewardLog(altitude: definition.initialPosition.z, verticalVelocity: 0)

    #expect(throws: ReferenceQuadrotorDenseReward.RewardError.negativeWeight) {
        _ = try reward.reward(scenario: definition, log: log, failure: nil, truncated: false)
    }
}

@Test func altitudeHoldReferenceUsesScenarioAuthority() throws {
    let attitude = try makeShortAttitudeScenario()
    let attitudeReference = try ReferenceQuadrotorAltitudeHoldReference(definition: attitude)
    let lift = try makeShortLiftScenario(
        kind: .liftHover,
        id: "KUY-RL-TEST/LIFT-REFERENCE",
        seed: 45,
        initialZ: 1.0,
        targetZ: 2.5
    )
    let liftReference = try ReferenceQuadrotorAltitudeHoldReference(definition: lift)

    #expect(attitude.liftEnvelope == nil)
    #expect(attitudeReference.targetPosition.x == attitude.initialPosition.x)
    #expect(attitudeReference.targetPosition.y == attitude.initialPosition.y)
    #expect(attitudeReference.targetPosition.z == attitude.initialPosition.z)
    #expect(attitudeReference.tolerance == ReferenceQuadrotorAltitudeHoldReference.attitudeTolerance)
    #expect(
        attitudeReference.referenceVerticalVelocity
            == ReferenceQuadrotorAltitudeHoldReference.attitudeReferenceVerticalVelocity
    )
    #expect(attitudeReference.referenceVerticalVelocity == attitude.safetyEnvelope.fallVelocityThreshold)
    #expect(liftReference.targetPosition.x == lift.initialPosition.x)
    #expect(liftReference.targetPosition.y == lift.initialPosition.y)
    #expect(liftReference.targetPosition.z == 2.5)
    #expect(liftReference.tolerance == lift.liftEnvelope?.tolerance)
    #expect(liftReference.referenceVerticalVelocity == lift.liftEnvelope?.maxVelocity)
}

@Test func referenceQuadrotorEnvironmentValidatesPhysicsOnlyWorldModelAdapter() async throws {
    let definition = try makeShortAttitudeScenario()
    let schedule = try SimulationSchedule.baseline(cutPeriodSteps: 1)
    var environment = ReferenceQuadrotorRLEnvironment(
        schedule: schedule,
        determinism: .tier1Baseline,
        motorNerveRateLimitPerSecond: 100.0,
        motorNerveSmoothingTimeConstant: nil,
        worldModelAdapter: PhysicsOnlyWorldModelAdapter()
    )

    _ = try environment.reset(seed: definition.config.seed, scenario: definition)
    let drive = try DriveIntent(index: DriveIndex(0), activation: 0.5)
    let step = try environment.step(action: .driveIntents([drive], corrections: []))

    #expect(step.reward.isFinite)
    #expect(step.info.stepCount == 1)
}

@Test func referenceQuadrotorEnvironmentRejectsInvalidWorldModelPredictionWithoutAdvancingState() async throws {
    let definition = try makeShortAttitudeScenario()
    let schedule = try SimulationSchedule.baseline(cutPeriodSteps: 1)
    let adapterConfiguration = WorldModelAdapterConfiguration(uncertaintyThreshold: 0.25)
    var environment = ReferenceQuadrotorRLEnvironment(
        schedule: schedule,
        determinism: .tier1Baseline,
        motorNerveRateLimitPerSecond: 100.0,
        motorNerveSmoothingTimeConstant: nil,
        worldModelAdapter: HighUncertaintyWorldModelAdapter(),
        worldModelAdapterConfiguration: adapterConfiguration
    )

    _ = try environment.reset(seed: definition.config.seed, scenario: definition)
    let drive = try DriveIntent(index: DriveIndex(0), activation: 0.5)
    do {
        _ = try environment.step(action: .driveIntents([drive], corrections: []))
        Issue.record("Expected world-model rejection")
    } catch WorldModelAdapterRejection.uncertaintyExceeded(let actual, let limit) {
        #expect(actual == 1.0)
        #expect(limit == adapterConfiguration.uncertaintyThreshold)
    }

    environment.worldModelAdapter = nil
    let step = try environment.step(action: .driveIntents([drive], corrections: []))
    #expect(step.info.stepCount == 1)
}

private func makeShortLiftScenario(
    kind: ReferenceQuadrotorScenarioKind,
    id: String,
    seed: UInt64,
    initialZ: Double,
    targetZ: Double
) throws -> ReferenceQuadrotorScenarioDefinition {
    let timeStep = try TimeStep(delta: 0.002)
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
    let config = try ScenarioConfig(
        id: ScenarioID(id),
        seed: ScenarioSeed(seed),
        duration: 0.02,
        timeStep: timeStep
    )
    return ReferenceQuadrotorScenarioDefinition(
        config: config,
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

private func scenario(
    from definition: ReferenceQuadrotorScenarioDefinition,
    torqueEvents: [TorqueDisturbanceEvent]? = nil,
    swapEvents: [SwapEvent]? = nil,
    hfEvents: [HFStressEvent]? = nil
) throws -> ReferenceQuadrotorScenarioDefinition {
    ReferenceQuadrotorScenarioDefinition(
        config: definition.config,
        kind: definition.kind,
        initialPosition: definition.initialPosition,
        initialAttitude: definition.initialAttitude,
        initialAngularVelocity: definition.initialAngularVelocity,
        safetyEnvelope: definition.safetyEnvelope,
        liftEnvelope: definition.liftEnvelope,
        torqueEvents: torqueEvents ?? definition.torqueEvents,
        actuatorDegradation: definition.actuatorDegradation,
        gyroDriftScale: definition.gyroDriftScale,
        swapEvents: swapEvents ?? definition.swapEvents,
        hfEvents: hfEvents ?? definition.hfEvents
    )
}

private func parameters(for kind: ReferenceQuadrotorScenarioKind) throws -> ReferenceQuadrotorParameters {
    guard kind == .singleLiftHover else { return .baseline }
    return try ReferenceQuadrotorParameters(
        mass: ReferenceQuadrotorParameters.baseline.mass,
        inertia: ReferenceQuadrotorParameters.baseline.inertia,
        armLength: ReferenceQuadrotorParameters.baseline.armLength,
        motorTimeConstant: ReferenceQuadrotorParameters.baseline.motorTimeConstant,
        maxThrust: 12.0,
        yawCoefficient: ReferenceQuadrotorParameters.baseline.yawCoefficient,
        gravity: ReferenceQuadrotorParameters.baseline.gravity,
        aerodynamics: ReferenceQuadrotorParameters.baseline.aerodynamics
    )
}

private func driveIntents(from action: EnvironmentAction) throws -> [DriveIntent] {
    switch action {
    case .driveIntents(let drives, _):
        return drives
    case .actuatorValues:
        throw TestPolicyActionError.expectedDriveIntents
    }
}

private enum TestPolicyActionError: Error {
    case expectedDriveIntents
}

private func loadBundledRobot(_ relativePath: String) throws -> LoadedKuyuRobot {
    let testFile = URL(fileURLWithPath: #filePath)
    let packageRoot = testFile
        .deletingLastPathComponent()
        .deletingLastPathComponent()
        .deletingLastPathComponent()
    let workspaceRoot = packageRoot.deletingLastPathComponent()
    let resourceURL = workspaceRoot
        .appendingPathComponent("kuyu")
        .appendingPathComponent("Sources")
        .appendingPathComponent("KuyuUI")
        .appendingPathComponent("Resources")
        .appendingPathComponent("Models")
        .appendingPathComponent(relativePath)
    return try KuyuModelLoader().loadRobot(path: resourceURL.path)
}

private func makeRewardLog(
    altitude: Double,
    verticalVelocity: Double,
    tiltRadians: Double = 0,
    omegaMagnitude: Double = 0,
    driveActivation: Double = 0
) throws -> WorldStepLog {
    let root = RigidBodySnapshot(
        id: "root",
        position: Axis3(x: 0, y: 0, z: altitude),
        velocity: Axis3(x: 0, y: 0, z: verticalVelocity),
        orientation: QuaternionSnapshot(w: 1, x: 0, y: 0, z: 0),
        angularVelocity: Axis3(x: 0, y: 0, z: 0)
    )
    return WorldStepLog(
        time: try WorldTime(stepIndex: 1, time: 0.001),
        events: [],
        sensorSamples: [],
        driveIntents: [try DriveIntent(index: DriveIndex(0), activation: driveActivation)],
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

private struct HighUncertaintyWorldModelAdapter: WorldModelEnvironmentAdapter {
    private let base = PhysicsOnlyWorldModelAdapter()

    func predict(reference: EnvironmentStep) throws -> WorldModelPrediction {
        try WorldModelPrediction(step: reference, uncertainty: 1.0)
    }

    func validate(predicted: EnvironmentStep, reference: EnvironmentStep) throws -> WorldModelAdapterValidation {
        try base.validate(predicted: predicted, reference: reference)
    }

    func validate(
        prediction: WorldModelPrediction,
        reference: EnvironmentStep,
        configuration: WorldModelAdapterConfiguration
    ) throws -> WorldModelAdapterValidation {
        try base.validate(prediction: prediction, reference: reference, configuration: configuration)
    }

    func accept(
        prediction: WorldModelPrediction,
        reference: EnvironmentStep,
        configuration: WorldModelAdapterConfiguration
    ) throws -> EnvironmentStep {
        try base.accept(prediction: prediction, reference: reference, configuration: configuration)
    }
}
