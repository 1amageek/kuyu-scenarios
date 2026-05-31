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

private func makeShortAttitudeScenario() throws -> ReferenceQuadrotorScenarioDefinition {
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
        id: ScenarioID("KUY-RL-TEST/ATT"),
        seed: ScenarioSeed(42),
        duration: 0.02,
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
    #expect(reward.descriptor.version == "3")
    #expect(lowReward < targetReward)
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

private func makeRewardLog(altitude: Double, verticalVelocity: Double) throws -> WorldStepLog {
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
