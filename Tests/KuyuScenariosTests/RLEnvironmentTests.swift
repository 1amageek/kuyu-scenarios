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

@Test func referenceQuadrotorEnvironmentCanBeConstructedFromBundledDescriptors() throws {
    let schedule = try SimulationSchedule.baseline(cutPeriodSteps: 1)
    let quad = try loadBundledDescriptor("QuadRef/quadref.model.json")
    let single = try loadBundledDescriptor("SingleProp/singleprop.model.json")

    let quadEnvironment = try ReferenceQuadrotorRLEnvironment(
        loadedDescriptor: quad,
        schedule: schedule,
        determinism: .tier1Baseline
    )
    let singleEnvironment = try ReferenceQuadrotorRLEnvironment(
        loadedDescriptor: single,
        schedule: schedule,
        determinism: .tier1Baseline
    )

    #expect(quadEnvironment.descriptorId == "quadref-v0")
    #expect(quadEnvironment.driveCount == 4)
    #expect(quadEnvironment.actuatorCount == 4)
    #expect(singleEnvironment.descriptorId == "singleprop-v0")
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
    var environment = ReferenceQuadrotorRLEnvironment(
        schedule: schedule,
        determinism: .tier1Baseline,
        motorNerveRateLimitPerSecond: 100.0,
        motorNerveSmoothingTimeConstant: nil,
        worldModelAdapter: HighUncertaintyWorldModelAdapter()
    )

    _ = try environment.reset(seed: definition.config.seed, scenario: definition)
    let drive = try DriveIntent(index: DriveIndex(0), activation: 0.5)
    do {
        _ = try environment.step(action: .driveIntents([drive], corrections: []))
        Issue.record("Expected world-model rejection")
    } catch WorldModelAdapterRejection.uncertaintyExceeded(let actual, let limit) {
        #expect(actual == 1.0)
        #expect(limit == 0.0)
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

private func loadBundledDescriptor(_ relativePath: String) throws -> LoadedRobotDescriptor {
    let path = "../kuyu/Sources/KuyuUI/Resources/Models/\(relativePath)"
    return try RobotDescriptorLoader().loadDescriptor(path: path)
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
