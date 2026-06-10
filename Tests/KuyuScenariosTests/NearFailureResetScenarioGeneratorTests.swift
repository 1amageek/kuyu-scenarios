import Foundation
import Testing
import KuyuCore
import KuyuPhysics
@testable import KuyuScenarios

private func makeTemplate() throws -> ReferenceQuadrotorScenarioDefinition {
    let config = try ScenarioConfig(
        id: ScenarioID("NFR-TEMPLATE/SCN-0"),
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

private func tiltRadians(of attitude: EulerAngles) -> Double {
    acos(min(max(cos(attitude.roll) * cos(attitude.pitch), -1.0), 1.0))
}

private func omegaMagnitude(of omega: Axis3) -> Double {
    (omega.x * omega.x + omega.y * omega.y + omega.z * omega.z).squareRoot()
}

@Test func nearFailureResetGenerationIsDeterministic() throws {
    let template = try makeTemplate()
    let generator = NearFailureResetScenarioGenerator(
        template: template,
        space: try NearFailureResetScenarioGenerator.ResetStateSpace()
    )
    let first = try generator.generate(count: 16, baseSeed: 42)
    let second = try generator.generate(count: 16, baseSeed: 42)
    #expect(first == second)
}

@Test func nearFailureResetStatesStayInsideEnvelopeAndSpace() throws {
    let template = try makeTemplate()
    let space = try NearFailureResetScenarioGenerator.ResetStateSpace(
        tiltFractionRange: 0.6...0.95,
        omegaFractionRange: 0.3...0.8
    )
    let generator = NearFailureResetScenarioGenerator(template: template, space: space)
    let definitions = try generator.generate(count: 64, baseSeed: 7)
    let envelope = template.safetyEnvelope
    let tiltLimit = envelope.tiltSafeMaxDegrees * Double.pi / 180.0

    for definition in definitions {
        let tilt = tiltRadians(of: definition.initialAttitude)
        let omega = omegaMagnitude(of: definition.initialAngularVelocity)
        #expect(tilt < tiltLimit)
        #expect(omega < envelope.omegaSafeMax)

        let tiltFraction = tilt / tiltLimit
        let omegaFraction = omega / envelope.omegaSafeMax
        #expect(tiltFraction >= space.tiltFractionRange.lowerBound - 1e-9)
        #expect(tiltFraction <= space.tiltFractionRange.upperBound + 1e-9)
        #expect(omegaFraction >= space.omegaFractionRange.lowerBound - 1e-9)
        #expect(omegaFraction <= space.omegaFractionRange.upperBound + 1e-9)
    }
}

@Test func nearFailureResetSamplesDiverseTiltDirections() throws {
    let template = try makeTemplate()
    let generator = NearFailureResetScenarioGenerator(
        template: template,
        space: try NearFailureResetScenarioGenerator.ResetStateSpace()
    )
    let definitions = try generator.generate(count: 64, baseSeed: 11)
    #expect(definitions.contains { $0.initialAttitude.roll > 1e-3 })
    #expect(definitions.contains { $0.initialAttitude.roll < -1e-3 })
    #expect(definitions.contains { $0.initialAttitude.pitch > 1e-3 })
    #expect(definitions.contains { $0.initialAttitude.pitch < -1e-3 })
    #expect(definitions.contains { $0.initialAngularVelocity.z > 1e-3 })
    #expect(definitions.contains { $0.initialAngularVelocity.z < -1e-3 })
}

@Test func nearFailureResetCurriculumWidensTowardEnvelope() throws {
    let template = try makeTemplate()
    let space = try NearFailureResetScenarioGenerator.ResetStateSpace(
        tiltFractionRange: 0.3...0.9,
        omegaFractionRange: 0.1...0.8
    )
    let generator = NearFailureResetScenarioGenerator(template: template, space: space)
    let levels = 4
    let curriculum = try generator.generateCurriculum(
        levels: levels,
        scenariosPerLevel: 32,
        baseSeed: 99
    )
    #expect(curriculum.count == levels)

    let envelope = template.safetyEnvelope
    let tiltLimit = envelope.tiltSafeMaxDegrees * Double.pi / 180.0
    let tiltWidth = space.tiltFractionRange.upperBound - space.tiltFractionRange.lowerBound

    for (level, definitions) in curriculum.enumerated() {
        #expect(definitions.count == 32)
        let levelFraction = Double(level + 1) / Double(levels)
        let levelUpperBound = space.tiltFractionRange.lowerBound + tiltWidth * levelFraction
        for definition in definitions {
            let tiltFraction = tiltRadians(of: definition.initialAttitude) / tiltLimit
            #expect(tiltFraction >= space.tiltFractionRange.lowerBound - 1e-9)
            #expect(tiltFraction <= levelUpperBound + 1e-9)
        }
    }

    // The final level must actually reach beyond the first level's ceiling,
    // otherwise the curriculum never approaches the envelope.
    let firstLevelCeiling = space.tiltFractionRange.lowerBound + tiltWidth / Double(levels)
    let lastLevelMaxTilt = curriculum[levels - 1]
        .map { tiltRadians(of: $0.initialAttitude) / tiltLimit }
        .max() ?? 0
    #expect(lastLevelMaxTilt > firstLevelCeiling)
}

@Test func nearFailureResetPreservesTemplateFields() throws {
    let template = try makeTemplate()
    let generator = NearFailureResetScenarioGenerator(
        template: template,
        space: try NearFailureResetScenarioGenerator.ResetStateSpace()
    )
    let definitions = try generator.generate(count: 8, baseSeed: 3)
    for definition in definitions {
        #expect(definition.kind == template.kind)
        #expect(definition.initialPosition == template.initialPosition)
        #expect(definition.safetyEnvelope == template.safetyEnvelope)
        #expect(definition.torqueEvents.isEmpty)
        #expect(definition.actuatorDegradation == nil)
        #expect(definition.swapEvents.isEmpty)
        #expect(definition.hfEvents.isEmpty)
        #expect(definition.config.duration == template.config.duration)
        #expect(definition.config.timeStep == template.config.timeStep)
    }
    let identifiers = Set(definitions.map { $0.config.id.rawValue })
    #expect(identifiers.count == definitions.count)
}

@Test func nearFailureResetRejectsInvalidInputs() throws {
    let template = try makeTemplate()
    let generator = NearFailureResetScenarioGenerator(
        template: template,
        space: try NearFailureResetScenarioGenerator.ResetStateSpace()
    )
    #expect(throws: NearFailureResetScenarioGenerator.GeneratorError.invalidCount) {
        _ = try generator.generate(count: 0, baseSeed: 1)
    }
    #expect(throws: NearFailureResetScenarioGenerator.GeneratorError.invalidCount) {
        _ = try generator.generateCurriculum(levels: 0, scenariosPerLevel: 4, baseSeed: 1)
    }
    #expect(throws: NearFailureResetScenarioGenerator.ResetStateSpace.ValidationError.outOfBounds("tiltFractionRange")) {
        _ = try NearFailureResetScenarioGenerator.ResetStateSpace(tiltFractionRange: 0.5...1.0)
    }
    #expect(throws: NearFailureResetScenarioGenerator.ResetStateSpace.ValidationError.outOfBounds("omegaFractionRange")) {
        _ = try NearFailureResetScenarioGenerator.ResetStateSpace(omegaFractionRange: -0.1...0.5)
    }
}
