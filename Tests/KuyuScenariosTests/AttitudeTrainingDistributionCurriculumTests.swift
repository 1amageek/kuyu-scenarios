import Foundation
import Testing
import KuyuCore
import KuyuPhysics
@testable import KuyuScenarios

@Test func curriculumStageMappingCoversProgressRange() throws {
    let curriculum = AttitudeTrainingDistributionCurriculum(
        config: try AttitudeTrainingDistributionCurriculum.Config(stageCount: 3)
    )
    #expect(try curriculum.stage(forProgress: 0) == 0)
    #expect(try curriculum.stage(forProgress: 0.32) == 0)
    #expect(try curriculum.stage(forProgress: 0.5) == 1)
    #expect(try curriculum.stage(forProgress: 0.99) == 2)
    #expect(try curriculum.stage(forProgress: 1.0) == 2)
    // Finite values outside [0, 1] clamp instead of failing: early iterations
    // sit below the curriculum start, mastered runs sit past it.
    #expect(try curriculum.stage(forProgress: -0.5) == 0)
    #expect(try curriculum.stage(forProgress: 7.0) == 2)
}

@Test func curriculumStageMappingRejectsNonFiniteProgress() throws {
    let curriculum = AttitudeTrainingDistributionCurriculum(
        config: try AttitudeTrainingDistributionCurriculum.Config()
    )
    #expect(throws: AttitudeTrainingDistributionCurriculum.StageError.nonFiniteProgress) {
        _ = try curriculum.stage(forProgress: Double.nan)
    }
    #expect(throws: AttitudeTrainingDistributionCurriculum.StageError.nonFiniteProgress) {
        _ = try curriculum.stage(forProgress: Double.infinity)
    }
}

@Test func curriculumTiltBoundsExpandMonotonicallyTowardConfiguredUpperBound() throws {
    let config = try AttitudeTrainingDistributionCurriculum.Config(
        stageCount: 3,
        tiltLowerBoundDegrees: 10,
        tiltUpperBoundDegrees: 35
    )
    let curriculum = AttitudeTrainingDistributionCurriculum(config: config)
    var previousUpper = config.tiltLowerBoundDegrees
    for stage in 0..<config.stageCount {
        let upper = try curriculum.tiltUpperBoundDegrees(stage: stage)
        #expect(upper > previousUpper)
        previousUpper = upper
    }
    #expect(previousUpper == config.tiltUpperBoundDegrees)
}

@Test func curriculumScenariosRespectStageTiltBoundAndPinnedDuration() throws {
    let config = try AttitudeTrainingDistributionCurriculum.Config(
        stageCount: 3,
        scenariosPerStage: 5,
        episodeDuration: 20.0
    )
    let curriculum = AttitudeTrainingDistributionCurriculum(config: config)
    for stage in 0..<config.stageCount {
        let upperRadians = try curriculum.tiltUpperBoundDegrees(stage: stage) * Double.pi / 180.0
        let lowerRadians = config.tiltLowerBoundDegrees * Double.pi / 180.0
        for definition in try curriculum.definitions(stage: stage) {
            #expect(definition.config.duration == config.episodeDuration)
            #expect(definition.initialAttitude.roll >= lowerRadians - 1e-9)
            #expect(definition.initialAttitude.roll <= upperRadians + 1e-9)
        }
    }
}

@Test func curriculumGenerationIsDeterministic() throws {
    let config = try AttitudeTrainingDistributionCurriculum.Config()
    let first = try AttitudeTrainingDistributionCurriculum(config: config).allDefinitions()
    let second = try AttitudeTrainingDistributionCurriculum(config: config).allDefinitions()
    #expect(first == second)
}

@Test func curriculumDefinitionsAreDisjointFromFixedTrainingSuite() throws {
    let curriculum = AttitudeTrainingDistributionCurriculum(
        config: try AttitudeTrainingDistributionCurriculum.Config()
    )
    let curriculumKeys = Set(try curriculum.allDefinitions().map {
        "\($0.config.id.rawValue)#\($0.config.seed.rawValue)"
    })
    let baseKeys = Set(try KuyAtt1Suite().scenarios().map {
        "\($0.config.id.rawValue)#\($0.config.seed.rawValue)"
    })
    #expect(curriculumKeys.count == curriculum.config.stageCount * curriculum.config.scenariosPerStage)
    #expect(curriculumKeys.isDisjoint(with: baseKeys))
    for key in curriculumKeys {
        #expect(key.hasPrefix("GEN-"))
    }
}

@Test func curriculumAllDefinitionsMatchPerStageDefinitions() throws {
    let curriculum = AttitudeTrainingDistributionCurriculum(
        config: try AttitudeTrainingDistributionCurriculum.Config()
    )
    var flattened: [ReferenceQuadrotorScenarioDefinition] = []
    for stage in 0..<curriculum.config.stageCount {
        flattened.append(contentsOf: try curriculum.definitions(stage: stage))
    }
    #expect(try curriculum.allDefinitions() == flattened)
}

@Test func curriculumRejectsStageOutOfRange() throws {
    let curriculum = AttitudeTrainingDistributionCurriculum(
        config: try AttitudeTrainingDistributionCurriculum.Config(stageCount: 3)
    )
    #expect(throws: AttitudeTrainingDistributionCurriculum.StageError.stageOutOfRange(stage: 3, stageCount: 3)) {
        _ = try curriculum.definitions(stage: 3)
    }
    #expect(throws: AttitudeTrainingDistributionCurriculum.StageError.stageOutOfRange(stage: -1, stageCount: 3)) {
        _ = try curriculum.tiltUpperBoundDegrees(stage: -1)
    }
}

@Test func curriculumConfigValidatesInputs() throws {
    #expect(throws: AttitudeTrainingDistributionCurriculum.ConfigurationError.invalidStageCount(0)) {
        _ = try AttitudeTrainingDistributionCurriculum.Config(stageCount: 0)
    }
    #expect(throws: AttitudeTrainingDistributionCurriculum.ConfigurationError.invalidScenariosPerStage(-1)) {
        _ = try AttitudeTrainingDistributionCurriculum.Config(scenariosPerStage: -1)
    }
    #expect(throws: AttitudeTrainingDistributionCurriculum.ConfigurationError.invalidTiltBounds(lowerDegrees: 40, upperDegrees: 20)) {
        _ = try AttitudeTrainingDistributionCurriculum.Config(
            tiltLowerBoundDegrees: 40,
            tiltUpperBoundDegrees: 20
        )
    }
    #expect(throws: AttitudeTrainingDistributionCurriculum.ConfigurationError.invalidTiltBounds(lowerDegrees: 10, upperDegrees: 95)) {
        _ = try AttitudeTrainingDistributionCurriculum.Config(tiltUpperBoundDegrees: 95)
    }
    #expect(throws: AttitudeTrainingDistributionCurriculum.ConfigurationError.invalidTorqueMagnitudeUpperBound(-0.1)) {
        _ = try AttitudeTrainingDistributionCurriculum.Config(torqueMagnitudeUpperBound: -0.1)
    }
    #expect(throws: AttitudeTrainingDistributionCurriculum.ConfigurationError.invalidEpisodeDuration(0)) {
        _ = try AttitudeTrainingDistributionCurriculum.Config(episodeDuration: 0)
    }
}
