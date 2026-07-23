import Testing
@testable import KuyuScenarios

@Test func screeningProjectionPreservesControlCadenceAndStressSchedule() throws {
    let definition = try #require(
        ReferenceQuadrotorScenarioCatalog.scenarios(
            for: .attitude,
            suite: 8,
            episodeCount: 1
        ).first
    )

    let projected = try ReferenceQuadrotorScenarioScreeningProjector().projected(
        definition,
        maximumControlStepsPerEpisode: 1_000,
        controlPeriodSteps: 2
    )

    #expect(projected.config.duration == 2)
    #expect(projected.config.timeStep == definition.config.timeStep)
    #expect(projected.config.seed == definition.config.seed)
    #expect(projected.config.id != definition.config.id)
    #expect(projected.hfEvents == definition.hfEvents)
    #expect(projected.torqueEvents == definition.torqueEvents)
    #expect(projected.swapEvents == definition.swapEvents)
    #expect(definition.config.duration >= LongHorizonBenchmarkSuite.minimumLongHorizonDurationSeconds)
}

@Test func screeningProjectionIsDeterministicAndDoesNotExpandShortScenarios() throws {
    let definition = try #require(
        ReferenceQuadrotorScenarioCatalog.scenarios(
            for: .lift,
            suite: 6,
            episodeCount: 1
        ).first
    )
    let projector = ReferenceQuadrotorScenarioScreeningProjector()

    let first = try projector.projected(
        definition,
        maximumControlStepsPerEpisode: 500,
        controlPeriodSteps: 2
    )
    let second = try projector.projected(
        definition,
        maximumControlStepsPerEpisode: 500,
        controlPeriodSteps: 2
    )
    let unbounded = try projector.projected(
        definition,
        maximumControlStepsPerEpisode: 10_000,
        controlPeriodSteps: 2
    )

    #expect(first == second)
    #expect(first.liftEnvelope == definition.liftEnvelope)
    #expect(unbounded == definition)
}

@Test func screeningProjectionRejectsInvalidBudgets() throws {
    let definition = try #require(
        ReferenceQuadrotorScenarioCatalog.scenarios(
            for: .attitude,
            suite: 0,
            episodeCount: 1
        ).first
    )
    let projector = ReferenceQuadrotorScenarioScreeningProjector()

    #expect(throws: ReferenceQuadrotorScenarioScreeningProjector.ProjectionError.invalidMaximumControlSteps(0)) {
        _ = try projector.projected(
            definition,
            maximumControlStepsPerEpisode: 0,
            controlPeriodSteps: 2
        )
    }
    #expect(throws: ReferenceQuadrotorScenarioScreeningProjector.ProjectionError.invalidControlPeriodSteps(0)) {
        _ = try projector.projected(
            definition,
            maximumControlStepsPerEpisode: 100,
            controlPeriodSteps: 0
        )
    }
}
