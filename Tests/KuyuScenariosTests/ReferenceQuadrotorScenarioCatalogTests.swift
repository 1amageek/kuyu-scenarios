import Testing
@testable import KuyuScenarios

@Test func referenceQuadrotorScenarioCatalogResolvesA1AttitudeSuite() throws {
    let definitions = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .attitude,
        suite: 2,
        episodeCount: 2
    )

    #expect(definitions.count == 2)
    #expect(definitions.map(\.config.id.rawValue).allSatisfy { $0.hasPrefix("KUY-A1/Suite-2/") })
    #expect(definitions.map(\.config.seed.rawValue) == [42_000, 42_001])
    #expect(ReferenceQuadrotorScenarioCatalog.trackName(forSuite: 2) == "Suite-2")
}

@Test func referenceQuadrotorScenarioCatalogResolvesLongHorizonAttitudeSuite() throws {
    let definitions = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .attitude,
        suite: 7,
        episodeCount: 2
    )

    #expect(definitions.count == 2)
    #expect(definitions.map(\.config.id.rawValue).allSatisfy { $0.hasPrefix("LH-TRANSFER-") })
    #expect(ReferenceQuadrotorScenarioCatalog.trackName(forSuite: 7) == "morphologyTransfer")
}

@Test func referenceQuadrotorScenarioCatalogResolvesLiftSuiteVariations() throws {
    let base = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .lift,
        suite: 6,
        episodeCount: 1
    )
    let shifted = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .lift,
        suite: 7,
        episodeCount: 1
    )
    let stressed = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .lift,
        suite: 8,
        episodeCount: 1
    )

    let baseEnvelope = try #require(base.first?.liftEnvelope)
    let shiftedEnvelope = try #require(shifted.first?.liftEnvelope)
    let stressedEnvelope = try #require(stressed.first?.liftEnvelope)
    #expect(base.first?.config.id.rawValue == "KUY-LIFT-M2-S6/SCN-1")
    #expect(shifted.first?.config.id.rawValue == "KUY-LIFT-M2-S7/SCN-1")
    #expect(stressed.first?.config.id.rawValue == "KUY-LIFT-M2-S8/SCN-1")
    #expect(shiftedEnvelope.targetZ == baseEnvelope.targetZ + 0.05)
    #expect(stressedEnvelope.targetZ == baseEnvelope.targetZ - 0.02)
    #expect(stressed.first?.hfEvents.isEmpty == false)
}

@Test func referenceQuadrotorScenarioCatalogResolvesSingleLiftSuiteVariations() throws {
    let base = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .singleLift,
        suite: 6,
        episodeCount: 1
    )
    let shifted = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .singleLift,
        suite: 7,
        episodeCount: 1
    )
    let stressed = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .singleLift,
        suite: 8,
        episodeCount: 1
    )

    let baseEnvelope = try #require(base.first?.liftEnvelope)
    let shiftedEnvelope = try #require(shifted.first?.liftEnvelope)
    let stressedEnvelope = try #require(stressed.first?.liftEnvelope)
    #expect(base.first?.config.id.rawValue == "KUY-SLIFT-M2-S6/SCN-1")
    #expect(shifted.first?.config.id.rawValue == "KUY-SLIFT-M2-S7/SCN-1")
    #expect(stressed.first?.config.id.rawValue == "KUY-SLIFT-M2-S8/SCN-1")
    #expect(shiftedEnvelope.targetZ == baseEnvelope.targetZ + 0.02)
    #expect(stressedEnvelope.targetZ == baseEnvelope.targetZ - 0.01)
    #expect(stressed.first?.hfEvents.isEmpty == false)
}

@Test func referenceQuadrotorScenarioCatalogRejectsInvalidRequests() {
    #expect(throws: ReferenceQuadrotorScenarioCatalog.ResolutionError.invalidEpisodeCount(0)) {
        _ = try ReferenceQuadrotorScenarioCatalog.scenarios(
            for: .lift,
            suite: 6,
            episodeCount: 0
        )
    }
    #expect(throws: ReferenceQuadrotorScenarioCatalog.ResolutionError.unsupportedSuite(
        task: "Lift",
        suite: 99
    )) {
        _ = try ReferenceQuadrotorScenarioCatalog.scenarios(
            for: .lift,
            suite: 99,
            episodeCount: 1
        )
    }
}
