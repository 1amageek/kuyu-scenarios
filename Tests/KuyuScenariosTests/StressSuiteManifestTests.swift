import Foundation
import KuyuCore
import KuyuPhysics
import Testing
@testable import KuyuScenarios

@Test func stressSuiteManifestAcceptsReferenceCoverageWithReplay() throws {
    let definitions = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .attitude,
        suite: 8,
        episodeCount: 2
    )
    let manifest = try StressSuiteManifest.referenceQuadrotor(
        suiteID: "reference-attitude-stress",
        definitions: definitions,
        coverageTargets: [
            try target(.longHorizon, 2),
            try target(.hfLatencySpike, 2),
            try target(.torqueDisturbance, 1),
        ],
        replay: .performed(replayChecks(for: definitions))
    )

    #expect(manifest.profile == .referenceQuadrotor)
    #expect(manifest.replayRequirement == .performedRequired)
    #expect(manifest.replayEvidence.status == .performed)
    #expect(manifest.replayEvidence.checkCount == definitions.count)
    #expect(manifest.coverageCounts[.longHorizon] == 2)
    #expect(manifest.coverageCounts[.hfLatencySpike] == 2)
    #expect(manifest.coverageCounts[.torqueDisturbance] == 2)
}

@Test func stressSuiteManifestDerivesReferenceCoverageTargets() throws {
    let definitions = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .attitude,
        suite: 8,
        episodeCount: 2
    )
    let targets = try StressSuiteManifest.referenceQuadrotorCoverageTargets(for: definitions)
    let dimensions = Set(targets.map(\.dimension))

    #expect(dimensions.contains(.longHorizon))
    #expect(dimensions.contains(.hfLatencySpike))
    #expect(dimensions.contains(.torqueDisturbance))
    #expect(targets.allSatisfy { $0.minimumCount == 1 })
}

@Test func stressSuiteManifestAcceptsReferenceM2BenchmarkCoverage() throws {
    let benchmark = try LongHorizonBenchmarkSuite.makeDefault(
        scenariosPerTrack: 1,
        baseSeed: 60_000
    )
    let definitions = benchmark.cases.map(\.definition)
    let manifest = try StressSuiteManifest.referenceQuadrotorM2Benchmark(
        suiteID: "reference-m2-coverage",
        benchmark: benchmark,
        replay: .performed(replayChecks(for: definitions))
    )

    #expect(manifest.coverageCounts[.longHorizon] == 3)
    #expect(manifest.coverageCounts[.plannerDegradation] == 1)
    #expect(manifest.coverageCounts[.morphologyTransfer] == 1)
    #expect(manifest.coverageCounts[.partialObservability] == 1)
    #expect(manifest.coverageCounts[.torqueDisturbance] == 1)
    #expect(manifest.coverageCounts[.hfLatencySpike] == 1)
    #expect(manifest.coverageTargets.map(\.dimension) == StressSuiteManifest.requiredReferenceQuadrotorM2Dimensions)
    let evidence = try #require(manifest.referenceM2BenchmarkEvidence)
    #expect(evidence.isComplete)
    #expect(evidence.countByTrack[.longHorizonTask] == 1)
    #expect(evidence.countByTrack[.morphologyTransfer] == 1)
    #expect(evidence.countByTrack[.disturbanceDelayPartialObservability] == 1)
    #expect(evidence.plannerDegradationScenarioIDs == ["LH-TASK-0"])
    #expect(evidence.morphologyTransfers.first?.sourceRobotID == "reference-quadrotor.baseline")
    #expect(evidence.morphologyTransfers.first?.parameterDeltas.map(\.name).contains("mass") == true)

    let data = try JSONEncoder().encode(manifest)
    let decoded = try JSONDecoder().decode(StressSuiteManifest.self, from: data)
    #expect(decoded.referenceM2BenchmarkEvidence == evidence)
}

@Test func longHorizonBenchmarkSuiteDefaultCasesCarryReferenceM2Semantics() throws {
    let benchmark = try LongHorizonBenchmarkSuite.makeDefault(
        scenariosPerTrack: 2,
        baseSeed: 60_000
    )
    let counts = try benchmark.validatedReferenceM2TrackCounts()
    let transferCases = benchmark.cases.filter { $0.track == .morphologyTransfer }
    let disturbanceCases = benchmark.cases.filter { $0.track == .disturbanceDelayPartialObservability }

    #expect(counts[.longHorizonTask] == 2)
    #expect(counts[.morphologyTransfer] == 2)
    #expect(counts[.disturbanceDelayPartialObservability] == 2)
    #expect(transferCases.allSatisfy { $0.morphologyTransfer != nil })
    #expect(disturbanceCases.allSatisfy { !$0.definition.torqueEvents.isEmpty })
    #expect(disturbanceCases.allSatisfy { $0.definition.hfEvents.contains(where: { $0.kind == .latencySpike }) })
    #expect(disturbanceCases.allSatisfy { abs($0.definition.gyroDriftScale - 1.0) > 1e-12 })
}

@Test func longHorizonBenchmarkSuiteRejectsInvalidScenariosPerTrack() {
    #expect(throws: LongHorizonBenchmarkSuite.ValidationError.invalidScenariosPerTrack(0)) {
        _ = try LongHorizonBenchmarkSuite.makeDefault(
            scenariosPerTrack: 0,
            baseSeed: 60_000
        )
    }
}

@Test func longHorizonMorphologyTransferContractRejectsTagOnlyTransfer() throws {
    let delta = try LongHorizonMorphologyTransferContract.ParameterDelta(
        name: "mass",
        sourceValue: 1.0,
        targetValue: 1.2
    )

    #expect(throws: LongHorizonMorphologyTransferContract.ValidationError.identicalRobotIDs(
        "reference-quadrotor.baseline"
    )) {
        _ = try LongHorizonMorphologyTransferContract(
            sourceRobotID: "reference-quadrotor.baseline",
            targetRobotID: "reference-quadrotor.baseline",
            sourceReadiness: .dynamicSimulation,
            targetReadiness: .dynamicSimulation,
            parameterDeltas: [delta]
        )
    }
}

@Test func stressSuiteManifestRejectsIncompleteReferenceM2BenchmarkCoverage() throws {
    let definition = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .attitude,
        suite: 6,
        episodeCount: 1
    )[0]
    let benchmark = LongHorizonBenchmarkSuite(
        cases: [
            LongHorizonBenchmarkCase(track: .longHorizonTask, definition: definition),
        ]
    )

    #expect(throws: StressSuiteManifest.ValidationError.invalidReferenceM2Benchmark(
        .missingReferenceM2Track(.morphologyTransfer)
    )) {
        _ = try StressSuiteManifest.referenceQuadrotorM2Benchmark(
            suiteID: "reference-m2-incomplete",
            benchmark: benchmark,
            replay: .performed(replayChecks(for: [definition]))
        )
    }
}

@Test func stressSuiteManifestRejectsImbalancedReferenceM2BenchmarkTracks() throws {
    let balanced = try LongHorizonBenchmarkSuite.makeDefault(
        scenariosPerTrack: 1,
        baseSeed: 60_000
    )
    let extraTaskCase = try LongHorizonBenchmarkSuite.makeDefault(
        scenariosPerTrack: 1,
        baseSeed: 70_000
    ).cases.first { $0.track == .longHorizonTask }
    let benchmark = LongHorizonBenchmarkSuite(cases: balanced.cases + [try #require(extraTaskCase)])
    let definitions = benchmark.cases.map(\.definition)

    #expect(throws: StressSuiteManifest.ValidationError.invalidReferenceM2Benchmark(
        .imbalancedReferenceM2TrackCount(
            track: .longHorizonTask,
            expectedCount: 1,
            actualCount: 2
        )
    )) {
        _ = try StressSuiteManifest.referenceQuadrotorM2Benchmark(
            suiteID: "reference-m2-imbalanced",
            benchmark: benchmark,
            replay: .performed(replayChecks(for: definitions))
        )
    }
}

@Test func stressSuiteManifestRejectsReferenceM2MorphologyTransferWithoutContract() throws {
    let valid = try LongHorizonBenchmarkSuite.makeDefault(
        scenariosPerTrack: 1,
        baseSeed: 60_000
    )
    let cases = valid.cases.map { benchmarkCase in
        if benchmarkCase.track == .morphologyTransfer {
            return LongHorizonBenchmarkCase(
                track: benchmarkCase.track,
                definition: benchmarkCase.definition
            )
        }
        return benchmarkCase
    }
    let benchmark = LongHorizonBenchmarkSuite(cases: cases)
    let definitions = benchmark.cases.map(\.definition)
    let transferID = try #require(
        benchmark.cases.first { $0.track == .morphologyTransfer }?.definition.config.id.rawValue
    )

    #expect(throws: StressSuiteManifest.ValidationError.invalidReferenceM2Benchmark(
        .missingMorphologyTransferContract(transferID)
    )) {
        _ = try StressSuiteManifest.referenceQuadrotorM2Benchmark(
            suiteID: "reference-m2-tag-only-transfer",
            benchmark: benchmark,
            replay: .performed(replayChecks(for: definitions))
        )
    }
}

@Test func stressSuiteManifestRejectsReferenceM2PartialObservabilityWithoutEvidence() throws {
    let valid = try LongHorizonBenchmarkSuite.makeDefault(
        scenariosPerTrack: 1,
        baseSeed: 60_000
    )
    let cases = valid.cases.map { benchmarkCase in
        if benchmarkCase.track == .disturbanceDelayPartialObservability {
            return LongHorizonBenchmarkCase(
                track: benchmarkCase.track,
                definition: definitionWith(
                    benchmarkCase.definition,
                    gyroDriftScale: 1.0,
                    swapEvents: [],
                    hfEvents: benchmarkCase.definition.hfEvents.filter { $0.kind != .sensorGlitch }
                )
            )
        }
        return benchmarkCase
    }
    let benchmark = LongHorizonBenchmarkSuite(cases: cases)
    let definitions = benchmark.cases.map(\.definition)
    let partialID = try #require(
        benchmark.cases.first {
            $0.track == .disturbanceDelayPartialObservability
        }?.definition.config.id.rawValue
    )

    #expect(throws: StressSuiteManifest.ValidationError.invalidReferenceM2Benchmark(
        .missingPartialObservabilityEvidence(partialID)
    )) {
        _ = try StressSuiteManifest.referenceQuadrotorM2Benchmark(
            suiteID: "reference-m2-no-partial-observability",
            benchmark: benchmark,
            replay: .performed(replayChecks(for: definitions))
        )
    }
}

@Test func stressSuiteManifestRejectsReferenceM2PlannerDegradationEvidenceMismatch() throws {
    let valid = try LongHorizonBenchmarkSuite.makeDefault(
        scenariosPerTrack: 1,
        baseSeed: 60_000
    )
    let definitions = valid.cases.map(\.definition)
    let plannerID = try #require(
        valid.cases.first { $0.track == .longHorizonTask }?.definition.config.id.rawValue
    )
    let transferID = try #require(
        valid.cases.first { $0.track == .morphologyTransfer }?.definition.config.id.rawValue
    )
    let evidence = try StressSuiteManifest.ReferenceM2BenchmarkEvidence(
        tracks: try LongHorizonBenchmarkTrack.allCases.map {
            try StressSuiteManifest.ReferenceM2BenchmarkEvidence.TrackEvidence(track: $0, count: 1)
        },
        plannerDegradationScenarioIDs: [transferID],
        morphologyTransfers: [
            try StressSuiteManifest.ReferenceM2BenchmarkEvidence.MorphologyTransferEvidence(
                scenarioID: transferID,
                contract: try #require(
                    valid.cases.first { $0.track == .morphologyTransfer }?.morphologyTransfer
                )
            ),
        ],
        disturbanceScenarioIDs: ["LH-DISTURB-0"],
        latencyScenarioIDs: ["LH-DISTURB-0"],
        partialObservabilityScenarioIDs: ["LH-DISTURB-0"]
    )

    #expect(throws: StressSuiteManifest.ValidationError.invalidReferenceM2BenchmarkEvidence(
        "planner-degradation-record-missing-dimension:\(transferID)"
    )) {
        _ = try StressSuiteManifest(
            suiteID: "reference-m2-planner-evidence-mismatch",
            profile: .referenceQuadrotor,
            records: valid.cases.map { try StressSuiteManifest.referenceRecord(from: $0) },
            coverageTargets: try StressSuiteManifest.referenceQuadrotorM2CoverageTargets(),
            replayRequirement: .performedRequired,
            replay: .performed(replayChecks(for: definitions)),
            referenceM2BenchmarkEvidence: evidence
        )
    }
    #expect(plannerID == "LH-TASK-0")
}

@Test func stressSuiteManifestRejectsReferenceReplaySkip() throws {
    let definitions = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .attitude,
        suite: 8,
        episodeCount: 1
    )

    #expect(throws: StressSuiteManifest.ValidationError.replayNotPerformed("not-run")) {
        _ = try StressSuiteManifest.referenceQuadrotor(
            suiteID: "reference-attitude-stress",
            definitions: definitions,
            coverageTargets: [try target(.hfLatencySpike, 1)],
            replay: .notPerformed(reason: "not-run")
        )
    }
}

@Test func stressSuiteManifestRejectsMissingReplayCheck() throws {
    let definitions = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .attitude,
        suite: 8,
        episodeCount: 2
    )
    let missing = scenarioKey(definitions[1])

    #expect(throws: StressSuiteManifest.ValidationError.missingReplayChecks([missing])) {
        _ = try StressSuiteManifest.referenceQuadrotor(
            suiteID: "reference-attitude-stress",
            definitions: definitions,
            coverageTargets: [try target(.hfLatencySpike, 2)],
            replay: .performed(replayChecks(for: [definitions[0]]))
        )
    }
}

@Test func stressSuiteManifestRejectsUnmetCoverageTarget() throws {
    let definitions = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .attitude,
        suite: 8,
        episodeCount: 2
    )

    #expect(throws: StressSuiteManifest.ValidationError.unmetCoverageTarget(
        dimension: .actuatorDegradation,
        minimumCount: 1,
        actualCount: 0
    )) {
        _ = try StressSuiteManifest.referenceQuadrotor(
            suiteID: "reference-attitude-stress",
            definitions: definitions,
            coverageTargets: [try target(.actuatorDegradation, 1)],
            replay: .performed(replayChecks(for: definitions))
        )
    }
}

@Test func stressSuiteManifestAcceptsArticulatedExplicitReplaySkip() throws {
    let log = try makeArticulatedDynamicLog(
        events: [makeArticulatedStep(index: 0)],
        eventSchedule: StressEventSchedule(
            swapEvents: [],
            hfEvents: [
                try HFStressEvent(
                    kind: .latencySpike,
                    startTime: 0.0,
                    duration: 0.01,
                    magnitude: 0.2
                ),
            ]
        )
    )

    let manifest = try StressSuiteManifest.articulatedDynamic(
        suiteID: "roarm-dynamic-stress",
        logs: [log],
        coverageTargets: [
            try target(.articulatedDynamic, 1),
            try target(.hfLatencySpike, 1),
        ],
        replay: .notPerformed(reason: "articulated dynamic path records explicit replay skip")
    )

    #expect(manifest.profile == .articulatedDynamic)
    #expect(manifest.replayRequirement == .explicitSkipAllowed)
    #expect(manifest.replayEvidence.status == .notPerformed)
    #expect(manifest.coverageCounts[.articulatedDynamic] == 1)
    #expect(manifest.coverageCounts[.hfLatencySpike] == 1)
}

@Test func stressSuiteManifestDecodeRejectsCoverageCountTampering() throws {
    let definitions = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .attitude,
        suite: 8,
        episodeCount: 1
    )
    let manifest = try StressSuiteManifest.referenceQuadrotor(
        suiteID: "reference-attitude-stress",
        definitions: definitions,
        coverageTargets: [try target(.hfLatencySpike, 1)],
        replay: .performed(replayChecks(for: definitions))
    )
    let tampered = TamperedStressSuiteManifestPayload(
        suiteID: manifest.suiteID,
        profile: manifest.profile,
        records: manifest.records,
        coverageTargets: manifest.coverageTargets,
        coverageCounts: [.hfLatencySpike: 0],
        replayRequirement: manifest.replayRequirement,
        replayEvidence: manifest.replayEvidence
    )
    let data = try JSONEncoder().encode(tampered)

    #expect(throws: StressSuiteManifest.ValidationError.decodedCoverageCountMismatch) {
        _ = try JSONDecoder().decode(StressSuiteManifest.self, from: data)
    }
}

@Test func stressSuiteManifestArtifactStoreRoundTripsValidatedManifest() throws {
    let root = temporaryStressManifestRoot("stress-manifest-store-roundtrip")
    defer { removeStressManifestRoot(root) }
    let url = root
        .appendingPathComponent("stress", isDirectory: true)
        .appendingPathComponent("reference-stress.json", isDirectory: false)
    let manifest = try makeReferenceStressManifest()
    let store = StressSuiteManifestArtifactStore()

    let writtenURL = try store.write(manifest, to: url, artifactRoot: root)
    let loaded = try store.validatedManifest(at: writtenURL, artifactRoot: root)

    #expect(writtenURL == url.standardizedFileURL)
    #expect(loaded == manifest)
}

@Test func stressSuiteManifestArtifactStoreRejectsTamperedManifest() throws {
    let root = temporaryStressManifestRoot("stress-manifest-store-tampered")
    defer { removeStressManifestRoot(root) }
    let url = root
        .appendingPathComponent("stress", isDirectory: true)
        .appendingPathComponent("reference-stress.json", isDirectory: false)
    let manifest = try makeReferenceStressManifest()
    _ = try StressSuiteManifestArtifactStore().write(manifest, to: url, artifactRoot: root)
    let tampered = TamperedStressSuiteManifestPayload(
        suiteID: manifest.suiteID,
        profile: manifest.profile,
        records: manifest.records,
        coverageTargets: manifest.coverageTargets,
        coverageCounts: [.hfLatencySpike: 0],
        replayRequirement: manifest.replayRequirement,
        replayEvidence: manifest.replayEvidence
    )
    try JSONEncoder().encode(tampered).write(to: url, options: [.atomic])

    #expect(throws: StressSuiteManifestArtifactStore.StoreError.invalidManifest(url.path)) {
        _ = try StressSuiteManifestArtifactStore().validatedManifest(at: url, artifactRoot: root)
    }
}

@Test func stressSuiteManifestArtifactStoreRejectsSymlinkedManifestEscape() throws {
    let root = temporaryStressManifestRoot("stress-manifest-store-root")
    let externalRoot = temporaryStressManifestRoot("stress-manifest-store-external")
    defer {
        removeStressManifestRoot(root)
        removeStressManifestRoot(externalRoot)
    }
    try FileManager.default.createDirectory(at: root, withIntermediateDirectories: true)
    try FileManager.default.createDirectory(at: externalRoot, withIntermediateDirectories: true)
    let symlinkURL = root.appendingPathComponent("stress", isDirectory: true)
    try FileManager.default.createSymbolicLink(at: symlinkURL, withDestinationURL: externalRoot)
    let outputURL = symlinkURL.appendingPathComponent("reference-stress.json", isDirectory: false)
    let escapedPath = externalRoot
        .appendingPathComponent("reference-stress.json", isDirectory: false)
        .standardizedFileURL
        .path

    #expect(throws: StressSuiteManifestArtifactStore.StoreError.manifestEscapesArtifactRoot(escapedPath)) {
        _ = try StressSuiteManifestArtifactStore().write(
            makeReferenceStressManifest(),
            to: outputURL,
            artifactRoot: root
        )
    }
}

@Test func stressSuiteManifestArtifactStoreRejectsSymlinkedManifestReadEscape() throws {
    let root = temporaryStressManifestRoot("stress-manifest-store-read-root")
    let externalRoot = temporaryStressManifestRoot("stress-manifest-store-read-external")
    defer {
        removeStressManifestRoot(root)
        removeStressManifestRoot(externalRoot)
    }
    try FileManager.default.createDirectory(at: root, withIntermediateDirectories: true)
    try FileManager.default.createDirectory(at: externalRoot, withIntermediateDirectories: true)
    let symlinkURL = root.appendingPathComponent("stress", isDirectory: true)
    try FileManager.default.createSymbolicLink(at: symlinkURL, withDestinationURL: externalRoot)
    let externalManifestURL = externalRoot.appendingPathComponent(
        "reference-stress.json",
        isDirectory: false
    )
    _ = try StressSuiteManifestArtifactStore().write(
        makeReferenceStressManifest(),
        to: externalManifestURL,
        artifactRoot: externalRoot
    )
    let symlinkedManifestURL = symlinkURL.appendingPathComponent(
        "reference-stress.json",
        isDirectory: false
    )
    let escapedPath = externalManifestURL.standardizedFileURL.path

    #expect(throws: StressSuiteManifestArtifactStore.StoreError.manifestEscapesArtifactRoot(escapedPath)) {
        _ = try StressSuiteManifestArtifactStore().validatedManifest(
            at: symlinkedManifestURL,
            artifactRoot: root
        )
    }
}

private func target(
    _ dimension: StressSuiteManifest.StressDimension,
    _ minimumCount: Int
) throws -> StressSuiteManifest.CoverageTarget {
    try StressSuiteManifest.CoverageTarget(dimension: dimension, minimumCount: minimumCount)
}

private func makeReferenceStressManifest() throws -> StressSuiteManifest {
    let definitions = try ReferenceQuadrotorScenarioCatalog.scenarios(
        for: .attitude,
        suite: 8,
        episodeCount: 1
    )
    return try StressSuiteManifest.referenceQuadrotor(
        suiteID: "reference-attitude-stress",
        definitions: definitions,
        coverageTargets: [try target(.hfLatencySpike, 1)],
        replay: .performed(replayChecks(for: definitions))
    )
}

private func temporaryStressManifestRoot(_ prefix: String) -> URL {
    FileManager.default.temporaryDirectory
        .appendingPathComponent("\(prefix)-\(UUID().uuidString)", isDirectory: true)
}

private func removeStressManifestRoot(_ url: URL) {
    guard FileManager.default.fileExists(atPath: url.path) else { return }
    do {
        try FileManager.default.removeItem(at: url)
    } catch {
        Issue.record("Failed to remove temporary stress manifest root \(url.path): \(error)")
    }
}

private func replayChecks(
    for definitions: [ReferenceQuadrotorScenarioDefinition]
) -> [ReplayCheckResult] {
    definitions.map { definition in
        ReplayCheckResult(
            scenarioId: definition.config.id,
            seed: definition.config.seed,
            tier: .tier1,
            passed: true,
            issues: [],
            residuals: .zero
        )
    }
}

private func scenarioKey(_ definition: ReferenceQuadrotorScenarioDefinition) -> String {
    "\(definition.config.id.rawValue):\(definition.config.seed.rawValue)"
}

private func definitionWith(
    _ definition: ReferenceQuadrotorScenarioDefinition,
    gyroDriftScale: Double,
    swapEvents: [SwapEvent],
    hfEvents: [HFStressEvent]
) -> ReferenceQuadrotorScenarioDefinition {
    ReferenceQuadrotorScenarioDefinition(
        config: definition.config,
        kind: definition.kind,
        initialPosition: definition.initialPosition,
        initialAttitude: definition.initialAttitude,
        initialAngularVelocity: definition.initialAngularVelocity,
        safetyEnvelope: definition.safetyEnvelope,
        liftEnvelope: definition.liftEnvelope,
        torqueEvents: definition.torqueEvents,
        actuatorDegradation: definition.actuatorDegradation,
        gyroDriftScale: gyroDriftScale,
        swapEvents: swapEvents,
        hfEvents: hfEvents
    )
}

private struct TamperedStressSuiteManifestPayload: Encodable {
    let suiteID: String
    let profile: StressSuiteManifest.Profile
    let records: [StressSuiteManifest.ScenarioRecord]
    let coverageTargets: [StressSuiteManifest.CoverageTarget]
    let coverageCounts: [StressSuiteManifest.StressDimension: Int]
    let replayRequirement: StressSuiteManifest.ReplayRequirement
    let replayEvidence: StressSuiteManifest.ReplayEvidence
}

private func makeArticulatedDynamicLog(
    events: [WorldStepLog],
    eventSchedule: StressEventSchedule? = nil
) throws -> SimulationLog {
    SimulationLog(
        scenarioId: try ScenarioID("ROARM-DYN-STRESS-1"),
        seed: ScenarioSeed(91),
        timeStep: try TimeStep(delta: 0.01),
        determinism: .tier1Baseline,
        configHash: "articulated-stress-test",
        events: events,
        eventSchedule: eventSchedule
    )
}

private func makeArticulatedStep(index: UInt64) throws -> WorldStepLog {
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
        safetyTrace: try SafetyTrace(omegaMagnitude: 0.0, tiltRadians: 0.0),
        plantState: PlantStateSnapshot(root: root),
        disturbances: DisturbanceSnapshot(
            forceWorld: Axis3(x: 0, y: 0, z: 0),
            torqueBody: Axis3(x: 0, y: 0, z: 0)
        )
    )
}
