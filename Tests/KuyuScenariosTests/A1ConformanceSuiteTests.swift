import Testing
import KuyuCore
@testable import KuyuScenarios

private func sensorEvents(_ defs: [ReferenceQuadrotorScenarioDefinition]) -> [SensorSwapEvent] {
    defs.flatMap { $0.swapEvents }.compactMap { event in
        if case .sensor(let s) = event { return s }
        return nil
    }
}

private func actuatorEvents(_ defs: [ReferenceQuadrotorScenarioDefinition]) -> [ActuatorSwapEvent] {
    defs.flatMap { $0.swapEvents }.compactMap { event in
        if case .actuator(let a) = event { return a }
        return nil
    }
}

@Test func a1WarmupInjectsNoStress() throws {
    let defs = try A1ConformanceSuite(level: .warmup).scenarios()
    #expect(defs.count == 3)
    #expect(defs.allSatisfy { $0.swapEvents.isEmpty })
    #expect(defs.allSatisfy { $0.hfEvents.isEmpty })
    #expect(defs.allSatisfy { $0.torqueEvents.isEmpty })
    #expect(defs.first?.config.id.rawValue == "KUY-A1/Suite-0/SCN-1")
}

@Test func a1SensorSuiteCoversAllModifiers() throws {
    let defs = try A1ConformanceSuite(level: .sensorSwappability).scenarios()
    let events = sensorEvents(defs)
    #expect(!events.isEmpty)
    // Every A1 sensor modifier dimension must be exercised by at least one event.
    #expect(events.contains { $0.gainScale != 1.0 })
    #expect(events.contains { $0.biasShift != 0.0 })
    #expect(events.contains { $0.noiseScale != 1.0 })
    #expect(events.contains { $0.delayShiftSteps != 0 })
    #expect(events.contains { $0.bandwidthScale < 1.0 })
    #expect(events.contains { $0.dropoutProbability > 0.0 })
}

@Test func a1ActuatorSuiteCoversAllModifiers() throws {
    let defs = try A1ConformanceSuite(level: .actuatorSwappability).scenarios()
    let events = actuatorEvents(defs)
    #expect(!events.isEmpty)
    #expect(events.contains { $0.maxOutputScale != 1.0 })
    #expect(events.contains { $0.lagScale != 1.0 })
    #expect(events.contains { $0.gainScale != 1.0 })
    #expect(events.contains { $0.deadzoneShift != 0.0 })
    #expect(events.contains { $0.rateLimitScale < 1.0 })
    #expect(events.contains { $0.asymmetryScale < 1.0 })
}

@Test func a1ReflexHFSuiteCoversAllHFKinds() throws {
    let defs = try A1ConformanceSuite(level: .reflexHF).scenarios()
    let kinds = Set(defs.flatMap { $0.hfEvents }.map(\.kind))
    #expect(kinds.contains(.impulse))
    #expect(kinds.contains(.vibration))
    #expect(kinds.contains(.sensorGlitch))
    #expect(kinds.contains(.actuatorSaturation))
    #expect(kinds.contains(.latencySpike))
}

@Test func a1BundleGatingInjectsAllChannelShocks() throws {
    let defs = try A1ConformanceSuite(level: .bundleGatingStress).scenarios()
    let events = sensorEvents(defs)
    #expect(!events.isEmpty)
    // Salience/normalization shocks must target every ascending channel simultaneously.
    #expect(events.allSatisfy { Set($0.targetChannels) == Set([0, 1, 2, 3, 4, 5]) })
}

@Test func a1CombinedSuiteMixesEveryStressor() throws {
    let defs = try A1ConformanceSuite(level: .combined).scenarios()
    #expect(!sensorEvents(defs).isEmpty)
    #expect(!actuatorEvents(defs).isEmpty)
    #expect(defs.contains { !$0.hfEvents.isEmpty })
}

@Test func a1AllEventsFallWithinRunDuration() throws {
    for level in A1ConformanceSuite.Level.allCases {
        let suite = A1ConformanceSuite(level: level)
        let defs = try suite.scenarios()
        for def in defs {
            let duration = def.config.duration
            for event in def.swapEvents {
                let window: (Double, Double)
                switch event {
                case .sensor(let s): window = (s.startTime, s.startTime + s.duration)
                case .actuator(let a): window = (a.startTime, a.startTime + a.duration)
                }
                #expect(window.0 >= 0)
                #expect(window.1 <= duration)
            }
            for hf in def.hfEvents {
                #expect(hf.startTime >= 0)
                #expect(hf.startTime + hf.duration <= duration)
            }
        }
    }
}

@Test func a1SuiteIDsMatchTaxonomy() {
    #expect(A1ConformanceSuite.Level.warmup.suiteID == "Suite-0")
    #expect(A1ConformanceSuite.Level.combined.suiteID == "Suite-5")
    #expect(A1ConformanceSuite.Level.allCases.count == 6)
}
