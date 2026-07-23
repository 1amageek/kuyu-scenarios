import KuyuPhysics

public extension LongHorizonBenchmarkSuite {
    func validatedReferenceM2TrackCounts() throws -> [LongHorizonBenchmarkTrack: Int] {
        let counts = try balancedReferenceM2TrackCounts()
        for benchmarkCase in cases {
            try validateLongHorizonDuration(benchmarkCase.definition)
            try validateTrackSemantics(benchmarkCase)
        }
        return counts
    }
}

extension LongHorizonBenchmarkSuite {
    static func morphologyTransferContract(
        index: Int
    ) throws -> LongHorizonMorphologyTransferContract {
        let base = ReferenceQuadrotorParameters.baseline
        let targetMassScale = 1.08 + Double(index % 3) * 0.02
        let targetArmScale = 0.94 + Double(index % 2) * 0.04
        return try LongHorizonMorphologyTransferContract(
            sourceRobotID: "reference-quadrotor.baseline",
            targetRobotID: "reference-quadrotor.m2-transfer-\(index)",
            sourceReadiness: .dynamicSimulation,
            targetReadiness: .dynamicSimulation,
            parameterDeltas: [
                try LongHorizonMorphologyTransferContract.ParameterDelta(
                    name: "mass",
                    sourceValue: base.mass,
                    targetValue: base.mass * targetMassScale
                ),
                try LongHorizonMorphologyTransferContract.ParameterDelta(
                    name: "armLength",
                    sourceValue: base.armLength,
                    targetValue: base.armLength * targetArmScale
                ),
                try LongHorizonMorphologyTransferContract.ParameterDelta(
                    name: "inertia.x",
                    sourceValue: base.inertia.x,
                    targetValue: base.inertia.x * targetMassScale
                ),
            ]
        )
    }

    private func validateLongHorizonDuration(
        _ definition: ReferenceQuadrotorScenarioDefinition
    ) throws {
        let duration = definition.config.duration
        guard duration.isFinite else {
            throw ValidationError.nonFiniteBenchmarkDuration(definition.config.id.rawValue)
        }
        guard duration >= Self.minimumLongHorizonDurationSeconds else {
            throw ValidationError.nonLongHorizonScenario(
                scenarioID: definition.config.id.rawValue,
                duration: duration
            )
        }
    }

    private func validateTrackSemantics(
        _ benchmarkCase: LongHorizonBenchmarkCase
    ) throws {
        let definition = benchmarkCase.definition
        switch benchmarkCase.track {
        case .longHorizonTask:
            try validateNoMorphologyContract(benchmarkCase)
        case .morphologyTransfer:
            guard benchmarkCase.morphologyTransfer != nil else {
                throw ValidationError.missingMorphologyTransferContract(definition.config.id.rawValue)
            }
        case .disturbanceDelayPartialObservability:
            try validateNoMorphologyContract(benchmarkCase)
            guard !definition.torqueEvents.isEmpty else {
                throw ValidationError.missingDisturbanceEvidence(definition.config.id.rawValue)
            }
            guard definition.hfEvents.contains(where: { $0.kind == .latencySpike }) else {
                throw ValidationError.missingLatencyEvidence(definition.config.id.rawValue)
            }
            guard Self.hasPartialObservabilityEvidence(definition) else {
                throw ValidationError.missingPartialObservabilityEvidence(definition.config.id.rawValue)
            }
        }
    }

    private func validateNoMorphologyContract(
        _ benchmarkCase: LongHorizonBenchmarkCase
    ) throws {
        guard benchmarkCase.morphologyTransfer == nil else {
            throw ValidationError.unexpectedMorphologyTransferContract(
                track: benchmarkCase.track,
                scenarioID: benchmarkCase.definition.config.id.rawValue
            )
        }
    }

    private static func hasPartialObservabilityEvidence(
        _ definition: ReferenceQuadrotorScenarioDefinition
    ) -> Bool {
        if abs(definition.gyroDriftScale - 1.0) > 1e-12 {
            return true
        }
        if definition.swapEvents.contains(where: { event in
            if case .sensor = event { return true }
            return false
        }) {
            return true
        }
        return definition.hfEvents.contains { $0.kind == .sensorGlitch }
    }
}
