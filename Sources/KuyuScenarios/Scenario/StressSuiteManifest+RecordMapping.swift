import Foundation
import KuyuCore
import KuyuPhysics

extension StressSuiteManifest {
    static func referenceRecord(
        from definition: ReferenceQuadrotorScenarioDefinition
    ) throws -> ScenarioRecord {
        try referenceRecord(from: definition, additionalDimensions: [])
    }

    static func referenceRecord(
        from benchmarkCase: LongHorizonBenchmarkCase
    ) throws -> ScenarioRecord {
        try referenceRecord(
            from: benchmarkCase.definition,
            additionalDimensions: dimensions(from: benchmarkCase.track)
        )
    }

    static func referenceRecord(
        from definition: ReferenceQuadrotorScenarioDefinition,
        additionalDimensions: [StressDimension]
    ) throws -> ScenarioRecord {
        do {
            return try ScenarioRecord(
                scenarioID: definition.config.id.rawValue,
                seed: definition.config.seed.rawValue,
                duration: definition.config.duration,
                timeStep: definition.config.timeStep,
                configHash: nil,
                dimensions: dimensions(from: definition) + additionalDimensions
            )
        } catch let error as ScenarioRecord.ValidationError {
            throw ValidationError.invalidScenarioRecord(error)
        }
    }

    static func articulatedRecord(from log: SimulationLog) throws -> ScenarioRecord {
        guard !log.events.isEmpty else {
            throw ValidationError.emptySimulationLog(log.scenarioId.rawValue)
        }
        do {
            return try ScenarioRecord(
                scenarioID: log.scenarioId.rawValue,
                seed: log.seed.rawValue,
                duration: Double(log.events.count) * log.timeStep.delta,
                timeStep: log.timeStep,
                configHash: log.configHash,
                dimensions: dimensions(from: log),
                failureReason: log.failureReason
            )
        } catch let error as ScenarioRecord.ValidationError {
            throw ValidationError.invalidScenarioRecord(error)
        }
    }

    static func dimensions(
        from definition: ReferenceQuadrotorScenarioDefinition
    ) -> [StressDimension] {
        var dimensions: [StressDimension] = []
        let attitude = definition.initialAttitude
        if abs(attitude.roll) > 1e-12 || abs(attitude.pitch) > 1e-12 || abs(attitude.yaw) > 1e-12 {
            dimensions.append(.initialAttitude)
        }
        let angularVelocity = definition.initialAngularVelocity
        let omega = (
            angularVelocity.x * angularVelocity.x
                + angularVelocity.y * angularVelocity.y
                + angularVelocity.z * angularVelocity.z
        ).squareRoot()
        if omega > 1e-12 {
            dimensions.append(.initialAngularVelocity)
        }
        if !definition.torqueEvents.isEmpty {
            dimensions.append(.torqueDisturbance)
        }
        if definition.actuatorDegradation != nil {
            dimensions.append(.actuatorDegradation)
        }
        if abs(definition.gyroDriftScale - 1.0) > 1e-12 {
            dimensions.append(.gyroDrift)
        }
        dimensions.append(contentsOf: swapDimensions(definition.swapEvents))
        dimensions.append(contentsOf: hfDimensions(definition.hfEvents))
        if definition.liftEnvelope != nil {
            dimensions.append(.liftEnvelope)
        }
        if definition.config.duration >= 20.0 {
            dimensions.append(.longHorizon)
        }
        return dimensions
    }

    static func dimensions(from track: LongHorizonBenchmarkTrack) -> [StressDimension] {
        switch track {
        case .longHorizonTask:
            return [.longHorizon, .plannerDegradation]
        case .morphologyTransfer:
            return [.longHorizon, .morphologyTransfer]
        case .disturbanceDelayPartialObservability:
            return [.longHorizon, .partialObservability]
        }
    }

    static func dimensions(from log: SimulationLog) -> [StressDimension] {
        var dimensions: [StressDimension] = [.articulatedDynamic]
        if let schedule = log.eventSchedule {
            dimensions.append(contentsOf: swapDimensions(schedule.swapEvents))
            dimensions.append(contentsOf: hfDimensions(schedule.hfEvents))
        }
        if log.failureReason != nil {
            dimensions.append(.failureEvidence)
        }
        return dimensions
    }

    static func swapDimensions(_ events: [SwapEvent]) -> [StressDimension] {
        events.map { event in
            switch event {
            case .sensor:
                .sensorSwap
            case .actuator:
                .actuatorSwap
            }
        }
    }

    static func hfDimensions(_ events: [HFStressEvent]) -> [StressDimension] {
        events.map { event in
            switch event.kind {
            case .impulse:
                .hfImpulse
            case .vibration:
                .hfVibration
            case .sensorGlitch:
                .hfSensorGlitch
            case .actuatorSaturation:
                .hfActuatorSaturation
            case .latencySpike:
                .hfLatencySpike
            }
        }
    }
}
