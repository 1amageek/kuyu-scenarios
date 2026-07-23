import Foundation

public extension StressSuiteManifest {
    enum Profile: String, Sendable, Codable, Equatable {
        case referenceQuadrotor
        case articulatedDynamic
    }

    enum ReplayRequirement: String, Sendable, Codable, Equatable {
        case performedRequired
        case explicitSkipAllowed
    }

    enum StressDimension: String, Sendable, Codable, Hashable, CaseIterable {
        case initialAttitude
        case initialAngularVelocity
        case torqueDisturbance
        case actuatorDegradation
        case gyroDrift
        case sensorSwap
        case actuatorSwap
        case hfImpulse
        case hfVibration
        case hfSensorGlitch
        case hfActuatorSaturation
        case hfLatencySpike
        case liftEnvelope
        case longHorizon
        case plannerDegradation
        case morphologyTransfer
        case partialObservability
        case articulatedDynamic
        case failureEvidence
    }

    enum ValidationError: Error, Equatable {
        case emptySuiteID
        case emptyRecords
        case emptyCoverageTargets
        case duplicateScenarioRecord(String)
        case duplicateCoverageTarget(StressDimension)
        case invalidCoverageTarget(CoverageTarget.ValidationError)
        case invalidScenarioRecord(ScenarioRecord.ValidationError)
        case emptySimulationLog(String)
        case unmetCoverageTarget(dimension: StressDimension, minimumCount: Int, actualCount: Int)
        case replayChecksEmpty
        case replayCheckFailed(String)
        case missingReplayChecks([String])
        case unexpectedReplayChecks([String])
        case replayNotPerformed(String)
        case replaySkipReasonEmpty
        case decodedCoverageCountMismatch
        case decodedReplayEvidenceInvalid(String)
        case invalidReferenceM2Benchmark(LongHorizonBenchmarkSuite.ValidationError)
        case invalidReferenceM2BenchmarkEvidence(String)
        case unexpectedReferenceM2BenchmarkEvidence(Profile)
    }
}
