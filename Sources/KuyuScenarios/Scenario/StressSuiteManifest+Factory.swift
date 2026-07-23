import Foundation
import KuyuCore

public extension StressSuiteManifest {
    static func referenceQuadrotor(
        suiteID: String,
        definitions: [ReferenceQuadrotorScenarioDefinition],
        coverageTargets: [CoverageTarget],
        replay: ReplayVerification
    ) throws -> StressSuiteManifest {
        return try StressSuiteManifest(
            suiteID: suiteID,
            profile: .referenceQuadrotor,
            records: definitions.map { try referenceRecord(from: $0) },
            coverageTargets: coverageTargets,
            replayRequirement: .performedRequired,
            replay: replay
        )
    }

    static func referenceQuadrotorCoverageTargets(
        for definitions: [ReferenceQuadrotorScenarioDefinition]
    ) throws -> [CoverageTarget] {
        let records = try definitions.map { try referenceRecord(from: $0) }
        let coverageCounts = makeCoverageCounts(records: records)
        return try StressDimension.allCases.compactMap { dimension in
            guard coverageCounts[dimension, default: 0] > 0 else {
                return nil
            }
            return try CoverageTarget(dimension: dimension, minimumCount: 1)
        }
    }

    static func referenceQuadrotorM2Benchmark(
        suiteID: String,
        benchmark: LongHorizonBenchmarkSuite,
        replay: ReplayVerification
    ) throws -> StressSuiteManifest {
        do {
            _ = try benchmark.validatedReferenceM2TrackCounts()
        } catch let error as LongHorizonBenchmarkSuite.ValidationError {
            throw ValidationError.invalidReferenceM2Benchmark(error)
        }

        return try StressSuiteManifest(
            suiteID: suiteID,
            profile: .referenceQuadrotor,
            records: benchmark.cases.map { try referenceRecord(from: $0) },
            coverageTargets: try referenceQuadrotorM2CoverageTargets(),
            replayRequirement: .performedRequired,
            replay: replay,
            referenceM2BenchmarkEvidence: try ReferenceM2BenchmarkEvidence(benchmark: benchmark)
        )
    }

    static func referenceQuadrotorM2CoverageTargets() throws -> [CoverageTarget] {
        try requiredReferenceQuadrotorM2Dimensions.map { dimension in
            try CoverageTarget(dimension: dimension, minimumCount: 1)
        }
    }

    static func articulatedDynamic(
        suiteID: String,
        logs: [SimulationLog],
        coverageTargets: [CoverageTarget],
        replay: ReplayVerification
    ) throws -> StressSuiteManifest {
        try StressSuiteManifest(
            suiteID: suiteID,
            profile: .articulatedDynamic,
            records: logs.map { try articulatedRecord(from: $0) },
            coverageTargets: coverageTargets,
            replayRequirement: .explicitSkipAllowed,
            replay: replay
        )
    }
}
