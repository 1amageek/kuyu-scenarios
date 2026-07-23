import Foundation

extension StressSuiteManifest {
    static func validateUniqueRecords(_ records: [ScenarioRecord]) throws {
        var seen: Set<String> = []
        for record in records {
            guard seen.insert(record.key).inserted else {
                throw ValidationError.duplicateScenarioRecord(record.key)
            }
        }
    }

    static func validateUniqueTargets(_ targets: [CoverageTarget]) throws {
        var seen: Set<StressDimension> = []
        for target in targets {
            guard seen.insert(target.dimension).inserted else {
                throw ValidationError.duplicateCoverageTarget(target.dimension)
            }
        }
    }

    static func makeCoverageCounts(records: [ScenarioRecord]) -> [StressDimension: Int] {
        var counts: [StressDimension: Int] = [:]
        for record in records {
            for dimension in record.dimensions {
                counts[dimension, default: 0] += 1
            }
        }
        return counts
    }

    static func validateCoverage(
        targets: [CoverageTarget],
        coverageCounts: [StressDimension: Int]
    ) throws {
        for target in targets {
            let actual = coverageCounts[target.dimension, default: 0]
            guard actual >= target.minimumCount else {
                throw ValidationError.unmetCoverageTarget(
                    dimension: target.dimension,
                    minimumCount: target.minimumCount,
                    actualCount: actual
                )
            }
        }
    }
}
