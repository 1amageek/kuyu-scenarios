import Foundation

public extension StressSuiteManifest {
    struct CoverageTarget: Sendable, Codable, Equatable {
        public enum ValidationError: Error, Equatable {
            case invalidMinimumCount(Int)
        }

        public let dimension: StressDimension
        public let minimumCount: Int

        private enum CodingKeys: String, CodingKey {
            case dimension
            case minimumCount
        }

        public init(dimension: StressDimension, minimumCount: Int) throws {
            guard minimumCount > 0 else {
                throw ValidationError.invalidMinimumCount(minimumCount)
            }
            self.dimension = dimension
            self.minimumCount = minimumCount
        }

        public init(from decoder: Decoder) throws {
            let container = try decoder.container(keyedBy: CodingKeys.self)
            try self.init(
                dimension: try container.decode(StressDimension.self, forKey: .dimension),
                minimumCount: try container.decode(Int.self, forKey: .minimumCount)
            )
        }
    }
}
