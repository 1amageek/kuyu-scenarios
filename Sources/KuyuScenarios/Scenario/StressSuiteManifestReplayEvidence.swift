import Foundation

public extension StressSuiteManifest {
    struct ReplayEvidence: Sendable, Codable, Equatable {
        public enum Status: String, Sendable, Codable, Equatable {
            case performed
            case notPerformed
        }

        public let status: Status
        public let checkCount: Int
        public let skippedReason: String?
        public let passed: Bool

        private enum CodingKeys: String, CodingKey {
            case status
            case checkCount
            case skippedReason
            case passed
        }

        public init(status: Status, checkCount: Int, skippedReason: String?, passed: Bool) {
            self.status = status
            self.checkCount = checkCount
            self.skippedReason = skippedReason
            self.passed = passed
        }

        public init(from decoder: Decoder) throws {
            let container = try decoder.container(keyedBy: CodingKeys.self)
            self.init(
                status: try container.decode(Status.self, forKey: .status),
                checkCount: try container.decode(Int.self, forKey: .checkCount),
                skippedReason: try container.decodeIfPresent(String.self, forKey: .skippedReason),
                passed: try container.decode(Bool.self, forKey: .passed)
            )
        }
    }
}
