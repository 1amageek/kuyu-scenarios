import Foundation

/// Bridges planner outputs into bounded descending channels at a fixed rate.
/// The bridge keeps the last valid vector when planner input is absent.
public struct PlannerExecutorBridge: Sendable {
    public enum BridgeError: Error, Equatable {
        case nonPositiveUpdatePeriod
        case nonFiniteUpdatePeriod
        case nonFiniteClampRange
    }

    public enum SnapshotStatus: String, Sendable, Codable, Equatable {
        case updatedFromProgram
        case heldWithinUpdatePeriod
        case heldDisconnected
        case heldInvalidTime
        case heldNonMonotonicTime
    }

    public struct Snapshot: Sendable, Codable, Equatable {
        public let requestedTime: Double?
        public let lastUpdateTime: Double?
        public let vector: [Double]
        public let programAvailable: Bool
        public let status: SnapshotStatus

        public init(
            requestedTime: Double?,
            lastUpdateTime: Double?,
            vector: [Double],
            programAvailable: Bool,
            status: SnapshotStatus
        ) {
            self.requestedTime = requestedTime
            self.lastUpdateTime = lastUpdateTime
            self.vector = vector
            self.programAvailable = programAvailable
            self.status = status
        }
    }

    public let channelCount: Int
    public let updatePeriod: Double
    public let clampRange: ClosedRange<Double>

    private var lastUpdateTime: Double?
    private var lastVector: [Double]

    public init(
        channelCount: Int,
        updatePeriod: Double,
        clampRange: ClosedRange<Double> = -1.0...1.0
    ) throws {
        guard updatePeriod.isFinite else {
            throw BridgeError.nonFiniteUpdatePeriod
        }
        guard updatePeriod > 0 else {
            throw BridgeError.nonPositiveUpdatePeriod
        }
        guard clampRange.lowerBound.isFinite, clampRange.upperBound.isFinite else {
            throw BridgeError.nonFiniteClampRange
        }
        self.channelCount = max(0, channelCount)
        self.updatePeriod = updatePeriod
        self.clampRange = clampRange
        self.lastUpdateTime = nil
        self.lastVector = [Double](repeating: 0.0, count: max(0, channelCount))
    }

    public mutating func descendingVector(
        at time: Double,
        program: DescendingIntentProgram?
    ) -> [Double] {
        descendingSnapshot(at: time, program: program).vector
    }

    public mutating func descendingSnapshot(
        at time: Double,
        program: DescendingIntentProgram?
    ) -> Snapshot {
        guard time.isFinite else {
            return snapshot(
                requestedTime: nil,
                programAvailable: program != nil,
                status: .heldInvalidTime
            )
        }

        if let lastUpdateTime, time < lastUpdateTime {
            return snapshot(
                requestedTime: time,
                programAvailable: program != nil,
                status: .heldNonMonotonicTime
            )
        }

        guard shouldUpdate(now: time) else {
            return snapshot(
                requestedTime: time,
                programAvailable: program != nil,
                status: program == nil ? .heldDisconnected : .heldWithinUpdatePeriod
            )
        }

        let status: SnapshotStatus
        if let program {
            lastVector = normalize(program.vector(at: time))
            status = .updatedFromProgram
        } else {
            status = .heldDisconnected
        }
        lastUpdateTime = time

        return snapshot(
            requestedTime: time,
            programAvailable: program != nil,
            status: status
        )
    }

    private func shouldUpdate(now: Double) -> Bool {
        guard now.isFinite else { return false }
        guard let last = lastUpdateTime else { return true }
        guard now >= last else { return false }
        return (now - last) >= updatePeriod
    }

    private func normalize(_ raw: [Double]) -> [Double] {
        var values = raw.prefix(channelCount).map { value -> Double in
            guard value.isFinite else { return 0.0 }
            return min(max(value, clampRange.lowerBound), clampRange.upperBound)
        }
        if values.count < channelCount {
            values.append(contentsOf: [Double](repeating: 0.0, count: channelCount - values.count))
        }
        return values
    }

    private func snapshot(
        requestedTime: Double?,
        programAvailable: Bool,
        status: SnapshotStatus
    ) -> Snapshot {
        Snapshot(
            requestedTime: requestedTime,
            lastUpdateTime: lastUpdateTime,
            vector: lastVector,
            programAvailable: programAvailable,
            status: status
        )
    }
}
