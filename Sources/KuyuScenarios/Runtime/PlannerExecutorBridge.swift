import Foundation

/// Bridges planner outputs into bounded descending channels at a fixed rate.
/// The bridge keeps the last valid vector when planner input is absent.
public struct PlannerExecutorBridge: Sendable {
    public enum BridgeError: Error, Equatable {
        case nonPositiveUpdatePeriod
        case nonFiniteUpdatePeriod
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
        if shouldUpdate(now: time) {
            if let program {
                lastVector = normalize(program.vector(at: time))
            }
            lastUpdateTime = time
        }
        return lastVector
    }

    private func shouldUpdate(now: Double) -> Bool {
        guard now.isFinite else { return false }
        guard let last = lastUpdateTime else { return true }
        if now < last { return true }
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
}
