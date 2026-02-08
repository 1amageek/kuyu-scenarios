import Foundation

public struct DescendingIntentProgram: Sendable, Codable, Equatable {
    public struct Keyframe: Sendable, Codable, Equatable {
        public enum KeyframeError: Error, Equatable {
            case nonFiniteTime
            case negativeTime
            case emptyValues
            case nonFiniteValue(index: Int)
        }

        public let time: Double
        public let values: [Double]

        public init(time: Double, values: [Double]) throws {
            guard time.isFinite else {
                throw KeyframeError.nonFiniteTime
            }
            guard time >= 0 else {
                throw KeyframeError.negativeTime
            }
            guard !values.isEmpty else {
                throw KeyframeError.emptyValues
            }
            for (index, value) in values.enumerated() where !value.isFinite {
                throw KeyframeError.nonFiniteValue(index: index)
            }
            self.time = time
            self.values = values
        }
    }

    public enum ProgramError: Error, Equatable {
        case empty
        case nonIncreasingTime(index: Int)
        case inconsistentVectorSize(expected: Int, actual: Int, index: Int)
    }

    public let keyframes: [Keyframe]

    public var channelCount: Int {
        keyframes.first?.values.count ?? 0
    }

    public init(keyframes: [Keyframe]) throws {
        guard !keyframes.isEmpty else {
            throw ProgramError.empty
        }

        let expectedCount = keyframes[0].values.count
        var previousTime = keyframes[0].time
        for index in 1..<keyframes.count {
            let frame = keyframes[index]
            if frame.time <= previousTime {
                throw ProgramError.nonIncreasingTime(index: index)
            }
            if frame.values.count != expectedCount {
                throw ProgramError.inconsistentVectorSize(
                    expected: expectedCount,
                    actual: frame.values.count,
                    index: index
                )
            }
            previousTime = frame.time
        }

        self.keyframes = keyframes
    }

    public func vector(at time: Double) -> [Double] {
        guard let first = keyframes.first else { return [] }
        guard time.isFinite else { return first.values }
        if time <= first.time {
            return first.values
        }

        guard let last = keyframes.last else { return first.values }
        if time >= last.time {
            return last.values
        }

        for index in 1..<keyframes.count {
            let next = keyframes[index]
            if time > next.time {
                continue
            }
            let prev = keyframes[index - 1]
            let dt = next.time - prev.time
            if dt <= 0 {
                return next.values
            }
            let alpha = (time - prev.time) / dt
            return zip(prev.values, next.values).map { lhs, rhs in
                lhs + ((rhs - lhs) * alpha)
            }
        }

        return last.values
    }
}
