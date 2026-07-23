import Testing
@testable import KuyuScenarios

@Test func plannerExecutorBridgeSnapshotsHoldWhenPlannerDisconnects() throws {
    var bridge = try PlannerExecutorBridge(channelCount: 2, updatePeriod: 0.1)
    let program = try DescendingIntentProgram(keyframes: [
        try DescendingIntentProgram.Keyframe(time: 0.0, values: [0.2, -0.2]),
        try DescendingIntentProgram.Keyframe(time: 1.0, values: [1.0, -1.0]),
    ])

    let connected = bridge.descendingSnapshot(at: 0.1, program: program)
    let disconnected = bridge.descendingSnapshot(at: 0.2, program: nil)

    #expect(connected.status == .updatedFromProgram)
    #expect(disconnected.status == .heldDisconnected)
    #expect(disconnected.programAvailable == false)
    #expect(disconnected.vector == connected.vector)
}

@Test func plannerExecutorBridgeSnapshotsHoldWithinFixedRate() throws {
    var bridge = try PlannerExecutorBridge(channelCount: 2, updatePeriod: 0.1)
    let program = try DescendingIntentProgram(keyframes: [
        try DescendingIntentProgram.Keyframe(time: 0.0, values: [0.0, 0.0]),
        try DescendingIntentProgram.Keyframe(time: 1.0, values: [1.0, -1.0]),
    ])

    let first = bridge.descendingSnapshot(at: 0.0, program: program)
    let held = bridge.descendingSnapshot(at: 0.05, program: program)

    #expect(first.status == .updatedFromProgram)
    #expect(held.status == .heldWithinUpdatePeriod)
    #expect(held.programAvailable)
    #expect(held.vector == first.vector)
}

@Test func plannerExecutorBridgeDoesNotReplayPastPlannerValuesOnNonMonotonicTime() throws {
    var bridge = try PlannerExecutorBridge(channelCount: 2, updatePeriod: 0.1)
    let program = try DescendingIntentProgram(keyframes: [
        try DescendingIntentProgram.Keyframe(time: 0.0, values: [0.0, 0.0]),
        try DescendingIntentProgram.Keyframe(time: 1.0, values: [1.0, -1.0]),
    ])

    let current = bridge.descendingSnapshot(at: 0.2, program: program)
    let stale = bridge.descendingSnapshot(at: 0.1, program: program)

    #expect(current.status == .updatedFromProgram)
    #expect(stale.status == .heldNonMonotonicTime)
    #expect(stale.vector == current.vector)
    #expect(stale.lastUpdateTime == current.lastUpdateTime)
}

@Test func plannerExecutorBridgeRejectsInvalidClampRanges() {
    #expect(throws: PlannerExecutorBridge.BridgeError.nonFiniteClampRange) {
        _ = try PlannerExecutorBridge(
            channelCount: 2,
            updatePeriod: 0.1,
            clampRange: -Double.infinity...1.0
        )
    }
}

@Test func plannerExecutorBridgeSnapshotsClampAndPadProgramVectors() throws {
    var bridge = try PlannerExecutorBridge(channelCount: 3, updatePeriod: 0.1, clampRange: 0.0...1.0)
    let program = try DescendingIntentProgram(keyframes: [
        try DescendingIntentProgram.Keyframe(time: 0.0, values: [2.0, -2.0]),
    ])

    let snapshot = bridge.descendingSnapshot(at: 0.0, program: program)

    #expect(snapshot.status == .updatedFromProgram)
    #expect(snapshot.vector == [1.0, 0.0, 0.0])
}
