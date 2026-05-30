import KuyuCore
import KuyuPhysics

/// A1 conformance evaluation suites (Suite-0 .. Suite-5) for quadrotor attitude
/// stabilization. Unlike `KuyAtt1Suite` (a flat warmup set), each level here injects
/// the mid-run swap/HF/gating stress required by `A1_MANAS_CONFORMANCE_SUITE.md`.
///
/// The swap/HF injection machinery lives in `kuyu-physics`
/// (`SwappableSensorField`, `SwappableActuatorEngine`, `TorqueDisturbanceField`);
/// this suite is the scenario-layer authority that populates the event sets.
public struct A1ConformanceSuite: ReferenceQuadrotorScenarioSuite {
    /// A1 suite taxonomy. Raw value matches the integer suite ID used by the
    /// regression/learning runners.
    public enum Level: Int, Sendable, CaseIterable, Codable {
        case warmup = 0
        case sensorSwappability = 1
        case actuatorSwappability = 2
        case reflexHF = 3
        case bundleGatingStress = 4
        case combined = 5

        /// Declared suite identifier (`Suite-0` .. `Suite-5`).
        public var suiteID: String { "Suite-\(rawValue)" }

        public var title: String {
            switch self {
            case .warmup: "Warmup"
            case .sensorSwappability: "Sensor Swappability"
            case .actuatorSwappability: "Actuator Swappability"
            case .reflexHF: "Reflex HF Training"
            case .bundleGatingStress: "Bundle/Gating Stress"
            case .combined: "Combined"
            }
        }
    }

    public let level: Level
    public let seeds: [UInt64]
    public let duration: Double

    public init(level: Level, seeds: [UInt64] = [2001, 2002, 2003], duration: Double = 20.0) {
        self.level = level
        self.seeds = seeds
        self.duration = duration
    }

    public func scenarios() throws -> [ReferenceQuadrotorScenarioDefinition] {
        let timeStep = try TimeStep(delta: 0.001)
        let envelope = try SafetyEnvelope(
            omegaSafeMax: 20.0,
            tiltSafeMaxDegrees: 60.0,
            sustainedViolationSeconds: 0.200,
            groundZ: 0.0,
            fallDurationSeconds: 0.5,
            fallVelocityThreshold: 0.05
        )
        let initialPosition = Axis3(x: 0, y: 0, z: 2.0)

        var results: [ReferenceQuadrotorScenarioDefinition] = []
        results.reserveCapacity(seeds.count)
        for (index, seed) in seeds.enumerated() {
            let config = try ScenarioConfig(
                id: try ScenarioID("KUY-A1/\(level.suiteID)/SCN-\(index + 1)"),
                seed: ScenarioSeed(seed),
                duration: duration,
                timeStep: timeStep
            )
            let injection = try makeInjection(for: level)
            results.append(ReferenceQuadrotorScenarioDefinition(
                config: config,
                kind: .hoverStart,
                initialPosition: initialPosition,
                initialAttitude: EulerAngles.degrees(roll: 8, pitch: 0, yaw: 0),
                initialAngularVelocity: Axis3(x: 0, y: 0, z: 0),
                safetyEnvelope: envelope,
                torqueEvents: injection.torqueEvents,
                actuatorDegradation: nil,
                gyroDriftScale: injection.gyroDriftScale,
                swapEvents: injection.swapEvents,
                hfEvents: injection.hfEvents
            ))
        }
        return results
    }

    private struct Injection {
        var swapEvents: [SwapEvent] = []
        var hfEvents: [HFStressEvent] = []
        var torqueEvents: [TorqueDisturbanceEvent] = []
        var gyroDriftScale: Double = 1.0
    }

    private func makeInjection(for level: Level) throws -> Injection {
        switch level {
        case .warmup:
            return Injection()
        case .sensorSwappability:
            return Injection(swapEvents: try sensorSwaps())
        case .actuatorSwappability:
            return Injection(swapEvents: try actuatorSwaps())
        case .reflexHF:
            var injection = Injection(hfEvents: try hfStress())
            injection.gyroDriftScale = 1.5
            return injection
        case .bundleGatingStress:
            return Injection(swapEvents: try gatingShocks())
        case .combined:
            return try combinedInjection()
        }
    }

    /// Combined stress runs a representative subset of every stressor *concurrently*
    /// within the run window (combined = simultaneous, not sequential), so the layers
    /// interact rather than recover one-at-a-time.
    private func combinedInjection() throws -> Injection {
        let swaps: [SwapEvent] = [
            .sensor(try SensorSwapEvent(
                kind: .calibShift,
                startTime: 5.0,
                duration: 4.0,
                targetChannels: [0, 1, 2],
                gainScale: 1.3,
                biasShift: 0.02,
                noiseScale: 1.5,
                dropoutProbability: 0.0,
                delayShiftSteps: 0,
                bandwidthScale: 0.5
            )),
            .actuator(try ActuatorSwapEvent(
                kind: .rateLimitShift,
                startTime: 6.0,
                duration: 4.0,
                motorIndex: 2,
                gainScale: 0.9,
                lagScale: 1.0,
                maxOutputScale: 1.0,
                deadzoneShift: 0.0,
                rateLimitScale: 0.25,
                asymmetryScale: 0.5
            )),
            .sensor(try SensorSwapEvent(
                kind: .swapUnit,
                startTime: 12.0,
                duration: 0.5,
                targetChannels: [0, 1, 2, 3, 4, 5],
                gainScale: 2.0,
                biasShift: 0.0,
                noiseScale: 2.5,
                dropoutProbability: 0.0,
                delayShiftSteps: 0
            )),
        ]
        let hf: [HFStressEvent] = [
            try HFStressEvent(kind: .impulse, startTime: 7.0, duration: 0.020, magnitude: 0.20),
            try HFStressEvent(kind: .vibration, startTime: 8.0, duration: 3.0, magnitude: 0.05),
            try HFStressEvent(kind: .sensorGlitch, startTime: 10.0, duration: 0.5, magnitude: 0.10),
        ]
        return Injection(swapEvents: swaps, hfEvents: hf, torqueEvents: [], gyroDriftScale: 1.5)
    }

    // MARK: - Suite-1: sensor swaps covering gain/bias/noise/delay/bandwidth/dropout

    private func sensorSwaps(startBase: Double = 5.0) throws -> [SwapEvent] {
        let gyroChannels: [UInt32] = [0, 1, 2]
        let accelChannels: [UInt32] = [3, 4, 5]
        return [
            .sensor(try SensorSwapEvent(
                kind: .calibShift,
                startTime: startBase,
                duration: 3.0,
                targetChannels: gyroChannels,
                gainScale: 1.3,
                biasShift: 0.02,
                noiseScale: 1.5,
                dropoutProbability: 0.0,
                delayShiftSteps: 0
            )),
            .sensor(try SensorSwapEvent(
                kind: .bandwidthChange,
                startTime: startBase + 4.0,
                duration: 3.0,
                targetChannels: accelChannels,
                gainScale: 1.0,
                biasShift: 0.0,
                noiseScale: 2.0,
                dropoutProbability: 0.0,
                delayShiftSteps: 0,
                bandwidthScale: 0.25
            )),
            .sensor(try SensorSwapEvent(
                kind: .dropoutBurst,
                startTime: startBase + 8.0,
                duration: 2.0,
                targetChannels: [0, 3],
                gainScale: 1.0,
                biasShift: 0.0,
                noiseScale: 1.0,
                dropoutProbability: 0.10,
                delayShiftSteps: 5
            )),
        ]
    }

    // MARK: - Suite-2: actuator swaps covering maxOutput/lag/gain/deadzone/rateLimit/asymmetry

    private func actuatorSwaps(startBase: Double = 5.0) throws -> [SwapEvent] {
        return [
            .actuator(try ActuatorSwapEvent(
                kind: .maxOutputShift,
                startTime: startBase,
                duration: 4.0,
                motorIndex: 0,
                gainScale: 1.0,
                lagScale: 1.5,
                maxOutputScale: 0.75,
                deadzoneShift: 0.0
            )),
            .actuator(try ActuatorSwapEvent(
                kind: .gainShift,
                startTime: startBase + 5.0,
                duration: 4.0,
                motorIndex: 1,
                gainScale: 0.8,
                lagScale: 1.0,
                maxOutputScale: 1.0,
                deadzoneShift: 0.05
            )),
            .actuator(try ActuatorSwapEvent(
                kind: .rateLimitShift,
                startTime: startBase + 9.0,
                duration: 3.0,
                motorIndex: 2,
                gainScale: 1.0,
                lagScale: 1.0,
                maxOutputScale: 1.0,
                deadzoneShift: 0.0,
                rateLimitScale: 0.20,
                asymmetryScale: 0.5
            )),
        ]
    }

    // MARK: - Suite-3: HF stress (impulse/vibration/glitch/saturation/latency)

    private func hfStress(startBase: Double = 5.0) throws -> [HFStressEvent] {
        return [
            try HFStressEvent(kind: .impulse, startTime: startBase, duration: 0.020, magnitude: 0.20),
            try HFStressEvent(kind: .vibration, startTime: startBase + 3.0, duration: 2.0, magnitude: 0.05),
            try HFStressEvent(kind: .sensorGlitch, startTime: startBase + 6.0, duration: 0.5, magnitude: 0.10),
            try HFStressEvent(kind: .actuatorSaturation, startTime: startBase + 7.5, duration: 0.5, magnitude: 0.6),
            try HFStressEvent(kind: .latencySpike, startTime: startBase + 9.0, duration: 0.01, magnitude: 4.0),
        ]
    }

    // MARK: - Suite-4: Bundle/Gating stress (abrupt all-channel salience + normalization shocks)

    /// Kuyu-side injectable proxy for Bundle/Gating stress. Bundle/Gating itself lives
    /// in Manas; the injectable stressor is an abrupt simultaneous shock across every
    /// ascending channel: a salience spike (large gain + noise burst) followed by a
    /// normalization collapse (gain crush + bias offset). Recovery from these tests the
    /// gating layer's resistance to runaway/collapse.
    private func gatingShocks(startBase: Double = 6.0) throws -> [SwapEvent] {
        let allChannels: [UInt32] = [0, 1, 2, 3, 4, 5]
        return [
            .sensor(try SensorSwapEvent(
                kind: .swapUnit,
                startTime: startBase,
                duration: 0.5,
                targetChannels: allChannels,
                gainScale: 2.5,
                biasShift: 0.0,
                noiseScale: 3.0,
                dropoutProbability: 0.0,
                delayShiftSteps: 0
            )),
            .sensor(try SensorSwapEvent(
                kind: .swapUnit,
                startTime: startBase + 6.0,
                duration: 0.5,
                targetChannels: allChannels,
                gainScale: 0.3,
                biasShift: 0.1,
                noiseScale: 1.0,
                dropoutProbability: 0.0,
                delayShiftSteps: 0
            )),
        ]
    }
}
