import KuyuCore
import KuyuPhysics

public protocol ReferenceQuadrotorEnvironmentPolicy {
    var policyID: String { get }
    mutating func action(for observation: EnvironmentObservation) async throws -> EnvironmentAction
}

public struct KuyAtt1BaselineEnvironmentPolicy: ReferenceQuadrotorEnvironmentPolicy {
    public let policyID: String
    private var cut: ImuRateDampingDriveCut

    public init(
        definition: ReferenceQuadrotorScenarioDefinition,
        parameters: ReferenceQuadrotorParameters = .baseline,
        gains: ImuRateDampingCutGains,
        mode: KuyAtt1BaselineMode
    ) throws {
        let hoverThrust = parameters.mass * parameters.gravity / 4.0 * gains.hoverThrustScale
        let initialAttitude: EulerAngles
        let tiltCorrectionTimeConstant: Double?

        switch mode {
        case .teacher:
            initialAttitude = definition.initialAttitude
            tiltCorrectionTimeConstant = nil
            policyID = "teacherBaseline"
        case .sensor:
            initialAttitude = EulerAngles(roll: 0, pitch: 0, yaw: 0)
            tiltCorrectionTimeConstant = 0.4
            policyID = "sensorBaseline"
        }

        cut = try ImuRateDampingDriveCut(
            hoverThrust: hoverThrust,
            kp: gains.kp,
            kd: gains.kd,
            yawDamping: gains.yawDamping,
            armLength: parameters.armLength,
            yawCoefficient: parameters.yawCoefficient,
            maxThrust: parameters.maxThrust,
            initialRoll: initialAttitude.roll,
            initialPitch: initialAttitude.pitch,
            tiltCorrectionTimeConstant: tiltCorrectionTimeConstant
        )
    }

    public mutating func action(for observation: EnvironmentObservation) async throws -> EnvironmentAction {
        let output = try cut.update(samples: observation.sensorSamples, time: observation.time)
        switch output {
        case .driveIntents(let drives, let corrections):
            return .driveIntents(drives, corrections: corrections)
        case .actuatorValues(let values):
            return .actuatorValues(values)
        }
    }
}

public struct KuyLiftBaselineEnvironmentPolicy: ReferenceQuadrotorEnvironmentPolicy {
    public enum PolicyError: Error, Equatable {
        case missingLiftEnvelope
    }

    public let policyID: String
    public let targetZ: Double
    public let hoverThrust: Double
    public let maxThrust: Double
    public let kp: Double
    public let kd: Double
    public let driveCount: Int

    public init(
        definition: ReferenceQuadrotorScenarioDefinition,
        parameters: ReferenceQuadrotorParameters = .baseline,
        mode: KuyAtt1BaselineMode,
        hoverThrustScale: Double = 1.0,
        kp: Double = 6.0,
        kd: Double = 4.0
    ) throws {
        guard let envelope = definition.liftEnvelope else {
            throw PolicyError.missingLiftEnvelope
        }

        self.policyID = mode == .teacher ? "teacherBaseline" : "sensorBaseline"
        self.targetZ = envelope.targetZ
        self.driveCount = definition.kind == .singleLiftHover ? 1 : 4
        self.hoverThrust = parameters.mass * parameters.gravity / Double(driveCount) * hoverThrustScale
        self.maxThrust = parameters.maxThrust
        self.kp = kp
        self.kd = kd
    }

    public mutating func action(for observation: EnvironmentObservation) async throws -> EnvironmentAction {
        let z = observation.plantState.root.position.z
        let vz = observation.plantState.root.velocity.z
        let error = targetZ - z
        let desiredThrust = hoverThrust + kp * error - kd * vz
        let throttle = clamp(desiredThrust / max(maxThrust, 1e-6), lower: 0.0, upper: 1.0)
        let drives = try (0..<driveCount).map { index in
            try DriveIntent(index: DriveIndex(UInt32(index)), activation: throttle)
        }
        return .driveIntents(drives, corrections: [])
    }

    private func clamp(_ value: Double, lower: Double, upper: Double) -> Double {
        min(max(value, lower), upper)
    }
}
