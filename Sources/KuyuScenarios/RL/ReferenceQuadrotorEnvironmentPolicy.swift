import KuyuCore
import KuyuPhysics

public protocol ReferenceQuadrotorEnvironmentPolicy {
    var policyID: String { get }
    mutating func action(for observation: EnvironmentObservation) async throws -> EnvironmentAction
}

public struct KuyAtt1BaselineEnvironmentPolicy: ReferenceQuadrotorEnvironmentPolicy {
    public let policyID: String
    private var controller: Controller

    public init(
        definition: ReferenceQuadrotorScenarioDefinition,
        parameters: ReferenceQuadrotorParameters = .baseline,
        gains: ImuRateDampingCutGains,
        mode: KuyAtt1BaselineMode,
        teacherConfig: PrivilegedAltitudeHoldTeacherConfig = .activeAltitudeHold
    ) throws {
        switch mode {
        case .teacher:
            self.policyID = "teacherActiveAltitudeHold"
            self.controller = .privileged(
                try KuyAtt1PrivilegedAltitudeHoldTeacher(
                    definition: definition,
                    parameters: parameters,
                    gains: gains,
                    config: teacherConfig
                )
            )
        case .sensor:
            self.policyID = "sensorBaseline"
            self.controller = .sensor(
                try Self.makeSensorCut(
                    parameters: parameters,
                    gains: gains
                )
            )
        }
    }

    public mutating func action(for observation: EnvironmentObservation) async throws -> EnvironmentAction {
        switch controller {
        case .sensor(var cut):
            let output = try cut.update(samples: observation.sensorSamples, time: observation.time)
            controller = .sensor(cut)
            switch output {
            case .driveIntents(let drives, let corrections):
                return .driveIntents(drives, corrections: corrections)
            case .actuatorValues(let values):
                return .actuatorValues(values)
            }
        case .privileged(var teacher):
            let action = try teacher.action(for: observation)
            controller = .privileged(teacher)
            return action
        }
    }

    private static func makeSensorCut(
        parameters: ReferenceQuadrotorParameters,
        gains: ImuRateDampingCutGains
    ) throws -> ImuRateDampingDriveCut {
        let hoverThrust = parameters.mass * parameters.gravity / 4.0 * gains.hoverThrustScale

        return try ImuRateDampingDriveCut(
            hoverThrust: hoverThrust,
            kp: gains.kp,
            kd: gains.kd,
            yawDamping: gains.yawDamping,
            armLength: parameters.armLength,
            yawCoefficient: parameters.yawCoefficient,
            maxThrust: parameters.maxThrust,
            initialRoll: 0,
            initialPitch: 0,
            tiltCorrectionTimeConstant: 0.4
        )
    }

    private enum Controller: Sendable {
        case sensor(ImuRateDampingDriveCut)
        case privileged(KuyAtt1PrivilegedAltitudeHoldTeacher)
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

        self.policyID = mode == .teacher ? "teacherActiveAltitudeHold" : "sensorBaseline"
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
