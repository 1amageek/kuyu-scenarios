import KuyuCore
import KuyuPhysics

public struct KuyAtt1PrivilegedAltitudeHoldTeacher: Sendable {
    public enum TeacherError: Error, Sendable, Equatable {
        case missingCollectiveDrive
        case nonFiniteState
    }

    private var cut: ImuRateDampingDriveCut
    private let reference: ReferenceQuadrotorAltitudeHoldReference
    private let parameters: ReferenceQuadrotorParameters
    private let config: PrivilegedAltitudeHoldTeacherConfig

    public init(
        definition: ReferenceQuadrotorScenarioDefinition,
        parameters: ReferenceQuadrotorParameters = .baseline,
        gains: ImuRateDampingCutGains,
        config: PrivilegedAltitudeHoldTeacherConfig
    ) throws {
        let hoverThrust = parameters.mass * parameters.gravity / 4.0 * gains.hoverThrustScale

        self.cut = try ImuRateDampingDriveCut(
            hoverThrust: hoverThrust,
            kp: gains.kp,
            kd: gains.kd,
            yawDamping: gains.yawDamping,
            armLength: parameters.armLength,
            yawCoefficient: parameters.yawCoefficient,
            maxThrust: parameters.maxThrust,
            initialRoll: definition.initialAttitude.roll,
            initialPitch: definition.initialAttitude.pitch,
            tiltCorrectionTimeConstant: nil
        )
        self.reference = try ReferenceQuadrotorAltitudeHoldReference(definition: definition)
        self.parameters = parameters
        self.config = config
    }

    public mutating func action(for observation: EnvironmentObservation) throws -> EnvironmentAction {
        let output = try cut.update(samples: observation.sensorSamples, time: observation.time)
        switch output {
        case .driveIntents(let drives, let corrections):
            let adjusted = try adjustedDriveIntents(drives: drives, root: observation.plantState.root)
            return .driveIntents(adjusted, corrections: corrections)
        case .actuatorValues(let values):
            return .actuatorValues(values)
        }
    }

    public mutating func driveIntents(for event: WorldStepLog) throws -> [DriveIntent] {
        let output = try cut.update(samples: event.sensorSamples, time: event.time)
        switch output {
        case .driveIntents(let drives, _):
            return try adjustedDriveIntents(drives: drives, root: event.plantState.root)
        case .actuatorValues:
            return []
        }
    }

    private func adjustedDriveIntents(
        drives: [DriveIntent],
        root: RigidBodySnapshot
    ) throws -> [DriveIntent] {
        guard root.position.z.isFinite, root.velocity.z.isFinite else {
            throw TeacherError.nonFiniteState
        }
        guard let throttleIndex = drives.firstIndex(where: { $0.index.rawValue == 0 }) else {
            throw TeacherError.missingCollectiveDrive
        }

        let altitudeError = reference.targetPosition.z - root.position.z
        let correctionThrust = (config.altitudeKp * altitudeError)
            - (config.verticalVelocityKd * root.velocity.z)
        let correctionThrottle = correctionThrust / max(4.0 * parameters.maxThrust, 1e-6)
        let limitedCorrection = clamp(
            correctionThrottle,
            lower: -config.maxThrottleCorrection,
            upper: config.maxThrottleCorrection
        )
        let original = drives[throttleIndex]
        let adjustedThrottle = clamp(original.activation + limitedCorrection, lower: 0, upper: 1)

        var adjusted = drives
        adjusted[throttleIndex] = try DriveIntent(
            index: original.index,
            activation: adjustedThrottle,
            parameters: original.parameters
        )
        return adjusted
    }

    private func clamp(_ value: Double, lower: Double, upper: Double) -> Double {
        min(max(value, lower), upper)
    }
}
