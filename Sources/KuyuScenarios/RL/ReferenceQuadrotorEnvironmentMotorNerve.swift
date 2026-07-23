import KuyuCore
import KuyuPhysics

enum ReferenceQuadrotorEnvironmentMotorNerve: MotorNerveEndpoint, MotorNerveTraceProvider, Sendable {
    case driveMixer(FixedQuadMotorNerve)
    case normalizedMotor(FixedQuadNormalizedMotorNerve)

    var lastTrace: MotorNerveTrace? {
        switch self {
        case .driveMixer(let nerve):
            nerve.lastTrace
        case .normalizedMotor(let nerve):
            nerve.lastTrace
        }
    }

    mutating func update(
        input: [DriveIntent],
        corrections: [ReflexCorrection],
        telemetry: MotorNerveTelemetry,
        time: WorldTime
    ) throws -> [ActuatorValue] {
        switch self {
        case .driveMixer(var nerve):
            let output = try nerve.update(
                input: input,
                corrections: corrections,
                telemetry: telemetry,
                time: time
            )
            self = .driveMixer(nerve)
            return output
        case .normalizedMotor(var nerve):
            let output = try nerve.update(
                input: input,
                corrections: corrections,
                telemetry: telemetry,
                time: time
            )
            self = .normalizedMotor(nerve)
            return output
        }
    }
}
