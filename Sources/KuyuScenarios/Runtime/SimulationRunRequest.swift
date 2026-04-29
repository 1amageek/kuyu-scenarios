import KuyuCore
import KuyuPhysics

public enum ControllerSelection: String, CaseIterable, Identifiable, Sendable {
    /// Legacy alias retained for compatibility. New UI/runtime code should use `.teacherBaseline`.
    case baseline = "Baseline"
    case teacherBaseline = "Teacher Baseline"
    case sensorBaseline = "Sensor Baseline"
    case manasMLX = "ManasMLX"

    public static let allCases: [ControllerSelection] = [
        .teacherBaseline,
        .sensorBaseline,
        .manasMLX,
    ]

    public var id: String { rawValue }

    public var isBaselineController: Bool {
        switch self {
        case .baseline, .teacherBaseline, .sensorBaseline:
            return true
        case .manasMLX:
            return false
        }
    }

    public var kuyAtt1BaselineMode: KuyAtt1BaselineMode? {
        switch self {
        case .baseline, .teacherBaseline:
            return .teacher
        case .sensorBaseline:
            return .sensor
        case .manasMLX:
            return nil
        }
    }
}

public enum SimulationTaskMode: String, CaseIterable, Identifiable, Sendable {
    case attitude = "Attitude"
    case lift = "Lift"
    case singleLift = "Single Lift"

    public var id: String { rawValue }
}

public struct SimulationRunRequest: Sendable {
    public let controller: ControllerSelection
    public let taskMode: SimulationTaskMode
    public let gains: ImuRateDampingCutGains
    public let cutPeriodSteps: UInt64
    public let noise: IMU6NoiseConfig
    public let determinism: DeterminismConfig
    public let modelDescriptorPath: String
    public let overrideParameters: ReferenceQuadrotorParameters?
    public let useAux: Bool
    public let useQualityGating: Bool
    public let descendingVector: [Double]?
    public let descendingProgram: DescendingIntentProgram?

    public init(
        controller: ControllerSelection,
        taskMode: SimulationTaskMode = .attitude,
        gains: ImuRateDampingCutGains,
        cutPeriodSteps: UInt64,
        noise: IMU6NoiseConfig,
        determinism: DeterminismConfig,
        modelDescriptorPath: String,
        overrideParameters: ReferenceQuadrotorParameters?,
        useAux: Bool,
        useQualityGating: Bool,
        descendingVector: [Double]? = nil,
        descendingProgram: DescendingIntentProgram? = nil
    ) {
        self.controller = controller
        self.taskMode = taskMode
        self.gains = gains
        self.cutPeriodSteps = cutPeriodSteps
        self.noise = noise
        self.determinism = determinism
        self.modelDescriptorPath = modelDescriptorPath
        self.overrideParameters = overrideParameters
        self.useAux = useAux
        self.useQualityGating = useQualityGating
        self.descendingVector = descendingVector
        self.descendingProgram = descendingProgram
    }
}
