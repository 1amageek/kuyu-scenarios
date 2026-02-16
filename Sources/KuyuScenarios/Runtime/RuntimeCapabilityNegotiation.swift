import KuyuPhysics

public struct RuntimeCapabilitySnapshot: Sendable, Equatable {
    public let sensorSet: [String]
    public let actuatorLimits: [String: ClosedRange<Double>]
    public let updateRates: [String: Double]
    public let latencyBudgets: RobotDescriptor.LatencyBudgetsMs?

    public init(descriptor: RobotDescriptor) {
        sensorSet = descriptor.sensors.map(\.id).sorted()

        var limits: [String: ClosedRange<Double>] = [:]
        for actuator in descriptor.actuators {
            limits[actuator.id] = actuator.limits.min...actuator.limits.max
        }
        actuatorLimits = limits

        var rates: [String: Double] = [:]
        for sensor in descriptor.sensors {
            rates["sensor:\(sensor.id)"] = sensor.rateHz
        }
        for signal in descriptor.signals.actuator {
            if let rate = signal.rateHz {
                rates["actuator:\(signal.id)"] = rate
            }
        }
        updateRates = rates
        latencyBudgets = descriptor.control.latencyBudgetsMs
    }

    public func negotiate(_ request: RuntimeCapabilityRequest) throws -> RuntimeNegotiationResult {
        for sensorID in request.requiredSensors where !sensorSet.contains(sensorID) {
            throw RuntimeNegotiationError.missingSensor(sensorID)
        }

        for actuatorID in request.requiredActuatorIDs where actuatorLimits[actuatorID] == nil {
            throw RuntimeNegotiationError.missingActuator(actuatorID)
        }

        for (rateKey, minRate) in request.minimumRates {
            guard let available = updateRates[rateKey] else {
                throw RuntimeNegotiationError.missingRateKey(rateKey)
            }
            if available < minRate {
                throw RuntimeNegotiationError.insufficientRate(
                    key: rateKey,
                    required: minRate,
                    available: available
                )
            }
        }

        if request.requireLatencyBudgets && latencyBudgets == nil {
            throw RuntimeNegotiationError.missingLatencyBudgets
        }

        return RuntimeNegotiationResult(snapshot: self)
    }
}

public struct RuntimeCapabilityRequest: Sendable, Equatable {
    public let requiredSensors: [String]
    public let requiredActuatorIDs: [String]
    public let minimumRates: [String: Double]
    public let requireLatencyBudgets: Bool

    public init(
        requiredSensors: [String] = [],
        requiredActuatorIDs: [String] = [],
        minimumRates: [String: Double] = [:],
        requireLatencyBudgets: Bool = false
    ) {
        self.requiredSensors = requiredSensors
        self.requiredActuatorIDs = requiredActuatorIDs
        self.minimumRates = minimumRates
        self.requireLatencyBudgets = requireLatencyBudgets
    }
}

public struct RuntimeNegotiationResult: Sendable, Equatable {
    public let snapshot: RuntimeCapabilitySnapshot

    public init(snapshot: RuntimeCapabilitySnapshot) {
        self.snapshot = snapshot
    }
}

public enum RuntimeNegotiationError: Error, Equatable {
    case missingSensor(String)
    case missingActuator(String)
    case missingRateKey(String)
    case insufficientRate(key: String, required: Double, available: Double)
    case missingLatencyBudgets
}
