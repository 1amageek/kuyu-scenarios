import Logging
import KuyuCore
import KuyuPhysics

public enum RuntimeBackendKind: String, Sendable, Codable, Equatable {
    case simulation
    case robot
}

public enum RuntimeTransportKind: String, Sendable, Codable, Equatable {
    case inProcess
    case sharedMemory
    case grpc
    case ros2
}

public struct RuntimeSessionStartRequest: Sendable, Equatable {
    public let capabilityRequest: RuntimeCapabilityRequest
    public let enforceReflexPathLowLatency: Bool

    public init(
        capabilityRequest: RuntimeCapabilityRequest = RuntimeCapabilityRequest(),
        enforceReflexPathLowLatency: Bool = true
    ) {
        self.capabilityRequest = capabilityRequest
        self.enforceReflexPathLowLatency = enforceReflexPathLowLatency
    }
}

public struct RuntimeSessionStartResult: Sendable, Equatable {
    public let adapterID: String
    public let backendKind: RuntimeBackendKind
    public let transport: RuntimeTransportKind
    public let negotiation: RuntimeNegotiationResult

    public init(
        adapterID: String,
        backendKind: RuntimeBackendKind,
        transport: RuntimeTransportKind,
        negotiation: RuntimeNegotiationResult
    ) {
        self.adapterID = adapterID
        self.backendKind = backendKind
        self.transport = transport
        self.negotiation = negotiation
    }
}

public struct RuntimeLoopCheckResult: Sendable, Equatable {
    public let stepCount: Int
    public let durationSeconds: Double
    public let averageStepRateHz: Double
    public let cutUpdateCount: Int
    public let motorNerveUpdateCount: Int
    public let failureReason: FailureReason?
    public let failureTime: Double?

    public init(
        stepCount: Int,
        durationSeconds: Double,
        averageStepRateHz: Double,
        cutUpdateCount: Int,
        motorNerveUpdateCount: Int,
        failureReason: FailureReason?,
        failureTime: Double?
    ) {
        self.stepCount = stepCount
        self.durationSeconds = durationSeconds
        self.averageStepRateHz = averageStepRateHz
        self.cutUpdateCount = cutUpdateCount
        self.motorNerveUpdateCount = motorNerveUpdateCount
        self.failureReason = failureReason
        self.failureTime = failureTime
    }
}

public enum RuntimeSessionError: Error, Equatable {
    case reflexLatencyRequiresLowLatencyTransport(RuntimeTransportKind)
    case missingLatencyBudgetDeclaration
    case emptySimulationLog
    case nonMonotonicSimulationTime
}

public enum RuntimeSessionEventLevel: String, Sendable, Codable, Equatable {
    case info
    case warning
    case error
}

public struct RuntimeSessionEvent: Sendable, Codable, Equatable {
    public let level: RuntimeSessionEventLevel
    public let code: String
    public let message: String
    public let metadata: [String: String]

    public init(
        level: RuntimeSessionEventLevel,
        code: String,
        message: String,
        metadata: [String: String] = [:]
    ) {
        self.level = level
        self.code = code
        self.message = message
        self.metadata = metadata
    }
}

public protocol RuntimeControlAdapter {
    var adapterID: String { get }
    var backendKind: RuntimeBackendKind { get }
    var transport: RuntimeTransportKind { get }
    var capabilitySnapshot: RuntimeCapabilitySnapshot { get }
}

public struct SimulationRuntimeAdapter: RuntimeControlAdapter {
    public let adapterID: String
    public let transport: RuntimeTransportKind
    public let capabilitySnapshot: RuntimeCapabilitySnapshot

    public var backendKind: RuntimeBackendKind { .simulation }

    public init(
        adapterID: String = "simulation.default",
        transport: RuntimeTransportKind = .inProcess,
        embodiment: EmbodimentContract
    ) {
        self.adapterID = adapterID
        self.transport = transport
        self.capabilitySnapshot = RuntimeCapabilitySnapshot(embodiment: embodiment)
    }
}

public enum RuntimeSessionCoordinator {
    public static func startSession(
        adapter: some RuntimeControlAdapter,
        request: RuntimeSessionStartRequest
    ) throws -> RuntimeSessionStartResult {
        try startSession(
            adapter: adapter,
            request: request,
            logger: nil,
            eventSink: nil
        )
    }

    public static func startSession(
        adapter: some RuntimeControlAdapter,
        request: RuntimeSessionStartRequest,
        logger: Logger?,
        eventSink: ((RuntimeSessionEvent) -> Void)?
    ) throws -> RuntimeSessionStartResult {
        emit(
            RuntimeSessionEvent(
                level: .info,
                code: "session.start",
                message: "Runtime session negotiation started.",
                metadata: [
                    "adapterID": adapter.adapterID,
                    "backendKind": adapter.backendKind.rawValue,
                    "transport": adapter.transport.rawValue
                ]
            ),
            logger: logger,
            sink: eventSink
        )

        let negotiation: RuntimeNegotiationResult
        do {
            negotiation = try adapter.capabilitySnapshot.negotiate(request.capabilityRequest)
        } catch {
            emit(
                RuntimeSessionEvent(
                    level: .error,
                    code: "session.negotiation_failed",
                    message: "Capability negotiation failed.",
                    metadata: [
                        "adapterID": adapter.adapterID,
                        "reason": String(describing: error)
                    ]
                ),
                logger: logger,
                sink: eventSink
            )
            throw error
        }

        emit(
            RuntimeSessionEvent(
                level: .info,
                code: "session.negotiation_ok",
                message: "Capability negotiation succeeded.",
                metadata: [
                    "adapterID": adapter.adapterID
                ]
            ),
            logger: logger,
            sink: eventSink
        )

        if request.enforceReflexPathLowLatency {
            guard adapter.transport == .inProcess || adapter.transport == .sharedMemory else {
                emit(
                    RuntimeSessionEvent(
                        level: .error,
                        code: "session.latency_policy_rejected",
                        message: "Transport violates reflex low-latency policy.",
                        metadata: [
                            "adapterID": adapter.adapterID,
                            "transport": adapter.transport.rawValue
                        ]
                    ),
                    logger: logger,
                    sink: eventSink
                )
                throw RuntimeSessionError.reflexLatencyRequiresLowLatencyTransport(adapter.transport)
            }
            guard negotiation.snapshot.latencyBudgets != nil else {
                emit(
                    RuntimeSessionEvent(
                        level: .error,
                        code: "session.latency_budget_missing",
                        message: "Latency budget declaration missing.",
                        metadata: [
                            "adapterID": adapter.adapterID
                        ]
                    ),
                    logger: logger,
                    sink: eventSink
                )
                throw RuntimeSessionError.missingLatencyBudgetDeclaration
            }
            emit(
                RuntimeSessionEvent(
                    level: .info,
                    code: "session.latency_policy_ok",
                    message: "Latency policy validated.",
                    metadata: [
                        "adapterID": adapter.adapterID,
                        "transport": adapter.transport.rawValue
                    ]
                ),
                logger: logger,
                sink: eventSink
            )
        }

        let result = RuntimeSessionStartResult(
            adapterID: adapter.adapterID,
            backendKind: adapter.backendKind,
            transport: adapter.transport,
            negotiation: negotiation
        )
        emit(
            RuntimeSessionEvent(
                level: .info,
                code: "session.ready",
                message: "Runtime session established.",
                metadata: [
                    "adapterID": adapter.adapterID,
                    "backendKind": adapter.backendKind.rawValue,
                    "transport": adapter.transport.rawValue
                ]
            ),
            logger: logger,
            sink: eventSink
        )
        return result
    }

    public static func verifySimulationRun(
        session: RuntimeSessionStartResult,
        log: SimulationLog,
        logger: Logger?,
        eventSink: ((RuntimeSessionEvent) -> Void)?
    ) throws -> RuntimeLoopCheckResult {
        guard !log.events.isEmpty else {
            emit(
                RuntimeSessionEvent(
                    level: .error,
                    code: "session.loop_empty",
                    message: "Simulation log has no events.",
                    metadata: ["adapterID": session.adapterID]
                ),
                logger: logger,
                sink: eventSink
            )
            throw RuntimeSessionError.emptySimulationLog
        }

        var previousTime = -Double.greatestFiniteMagnitude
        for event in log.events {
            if !event.time.time.isFinite || event.time.time < previousTime {
                emit(
                    RuntimeSessionEvent(
                        level: .error,
                        code: "session.loop_non_monotonic_time",
                        message: "Simulation time is non-monotonic.",
                        metadata: ["adapterID": session.adapterID]
                    ),
                    logger: logger,
                    sink: eventSink
                )
                throw RuntimeSessionError.nonMonotonicSimulationTime
            }
            previousTime = event.time.time
        }

        let firstTime = log.events.first?.time.time ?? 0.0
        let lastTime = log.events.last?.time.time ?? firstTime
        let duration = max(lastTime - firstTime, log.timeStep.delta)
        let stepCount = log.events.count
        let cutUpdateCount = log.events.reduce(0) { partial, event in
            partial + (event.events.contains(.cutUpdate) ? 1 : 0)
        }
        let motorUpdateCount = log.events.reduce(0) { partial, event in
            partial + (event.events.contains(.motorNerveUpdate) ? 1 : 0)
        }
        let averageStepRate = Double(stepCount) / duration

        if let failureReason = log.failureReason {
            emit(
                RuntimeSessionEvent(
                    level: .warning,
                    code: "session.loop_failure_reported",
                    message: "Simulation run reported failure.",
                    metadata: [
                        "adapterID": session.adapterID,
                        "failureReason": failureReason.rawValue,
                        "failureTime": String(log.failureTime ?? -1.0)
                    ]
                ),
                logger: logger,
                sink: eventSink
            )
        }

        let result = RuntimeLoopCheckResult(
            stepCount: stepCount,
            durationSeconds: duration,
            averageStepRateHz: averageStepRate,
            cutUpdateCount: cutUpdateCount,
            motorNerveUpdateCount: motorUpdateCount,
            failureReason: log.failureReason,
            failureTime: log.failureTime
        )

        let okEvent = RuntimeSessionEvent(
            level: .info,
            code: "session.loop_check_ok",
            message: "Simulation loop interoperability check passed.",
            metadata: [
                "adapterID": session.adapterID,
                "stepCount": String(stepCount),
                "durationSeconds": String(duration),
                "stepRateHz": String(averageStepRate),
                "cutUpdateCount": String(cutUpdateCount),
                "motorNerveUpdateCount": String(motorUpdateCount)
            ]
        )
        emit(okEvent, logger: logger, sink: eventSink)

        return result
    }

    private static func emit(
        _ event: RuntimeSessionEvent,
        logger: Logger?,
        sink: ((RuntimeSessionEvent) -> Void)?
    ) {
        sink?(event)
        guard let logger else { return }
        let metadata = Logger.Metadata(
            uniqueKeysWithValues: event.metadata.map { key, value in
                (key, Logger.MetadataValue.string(value))
            }
        )
        switch event.level {
        case .info:
            logger.info("\(event.message)", metadata: metadata)
        case .warning:
            logger.warning("\(event.message)", metadata: metadata)
        case .error:
            logger.error("\(event.message)", metadata: metadata)
        }
    }
}
