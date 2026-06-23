import KuyuCore
import KuyuPhysics

public enum ReferenceQuadrotorBaselineReplayRuntimeError: Error, Sendable, Equatable {
    case unsupportedController(String)
}

public struct ReferenceQuadrotorBaselineReplayRuntime: Sendable {
    public init() {}

    public func run(
        request: SimulationRunRequest,
        parameters: ReferenceQuadrotorParameters,
        schedule: SimulationSchedule,
        definitions: [ReferenceQuadrotorScenarioDefinition]? = nil,
        control: SimulationControl? = nil,
        telemetry: WorldStepTelemetry? = nil
    ) async throws -> KuyAtt1RunOutput {
        guard request.controller.isBaselineController else {
            throw ReferenceQuadrotorBaselineReplayRuntimeError.unsupportedController(request.controller.rawValue)
        }

        switch request.taskMode {
        case .attitude:
            let runner = KuyAtt1Runner(
                parameters: parameters,
                schedule: schedule,
                determinism: request.determinism,
                noise: request.noise,
                gains: request.gains,
                baselineMode: request.controller.kuyAtt1BaselineMode ?? .teacher,
                replayVerification: true
            )
            return try await runner.runWithLogs(
                definitions: definitions,
                control: control,
                telemetry: telemetry
            )
        case .lift:
            return try await runLiftBaseline(
                request: request,
                parameters: parameters,
                schedule: schedule,
                definitions: definitions,
                control: control,
                telemetry: telemetry
            )
        case .singleLift:
            return try await runSingleLiftBaseline(
                request: request,
                parameters: parameters,
                schedule: schedule,
                definitions: definitions,
                control: control,
                telemetry: telemetry
            )
        }
    }

    private func runLiftBaseline(
        request: SimulationRunRequest,
        parameters: ReferenceQuadrotorParameters,
        schedule: SimulationSchedule,
        definitions overrideDefinitions: [ReferenceQuadrotorScenarioDefinition]?,
        control: SimulationControl?,
        telemetry: WorldStepTelemetry?
    ) async throws -> KuyAtt1RunOutput {
        let definitions = try overrideDefinitions ?? KuyLiftSuite().scenarios()
        let suite = ResolvedReferenceQuadrotorScenarioSuite(definitions: definitions)
        let runner = ReferenceQuadrotorScenarioRunner<ImuRateDampingDriveCut, LiftMotorNerve>(
            parameters: parameters,
            schedule: schedule,
            determinism: request.determinism,
            noise: request.noise,
            environment: .standard,
            hoverThrustScale: request.gains.hoverThrustScale
        )
        let validation = KuyAtt1Validation(runner: runner, suite: suite)
        let output = try await validation.runWithLogs(
            cutFactory: { definition in
                try Self.makeLiftCut(
                    definition: definition,
                    parameters: parameters,
                    gains: request.gains
                )
            },
            motorNerveFactory: { _ in
                LiftMotorNerve(motorMaxThrusts: try MotorMaxThrusts.uniform(parameters.maxThrust))
            },
            control: control,
            telemetry: telemetry
        )
        return Self.makeOutput(
            result: output.result,
            logs: output.logs,
            manifest: output.manifest
        )
    }

    private func runSingleLiftBaseline(
        request: SimulationRunRequest,
        parameters: ReferenceQuadrotorParameters,
        schedule: SimulationSchedule,
        definitions overrideDefinitions: [ReferenceQuadrotorScenarioDefinition]?,
        control: SimulationControl?,
        telemetry: WorldStepTelemetry?
    ) async throws -> KuyAtt1RunOutput {
        let tunedParameters = try ReferenceQuadrotorSingleLiftParameterTuning.tuned(
            parameters: parameters,
            hoverThrustScale: request.gains.hoverThrustScale
        )
        let definitions = try overrideDefinitions ?? KuySingleLiftSuite().scenarios()
        let suite = ResolvedReferenceQuadrotorScenarioSuite(definitions: definitions)
        let runner = ReferenceQuadrotorScenarioRunner<SinglePropHoverCut, FixedSinglePropMotorNerve>(
            parameters: tunedParameters,
            schedule: schedule,
            determinism: request.determinism,
            noise: request.noise,
            environment: .standard,
            hoverThrustScale: request.gains.hoverThrustScale
        )
        let validation = KuyAtt1Validation(runner: runner, suite: suite)
        let output = try await validation.runWithLogs(
            cutFactory: { definition in
                try Self.makeSingleLiftCut(
                    definition: definition,
                    parameters: tunedParameters,
                    hoverThrustScale: request.gains.hoverThrustScale
                )
            },
            motorNerveFactory: { _ in
                FixedSinglePropMotorNerve(config: FixedSinglePropMotorNerve.Config(
                    maxThrust: tunedParameters.maxThrust,
                    rateLimitPerSecond: 100.0,
                    smoothingTimeConstant: nil,
                    baseThrottle: 0.0
                ))
            },
            control: control,
            telemetry: telemetry
        )
        return Self.makeOutput(
            result: output.result,
            logs: output.logs,
            manifest: output.manifest
        )
    }

    private static func makeLiftCut(
        definition: ReferenceQuadrotorScenarioDefinition,
        parameters: ReferenceQuadrotorParameters,
        gains: ImuRateDampingCutGains
    ) throws -> ImuRateDampingDriveCut {
        try ImuRateDampingDriveCut(
            hoverThrust: parameters.mass * parameters.gravity / 4.0 * gains.hoverThrustScale,
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
    }

    private static func makeSingleLiftCut(
        definition: ReferenceQuadrotorScenarioDefinition,
        parameters: ReferenceQuadrotorParameters,
        hoverThrustScale: Double
    ) throws -> SinglePropHoverCut {
        let targetZ = try ReferenceQuadrotorAltitudeHoldReference(definition: definition).targetPosition.z
        return try SinglePropHoverCut(
            targetZ: targetZ,
            hoverThrust: parameters.mass * parameters.gravity * hoverThrustScale,
            maxThrust: parameters.maxThrust
        )
    }

    private nonisolated static func makeOutput(
        result: SuiteRunResult,
        logs: [ScenarioLogEntry],
        manifest: [ReferenceQuadrotorScenarioManifest]
    ) -> KuyAtt1RunOutput {
        let aggregate = EvaluationAggregate.from(evaluations: result.evaluations)
        let summary = ValidationSummary(
            suitePassed: result.passed,
            evaluations: result.evaluations,
            replay: result.replay,
            manifest: manifest,
            aggregate: aggregate
        )
        return KuyAtt1RunOutput(result: result, summary: summary, logs: logs)
    }
}

private struct ResolvedReferenceQuadrotorScenarioSuite: ReferenceQuadrotorScenarioSuite {
    let definitions: [ReferenceQuadrotorScenarioDefinition]

    func scenarios() throws -> [ReferenceQuadrotorScenarioDefinition] {
        definitions
    }
}
