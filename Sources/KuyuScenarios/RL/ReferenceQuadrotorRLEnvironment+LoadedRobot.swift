import KuyuCore
import KuyuPhysics

public extension ReferenceQuadrotorRLEnvironment {
    init(
        loadedRobot: LoadedKuyuRobot,
        schedule: SimulationSchedule,
        determinism: DeterminismConfig,
        noise: IMU6NoiseConfig = .zero,
        worldEnvironment: WorldEnvironment = .standard,
        hoverThrustScale: Double = 1.0,
        actionRealization: ReferenceQuadrotorActionRealization = .driveMixer,
        motorNerveRateLimitPerSecond: Double = 2.0,
        motorNerveSmoothingTimeConstant: Double? = 0.08,
        rewardFunction: ReferenceQuadrotorDenseReward = ReferenceQuadrotorDenseReward(),
        safetyCostFunction: ReferenceQuadrotorSafetyCost? = nil,
        worldModelAdapter: (any WorldModelEnvironmentAdapter)? = nil,
        worldModelAdapterConfiguration: WorldModelAdapterConfiguration =
            WorldModelAdapterConfiguration(),
        canonicalExecutor: any ReferenceQuadrotorCanonicalExecuting =
            ReferenceQuadrotorScalarDynamicsExecutor()
    ) throws {
        let embodiment = loadedRobot.embodiment
        let driveCount = embodiment.control.driveChannels.count
        let actuatorCount = embodiment.signals.actuator.count
        guard (driveCount == 4 && actuatorCount == 4)
                || (driveCount == 1 && actuatorCount == 1) else {
            throw EnvironmentError.unsupportedRobotShape(
                robotID: loadedRobot.manifest.robotID,
                driveCount: driveCount,
                actuatorCount: actuatorCount
            )
        }

        guard motorNerveRateLimitPerSecond.isFinite,
              motorNerveRateLimitPerSecond > 0 else {
            throw EnvironmentError.invalidMotorNerveRateLimit(
                motorNerveRateLimitPerSecond
            )
        }
        let embodimentResolution = try ReferenceQuadrotorEmbodimentResolver()
            .resolution(for: loadedRobot)
        let effectiveRateLimit = min(
            motorNerveRateLimitPerSecond,
            embodimentResolution.normalizedActuatorRateLimitPerSecond
        )

        self.init(
            parameters: embodimentResolution.parameters,
            schedule: schedule,
            determinism: determinism,
            noise: noise,
            worldEnvironment: worldEnvironment,
            hoverThrustScale: hoverThrustScale,
            robotManifestID: loadedRobot.manifest.robotID,
            driveCount: driveCount,
            actuatorCount: actuatorCount,
            actionRealization: actionRealization,
            motorNerveRateLimitPerSecond: effectiveRateLimit,
            motorNerveSmoothingTimeConstant: motorNerveSmoothingTimeConstant,
            rewardFunction: rewardFunction,
            safetyCostFunction: safetyCostFunction,
            worldModelAdapter: worldModelAdapter,
            worldModelAdapterConfiguration: worldModelAdapterConfiguration,
            canonicalExecutor: canonicalExecutor
        )
    }
}
