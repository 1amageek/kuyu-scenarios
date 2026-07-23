import simd
import KuyuPhysics

extension ReferenceQuadrotorRLEnvironment {
    func scaledNoise(for definition: ReferenceQuadrotorScenarioDefinition) throws -> IMU6NoiseConfig {
        try IMU6NoiseConfig(
            gyroNoiseStdDev: noise.gyroNoiseStdDev,
            gyroBias: noise.gyroBias,
            gyroRandomWalkSigma: noise.gyroRandomWalkSigma * definition.gyroDriftScale,
            accelNoiseStdDev: noise.accelNoiseStdDev,
            accelBias: noise.accelBias,
            accelRandomWalkSigma: noise.accelRandomWalkSigma,
            delaySteps: noise.delaySteps
        )
    }

    func disturbance(
        definition: ReferenceQuadrotorScenarioDefinition,
        store: ReferenceQuadrotorWorldStore
    ) -> TorqueDisturbanceField {
        TorqueDisturbanceField(
            events: definition.torqueEvents,
            hfEvents: definition.hfEvents,
            store: store
        )
    }

    func imuSensor(
        definition: ReferenceQuadrotorScenarioDefinition,
        store: ReferenceQuadrotorWorldStore,
        scaledNoise: IMU6NoiseConfig
    ) throws -> IMU6SensorField {
        try IMU6SensorField(
            parameters: parameters,
            mixer: mixer,
            store: store,
            timeStep: definition.config.timeStep,
            environment: worldEnvironment,
            noiseSeed: definition.config.seed.rawValue,
            gyroNoiseStdDev: scaledNoise.gyroNoiseStdDev,
            gyroBias: scaledNoise.gyroBias,
            gyroRandomWalkSigma: scaledNoise.gyroRandomWalkSigma,
            accelNoiseStdDev: scaledNoise.accelNoiseStdDev,
            accelBias: scaledNoise.accelBias,
            accelRandomWalkSigma: scaledNoise.accelRandomWalkSigma,
            delaySteps: scaledNoise.delaySteps
        )
    }

    func buildStore(definition: ReferenceQuadrotorScenarioDefinition) throws -> ReferenceQuadrotorWorldStore {
        let isSingleLift = definition.kind == .singleLiftHover
        let orientation = isSingleLift
            ? simd_quatd(angle: 0.0, axis: SIMD3<Double>(0, 0, 1))
            : definition.initialAttitude.toQuaternion()
        let angularVelocity = isSingleLift
            ? SIMD3<Double>(repeating: 0.0)
            : SIMD3<Double>(
                definition.initialAngularVelocity.x,
                definition.initialAngularVelocity.y,
                definition.initialAngularVelocity.z
            )
        let state = try ReferenceQuadrotorState(
            position: SIMD3<Double>(
                definition.initialPosition.x,
                definition.initialPosition.y,
                definition.initialPosition.z
            ),
            velocity: SIMD3<Double>(repeating: 0),
            orientation: orientation,
            angularVelocity: angularVelocity
        )
        let hoverThrust = isSingleLift
            ? parameters.mass * parameters.gravity * hoverThrustScale
            : parameters.mass * parameters.gravity / 4.0 * hoverThrustScale
        let thrusts = isSingleLift
            ? try MotorThrusts(f1: hoverThrust, f2: 0, f3: 0, f4: 0)
            : try MotorThrusts.uniform(hoverThrust)
        return ReferenceQuadrotorWorldStore(state: state, motorThrusts: thrusts)
    }
}
