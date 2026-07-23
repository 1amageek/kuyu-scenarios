import KuyuCore

extension ReferenceQuadrotorRLEnvironment {
    mutating func initialObservation() throws -> EnvironmentObservation {
        guard var simulator else { throw EnvironmentError.notReset }
        let time = try WorldTime(stepIndex: 0, time: 0.0)
        let environmentObservation: EnvironmentObservation
        switch simulator {
        case .quad(var quad):
            environmentObservation = EnvironmentObservation(
                time: time,
                sensorSamples: try quad.sensor.sample(time: time),
                plantState: quad.plant.snapshot(),
                safetyTrace: try quad.plant.safetyTrace(),
                actuatorTelemetry: quad.actuator.telemetrySnapshot(),
                disturbances: quad.disturbance.snapshot()
            )
            simulator = .quad(quad)
        case .lift(var lift):
            environmentObservation = EnvironmentObservation(
                time: time,
                sensorSamples: try lift.sensor.sample(time: time),
                plantState: lift.plant.snapshot(),
                safetyTrace: try lift.plant.safetyTrace(),
                actuatorTelemetry: lift.actuator.telemetrySnapshot(),
                disturbances: lift.disturbance.snapshot()
            )
            simulator = .lift(lift)
        case .single(var single):
            environmentObservation = EnvironmentObservation(
                time: time,
                sensorSamples: try single.sensor.sample(time: time),
                plantState: single.plant.snapshot(),
                safetyTrace: try single.plant.safetyTrace(),
                actuatorTelemetry: single.actuator.telemetrySnapshot(),
                disturbances: single.disturbance.snapshot()
            )
            simulator = .single(single)
        }
        self.simulator = simulator
        return environmentObservation
    }
}
