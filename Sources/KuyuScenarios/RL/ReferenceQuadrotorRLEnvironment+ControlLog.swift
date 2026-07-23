import KuyuCore

extension ReferenceQuadrotorRLEnvironment {
    func log(
        _ log: WorldStepLog,
        applying application: WorldControlApplication
    ) -> WorldStepLog {
        WorldStepLog(
            time: log.time,
            events: application.events + log.events,
            sensorSamples: log.sensorSamples,
            driveIntents: application.driveIntents,
            reflexCorrections: application.reflexCorrections,
            actuatorValues: application.actuatorValues,
            actuatorTelemetry: log.actuatorTelemetry,
            motorNerveTrace: application.motorNerveTrace,
            safetyTrace: log.safetyTrace,
            plantState: log.plantState,
            disturbances: log.disturbances
        )
    }
}
