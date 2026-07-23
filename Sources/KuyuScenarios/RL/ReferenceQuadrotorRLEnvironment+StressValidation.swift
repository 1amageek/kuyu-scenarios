extension ReferenceQuadrotorRLEnvironment {
    func validateSingleLiftStress(definition: ReferenceQuadrotorScenarioDefinition) throws {
        guard definition.torqueEvents.isEmpty else {
            throw EnvironmentError.unsupportedSingleLiftStress("torqueEvents")
        }
        guard definition.actuatorDegradation == nil else {
            throw EnvironmentError.unsupportedSingleLiftStress("actuatorDegradation")
        }
        for event in definition.hfEvents {
            switch event.kind {
            case .impulse, .vibration:
                throw EnvironmentError.unsupportedSingleLiftStress("hfEvent.\(event.kind.rawValue)")
            default:
                break
            }
        }
        for event in definition.swapEvents {
            switch event {
            case .actuator(let actuator) where actuator.motorIndex != 0:
                throw EnvironmentError.unsupportedSingleLiftStress("actuatorSwap.motorIndex.\(actuator.motorIndex)")
            case .sensor(let sensor) where sensor.targetChannels.contains(where: { $0 > 7 }):
                throw EnvironmentError.unsupportedSingleLiftStress("sensorSwap.targetChannel")
            default:
                break
            }
        }
    }
}
