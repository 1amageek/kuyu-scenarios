import KuyuCore
import KuyuPhysics

extension ReferenceQuadrotorRLEnvironment {
    func validatedResetPlan(
        seed: ScenarioSeed,
        scenario: ReferenceQuadrotorScenarioDefinition
    ) throws -> (physicsStepCount: Int, controlStepCount: Int, simulationConfig: SimulationConfig) {
        guard seed == scenario.config.seed else {
            throw EnvironmentError.seedMismatch(expected: scenario.config.seed.rawValue, actual: seed.rawValue)
        }
        guard schedule.sensor.periodSteps == 1,
              schedule.motorNerve?.periodSteps == schedule.cut.periodSteps else {
            throw EnvironmentError.unsupportedControlSchedule(
                cutPeriodSteps: schedule.cut.periodSteps,
                motorNervePeriodSteps: schedule.motorNerve?.periodSteps,
                sensorPeriodSteps: schedule.sensor.periodSteps
            )
        }
        let horizon: SimulationStepHorizon
        do {
            horizon = try SimulationStepHorizon(
                duration: scenario.config.duration,
                physicsTimeStep: scenario.config.timeStep.delta,
                controlPeriodSteps: schedule.cut.periodSteps
            )
        } catch {
            throw EnvironmentError.invalidStepLimit
        }
        if actionRealization.kind == .temporalCTBR {
            switch scenario.kind {
            case .hoverStart, .impulseTorqueShock, .sustainedWindTorque, .sensorDriftStress, .actuatorDegradation:
                break
            case .singleLiftHover, .liftHover:
                throw EnvironmentError.unsupportedActionRealization(
                    actionKind: actionRealization.kind,
                    scenarioKind: scenario.kind
                )
            }
        }
        return (
            physicsStepCount: horizon.physicsStepCount,
            controlStepCount: horizon.controlStepCount,
            simulationConfig: SimulationConfig(
                scenario: scenario.config,
                schedule: schedule,
                determinism: determinism,
                environment: worldEnvironment
            )
        )
    }
}
