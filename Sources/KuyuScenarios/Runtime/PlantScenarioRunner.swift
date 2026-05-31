import KuyuCore
import KuyuPhysics

public protocol PlantScenarioRunner {
    associatedtype Scenario
    associatedtype Cut: CutInterface
    associatedtype Nerve: MotorNerveEndpoint

    func runScenario(
        definition: Scenario,
        cut: sending Cut,
        motorNerve: Nerve?,
        control: SimulationControl?,
        telemetry: WorldStepTelemetry?
    ) async throws -> SimulationLog
}
