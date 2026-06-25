import KuyuCore
import KuyuPhysics

public protocol PlantScenarioRunner {
    associatedtype Scenario
    associatedtype Cut: CutInterface
    associatedtype Nerve: MotorNerveEndpoint

    nonisolated(nonsending) func runScenario(
        definition: Scenario,
        cut: Cut,
        motorNerve: Nerve?,
        control: SimulationControl?,
        telemetry: WorldStepTelemetry?
    ) async throws -> SimulationLog
}
