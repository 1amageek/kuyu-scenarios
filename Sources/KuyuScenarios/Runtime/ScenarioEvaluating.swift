import KuyuCore
import KuyuPhysics

public protocol ScenarioEvaluating {
    associatedtype Scenario

    func evaluate(definition: Scenario, log: SimulationLog) -> ScenarioEvaluation
}
