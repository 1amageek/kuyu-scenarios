import KuyuCore
import KuyuPhysics

public protocol ReferenceQuadrotorScenarioSuite {
    func scenarios() throws -> [ReferenceQuadrotorScenarioDefinition]
}
