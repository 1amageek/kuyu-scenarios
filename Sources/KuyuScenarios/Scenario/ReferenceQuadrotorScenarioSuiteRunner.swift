import KuyuCore
import KuyuPhysics

public typealias ReferenceQuadrotorScenarioSuiteRunner<Cut: CutInterface, Nerve: MotorNerveEndpoint> = PlantScenarioSuiteRunner<
    ReferenceQuadrotorScenarioRunner<Cut, Nerve>,
    ReferenceQuadrotorScenarioEvaluator
>
