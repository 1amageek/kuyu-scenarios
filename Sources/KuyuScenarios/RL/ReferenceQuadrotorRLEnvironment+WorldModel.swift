import KuyuCore

extension ReferenceQuadrotorRLEnvironment {
    func validateWorldModelPrediction(reference output: EnvironmentStep) throws {
        guard let worldModelAdapter else { return }
        let prediction = try worldModelAdapter.predict(reference: output)
        _ = try worldModelAdapter.validate(
            prediction: prediction,
            reference: output,
            configuration: worldModelAdapterConfiguration
        )
    }
}
