import KuyuCore

extension ReferenceQuadrotorRLEnvironment {
    func episodeInfo(
        definition: ReferenceQuadrotorScenarioDefinition,
        configHash: String,
        stepCount: Int,
        rewardSum: Double,
        terminalFailure: FailureEvent?,
        terminalReason: String?
    ) -> EpisodeInfo {
        EpisodeInfo(
            scenarioId: definition.config.id,
            seed: definition.config.seed,
            configHash: configHash,
            robotManifestID: robotManifestID,
            policyId: nil,
            rewardDescriptor: rewardFunction.descriptor,
            stepCount: stepCount,
            rewardSum: rewardSum,
            failureReason: terminalFailure?.reason,
            failureTime: terminalFailure?.time,
            terminalReason: terminalReason
        )
    }
}
