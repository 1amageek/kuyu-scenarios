# kuyu-scenarios

Scenario definitions, evaluation suites, and logging for the Kuyu simulation environment.

## Overview

kuyu-scenarios provides the evaluation infrastructure for testing controllers. It defines scenario suites, runs simulations with specific configurations, evaluates performance metrics, and produces structured logs.

## Responsibility Boundary

`kuyu-scenarios` is the authority for simulation-task semantics. Code outside
this package may execute or learn from scenarios, but it should not reinterpret
their target state, safety rules, reward meaning, or pass/fail semantics.

| Owned here | Not owned here |
|---|---|
| Scenario definitions and deterministic seeds | Training loop scheduling |
| Safety, failure, truncation, and task-quality semantics | PPO/BC/evolution algorithms |
| Reward functions and `RewardDescriptor` provenance | Checkpoint selection or publication |
| Scenario-derived references such as altitude-hold target/tolerance | Dataset persistence formats beyond scenario logs |
| Simulation logs and replay validation | MLX model architecture or optimizer details |

### Reliability Contract

- Scenario-derived quantities must be represented by typed APIs rather than
  duplicated fallbacks in backend packages.
- Reward changes must bump `RewardDescriptor.version` when behavior changes
  without a config-weight change.
- `ScenarioTerminalFacts` must validate terminal state consistency before the
  facts are persisted by downstream dataset or artifact writers.
- Reference-quadrotor suite IDs must resolve through
  `ReferenceQuadrotorScenarioCatalog`; backend and training packages must not
  keep independent suite-ID-to-scenario semantics.
- Stress-suite coverage must be expressed through `StressSuiteManifest`, with
  explicit coverage targets and replay evidence, so downstream packages do not
  reinterpret disturbance, degradation, drift, swap, high-frequency stress, or
  M2 long-horizon/planner/morphology/partial-observability semantics locally.
- Persisted stress-suite manifests must be written and reloaded through
  `StressSuiteManifestArtifactStore` so decoded coverage counts, replay
  evidence, semantic M2 evidence, and artifact-root containment remain
  scenario-owned at saved-artifact boundaries.
- Reference M2 benchmark cases must validate through
  `LongHorizonBenchmarkSuite.validatedReferenceM2TrackCounts`; morphology
  transfer requires `LongHorizonMorphologyTransferContract`, and
  disturbance/delay/partial-observability cases must carry concrete
  disturbance, latency, and sensor-observability evidence before a manifest can
  be accepted. Accepted reference M2 manifests persist
  `ReferenceM2BenchmarkEvidence`, so downstream project-evidence packs can
  distinguish semantic case evidence from dimension counts alone.
- Planner degradation must flow through `DescendingIntentProgram` and
  `PlannerExecutorBridge` so missing or disconnected conscious-layer input
  degrades to bounded hold behavior instead of becoming a direct actuator
  command path or a backend-local convention.
- Baseline reference-quadrotor starter execution must use
  `ReferenceQuadrotorBaselineReplayRuntime` so attitude, lift, and single-lift
  tasks all record deterministic replay evidence.
- Attitude scenarios without a `LiftEnvelope` still own an altitude-hold
  reference: the initial z position is the hover target, with the package-level
  tolerance and velocity reference exposed through
  `ReferenceQuadrotorAltitudeHoldReference`.
- Backends such as `kuyu-mlx` must consume these scenario references instead of
  hard-coding fallback target heights.

Package-local reliability milestones are defined in
`RELIABILITY_MILESTONES.md`, and package-local verification evidence is recorded
in `RELIABILITY_EVIDENCE.md`.

### Evaluation Suites

- **KuyAtt1Suite** — Attitude stabilization scenarios (hover, step response, disturbance rejection).
- **KuyLiftSuite** / **KuySingleLiftSuite** — Lift control scenarios for single-propeller platforms.
- **ReferenceQuadrotorScenarioCatalog** — Canonical task/suite resolver for runnable starter and regression scenario selection.
- **ParametricScenarioGenerator** — Generates scenario variants by sweeping parameters.
- **StressSuiteManifest** — Typed stress coverage and replay-evidence gate for reference quadrotor, reference M2 benchmark, and articulated dynamic scenario bundles; schema, coding, factory, record mapping, coverage validation, replay validation, and Reference M2 evidence checks live in focused files so coverage contracts remain auditable as they grow.
- **StressSuiteManifestArtifactStore** — Scenario-owned saved-artifact boundary for stress-suite manifests with validation and artifact-root containment.
- **StressSuiteManifest.ReferenceM2BenchmarkEvidence** — Persisted reference M2 track, morphology-transfer, disturbance, latency, and partial-observability case evidence for downstream adoption gates.
- **LongHorizonMorphologyTransferContract** — Typed descriptor contrast required before reference M2 morphology-transfer cases can be accepted.

### Runtime

- **`PlantScenarioRunner`** — Executes a single scenario with a given controller and produces logs.
- **`PlantScenarioSuiteRunner`** — Runs a full suite of scenarios and aggregates results.
- **`ReferenceQuadrotorRLEnvironment`** — Scenario-owned RL reset/step environment for reference attitude, lift, and single-lift tasks. One `step` applies the current policy action before physics, holds it for one Cut/MotorNerve control period, aggregates reward, and checks failure after every enclosed physics tick. State, reset, step, observation, world-model validation, simulator construction, stress validation, and support helpers live in focused files so these semantics remain auditable.
- **`ReferenceQuadrotorEnvironmentExecutionContract`** — Fail-closed schema-v2 identity for an RL rollout. It binds the canonical executor version in addition to simulation, action realization, morphology parameters, and MotorNerve settings, so Swift scalar, Mojo numeric variants, and MLX tensor execution cannot share an ambiguous `configHash`.
- **`ReferenceQuadrotorScenarioRunner`** — Accepts one canonical executor and propagates it to every canonical Plant path and the full-quadrotor `IMU6SensorField` for the complete run. The constrained single-prop sensor retains its dedicated analytical field. Backend failures remain run failures; the runner does not select a scalar fallback.
- **`ReferenceQuadrotorBaselineReplayRuntime`** — Executes baseline reference-quadrotor attitude/lift/single-lift suites with replay verification enabled.
- **`ScenarioRunner`** — Sequential execution of independent scenarios.
- **`DescendingIntentProgram`** — Time-varying descending channel commands (keyframe interpolation).
- **`PlannerExecutorBridge`** — Fixed-rate descending channel bridge with snapshot evidence for planner disconnect and hold behavior.

### Evaluation Metrics

- **`ScenarioEvaluation`** — Pass/fail with detailed metrics (settling time, overshoot, steady-state error).
- **`ControlQualityMetrics`** — Quantitative control performance.
- **`RobustnessStatistics`** — Suite-level robustness analysis.
- **`SafetyEnvelope`** — Safety boundary checking.

### Logging

- **`ScenarioLogBundle`** — Structured log output per scenario run.
- **`KuyAtt1LogWriter`** — JSON-based log serialization.
- **`LogStore`** — In-memory log management.

## Package Structure

| Module | Dependencies | Description |
|--------|-------------|-------------|
| **KuyuScenarios** | KuyuCore, KuyuPhysics, swift-log, swift-configuration | All scenario infrastructure |

## Requirements

- Swift 6.2+
- macOS 26+

## Dependency Graph

```
KuyuCore
  |
  +-- KuyuPhysics
        |
        +-- KuyuScenarios (this package)
              |
              +-- kuyu-training  (uses scenarios for data collection)
              +-- kuyu           (runs scenarios via UI/CLI)
```

## License

See repository for license information.
