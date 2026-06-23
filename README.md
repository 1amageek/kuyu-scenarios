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
- Baseline reference-quadrotor starter execution must use
  `ReferenceQuadrotorBaselineReplayRuntime` so attitude, lift, and single-lift
  tasks all record deterministic replay evidence.
- Attitude scenarios without a `LiftEnvelope` still own an altitude-hold
  reference: the initial z position is the hover target, with the package-level
  tolerance and velocity reference exposed through
  `ReferenceQuadrotorAltitudeHoldReference`.
- Backends such as `kuyu-mlx` must consume these scenario references instead of
  hard-coding fallback target heights.

### Evaluation Suites

- **KuyAtt1Suite** — Attitude stabilization scenarios (hover, step response, disturbance rejection).
- **KuyLiftSuite** / **KuySingleLiftSuite** — Lift control scenarios for single-propeller platforms.
- **ReferenceQuadrotorScenarioCatalog** — Canonical task/suite resolver for runnable starter and regression scenario selection.
- **ParametricScenarioGenerator** — Generates scenario variants by sweeping parameters.

### Runtime

- **`PlantScenarioRunner`** — Executes a single scenario with a given controller and produces logs.
- **`PlantScenarioSuiteRunner`** — Runs a full suite of scenarios and aggregates results.
- **`ReferenceQuadrotorBaselineReplayRuntime`** — Executes baseline reference-quadrotor attitude/lift/single-lift suites with replay verification enabled.
- **`ScenarioRunner`** — Sequential execution of independent scenarios.
- **`DescendingIntentProgram`** — Time-varying descending channel commands (keyframe interpolation).

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
