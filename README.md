# kuyu-scenarios

Scenario definitions, evaluation suites, and logging for the Kuyu simulation environment.

## Overview

kuyu-scenarios provides the evaluation infrastructure for testing controllers. It defines scenario suites, runs simulations with specific configurations, evaluates performance metrics, and produces structured logs.

### Evaluation Suites

- **KuyAtt1Suite** — Attitude stabilization scenarios (hover, step response, disturbance rejection).
- **KuyLiftSuite** / **KuySingleLiftSuite** — Lift control scenarios for single-propeller platforms.
- **ParametricScenarioGenerator** — Generates scenario variants by sweeping parameters.

### Runtime

- **`PlantScenarioRunner`** — Executes a single scenario with a given controller and produces logs.
- **`PlantScenarioSuiteRunner`** — Runs a full suite of scenarios and aggregates results.
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
