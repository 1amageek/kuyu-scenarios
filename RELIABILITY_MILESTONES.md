# Kuyu Scenarios Reliability Milestones

This document defines the local reliability ladder for `kuyu-scenarios`.
`../KUYU_CAPABILITY_ROADMAP.md` owns the cross-package capability order. This
file owns the package-local sequence that must be completed before downstream
training, backend, or app code treats scenario semantics as stable. Evidence is
recorded in `RELIABILITY_EVIDENCE.md`.

## End State

`kuyu-scenarios` is reliable when scenario truth can be resolved, executed,
evaluated, logged, replay-checked, and consumed by downstream packages without
duplicating task, reward, safety, or pass/fail semantics.

```mermaid
flowchart LR
  Catalog["Scenario catalog"]
  Runtime["Scenario runtime"]
  Evaluation["Task / safety evaluation"]
  Replay["Replay validation"]
  Report["Typed reports"]
  Downstream["Training / MLX / app consumers"]

  Catalog --> Runtime
  Runtime --> Evaluation
  Runtime --> Replay
  Evaluation --> Report
  Replay --> Report
  Report --> Downstream
```

## Advancement Rule

New work in `kuyu-scenarios` should advance the first incomplete milestone
unless there is a blocking defect in an earlier milestone. A milestone is
complete only when contract, implementation, fail-closed validation, regression
tests, and evidence all agree.

| Requirement | Completion meaning |
|---|---|
| Contract | Scenario-owned semantics are documented and represented by public types. |
| Runtime path | Production execution uses the scenario-owned catalog, runtime, and evaluators. |
| Fail-closed gate | Invalid, stale, missing, or ambiguous scenario evidence is rejected. |
| Regression tests | Positive and negative cases cover the scenario invariant. |
| Evidence | The verification command and scope are recorded. |

## Milestones

| ID | Name | Status | Purpose | Completion gate |
|---|---|---|---|---|
| KS0 | Responsibility baseline | Complete | Keep `kuyu-scenarios` scoped to scenario truth, reward, safety, runtime, logging, and replay semantics. | README boundary, package-local reliability docs, root static gate, and package-level xcodebuild test. |
| KS1 | Catalog and suite authority | Complete for current reference-quadrotor suites | Ensure task and suite resolution goes through public scenario-owned catalog APIs. | `ReferenceQuadrotorScenarioCatalog` tests for attitude, long-horizon attitude, lift, single-lift, and invalid requests. |
| KS2 | Runtime and replay truth | Complete for current baseline paths | Ensure runnable scenario execution records deterministic replay evidence instead of relying on caller trust. | Baseline replay runtime tests, deterministic replay tests, and explicit disabled-replay recording tests. |
| KS3 | Task, reward, and safety semantics | Complete for current public evaluators | Ensure pass/fail, reward descriptors, altitude-hold references, and safety costs are scenario-owned and validated. | Scenario terminal fact tests, task-quality tests, dense-reward/reference tests, and safety-cost descriptor tests. |
| KS4 | Downstream report readiness | Complete for current conformance/report paths | Ensure downstream packages consume typed reports that fail closed on missing replay or empty suites. | `A1ConformanceReportFactory` positive and negative tests plus root boundary gate linkage. |
| KS5 | Individual reliability baseline | Complete for current package-local baseline | Make `kuyu-scenarios` independently auditable before additional integration work depends on it. | `RELIABILITY_EVIDENCE.md`, root individual reliability map, static gate requirements, and package-level xcodebuild test. |

## Dependency Order

```mermaid
flowchart TB
  KS0["KS0 Responsibility baseline"]
  KS1["KS1 Catalog authority"]
  KS2["KS2 Runtime replay truth"]
  KS3["KS3 Task / reward / safety"]
  KS4["KS4 Downstream reports"]
  KS5["KS5 Individual reliability"]

  KS0 --> KS1
  KS1 --> KS2
  KS2 --> KS3
  KS3 --> KS4
  KS4 --> KS5
```

## KS0: Responsibility Baseline

Status: complete.

Owned responsibility:

| Owned | Not owned |
|---|---|
| Scenario definitions, deterministic seeds, task-quality semantics, reward descriptors, safety envelopes, replay validation, scenario logs, and conformance reports | Training loop scheduling, PPO/BC/evolution algorithms, checkpoint selection, MLX model architecture, app rendering, or hardware-parity claims |

Acceptance evidence:

| Evidence | Required state |
|---|---|
| `README.md` | Documents responsibility boundary and reliability contract. |
| `RELIABILITY_MILESTONES.md` | Defines this package-local reliability ladder. |
| `RELIABILITY_EVIDENCE.md` | Records package-local verification commands and scope. |
| `../scripts/validate-unconscious-boundaries.sh` | Requires the scenario reliability files, links, tests, and source-safety gate. |
| Swift safety gate | Source validation rejects `try?`, `DispatchQueue`, `EventLoopFuture`, and `@unchecked Sendable` in package sources. |

## KS1: Catalog and Suite Authority

Status: complete for current reference-quadrotor suites.

Scenario selection must resolve through the package-owned catalog so downstream
training and MLX code cannot drift into separate suite identity maps.

Acceptance evidence:

| Invariant | Gate |
|---|---|
| A1 attitude suite resolution is package-owned. | `referenceQuadrotorScenarioCatalogResolvesA1AttitudeSuite`. |
| Long-horizon attitude suite resolution is package-owned. | `referenceQuadrotorScenarioCatalogResolvesLongHorizonAttitudeSuite`. |
| Lift and single-lift variations resolve through the same catalog. | `referenceQuadrotorScenarioCatalogResolvesLiftSuiteVariations` and `referenceQuadrotorScenarioCatalogResolvesSingleLiftSuiteVariations`. |
| Invalid catalog requests fail closed. | `referenceQuadrotorScenarioCatalogRejectsInvalidRequests`. |

## KS2: Runtime and Replay Truth

Status: complete for current baseline paths.

Runnable scenario output must carry replay evidence or explicitly record that
replay was disabled for the specific path. Baseline attitude, lift, and
single-lift replay execution is owned by `ReferenceQuadrotorBaselineReplayRuntime`.

Acceptance evidence:

| Invariant | Gate |
|---|---|
| Same-seed deterministic replay is bit-exact across stress runs. | `sameSeedReplayIsBitExactAcrossStress`. |
| Different seeds change trajectories. | `differentSeedChangesTrajectory`. |
| Baseline replay runtime performs lift and single-lift replay. | `baselineReplayRuntimePerformsLiftReplay` and `baselineReplayRuntimePerformsSingleLiftReplay`. |
| Baseline replay runtime rejects unsupported controllers. | `baselineReplayRuntimeRejectsNonBaselineController`. |
| Teacher runner records disabled replay explicitly when configured. | `kuyAtt1TeacherRunnerRecordsDisabledReplayExplicitly`. |

## KS3: Task, Reward, and Safety Semantics

Status: complete for current public evaluators.

Task success, terminal facts, altitude-hold references, reward descriptors, and
safety costs belong in this package. Downstream packages may consume them, but
must not reinterpret them.

Acceptance evidence:

| Invariant | Gate |
|---|---|
| Terminal fact combinations are validated before persistence. | `scenarioTerminalFactsValidateCompletedAndFailedStates` and `scenarioTerminalFactsRejectInvalidStateCombinations`. |
| Lift task quality accepts settled logs and rejects unsettled logs. | `taskQualityEvaluatorAcceptsSettledLiftLog` and `taskQualityEvaluatorRejectsUnsettledLiftLog`. |
| Attitude altitude references come from scenario authority. | `altitudeHoldReferenceUsesScenarioAuthority`. |
| Dense reward uses scenario altitude reference when no lift envelope exists. | `denseRewardPenalizesAttitudeAltitudeErrorWithoutLiftEnvelope`. |
| Safety cost is bounded, descriptor-backed, and input-validated. | `safetyCostIsZeroInsideMargin`, `safetyCostRisesLinearlyFromMarginToLimit`, `safetyCostCapsBeyondEnvelopeAndAddsViolationCost`, `safetyCostConfigRejectsInvalidValues`, and `safetyCostDescriptorTracksConfig`. |

## KS4: Downstream Report Readiness

Status: complete for current conformance/report paths.

Reports published from `kuyu-scenarios` must be safe for downstream packages to
consume without recomputing acceptance.

Acceptance evidence:

| Invariant | Gate |
|---|---|
| Empty conformance suites fail closed. | `a1ConformanceReportFactoryRejectsEmptySuiteList`. |
| Reports require replay to have been performed. | `a1ConformanceReportFactoryRequiresPerformedReplay`. |
| Empty replay checks fail closed. | `a1ConformanceReportFactoryRejectsEmptyReplayChecks`. |
| Only passing suites with passing replay are accepted. | `a1ConformanceReportFactoryAcceptsOnlyPassingSuitesWithPassingReplay`. |

## KS5: Individual Reliability Baseline

Status: complete for current package-local baseline.

The package is independently auditable when this file, `RELIABILITY_EVIDENCE.md`,
README linkage, root individual reliability linkage, the source-safety static
gate, and package-level xcodebuild tests all pass.

Remaining maintenance rule: any new scenario truth boundary must add a positive
and negative regression test before downstream packages may depend on it. Any
new downstream consumer that treats scenario output as accepted must consume a
typed scenario-owned report, replay validator, or stricter package-local gate.
