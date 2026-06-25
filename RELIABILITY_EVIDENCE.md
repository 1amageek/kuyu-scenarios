# Kuyu Scenarios Reliability Evidence

This file records package-local evidence for `RELIABILITY_MILESTONES.md`.
Entries must identify the verification command, the files or gates that prove
the claim, and what remains outside package-local reliability.

## Evidence Entries

| ID | Milestone | Status | Verification | Evidence | Scope |
|---|---|---|---|---|---|
| 2026-06-25-ks5-individual-reliability-baseline | KS5 individual reliability baseline | Passed for the current package-local self-verification baseline | `git -C /Users/1amageek/Desktop/Robot/unconscious diff --check`; `git -C /Users/1amageek/Desktop/Robot/unconscious/kuyu-scenarios diff --check`; `/Users/1amageek/Desktop/Robot/unconscious/scripts/validate-unconscious-boundaries.sh`; `TEST_TIMEOUT_SECONDS=120 /Users/1amageek/Desktop/Robot/unconscious/scripts/test.sh kuyu-scenarios` (63 tests) | `README.md`, `RELIABILITY_MILESTONES.md`, `RELIABILITY_EVIDENCE.md`, `Tests/KuyuScenariosTests/ReferenceQuadrotorScenarioCatalogTests.swift`, `Tests/KuyuScenariosTests/ReferenceQuadrotorBaselineReplayRuntimeTests.swift`, `Tests/KuyuScenariosTests/DeterminismReplayTests.swift`, `Tests/KuyuScenariosTests/A1ConformanceReportFactoryTests.swift`, `../INDIVIDUAL_RELIABILITY_MILESTONES.md`, `../scripts/validate-unconscious-boundaries.sh` | Proves `kuyu-scenarios` has a package-local reliability ladder, evidence file, README linkage, root individual reliability linkage, source-safety static gate, and package-level `xcodebuild` test coverage through `scripts/test.sh`. The covered baseline includes scenario catalog authority, deterministic replay checks, baseline replay runtime behavior, terminal facts, task-quality evaluation, safety cost descriptors, altitude-hold reference authority, and conformance report fail-closed behavior. This does not prove learned-policy quality, C6 long-horizon G1 success, RoArm M1 hardware parity, or every downstream consumer path. |
