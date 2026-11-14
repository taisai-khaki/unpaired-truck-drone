# Definitive 720-instance experiment

- Dataset status: FINAL_720_SEMANTICS_CORRECTED_FROZEN
- Execution commit: 8abdc6b7d9f82a7dc630468e4f108e48f2246e84
- Configuration SHA256: 3a1162b6fba0ec62fa1c8f0b1d84e968ec24cfc46a8b6002c4f463c9394da249
- Frozen instances: 720
- Method-instance runs: 2880
- Historical method rows resumed: 0
- Experiment-level repair/fallback: disabled

## Validity

| Method | Feasible | Architecture-valid |
|---|---:|---:|

The earlier runner attempt was stopped after one M3 record when its reuse-marker audit read objective-only components. The saved raw record confirms the backbone was reused and unchanged; that isolated row was not resumed or included in this dataset.
| feasibility_aware_alns | 720/720 | 720/720 |
| nils | 720/720 | 720/720 |
| no_unpairing | 720/720 | 720/720 |
| paired_baseline | 720/720 | 720/720 |

Invalid or architecture-invalid method outputs retained: 0 infeasible, 0 architecture-invalid.

## Overall results

| Method | Mean objective | Median objective | Mean HP tardiness | Mean elapsed wall time (s) |
|---|---:|---:|---:|---:|
| paired_baseline | 81764.3 | 6469.64 | 851.072 | 10.537 |
| no_unpairing | 76082.8 | 6307.44 | 784.214 | 18.695 |
| nils | 80225.4 | 6934.41 | 809.017 | 33.291 |
| feasibility_aware_alns | 50263.4 | 7420.47 | 554.33 | 60.793 |

## Results by size

| n | Method | Mean objective | Median objective | Mean HP tardiness | Mean elapsed wall time (s) |
|---:|---|---:|---:|---:|---:|
| 15 | feasibility_aware_alns | 1596.35 | 175.222 | 20.9779 | 8.876 |
| 15 | nils | 478.16 | 125.905 | 4.86642 | 0.687 |
| 15 | no_unpairing | 297.15 | 125.685 | 4.09094 | 0.527 |
| 15 | paired_baseline | 329.781 | 133.07 | 4.69109 | 0.526 |
| 50 | feasibility_aware_alns | 32184.9 | 21377.4 | 327.976 | 39.331 |
| 50 | nils | 25758.7 | 11445.5 | 253.365 | 7.195 |
| 50 | no_unpairing | 24794.3 | 12395.1 | 252.164 | 4.444 |
| 50 | paired_baseline | 30203 | 14328.8 | 285.379 | 4.190 |
| 75 | feasibility_aware_alns | 41337.7 | 17360.2 | 720.339 | 116.046 |
| 75 | nils | 56341.1 | 19379.3 | 910.069 | 32.283 |
| 75 | no_unpairing | 56735.4 | 18912.5 | 905.894 | 13.312 |
| 75 | paired_baseline | 63840.6 | 28103.7 | 1073.9 | 8.856 |
| 100 | feasibility_aware_alns | 133386 | 100522 | 1371.51 | 121.188 |
| 100 | nils | 248275 | 189068 | 2348.74 | 102.078 |
| 100 | no_unpairing | 233675 | 198266 | 2258.99 | 58.989 |
| 100 | paired_baseline | 245822 | 214114 | 2395.33 | 30.466 |

## Matched comparisons

| A vs B | Valid pairs | Wins–ties–losses (A) | Mean improvement (%) | Median improvement (%) | Wilcoxon p | Rank-biserial effect |
|---|---:|---:|---:|---:|---:|---:|
| paired_baseline vs no_unpairing | 720 | 234–0–486 | -41.0726 | -5.7237 | 5.74874e-18 | -0.3716 |
| no_unpairing vs nils | 720 | 379–49–292 | -5.1821 | 0.0887 | 0.00116812 | 0.1447 |
| paired_baseline vs nils | 720 | 267–1–452 | -34.3727 | -4.4877 | 2.76688e-05 | -0.1805 |
| nils vs feasibility_aware_alns | 720 | 324–0–396 | -26.3291 | -5.2363 | 1.15272e-06 | -0.2092 |

## Interpretation guardrails

The tables evaluate the solutions returned by each algorithm without harness repair or fallback. Paired tests use instances where both outputs pass full feasibility and their required architecture validator. Invalid outputs remain in the raw data and validation tables.

M3–M4 compares fixed-pair and no-cross-truck coordination; M4–M5 describes the incremental unrestricted cross-truck architecture. These are descriptive comparisons under the frozen common-backbone design. Runtime is elapsed wall time measured with the same monotonic clock as the algorithm deadline.

## Deadline audit

The configured limit was 120 seconds, but the implementation does not hard-kill an algorithm at that instant; it checks the deadline at operation/iteration boundaries. Actual elapsed times are retained in the raw results and summarized in `tables/time_limit_audit_by_size.csv`. Runs exceeded 120 seconds for ALNS (271/720; maximum 123.541 s), NILS (99/720; maximum 121.309 s), M4/no-unpairing (3/720; maximum 120.016 s), and M3/paired baseline (0/720). Thus, describe 120 seconds as the configured nominal limit, not a strict hard wall-clock cutoff. At n=100, ALNS averaged 1.4 iterations per run; those runs and their actual runtimes remain included without alteration.
