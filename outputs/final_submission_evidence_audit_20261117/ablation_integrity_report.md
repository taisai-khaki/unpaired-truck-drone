# M2/M6/M7 diagnostic gate

## Decision

**BLOCKED — do not launch the 2,160-run M2/M6/M7 experiment yet.** The gate ran 12/12 sampled method-instance cases. One M2 result is infeasible, and the frozen principal objectives do not reproduce exactly under the corrected schedule evaluator. No full M2/M6/M7 run was started, and no results have been substituted or synthesized.

## Diagnostic sample

The four frozen samples were `S0001_R01` (n=15), `S0034_R01` (n=50), `S0061_R09` (n=100), and `S0026_R08` (n=50). They span realized drone eligibility 0.24–0.76 and contain non-depot rendezvous opportunity. All exact input and saved-backbone hashes matched the frozen manifest.

| Method | Runs | Feasible and architecture-valid | Gate result |
|---|---:|---:|---|
| M2 simple-drone construction | 4 | 3/4 | Fail |
| M6 NILS without post-improvement local search | 4 | 4/4 | Pass on sample |
| M7 NILS without perturbation | 4 | 4/4 | Pass on sample |

The single invalid result is M2 on `S0061_R09` (n=100): trucks 1 and 2 exceed the 480-minute route shift. The method returned its constructed baseline with an infeasible status; it was preserved and not replaced with truck-only service. M2 is a constructive benchmark, so this failure does not prove mathematical infeasibility.

M6 and M7 reused the same saved truck-route hashes as M5 on all 8 NILS-ablation rows. Their flags were correct in every sampled run: M6 had local search disabled / perturbation enabled; M7 had local search enabled / perturbation disabled. The independently computed starting truck-backbone objective was shared across methods for each instance and matches the matched M5 truck-only reference. For all 12 rows, the recorded objective equals truck cost + drone cost + weighted tardiness to displayed precision.

## Frozen reference integrity

The immutable principal directory's manifest contains 5,782 artifact entries; none is missing or changed. All 720 inputs remain present and all 2,880 M3/M4/M5/M8 schedules remain feasible and architecture-valid after corrected re-evaluation. No previously accepted solution was invalidated. However, 227 stored M3/M4/M5 objective values change by more than `1e-6` because the current evaluator recomputes synchronization waiting from current readiness instead of retaining obsolete waits. Counts: M3 65, M4 62, M5 100, M8 0. The maximum absolute M5 objective change is 13,799.3240. Therefore, the strict requirement that archived objectives reproduce the historical tables within rounding is not met; corrected derivative summaries are not replacements for or edits to the immutable results.

The matched M1 reference reproduces on 720/720 instances. The real-data audit re-evaluates 56 M1–M7 records with no objective changes; 14 records at lambda 0.150/0.200 fail the 480-minute shift checks (all seven methods at each scale). Those are heuristic schedule failures, not mathematical infeasibility certificates.

## Initial-incumbent match audit

The saved M6/M7 diagnostic records were reconciled offline against the matched M5 rows; no optimization was rerun. All 8/8 M6/M7 records match M5's initial incumbent in route hash, method seed, and initial truck-backbone objective, and report the backbone as reused. The check is now persisted in `ablation_gate_diagnostics.csv` and `ablation_gate_summary.json`. Both M6 and M7 also retain their intended single-component ablation flags on all four sampled instances.

## Why no full run

The gate remains blocked because (1) M2 returns an infeasible schedule on `S0061_R09` (n=100), and (2) corrected-evaluator revalidation changes 227 frozen principal objective records. The initial-incumbent reproducibility gap has been closed, but it does not resolve either blocking condition.

Do not generate `new_method_results_2160.csv`, all-eight-method tables, or M5-vs-M6/M7 tests from historical values. Those files do not exist because those 2,160 runs were not performed. Resolve the gate policy and M2 handling first, rerun the small gate with explicit objective/initial-incumbent checks, and only then decide whether to run the full design.
