# NILS Timeout-Salvage Diagnostic

## Finding

The reported timeout defect is real. In the pre-fix M5 displacement search, reaching the deadline returned `None` immediately, discarding the ranked candidates already evaluated. The outer NILS loop also stopped as soon as the deadline passed, even if a candidate had been returned. The corrected code stops generating candidates, strictly checks the accumulated candidates within the existing cap of 12, and lets the main loop process a returned feasible candidate. It skips additional perturbation and local search after the deadline. No truck neighborhood, 2-opt objective, or NILS search architecture was changed.

The fix is verified by controlled-clock tests: a candidate generated before expiry is strictly validated, returned, and accepted by the outer loop after expiry. Per-run diagnostics record customers scanned, sortie options, local-screen rejections, full candidates evaluated, strict checks, and deadline termination.

## Matched Diagnostic

- Thirty matched instances: 10 each at `n=50`, `n=75`, and `n=100`, sampled across the design's scenario classes.
- Inputs are the same raw JSON files used by the November baseline: SHA-256 matched for all 30 files against the October archive copies.
- Baseline results: `outputs/full_revised_usync_20261101_paper_ready/tables/heuristic_study_runs.csv`; baseline execution commit: `5bce12bc37949b17d60f1dc38414fb9cffa20b4e`.
- Corrected run: commit `d4cc98b44f96268430be58e7b48fa06da1875735` plus the local NILS timeout patch; NILS source SHA-256 is recorded in `run_provenance.json`.
- Settings: 120-second limit, 25 outer iterations, 8 no-improvement iterations, NILS seed `2026`, strict-candidate cap 12, and the same feasible-output repair/fallback wrapper. The CSV's `random_seed` column is the instance-generation seed, not the NILS search seed.
- All 30 runs completed without execution errors and checkpointed individually. The baseline and rerun feasibility counts are retained; objective wins/losses use only matched feasible pairs.

| Size | Baseline feasible | Rerun feasible | Feasible pairs | Rerun wins/losses/ties | Baseline mean objective | Rerun mean objective | Mean drone share, baseline → rerun | Deadline terminations |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 50 | 10/10 | 10/10 | 10 | 3/2/5 | 56,666.22 | 50,388.35 | 4.00% → 4.60% | 10/10 |
| 75 | 10/10 | 5/10 | 5 | 5/0/0 | 66,739.99 | 59,709.39 | 0.00% → 1.33% | 5/10 |
| 100 | 10/10 | 0/10 | 0 | Not comparable | — | — | Not comparable | 0/10 |

The five feasible n=75 pairs all favor the rerun, but five other reruns failed during construction and fell back to truck-only schedules. At n=100, all ten current runs returned `CONSTRUCTION-FAILED` before displacement search; the fallback schedules were infeasible. Thus the n=100 zero-drone result in this rerun is **not evidence that NILS searched and chose not to use drones**. It never reached the candidate search. The earlier baseline's 10/10 feasible outcomes at n=100 are not reproduced in this environment.

For the runs that did search, the enumeration is large: n=50 generated a mean 34,606 sortie options and evaluated 29,457 full candidates; the five n=75 search runs averaged 39,516 options and 37,261 full candidates. The timeout defect can therefore matter, but the sample does not establish that it is the sole or primary cause of the historical M5–ALNS gap. The change is also not an equal-runtime comparison; actual runtimes are in the run CSV.

## Interpretation and Limits

- The code defect is confirmed and corrected; the historical n=75/n=100 M5 results should not be treated as a fair assessment until the construction/search discrepancy is resolved.
- The n=50 and feasible n=75 outcomes are diagnostic only. They are time-limited reruns, not replacement paper results, and their differences cannot all be causally assigned to timeout salvage without a contemporaneous no-salvage control.
- The results support investigating NILS initialization and feasibility repair separately. They do not prove mathematical infeasibility when construction fails.
- Distance-only 2-opt and the lack of systematic objective-driven truck relocation remain unchanged and require a separate experiment after feasible initialization is reliable.
- Do not use `outputs/nils_timeout_diagnostic_20261105/`: that interrupted attempt used the instance-generation seed as the NILS seed and omitted the benchmark wrapper. It is marked invalid and preserved only for audit.

## Initialization Environment Follow-Up

The `n=75` and `n=100` construction failures in this diagnostic were reproduced with Python 3.11.9 when OR-Tools was unavailable. The November baseline and current master have identical construction and feasibility code; with Python 3.14.3 and OR-Tools 9.15.6755, both code versions produced feasible starts on the same ten `n=100` instances before any NILS displacement. The earlier `n=100` zero-drone outcome is therefore a construction-environment failure, not a searched NILS solution. See `outputs/nils_initialization_audit_20261105/initialization_audit.md` and its route/timing comparison files. The original diagnostic run did not record its Python executable or dependency versions, so its exact runtime cannot be proven from its provenance file.

The corrected-runtime 30-instance diagnostic is complete. It produced feasible starts and final schedules on all 30 runs. Use the results and caveats in `outputs/nils_timeout_diagnostic_runtime_matched_20261105/diagnostic_findings.md` instead of the earlier `n=75`/`n=100` summaries above; the earlier comparisons were confounded by the missing OR-Tools dependency.

## Files

- `nils_timeout_diagnostic_runs.csv`: matched instance-level objectives, feasibility, drone use, runtime, search counters, and statuses.
- `nils_timeout_diagnostic_by_size.csv`: size summary; objective statistics exclude infeasible matched pairs.
- `run_provenance.json`: settings, source commits, hashes, and completion status.
