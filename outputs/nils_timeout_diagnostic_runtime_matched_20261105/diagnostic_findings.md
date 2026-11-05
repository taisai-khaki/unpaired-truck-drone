# Runtime-Matched NILS Timeout Diagnostic

## Scope and Provenance

This is a 30-instance diagnostic only: ten matched instances each at `n=50`, `n=75`, and `n=100`. It is not the 720-instance paper run and is not a replacement paper table. The same saved raw instances and baseline objective rows were used; the full instance IDs and run-level outputs are in `nils_timeout_diagnostic_runs.csv`.

- Baseline result source: `outputs/full_revised_usync_20261101_paper_ready/tables/heuristic_study_runs.csv`, SHA-256 `a9648b47e06cbb861e35ba8aef0eaecc4c52af30b8c7e3ab70bd877b25027812`.
- Baseline execution commit: `5bce12bc37949b17d60f1dc38414fb9cffa20b4e`.
- Diagnostic commit: `8e7c1b67bca74576757642d64583093220aa7887`, with a modified worktree. The exact NILS source SHA-256 is recorded in `run_provenance.json` (`768215a5c904506971e059a4e757ce8cff665f77ecf4ce6defb0cd7c3d8e7a42`).
- Runtime: Windows 11, Python 3.14.3, NumPy 2.4.3, pandas 2.3.3, OR-Tools 9.15.6755.
- Settings: NILS seed `2026`, 25 outer iterations, 8 no-improvement iterations, 120-second search limit, strict-candidate cap 12, greedy initial construction with a 3-second OR-Tools fallback, and the same feasible-output wrapper.
- Completion: 30/30 rows completed without execution errors; 30/30 baseline and rerun schedules are feasible; all 30 searches reached the time limit. The run started at 22:07 UTC and finished at 23:08 UTC on 2026-11-05.

## Results

Improvement is defined as `baseline objective - rerun objective`; positive values favor the rerun.

| Size | Feasible pairs | Wins / losses / ties | Baseline mean | Rerun mean | Mean improvement | Median improvement | Rerun drone share | Mean CPU (baseline / rerun) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 50 | 10 | 3 / 1 / 6 | 56,666.22 | 50,388.16 | 6,278.06 | 0.00 | 4.60% | 120.27 / 120.36 s |
| 75 | 10 | 10 / 0 / 0 | 131,645.30 | 123,119.56 | 8,525.74 | 4,413.41 | 1.33% | 121.63 / 121.69 s |
| 100 | 10 | 7 / 3 / 0 | 288,202.08 | 303,333.05 | -15,130.97 | 2,023.48 | 1.00% | 121.77 / 121.54 s |

At `n=100`, the rerun wins seven of ten pairs and has a better median objective, but three large losses make its mean objective worse. This is not a consistent mean-performance improvement. The corrected environment removes construction failure as an explanation for the low drone use, but the search-quality issue at large `n` remains unresolved. Average generated sortie options / fully evaluated candidates were 37,640 / 29,747 (`n=50`), 40,847 / 37,445 (`n=75`), and 26,823 / 22,887 (`n=100`).

## Interpretation

The previous Python 3.11 diagnostic had no OR-Tools and could not reproduce the saved `n=100` starts; its `n=75`/`n=100` comparisons must not be used. The initialization audit shows that both November and current source produce feasible starts on these ten `n=100` instances when run with Python 3.14.3 and OR-Tools 9.15.6755. The earlier construction failure was an environment/dependency mismatch, not a changed feasibility rule.

This rerun is descriptive, not a causal estimate of timeout salvage: the historical baseline did not record its Python or OR-Tools versions, and OR-Tools' short time-limited route construction can vary in route order. It also does not resolve NILS's weak large-instance truck-side search. No truck-neighborhood or 2-opt change was made, and no 720-instance rerun was started.

## Files

- `nils_timeout_diagnostic_runs.csv`: checkpointed instance-level results and search counters.
- `nils_timeout_diagnostic_by_size.csv`: basic by-size aggregation.
- `nils_timeout_diagnostic_robustness_by_size.csv`: mean/median objective and improvement, paired wins/losses/ties, feasibility, CPU, drone share, and search counts.
- `run_provenance.json`: commits, source hash, Python/dependency versions, settings, instance IDs, and completion status.
