# NILS Initialization Reconciliation

## Finding

The ten saved `n=100` instances do not fail because of a change to the construction algorithm or feasibility rules between the recorded November commit and current master. The construction module and feasibility checker are byte-identical to commit `5bce12bc37949b17d60f1dc38414fb9cffa20b4e`; the initialization-and-repair block inside `run_nils` is also identical. The intervening NILS changes are in displacement search and diagnostics, after initialization.

The failure reproduces under the diagnostic's Python 3.11.9 environment because OR-Tools is missing there. `requirements.txt` did not declare OR-Tools. The initializer catches the import failure and returns `None`, after which `run_nils` repeats the greedy constructor. Those greedy routes violate the 480-minute truck shift bound. Since no customer is drone-served yet, the initial repair cannot fix those shift violations; the result is correctly labeled `CONSTRUCTION-FAILED` rather than treated as mathematical infeasibility.

## Controlled Replay

The exact ten baseline instance files and NILS seed `2026` were replayed with `time_limit=0` to isolate construction and repair without running displacement search.

| Source code | Runtime | OR-Tools | Feasible starts |
|---|---|---:|---:|
| November commit `5bce12b` | Python 3.11.9 | unavailable | 0/10 |
| November commit `5bce12b` | Python 3.14.3 | 9.15.6755 | 10/10 |
| Current master `8e7c1b6` | Python 3.11.9 | unavailable | 0/10 |
| Current master `8e7c1b6` | Python 3.14.3 | 9.15.6755 | 10/10 |

With OR-Tools available, all ten starts pass strict feasibility: no shift, load, lower-window, battery, or synchronization errors. Maximum truck load is 263.52 kg against 500 kg capacity; the longest active route is 476.47 minutes against the 480-minute limit. Across old/current Python 3.14 replays, truck-to-customer assignments and truck loads match on all ten instances. Exact route order matches on nine; one three-second OR-Tools construction differs in route order but remains feasible. Customer arrival/readiness values match for 902 of 1,000 customer records; the differences are confined to that changed route order.

Without OR-Tools, all ten fail with `SHIFT` errors only (16 route-level shift violations total). The over-limit routes are recorded in `python311_no_ortools.json`. The diagnostic's earlier `n=100` result is therefore a construction-environment failure, not evidence that NILS searched and selected a zero-drone plan.

## Reproducibility Note

Historical run provenance recorded the source commit and config but not the Python executable or dependency versions, so the exact historical OR-Tools version cannot be proven. In addition, the current OR-Tools initializer receives a seed argument but does not apply it to its routing search parameters; its short time-limited search can return a different feasible route order. This is consistent with, but does not prove the cause of, the single route-order variation in the controlled replay. It does not explain the `CONSTRUCTION-FAILED` status.

The project now declares OR-Tools in `requirements.txt`. The timeout diagnostic records Python, NumPy, pandas, platform, and OR-Tools versions and stops before running if OR-Tools is absent. No feasibility constraint was weakened, no 720-instance experiment was started, and no NILS search operator was changed in this initialization audit.

## Audit Artifacts

- `baseline_5bce12b_python314.json` and `current_8e7c1b6_python314.json`: route sequences, assignments, truck loads, shift durations, and validation findings.
- `python311_no_ortools.json`: the failed greedy starts and exact shift violations.
- `instance_comparison.csv`, `truck_route_comparison.csv`, and `customer_timing_comparison.csv`: route-by-route and customer timing/readiness comparisons.
