# Algorithm Change Audit

## Proven differences

1. The June run used `src/experiments/heuristic_study.py:1885-1907`, executing seven methods sequentially on the generated in-memory instance.
2. The August run used `src/experiments/run_priority_service_speed_by_method.py:474-496`, reloading each raw JSON instance and executing the selected methods separately.
3. The August wrapper calls `_force_feasible_solution` and then `evaluate_solution` in `src/experiments/run_priority_service_speed_by_method.py:287-312`.
4. The August command-line option `--common-time-limit` defaults to `None` at `src/experiments/run_priority_service_speed_by_method.py:655-658`; the August logs do not record whether an override was supplied.
5. Standard M5 is not NILS-R. The current `run_nils` default is `risk_rollback=False` at `src/heuristics/nils.py:1017`, and the NILS-R logic is activated only through a separate method.

## Code-history comparison

The tracked July-to-August diff adds the NILS-R machinery and reporting infrastructure. The base `run_nils` body is not shown as changed apart from optional risk-rollback parameters and guarded calls. The June implementation itself predates Git, so this does not prove that the June and August base code were identical.

```text
config/experiment_doptimal_beta010_tables5_14.yaml |      6 +
 config/experiment_heuristic_study.yaml             |      6 +
 .../comparisons/nils_r_vs_alns_pairwise_detail.csv |    721 +
 .../comparisons/nils_r_vs_nils_pairwise_detail.csv |    721 +
 .../comparisons/table_nils_r_vs_alns_by_size.csv   |      6 +
 .../comparisons/table_nils_r_vs_nils_by_size.csv   |      6 +
 .../customer_delivery_observations.csv             |  27383 ++++
 .../customer_service_detail.csv                    | 136551 ++++++++++++++++++
 .../drone_decision_log.csv                         |   8590 ++
 .../fleet_size_sweep_nils_r.csv                    |   2881 +
 .../nils_r_reconstruction_log.csv                  |    533 +
 .../run_status_full.csv                            |   2881 +
 .../table_decision_quality_existing_methods.csv    |     21 +
 .../table_drone_use_decision_quality.csv           |     21 +
 .../table_fleet_size_marginal_value.csv            |     21 +
 .../table_nils_r_vs_alns_by_size.csv               |      6 +
 .../table_nils_r_vs_nils_by_size.csv               |      6 +
 .../table_priority_service_speed_by_method.csv     |     21 +
 src/data_models.py                                 |     36 +-
 src/experiments/aggregate_fleet_size_sweep.py      |    148 +
 src/experiments/aggregate_nils_r_comparisons.py    |    173 +
 .../build_decision_quality_existing_methods.py     |    196 +
 src/experiments/build_full_paper_report.py         |    326 +
 src/experiments/heuristic_study.py                 |      4 +
 src/experiments/run_fleet_size_sweep_nils_r.py     |    243 +
 .../run_priority_service_speed_by_method.py        |    648 +
 src/feasibility.py                                 |     11 +-
 src/heuristics/__init__.py                         |      1 +
 src/heuristics/baselines.py                        |     25 +
 src/heuristics/nils.py                             |    198 +-
 src/heuristics/nils_r.py                           |    444 +
 31 files changed, 182818 insertions(+), 16 deletions(-)
```

## Input and configuration findings

- All `720` instance IDs are present in both M5 outputs.
- The old M5 seed and raw JSON seed match for all `720` instances.
- The M5 customer-detail seed values align with the June seed for all `720` instances.
- The raw JSON files contain the current numerical instance data and are the correct reference for the August run.
- The original June in-memory objects were not preserved, so exact coordinate/demand/window equality between the June object and the later JSON object cannot be proven from the artifacts alone.
- Current tracked NILS settings are `max_outer_iter=25`, `max_no_improve=8`, `time_limit_seconds=180`, and `random_seed=2026`.

## Interpretation

The discrepancy is a run-lineage mismatch. It is proven that different drivers and unpinned source states were used. In the nine replayed cases, current direct NILS, the priority-service driver, the feasibility wrapper, regenerated current instances, and JSON reload produced identical objectives; the tested shared-state cases also matched. Current code reproduced the August objective for all sampled n=15 and n=50 cases, but not for the sampled n=75 and n=100 cases. Therefore the available evidence does not isolate one historical algorithm line as the sole cause, and the August full M5 result is not fully reproducible from the current checkout.

The wrapper and current serialization pathway are not the observed cause in the replay: direct and driver objectives matched in all nine cases, wrapper actions were unchanged, and regenerated/reloaded objectives matched direct NILS. Shared-state checks were run on the n=15 and n=50 representatives and also matched. The remaining uncertainty is the unavailable June source and unrecorded August source/runtime state, especially for the larger instances.
