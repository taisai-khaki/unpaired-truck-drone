# Paper Results Reconciliation

## Scope and run record

This report compares the M5–M8 values in the submitted `Drone_paper (14).pdf` with the fresh matched rerun in this folder. The rerun covers 720 saved instances and four methods: M1 truck-only, M5 NILS, M8 feasibility-aware ALNS, and M9 NILS-R. It completed from 2026-11-01 to 2026-11-03 with 2,880 method-instance records and no execution errors. M5, M8, and M9 produced feasible solutions on all 720 instances; M1 did so on 483.

The execution commit, source checksum, configuration checksums, input checksum, runtime settings, and completion status are in `run_provenance.json`. The main configuration sets a 120-second cooperative time limit, `max_outer_iter: 25` for NILS, and `alns_max_iter: 40` for ALNS.

## M5–M8 Table 15 replacement

The following fresh results are based on 720 instance-matched pairs. Positive improvement means M5 has a lower objective than M8; negative means M5 has a higher objective.

| Size | Pairs | M5 wins | M8 wins | Ties | Mean Z (M5) | Mean Z (M8) | Median improvement M5 vs M8 | Mean CPU M5/M8 (s) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| All | 720 | 291 | 428 | 1 | 119,472.22 | 50,848.95 | -13.72% | 82.62 / 18.53 |
| 15 | 230 | 139 | 90 | 1 | 427.70 | 1,596.35 | 14.93% | 10.33 / 2.58 |
| 50 | 210 | 148 | 62 | 0 | 48,428.60 | 33,299.67 | 13.98% | 109.55 / 10.66 |
| 75 | 90 | 2 | 88 | 0 | 125,022.89 | 43,342.75 | -221.48% | 121.66 / 28.57 |
| 100 | 190 | 2 | 188 | 0 | 339,471.37 | 133,422.65 | -186.33% | 121.87 / 41.79 |

All matched pairs are feasible for both M5 and M8. The Wilcoxon signed-rank p-values are recorded at full available precision in `tables/table_15_matched_m5_m8.csv`; the LaTeX table reports values below 0.001 as `<0.001`. The detailed paired rows are in `tables/table_15_matched_m5_m8_instance_detail.csv`.

The fresh full-design result does **not** support a blanket claim that M5 outperforms M8. M5 wins more often at sizes 15 and 50, but M8 wins 88/90 pairs at size 75 and 188/190 at size 100. In the full design M8 has the lower mean and median objective. The manuscript's Table 15 and any associated superiority statements need replacement or revision using the fresh table.

## Submitted-table versus fresh-run values

Values below reproduce the submitted Table 15 summary and compare it with the fresh rerun. The submitted values and fresh values are separate result sets; they must not be pooled.

| Size | Submitted wins M5/M8/ties | Fresh wins M5/M8/ties | Submitted mean Z M5/M8 | Fresh mean Z M5/M8 | Submitted median improvement | Fresh median improvement |
|---:|---:|---:|---:|---:|---:|---:|
| All | 401 / 314 / 5 | 291 / 428 / 1 | 48,038.47 / 45,022.75 | 119,472.22 / 50,848.95 | 1.88% | -13.72% |
| 15 | 138 / 87 / 5 | 139 / 90 / 1 | 212.97 / 870.85 | 427.70 / 1,596.35 | 1.32% | 14.93% |
| 50 | 156 / 54 / 0 | 148 / 62 / 0 | 14,036.32 / 21,024.69 | 48,428.60 / 33,299.67 | 27.10% | 13.98% |
| 75 | 37 / 53 / 0 | 2 / 88 / 0 | 39,808.06 / 33,916.25 | 125,022.89 / 43,342.75 | -7.77% | -221.48% |
| 100 | 70 / 120 / 0 | 2 / 188 / 0 | 147,412.44 / 130,254.94 | 339,471.37 / 133,422.65 | -27.95% | -186.33% |

The discrepancies are material, especially for M5 at sizes 75 and 100. The run-level replay audit against the archived October aggregate shows 76/720 NILS objective mismatches and 83/720 NILS-R mismatches, with no feasibility mismatches. ALNS and truck-only objectives match their archived values on all 720 instances. This supports treating the fresh results as a distinct time-limited rerun; it does not by itself identify the provenance of every earlier submitted value.

## Computational-effort limitation

The instance pairing is controlled, but realized runtimes are not matched. Although both solvers had a 120-second configured cap, ALNS stopped at its 40-iteration cap on many cases, resulting in a substantially lower average runtime than NILS. Therefore, the fresh table supports an objective comparison under the recorded settings, but not a claim of equal realized computational effort. A time-matched superiority claim requires a separate controlled comparison with equivalent wall-clock budgets and reported actual runtimes.

The fresh diagnostics also do not preserve actual iteration counts: the exporter replaced the search `components` dictionary when reevaluating the final objective, leaving `iterations` and related counters at zero in the aggregate file. The source exporter is now fixed to preserve those diagnostics, but the historical counters cannot be reconstructed from this run. The execution commit in `run_provenance.json` remains the correct code version for these results.

## Other deliverables and exclusions

- The `tables/` directory contains regenerated paper tables, the M5–M8 full matched Table 15, and the size-wise M9 comparisons against NILS and ALNS.
- `table_priority_service_speed_by_method.csv`, `table_drone_use_decision_quality.csv`, `customer_service_detail.csv`, and the NILS-R reconstruction and decision logs contain the requested service diagnostics.
- The separate matched available-drone fleet-size sweep is not included. Prior August sweep files were intentionally excluded because their run provenance and timing convention were not established as equivalent to this fresh run.
- `drone_decision_log.csv` records NILS-R rollback/admission actions, not every candidate evaluation made internally by M5 or M8.
