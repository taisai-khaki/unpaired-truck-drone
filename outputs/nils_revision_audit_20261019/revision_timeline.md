# Revision Timeline

## Evidence reviewed

- June M5: `outputs/doptimal_full_beta010_tables5_14_rerun2_20260601_144538/tables/heuristic_study_runs.csv`.
- August M5/M9: `outputs/paper_report_full_route_metrics_20260827/run_status_full.csv`.
- Corrected ALNS: `outputs/corrected_alns_final_report_20261019/corrected_alns_full_runs.csv`.
- Raw JSON reference: `outputs/doptimal_full_beta010_tables5_14_rerun2_20260601_144538/raw/`.

## Chronology

- June 1--9, 2026: the submitted heuristic study ran through `run-heuristic-study`; its run metadata records `720` instances and seven methods, including standard NILS.
- July 24, 2026: commit `2d847ce90` is the first tracked repository commit. The June source tree is therefore not available as a Git object.
- August 27, 2026: the priority-service driver ran standard NILS, NILS-R, truck-only, and feasibility-aware ALNS. Its run logs record the August M5 values.
- August 28, 2026: commit `1554ed832` captures the NILS-R and priority-service implementation after the August run had started and finished. The exact Git SHA used by the August process was not recorded.
- October 19, 2026: commit `c952b5c70` captures the corrected ALNS benchmark and fairness reports.

## Version conclusion

The June source version is unavailable. The August run is reproducible from the current priority-service driver for sampled instances and appears to correspond to the working tree later captured by `1554ed832`, but this cannot be promoted to a proven run SHA.

Recent history:

```text
c952b5c 2026-10-19 19:51:01 -0600 Add corrected full ALNS benchmark and fairness report
1554ed8 2026-08-27 21:53:11 -0600 Add NILS-R full report and fleet sweep
2d847ce 2026-07-24 01:17:27 -0600 Initial project upload with reviewer results
```
