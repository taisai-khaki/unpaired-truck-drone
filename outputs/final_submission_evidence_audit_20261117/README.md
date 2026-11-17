# Final submission evidence and integrity audit

Audit date: 2026-11-17. Repository branch at start: `codex/utdrp-latest`. Frozen principal experiment: execution commit `8abdc6b7d9f82a7dc630468e4f108e48f2246e84`, 720 instances / 2,880 M3/M4/M5/M8 records.

## Status

**BLOCKED.** The frozen experiment files are intact and every previously accepted M3/M4/M5/M8 schedule remains feasible, but 227 objective values change when recomputed with the corrected readiness-based timing evaluator. The exact MILP audit certifies one of seven cases; n=7 is invalid due an unnecessary swap and n=9 has no feasible CBC incumbent. In the 12-run M2/M6/M7 diagnostic, one M2 construction is infeasible; no full 2,160-run study was launched. M6/M7 initial backbone, seed, and starting-objective equivalence is now explicitly verified on 8/8 rows. Table 2 also describes a 20-km range as calibrating endurance even though the code uses an independent 1,200-Wh base capacity.

## Contents

- `MANUSCRIPT_ACTIONS.md`: exact manuscript edits, copy-ready replacement paragraphs, evidence matrix, and unresolved gates.
- `MILP_FORMULATION_CODE_MAP.md`: equation-to-code mapping, solver and objective findings, energy/capacity audit, and exact-case results.
- `ablation_integrity_report.md`: diagnostic sample, M2 failure, M6/M7 flags, reference-integrity result, and reason the 2,160 runs were not started.
- `corrected_tables_principal.tex`: copy-ready corrected principal tables based only on immutable M1/M3/M4/M5/M8 records and the current evaluator. It does not contain new M2/M6/M7 values.
- `exact_table5.tex`: separate exact-MILP evidence table. It is an addition; it must not replace the manuscript's NILS feasibility-calibration Table 5.
- `exact_results.csv`, `exact_certificates.csv`, `exact_objective_reconciliation.csv`, `exact_constraint_diagnostics.csv`, `exact_solver_logs/`: native CBC evidence for the seven archived small cases.
- `frozen_720_revalidation.csv` and related `frozen_720_corrected_*` files: read-only schedule evaluation and corrected derivatives. No optimization was rerun for M1/M3/M4/M5/M8.
- `high_priority_on_time_clustered_ci.json`: paired instance-cluster bootstrap interval for the M5--M8 high-priority on-time-rate difference; customers are not resampled independently.
- `real_data_revalidation.csv` and `real_data_corrected_table15.csv`: read-only audit of all 56 existing real-data method/scale records.
- `ablation_gate_diagnostics.csv`, `ablation_gate_summary.json`: the 12-run gate only; the 2,160 full runs were not performed.
- `environment_manifest.json`, `source_sha256_manifest.csv`, `audit_artifact_sha256_manifest.csv`: environment and controlling-paper fingerprint, source/config/test hashes, and hashes of audit artifacts (the artifact manifest excludes itself by design).

## Reproduction

Run from the repository root:

```powershell
python -m src.experiments.final_submission_evidence_audit --revalidate-frozen
python -m src.experiments.final_submission_evidence_audit --revalidate-references
python -m src.experiments.final_submission_evidence_audit --finalize-exact
python -m src.experiments.final_submission_evidence_audit --clustered-priority-ci
python -m src.experiments.final_submission_evidence_audit --finalize-ablation-gate-audit
python -m src.experiments.final_submission_evidence_audit --write-manifests
```

`--run-exact --exact-time-limit 120` runs new MILP solves and is not needed to reproduce the finalized report from retained native logs. `--run-ablation-gate` reruns the 12-instance diagnostic only. Do not launch the full 2,160-run experiment until the gate and reference-integrity issues in `ablation_integrity_report.md` are resolved. No M1/M3/M4/M5/M8 optimization rerun is authorized by this audit.

## Integrity boundaries

The immutable 720-instance directory and the earlier legacy-refresh directory were not edited. The pre-existing untracked `outputs/final_m3_m4_m5_alns_720_semantics_corrected_20261112/` directory was not staged or changed. This audit does not alter the manuscript PDF.
