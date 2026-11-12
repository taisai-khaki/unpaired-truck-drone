# Manuscript Semantics Reconciliation

This report covers targeted implementation validation and the matched truck-only reference audit. It does not report a full 720-instance M3/M4/M5/ALNS rerun.

## Historical Support-Action Counterfactual

The saved historical routes are held fixed; only state-required `z1`/`z2` actions are substituted.
- Historical target runs: 32; NILS-family runs: 24.
- NILS-family `z1` events forced by launch labeling: 26.
- NILS-family `z2` events forced by recovery labeling: 66.
- Objective changed in 18 runs; feasibility changed in 0 runs.
- Sum of objective deltas: -34025.812905 (diagnostic-only arithmetic total).

## Updated Targeted Runs

- Runs: 32; feasible: 32; architecture-validator passes: 32.
- Exact route-intersection / `u_sync` match: 32/32.
- NILS-family unnecessary launch reloads: 0; unnecessary recovery swaps: 0.
- ALNS runs pass M5 validator: True.
- ALNS sortie-type counts: `{"depot-depot": 4}`.

| Method | Runs | Feasible | Architecture pass | Mean objective | Mean CPU (s) |
|---|---:|---:|---:|---:|---:|
| feasibility_aware_alns | 8 | 8 | 8 | 52127.447405 | 68.44 |
| nils | 8 | 8 | 8 | 81056.663816 | 45.72 |
| no_unpairing | 8 | 8 | 8 | 72557.097229 | 21.54 |
| paired_baseline | 8 | 8 | 8 | 76543.674599 | 12.11 |

## Matched Truck-Only Reference

- Evaluated saved backbones: 720/720.
- Feasible: 720; infeasible: 0.
- Audit status: **PASS_REFERENCE_AUDIT**.
- No full M3/M4/M5/ALNS experiment was run.

| Size | Instances | Feasible | Infeasible | Mean objective | Median objective |
|---:|---:|---:|---:|---:|---:|
| 15 | 230 | 230 | 0 | 6196.523223 | 1462.150377 |
| 50 | 210 | 210 | 0 | 123678.707931 | 95350.344623 |
| 75 | 90 | 90 | 0 | 126827.793032 | 51979.798276 |
| 100 | 190 | 190 | 0 | 342742.403262 | 324347.862664 |
