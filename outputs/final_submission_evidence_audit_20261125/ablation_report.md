# M2/M6/M7 full-design results

Completed new method-instance runs: 2160/2160.
The original M1/M3/M4/M5/M8 artifacts remain immutable; their objective values in this merged analysis are read-only corrected-evaluator re-evaluations.
Invalid new runs remain in the raw records and are excluded from objective averages and paired wins/losses/ties.

## M1-M8 size-balanced ranking

| Rank | Method | Valid | Size-balanced objective index | Mean objective |
|---:|---|---:|---:|---:|
| 1 | M8 | 720/720 | 35.25 | 50,263.42 |
| 2 | M4 | 720/720 | 39.41 | 76,023.80 |
| 3 | M5 | 720/720 | 41.29 | 80,081.14 |
| 4 | M3 | 720/720 | 44.24 | 81,735.49 |
| 5 | M7 | 720/720 | 52.38 | 97,469.49 |
| 6 | M6 | 720/720 | 57.00 | 95,629.47 |
| 7 | M2 | 487/720 | 97.62 | 67,258.02 |
| 8 | M1 | 720/720 | 100.00 | 144,351.79 |

## Feasibility by new method

- M2: 487/720 valid; failure types {'INFEASIBLE_SCHEDULE': 233}.
- M6: 720/720 valid; failure types {}.
- M7: 720/720 valid; failure types {}.

## M5 ablations

| Comparison | Size | Pairs | M5 wins | Ablation wins | Ties | Median M5 improvement (%) | Wilcoxon p | Rank-biserial |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| M5_vs_M6 | All | 720 | 563 | 151 | 6 | 17.80 | 0.00000 | 0.577 |
| M5_vs_M6 | 15 | 230 | 193 | 32 | 5 | 17.80 | 0.00000 | 0.753 |
| M5_vs_M6 | 50 | 210 | 199 | 10 | 1 | 67.63 | 0.00000 | 0.935 |
| M5_vs_M6 | 75 | 90 | 71 | 19 | 0 | 25.24 | 0.00000 | 0.635 |
| M5_vs_M6 | 100 | 190 | 100 | 90 | 0 | 0.83 | 0.49620 | 0.057 |
| M5_vs_M7 | All | 720 | 523 | 46 | 151 | 12.39 | 0.00000 | 0.915 |
| M5_vs_M7 | 15 | 230 | 118 | 8 | 104 | 0.82 | 0.00000 | 0.957 |
| M5_vs_M7 | 50 | 210 | 159 | 19 | 32 | 23.50 | 0.00000 | 0.884 |
| M5_vs_M7 | 75 | 90 | 64 | 12 | 14 | 6.99 | 0.00000 | 0.852 |
| M5_vs_M7 | 100 | 190 | 182 | 7 | 1 | 13.69 | 0.00000 | 0.954 |

## Interpretation

The ablations estimate finite-time component contributions under the frozen design. They do not establish universal causal effects. Report any M6/M7 improvements by size and operational regime; do not assume full NILS is superior.
M2 is a constructive benchmark; infeasible constructions are retained and counted rather than replaced with truck-only schedules.
