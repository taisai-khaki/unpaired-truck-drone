# Corrected ALNS Execution Validation

## Protocol

- Protocol file: `outputs/corrected_alns_final_report_20261019/corrected_alns_protocol.json`.
- ALNS seed: `2237`; base seed: `2026`.
- Destroy-and-repair iterations: `40`.
- ALNS time limit: `0` seconds, so no premature ten-second cutoff.
- Matched comparison wall-clock budget: `240.0` seconds.

## Validation

- Rows: `720`; unique instances: `720`.
- Feasible final solutions: `720/720`.
- Iterations: `40` to `40`; every row completed 40.
- Candidate reassignment evaluations: `40` to `40`; every row recorded 40.
- CPU time: mean `47.971` seconds; maximum `213.622` seconds.
- The 40 iterations and candidate evaluations demonstrate that ALNS performed its prescribed destroy-and-repair loop, including the large instances.

## By size

| n | runs | feasible | iterations | candidate evaluations | mean CPU seconds | maximum CPU seconds |
|---:|---:|---:|---:|---:|---:|---:|
| 15 | 230 | 230 | 40--40 | 40--40 | 3.673 | 5.802 |
| 50 | 210 | 210 | 40--40 | 40--40 | 30.576 | 70.972 |
| 75 | 90 | 90 | 40--40 | 40--40 | 68.273 | 164.738 |
| 100 | 190 | 190 | 40--40 | 40--40 | 111.203 | 213.622 |

## Interpretation

The corrected ALNS results can be reused for both M5--ALNS and M9--ALNS comparisons. A common 240-second cap is a fairness ceiling, not evidence of equal effort; the report retains actual runtime and iteration counts. The low drone usage at larger sizes is therefore an observed ALNS solution outcome, not evidence that the ten-second initialization cutoff stopped the search.
