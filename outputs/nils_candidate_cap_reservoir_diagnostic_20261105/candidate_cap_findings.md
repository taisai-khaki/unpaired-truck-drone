# NILS Candidate-Cap Diagnostic

## Audit outcome

The experiment YAML contains `max_displacement_candidates: 100` and `max_repair_candidates: 100`, but the earlier `HeuristicConfig`, parser, and `run_nils()` did not read or pass either value. Git history shows these keys being added in configuration commits `c952b5c` and `fbaa88f`; the relevant historical NILS and parameter code contains no implementation or documented semantics. The requested latest manuscript files (`Drone_paper (12).pdf` and `(14).pdf`) are not present in Downloads, so the manuscript could not resolve the ambiguity.

The implemented, explicit interpretation is:

- `max_displacement_candidates` limits the number of move descriptors retained for screening in each displacement-extraction call (per outer iteration). `max_repair_candidates` limits sampled truck-insertion descriptors per repair step. A value of zero means unlimited.
- Both caps use seeded uniform reservoir sampling, not the first 100 candidates. This bounds candidate-pool memory and avoids enumeration-order selection bias.
- The raw candidate space is still enumerated to produce an unbiased reservoir. Thus `generated` can exceed 100; `retained` is capped at 100 per call/step. A separate direct sampler over the combinatorial route space would be a different algorithm and is not assumed here.
- Screening, battery checks, route/timing evaluation, and strict feasibility tests are unchanged. Counts distinguish descriptors generated, retained by the cap, screened, fully objective-evaluated, and strict-feasibility-accepted.

For the CSV counters, `generated` is the full descriptor population before sampling; `retained` is the reservoir subset; `screened` counts retained descriptors reaching candidate construction and the existing pre-objective checks, including rejected candidates; `fully_evaluated` counts candidates passing those checks and reaching `evaluate_solution`; and `strict_feasible` counts candidates accepted by the full feasibility checker. Repair counters use analogous insertion-candidate stages. Consequently, full objective evaluations can be below 100 when screened candidates are rejected.

## Matched 30-instance run

The rerun used the same 10 instances at each of `n=50,75,100` as the preceding uncapped diagnostic, the same NILS seed (`2026`), 25-iteration maximum, 8-iteration stagnation limit, and 120-second wall limit. One initial truck backbone was created, saved, and reused per instance. All 30 cached starts were feasible, all 30 completed, and all 30 final schedules were feasible. OR-Tools was `9.15.6755`; Python was `3.14.3`.

| Size | Mean iterations (uncapped → capped) | Mean full objective evaluations (uncapped → capped) | Mean CPU seconds (uncapped → capped) | Mean drone share (uncapped → capped) | Capped runs reaching time limit |
|---:|---:|---:|---:|---:|---:|
| 50 | 3.3 → 14.3 | 29,747 → 859 | 120.4 → 63.3 | 4.6% → 3.4% | 4/10 |
| 75 | 1.0 → 13.2 | 37,445 → 569 | 121.7 → 79.7 | 1.3% → 3.2% | 5/10 |
| 100 | 1.0 → 8.2 | 22,887 → 445 | 121.5 → 120.2 | 1.0% → 2.4% | 10/10 |

The capped run's mean raw generated descriptor counts were approximately 214,776 (`n=50`), 1,261,527 (`n=75`), and 3,395,175 (`n=100`) per instance. Mean retained counts were 1,430, 1,320, and 820, respectively; these totals span multiple iterations and remain within 100 per extraction call. Every run had at least two iterations. The `n=100` runs now average 8.2 iterations rather than one, but every one still reaches the 120-second limit.

## Interpretation and caveat

This resolves the silent YAML-to-code mismatch and demonstrates multiple feasible iterations at all sizes. It does **not** remove the raw combinatorial enumeration, and it does not establish that the cap's inferred semantics match the unavailable latest manuscript. The old uncapped diagnostic did not persist its initial truck backbones and used an earlier execution commit, so the old-to-new performance figures are descriptive, not a clean causal estimate of the cap alone. Do not replace paper tables or claim final benchmark superiority from this 30-instance diagnostic.

No 720-instance run was started. The run-level CSV, size summary, provenance, and cached initial backbones are in this directory.
