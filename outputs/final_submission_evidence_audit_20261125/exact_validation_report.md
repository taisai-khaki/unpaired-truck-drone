# Exact MILP audit summary

Status: BLOCKED (1/7 proven optima).
Solver: CBC 2.10.3; zero relative-gap request; 120-second per-instance limit.
Native CBC logs are retained. An absent lower-bound line is treated as equal to the incumbent only when CBC reports `Optimal solution found`.
A relative gap is left blank when the logged lower bound exceeds the incumbent or a valid bound is unavailable.
NILS comparisons use the corrected readiness-based schedule; the NILS optimality gap is reported only for certified cases.

## Case status

- n=3: CBC `Optimal solution found`, primal `27.078289251979005`, dual bound `27.078289251979005` (implied_by_native_cbc_optimal_termination), gap `0.0`, evaluator-consistent `True`, feasible `True`, architecture-valid `True`, certified `True`.
- n=4: CBC `Stopped on time limit`, primal `46.249081609036494`, dual bound `9.299` (cbc_log_lower_bound), gap `0.7989365480030831`, evaluator-consistent `True`, feasible `True`, architecture-valid `True`, certified `False`.
- n=5: CBC `Stopped on time limit`, primal `42.73231930434397`, dual bound `7.247` (cbc_log_lower_bound), gap `0.8304093922825456`, evaluator-consistent `True`, feasible `True`, architecture-valid `True`, certified `False`.
- n=6: CBC `Stopped on time limit`, primal `56.58939054824353`, dual bound `8.028` (cbc_log_lower_bound), gap `0.8581359523008827`, evaluator-consistent `True`, feasible `True`, architecture-valid `True`, certified `False`.
- n=7: CBC `Stopped on time limit`, primal `47.42722135563836`, dual bound `10.114` (cbc_log_lower_bound), gap `0.7867469417160446`, evaluator-consistent `True`, feasible `False`, architecture-valid `False`, certified `False`.
- n=8: CBC `Stopped on time limit`, primal `59.41847516130534`, dual bound `5.552` (cbc_log_lower_bound), gap `0.9065610488164195`, evaluator-consistent `True`, feasible `True`, architecture-valid `True`, certified `False`.
- n=9: CBC `Stopped on time limit`, primal `None`, dual bound `5.195` (cbc_log_lower_bound), gap `None`, evaluator-consistent `False`, feasible `False`, architecture-valid `False`, certified `False`.

## Material findings

- n=3 is certified: CBC ended with native optimal status, the extracted assignment passes the final validators, the model and evaluator objectives agree, and the optimum matches the best feasible archived NILS start within tolerance.
- n=4–6 and n=8 produced feasible, evaluator-consistent CBC incumbents, but stopped at the time limit with large nonzero gaps. Their incumbent objectives exceed archived feasible NILS objectives; this does not imply that the MILP optimum is worse.
- n=7's CBC incumbent is invalid because it contains an unnecessary battery swap at rendezvous node 5 (drone 3). It is not an exact benchmark.
- The archived n=8 objective discrepancy was entirely in priority-weighted tardiness: the MILP components were truck 37.4273779161, drone 21.9560914348, and tardiness 0; the prior evaluator added 1,368.8880485209 because stale readiness-based waiting produced 13.6888804852 minutes of high-priority tardiness at weight 100. Recomputing waits from readiness removes that stale delay and reproduces the archived MILP objective 59.3834693509. The fresh time-limited incumbent is 59.4184751613 and is not certified.
- n=9 stopped at the time limit with no feasible CBC incumbent. The native log explicitly says `No feasible solution found`; PuLP had populated variables with relaxation values, which the prior adapter mistakenly exposed as a candidate. The corrected finalizer discards those values, retains the 5.195 lower bound, and reports no primal objective or gap.
- The n=7 unnecessary swap reflects a model/validator convention gap: the MILP permits optional battery-restoration binaries, while the final validator rejects swaps that are not needed by battery state. Synchronization wait variables are also lower bounds without an explicit upper/equality link to exact readiness waiting. Fixed-route NILS-to-MILP compatibility was not run; do not claim complete exact-model reconciliation.
