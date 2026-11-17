# MILP formulation and implementation audit

## Equation-to-code map

| Paper equations | Implemented components | Audit result |
|---|---|---|
| (1)–(6), objective and cost components | `src/milp/objective.py::build_objective`; travel-time and distance preprocessing in `src/milp/model_builder.py` | Truck cost, drone distance/energy cost, and priority-weighted tardiness are summed in the stated units. Waiting cost is excluded, consistent with the paper. Independent objective reconciliation passes for every validated CBC incumbent. |
| (7)–(9), delivery timestamp and tardiness | `src/milp/constraints.py::add_tardiness_linearization`; `src/milp/extract_solution.py` | Arrival-time assignment is McCormick-linearized; tardiness lower bounds are minimized with positive priority weights. |
| (10)–(20), route start/return, flow, assignment, visitation, eligibility and payload | `add_return_to_depot`, `add_service_and_flow`, and `add_time_windows_and_shifts` in `src/milp/constraints.py`; variable construction in `src/milp/model_builder.py` | Route flow, single service, truck/drone visits, payload, eligibility, and route closure are represented. The n=9 native CBC log contains no feasible solution, so its PuLP relaxation values are not a route assignment. |
| (21)–(28), loaded-parcel state and truck inventory | `add_demand_and_load` in `src/milp/constraints.py` | Loaded-state conservation links service and reload; truck inventory accounts for customer deliveries and parcel reloads. Final extracted n=7/n=9 diagnostics still fail, so those records are not validation evidence. |
| (29)–(40), battery states, swap, reserve and energy | `add_energy` in `src/milp/constraints.py`; rates from `src/energy.py` and `src/drone_state.py` | Loaded/empty arc energy and residual reserve are modeled in Wh. The MILP allows `z2=1` as optional full restoration; it does not force a swap to be necessary. The n=7 incumbent is rejected for an unnecessary swap at node 5, drone 3. |
| (41)–(60), travel timing, readiness, rendezvous, support actions, handling and waiting | `add_timing` in `src/milp/constraints.py`; `compute_synchronization_waiting` in `src/timing.py`; extraction in `src/milp/extract_solution.py` | `u_sync` is linked to joint truck/drone customer-node visits; reload/swap remain separate; departure lower bounds charge `2*tau_hand*u_sync + t_rel*z1 + t_swap*z2` to both vehicles. Waiting variables are lower bounds, not exact max-readiness equalities and have no rendezvous upper link. This is the remaining timing-formulation mismatch. |
| (61)–(66), lower windows and shift | `add_time_windows_and_shifts` in `src/milp/constraints.py` | Lower time-window bounds and 480-minute route-return limits are implemented; upper windows contribute tardiness rather than hard infeasibility by default. |
| (67)–(70), subtours and domains | `_mtz_subtour` and variable declarations in `src/milp/model_builder.py` / `src/milp/constraints.py` | MTZ ordering constraints are used as described in Appendix A.6; binary/continuous variable domains are explicit. |
| Appendix A.1–A.2, delivery/tardiness linearization | `add_tardiness_linearization` in `src/milp/constraints.py` | Standard bounded McCormick products and positive-part tardiness are present. |
| Appendix A.3, aerodynamic preprocessing | `src/energy.py::aerodynamic_power_w`, `derive_drone_energy_rates`, and `src/instance_generator.py` | Speed is converted km/h → m/s; the code returns power in W, energy rates in Wh/min, and flight energy in Wh. Unit/regression tests pass. The battery-capacity calibration and Table 2 claim do not agree; see below. |
| Appendix A.4, fixed-pair restriction | architecture-specific constraints in `src/milp/model_builder.py` and benchmark move generation | The fixed-pair benchmark is restricted to its assigned truck; exact solves in this audit are the M5 instance formulation, not a fresh exact comparison of M3/M4/M5. |
| Appendix A.5, big-M | `config/default.yaml` and `config.constraints.big_m` passed to `src/milp/constraints.py` | The implementation uses one global `M=10,000` for timing, energy/load relaxations, and MTZ, rather than separately calibrated constants. It is larger than the 480-minute horizon and nominal 1,200-Wh battery; tightness/numerical conditioning are not established by this audit. |
| Appendix A.6, subtour elimination | `_mtz_subtour` | The code uses MTZ rather than enumerating exponential subset constraints, consistent with the manuscript. |

## Energy and capacity consistency

The implementation uses `c1=2.8037`, `c2=0.3177`, `c4=0.0296`, `c5=0.0279`, `g=9.8`, and `alpha=10°`. At 65 km/h and masses 3 kg empty / 5 kg loaded, the tested outputs are 512.4642 W and 1,003.3375 W, or 8.5411 and 16.7223 Wh/min. The 20-km loaded-flight energy at 65 km/h is 308.7192 Wh.

The actual default `max_battery_wh` is 1,200 Wh (`config/default.yaml`, `src/parameters.py`). The D-optimal endurance scenario multiplies that by 0.70/1.00/1.30, giving 840/1,200/1,560 Wh. At the paper's 10% reserve, the corresponding minimum reserved energy is 84/120/156 Wh. The nominal 20-km value does **not** calculate the implemented 1,200-Wh capacity; that capacity would imply about 77.74 km at the 65-km/h loaded-power point before reserve (about 69.97 km at 90% usable capacity). This is a material paper/code mismatch, not an optimization result.

The principal D-optimal experiment also creates drone speed from `truck_speed * speed_ratio`, with truck speed 35 km/h and ratios 1.25/2.00, i.e. 43.75/70 km/h. Thus 65 km/h is not the principal experiment's only or fixed drone speed. The manuscript's Table 2 should distinguish the nominal calibration value from the actual factor levels. Table 2 cites Moshref-Javadi et al. (2021) for aerodynamic parameters, whereas Appendix A.3 names Liu et al. (2017); the internal citation must be reconciled.

## Exact audit findings

The seven exact cases use the frozen small instances and seeds 5027, 6027, 7027, 8027, 9027, 10027, and 11027 for n=3,...,9. CBC 2.10.3 was run with a 120-second per-case budget and native logs retained.

| n | CBC evidence | Independent validation | NILS comparison |
|---:|---|---|---|
| 3 | Native optimal termination; primal/dual/evaluator = 27.0782893 | Feasible, architecture-valid, objective-consistent | NILS 27.0782893; this is the sole certified optimum |
| 4 | Time limit; primal 46.2491, bound 9.299, gap 79.89% | Valid incumbent | NILS 44.1406 is better; no optimality conclusion |
| 5 | Time limit; primal 42.7323, bound 7.247, gap 83.04% | Valid incumbent | NILS 39.8599 is better; no optimality conclusion |
| 6 | Time limit; primal 56.5894, bound 8.028, gap 85.81% | Valid incumbent | NILS 45.4320 is better; no optimality conclusion |
| 7 | Time limit; primal 47.4272, bound 10.114, gap 78.67% | Invalid: unnecessary swap at node 5 for drone 3 | No exact comparison |
| 8 | Time limit; primal 59.4185, bound 5.552, gap 90.66% | Valid incumbent; evaluator agrees | NILS 49.9081 is better; no optimality conclusion |
| 9 | Time limit; native log says `No feasible solution found`; lower bound 5.195 | No incumbent; PuLP relaxation values discarded | No primal objective or gap |

The archived n=8 discrepancy is accounted for exactly. The MILP objective was truck cost 37.4273779161 + drone cost 21.9560914348 + tardiness 0 = 59.3834693509. The prior evaluator added 1,368.8880485209 of weighted high-priority tardiness, from 13.6888804852 stale-wait minutes at weight 100. The timing scheduler retained a previous iteration's wait instead of recomputing it from current readiness. The corrected schedule evaluator removes that stale waiting and reproduces the archived MILP objective. A separate fresh CBC run found 59.4184752 at the time limit; it is not the same incumbent and is not an optimum certificate.

The former n=9 adapter bug was also concrete: it counted any non-`None` PuLP `varValue` as a solver solution. CBC's native log had no feasible integer incumbent, but PuLP had populated variables with relaxation values. The adapter now honors the explicit native no-feasible-solution result; the archived run is finalized as no incumbent, not as a malformed feasible plan.

## Remaining exact-model gate

One of seven optima is certified. n=7's optional swap variable conflicts with the validator's required-swap convention; exact replacement constraints or an explicit paper convention are needed. The wait variables in equations (59)–(60) do not force the exact readiness-based waiting used by the evaluator. A two-way fixed-route NILS-to-MILP feasibility check was not completed. Therefore the exact comparison and full formulation/evaluator reconciliation remain **BLOCKED**.
