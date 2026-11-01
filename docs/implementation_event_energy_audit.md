# Implementation Audit: Drone Events, Handling, Sorties, and Energy

**Audit target:** `Drone_paper (14).pdf` versus the current local implementation.

**Conclusion:** The implementation now represents physical truck-drone
rendezvous independently from parcel reload and battery swap. Aerodynamic
preprocessing is implemented for newly generated instances, and no full
computational rerun has been started.

## 1. Launch and recovery representation

The revised implementation contains route arcs, a physical rendezvous
indicator `u_sync`, parcel-reload variables `z1`/`rho`, and battery-swap
variables `z2`/`sigma`. Depot departure and final recovery remain implicit;
truck-supported intermediate rendezvous are represented by `u_sync` at jointly
visited customer nodes.

The code has the same basic representation:

- `src/milp/model_builder.py` creates `u_sync`, `z1`, and `z2`.
- `src/milp/constraints.py` links `u_sync` to joint truck/drone visitation and
  enforces `z1 <= u_sync` and `z2 <= u_sync`.
- `src/heuristics/nils.py` derives `u_sync` from shared non-depot route nodes;
  `z1` and `z2` remain independent operation flags.
- Depot launch and depot recovery remain implicit in a drone route beginning
  and ending at node `0`.

Therefore the implementation models **flight plus physical rendezvous and
synchronization support**. Depot launch/recovery remains implicit and is not
charged as a truck-based rendezvous.

The current accounting is:

- depot launch: implicit initial loaded state; no handling time;
- depot retrieval: implicit route return; no handling time;
- truck-based intermediate rendezvous: represented by `u_sync` and charged as
  one retrieval plus one subsequent launch;
- parcel reload: charged once through `z1`; and
- battery swap: charged once through `z2`.

Physical rendezvous, parcel reload, and battery swap are intentionally
additive. The model permits both `z1` and `z2` at the same rendezvous, so the
total handling time is not double counting: each term represents a distinct
operation.

## 2. Handling times

The revised departure equations charge:

1. customer service time;
2. readiness-based synchronization waiting whenever `u_sync=1`;
3. `2*tau_hand*u_sync` for retrieval plus subsequent launch;
4. parcel-reload time for `rho`/`z1`; and
5. battery-swap time for `sigma`/`z2`.

The implementation follows that equation structure in:

- `src/heuristics/nils.py`: route propagation adds rendezvous, reload, and
  swap handling after readiness and waiting;
- `src/milp/constraints.py`: departure constraints add the same additive terms
  and apply readiness-based waiting through `u_sync`;
- `src/feasibility.py`: the final schedule checker validates the same terms.

The physical handling time is stored as `handling_time_s`, with
`t_launch=t_retrieval=handling_time_s`. Legacy instance files without this
field fall back to their stored reload time, which matches the previous
short/medium/long design where reload and swap used the same value.

## 3. Sortie and launch/retrieval diagnostics

`src/heuristics/nils.py::compute_drone_operation_metrics` distinguishes:

- `active_drones_used`: number of drones with a loaded service in the final
  plan;
- `drone_served_customers`: distinct customers assigned to drones;
- `loaded_sorties`: loaded service transitions, one per loaded customer service
  segment;
- `drone_launches` and `drone_retrievals`: currently equal to the loaded-sortie
  count; and
- `reload_events` and `battery_swaps`: explicit `z1`/`z2` synchronization
  events.

Thus current diagnostic `drone_launches` is **not** the number of initially
activated drone routes. It is a derived loaded-sortie count. Initial route
activation is represented by `active_drones_used`.

The old Table 15 value at `n=100` is not compatible with this definition:

- `16.2%` drone share at `n=100` corresponds to approximately `16.2`
  drone-served customers per run;
- under the paper's single-customer-per-sortie assumption, individual loaded
  sorties should therefore be of the same order, not `2.99`; and
- `2.99` is consistent with an initial-activation/active-route counter for a
  fleet of three drones, or with an older route-level counter.

The same warning applies to the old Table 8 launch/retrieval columns. In
particular, unequal average launch and retrieval counts cannot represent a
closed set of individual sorties without additional event definitions.
Those tables must be regenerated from the current reconciled metrics, with
separate column names for active drones, loaded sorties, explicit reloads, and
battery swaps.

## 4. Energy preprocessing and units

The unit conversion used by the current implementation is internally valid for
fixed preprocessed rates:

- distances are treated as kilometres;
- speeds are stored in km/h;
- `src/distance_utils.py::travel_time_from_distance` converts travel time to
  minutes using `distance / speed * 60`;
- battery capacity is stored in Wh; and
- energy rates are stored as Wh/min, so rate times minutes gives Wh.

However, the paper's Appendix A.3 states that empty and loaded power are derived
from the aerodynamic model. The preprocessing is now implemented in
`src/energy.py` and is used for newly generated `SearchConfig` instances:

- `65 km/h` is converted to `18.0556 m/s` before evaluation;
- empty power is `512.4642 W`, or `8.5411 Wh/min`;
- loaded power is `1003.3375 W`, or `16.7223 Wh/min`; and
- newly generated instances persist the coefficients, units, powers, and rates
  in metadata. Legacy JSON instances retain their stored rates and are not
  silently recalibrated.

The MILP now uses equal loaded and empty flight times under the paper's
fixed-speed assumption; payload changes energy, not cruise time.

Therefore battery and flight-energy **units are consistent** for newly
generated instances. The historical JSON data are intentionally not silently
recalibrated.

### Battery capacity audit

The code's base battery is `B_d = 1200 Wh` in
`config/default.yaml` and the typed defaults. The endurance scenarios apply:

- low: `0.70 × 1200 = 840 Wh`;
- medium: `1.00 × 1200 = 1200 Wh`; and
- high: `1.30 × 1200 = 1560 Wh`.

The nominal `20 km` range is not used to derive `B_d` in the historical code.
With the calibrated loaded power, a 20-km all-loaded flight requires about
`308.72 Wh`; an all-empty flight requires about `157.68 Wh`. Thus `1200 Wh`
is a scenario baseline, not a battery capacity calculated from the stated
20-km range. Changing it to a range-calibrated value would alter feasibility,
drone assignments, objectives, and all benchmark comparisons.

## 5. Verification status

The readiness-based waiting changes are implemented and tested. The focused
architecture tests pass, the small MILP build creates the `u_sync` variables,
and the full test suite completed in this audit with `14 passed, 1 skipped`.

The focused rendezvous tests cover a physical rendezvous without reload/swap
and a rendezvous with both operations. No 720-instance rerun was started.
