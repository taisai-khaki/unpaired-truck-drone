# Assumptions

## Modeling assumptions
- One customer is served exactly once by either truck or drone.
- Depot is indexed as node 0 in the compact data model; separate return-time and departure-battery variables represent the paper's 0+ and 0− states.
- Vehicles may stay unused; arc binaries for that vehicle remain zero.
- Priority constraints are implemented as a precedence-style soft control, not as strict lexicographic constraints.

## Feasibility assumptions
- Truck load is reduced by served demand along served arcs.
- Drone battery decay is evaluated with loaded/empty rates, swap restoration, and the configured residual reserve.
- Waiting is recorded for diagnostics and affects timing, but is not a separate objective component.
- Physical truck-drone rendezvous use `u_sync`; parcel reload (`z1`) and battery swap (`z2`) remain separate operation flags and satisfy `z1 <= u_sync` and `z2 <= u_sync`.
- Customer deadlines are soft: only the lower target-window bound is hard, while lateness is penalized through weighted tardiness.

## Computational assumptions
- Small instances use exact MILP if `milp.enabled` is true.
- Medium and large instances use heuristic NILS by default.
- All generators and solvers are seeded by configuration for reproducibility.

## Practical assumptions used for implementation
- Unpaired coordination is represented by explicit truck-drone sync decisions; M5 is the canonical fully-unpaired NILS method and M8 is the independent ALNS benchmark.
- Subtour elimination uses MTZ constraints for tractability.
- If a field is absent in YAML input, typed default values are used (see `src/parameters.py`).
- If ambiguity is found in manuscript notation, the study logs the unresolved item and continues with a documented default.

## Heuristic-study assumptions
- `eligible_share` is implemented via a sampled per-instance customer eligibility mask stored in instance metadata.
- Synthetic instances generated from `SearchConfig` derive empty and loaded drone energy rates from the Appendix A.3 aerodynamic preprocessing in `src/energy.py`; persisted instances retain the derived rates and calibration provenance. Legacy JSON instances retain their stored rates and are not silently recalibrated.
- Endurance levels are mapped as battery multipliers: `low=0.70`, `medium=1.00`, `high=1.30` (relative to base battery).
- Handling-time levels are mapped to physical rendezvous and to the existing operation parameters: `short=30s`, `medium=60s`, `long=120s`. A rendezvous costs `2 * handling_time_s`; reload and swap costs are additive.
- Spatial pattern `uniform` maps to `dispersed`, and `clustered` maps to `dense_urban` in the generator.
- Depot remains at `(0,0)`; when customer coordinates are in positive ranges this behaves like a peripheral depot.
