# Model Audit - UTDRP-DP

This file records a faithful implementation mapping for the unpaired truck-drone routing model used in this codebase.
The implementation follows this structure:
- Truck-and-drone routing decision variables on arc sets.
- Exact customer assignment with single-service constraints.
- Capacity, battery, time and synchronization constraints.
- Priority-aware tardiness penalties in the objective.

## 1) Sets
- N = {0,1,...,n}: compact implementation node set; the model keeps separate start and return timing/battery states for the depot.
- V = {1,...,n}: customers.
- T = {1,...,|T|}: trucks.
- D = {1,...,|D|}: drones.
- C = {high, medium, low}: service priority classes.
- A_T = {(i,j,t): i in N, j in N, t in T}: truck arcs.
- A_D = {(i,j,d): i in N, j in N, d in D}: drone arcs.

## 2) Parameters
- Customer demand: q_i.
- Service windows: lb_i, ub_i.
- Priority penalties: p_high, p_medium, p_low.
- Truck params: capacity Q_t, speed v_t, cost per distance c_truck, energy cost e_truck.
- Drone params: capacity Q_d, speed v_d, max battery B_d, costs c_drone and e_drone.
- Fleet constants: swap time S_t, reload time R_t, shift horizon H, big-M M.
- Instance constants: generated coordinates, classes, demands, windows.

## 3) Decision variables
- x_truck[i,j,t] in {0,1}: truck t traverses arc (i,j).
- x_drone[i,j,d] in {0,1}: drone d traverses arc (i,j).
- u_truck[t,i] in {0,1}: truck t serves customer i.
- u_drone[d,i] in {0,1}: drone d serves customer i.
- z1[i,d,t], z2[i,d,t] in {0,1}: parcel-reload and battery-swap event flags at node i.
- lambda_loaded[i,j,d] in {0,1}: loaded-parcel state on drone arc (i,j).
- y_loaded[d,i] in {0,1}: loaded-arrival helper at customer i.
- w[t,i]: truck remaining load after leaving or arriving at i.
- r[d,i], r_departure[d,i]: drone battery on arrival and after node operations.
- a_truck[t,i], l_truck[t,i]: truck arrival/departure at i.
- a_drone[d,i], l_drone[d,i]: drone arrival/departure at i.
- tw_truck[t,i], tw_drone[d,i]: nonnegative waiting helpers.
- a_bar_truck[t,i], a_bar_drone[d,i]: McCormick-like arrival-time helpers.
- T_i: tardiness for customer i.

## 4) Constraints in plain English
- Depot return and route start/end: each used vehicle leaves/returns to depot in one arc count and no self loops.
- Exclusive service: every customer has exactly one of truck service or drone service.
- Vehicle continuity: in-degree equals out-degree for every visited node in each vehicle route.
- Load feasibility: truck load updates with customer deliveries and parcel transfers to drones and never exceeds truck capacity.
- Battery feasibility: drone battery uses loaded/empty energy, swap restoration, and the residual reserve at every visited node including return.
- Timing consistency: departure times are arrival + service + reload/swap handling + waiting; arrivals equal the preceding departure plus travel time.
- Window and shift constraints: service arrivals respect the hard lower bound, tardiness is soft after the upper deadline, and return times are bounded by the shift horizon.
- Subtour handling: MTZ constraints on customer nodes to block disconnected cycles.
- Tardiness linearization: tardiness lower bounded by actual arrival minus ub, inactive when customer not served.
- Priority precedence: high-priority served no later than medium/low where defined.

## 5) Objective
- Minimize total_cost = truck_cost + drone_cost + tardiness_cost.
- truck_cost and drone_cost are based on distance-derived travel time and rates; tardiness_cost uses class penalties and CUST-level tardiness.

## 6) Outputs and validation
- Extractor saves all binaries, arc variables, assignment matrices, timing, load, battery and objective components.
- Validator checks:
  - all customers served exactly once,
  - service continuity and depot start/end,
  - truck load and drone battery feasibility,
  - hard lower-window and shift compliance, with upper-window tardiness warnings,
  - synchronization consistency,
  - objective component consistency.

## 7) Ambiguity and TODO log
The current implementation decisions are:
1) synchronization waiting is recorded for diagnostics and propagated through timing helpers, but is not a separate objective term;
2) priority precedence constraints remain available as an optional strengthening layer, while priority protection is primarily represented by weighted tardiness;
3) loaded and empty drone energy rates are treated as preprocessed coefficients;
4) the compact depot index uses separate return-time and departure-battery variables to represent 0+ and 0− states;
5) the MILP uses MTZ constraints in the available backend; a lazy-cut callback is still a solver-specific optimization rather than a mathematical change.

These are listed in IMPORTANT_QUESTIONS_FOR_AUTHOR.md for author confirmation.
