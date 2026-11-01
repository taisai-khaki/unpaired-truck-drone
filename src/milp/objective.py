"""Objective and component accounting for the MILP."""
from __future__ import annotations

from typing import Dict, Tuple

from ..data_models import InstanceData
from .backend import gp


def build_objective(
    model,
    instance: InstanceData,
    x_truck: Dict[Tuple[int, int, int], object],
    x_drone: Dict[Tuple[int, int, int], object],
    y_loaded: Dict[Tuple[int, int], object],
    tw_truck: Dict[Tuple[int, int], object],
    tw_drone: Dict[Tuple[int, int], object],
    tardiness: Dict[int, object],
    t_truck: Dict[Tuple[int, int, int], float],
    t_empty: Dict[Tuple[int, int, int], float],
    t_loaded: Dict[Tuple[int, int, int], float],
    nodes: list[int],
    u_truck: Dict[Tuple[int, int], object],
    u_drone: Dict[Tuple[int, int], object],
    a_bar_truck: Dict[Tuple[int, int], object],
    a_bar_drone: Dict[Tuple[int, int], object],
    lambda_loaded: Dict[Tuple[int, int, int], object] | None = None,
    include_waiting: bool = False,
) -> Dict[str, object]:
    """Construct the exact objective and return component expressions."""
    truck_cost_terms = []
    drone_cost_terms = []
    waiting_terms = []

    customers = [i for i in nodes if i != 0]
    for t in instance.truck_ids():
        truck = instance.trucks[t - 1]
        for i in nodes:
            for j in nodes:
                if i == j:
                    continue
                distance_km = t_truck[(i, j, t)] * truck.speed_kmph / 60.0
                truck_cost_terms.append(
                    x_truck[(i, j, t)]
                    * (truck.cost_per_km * distance_km + truck.energy_cost_per_min * t_truck[(i, j, t)]),
                )
                if include_waiting:
                    waiting_terms.append(tw_truck[(t, i)])

    for d in instance.drone_ids():
        drone = instance.drones[d - 1]
        for i in nodes:
            for j in nodes:
                if i == j:
                    continue
                # Distance is derived from travel-time entries to avoid duplicate
                # pre-computation in constraints.
                distance_km = t_empty[(i, j, d)] * drone.speed_kmph / 60.0
                loaded_state = (
                    lambda_loaded.get((i, j, d), y_loaded.get((d, j), 0.0))
                    if lambda_loaded is not None
                    else y_loaded.get((d, j), 0.0)
                )
                energy = t_empty[(i, j, d)] * (
                    drone.energy_per_min_when_empty * x_drone[(i, j, d)]
                    + (drone.energy_per_min_when_loaded - drone.energy_per_min_when_empty) * loaded_state
                )
                drone_cost_terms.append(
                    drone.cost_per_km * distance_km * x_drone[(i, j, d)]
                    + drone.energy_cost_per_min * energy,
                )
                if include_waiting:
                    waiting_terms.append(tw_drone[(d, i)])

    tardiness_cost = gp.quicksum(
        tardiness[i] * instance.priority_penalties[instance.customer_map()[i].priority] for i in customers
    )
    truck_cost = gp.quicksum(truck_cost_terms)
    drone_cost = gp.quicksum(drone_cost_terms)
    waiting_cost = gp.quicksum(waiting_terms) if include_waiting else 0.0

    components = {
        "truck_cost": truck_cost,
        "drone_cost": drone_cost,
        "tardiness_cost": tardiness_cost,
        "waiting_cost": waiting_cost if include_waiting else 0.0,
    }

    objective = components["truck_cost"] + components["drone_cost"] + components["tardiness_cost"]
    if include_waiting:
        objective = objective + components["waiting_cost"]
    components["total"] = objective
    model.setObjective(objective, gp.GRB.MINIMIZE)
    return components

