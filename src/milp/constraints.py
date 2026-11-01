"""Constraint builders for the MILP formulation.

The implementation is intentionally conservative: constraints are written to be
numerically stable and aligned with the data model, while keeping several
manuscript choices explicit in configuration and TODO files.
"""
from __future__ import annotations

from typing import Dict, Tuple

from ..data_models import InstanceData
from .backend import gp


Var = object


def add_return_to_depot(
    model,
    instance: InstanceData,
    x_truck: Dict[Tuple[int, int, int], Var],
    x_drone: Dict[Tuple[int, int, int], Var],
    nodes: list[int],
) -> None:
    """Enforce depot conservation for every used vehicle."""
    customers = _customer_nodes(nodes)
    for t in instance.truck_ids():
        model.addConstr(
            gp.quicksum(x_truck[(0, j, t)] for j in customers)
            == gp.quicksum(x_truck[(j, 0, t)] for j in customers),
            name=f"truck_return_balance_{t}",
        )
        model.addConstr(
            gp.quicksum(x_truck[(0, j, t)] for j in customers) <= 1,
            name=f"truck_single_route_{t}",
        )
    for d in instance.drone_ids():
        model.addConstr(
            gp.quicksum(x_drone[(0, j, d)] for j in customers)
            == gp.quicksum(x_drone[(j, 0, d)] for j in customers),
            name=f"drone_return_balance_{d}",
        )
        model.addConstr(
            gp.quicksum(x_drone[(0, j, d)] for j in customers) <= 1,
            name=f"drone_single_route_{d}",
        )


def add_service_and_flow(
    model,
    instance: InstanceData,
    x_truck: Dict[Tuple[int, int, int], Var],
    x_drone: Dict[Tuple[int, int, int], Var],
    u_truck: Dict[Tuple[int, int], Var],
    u_drone: Dict[Tuple[int, int], Var],
    nodes: list[int],
) -> None:
    """Customer service, visit, and route-flow constraints."""
    customers = _customer_nodes(nodes)
    for j in customers:
        model.addConstr(
            gp.quicksum(u_truck[(t, j)] for t in instance.truck_ids())
            + gp.quicksum(u_drone[(d, j)] for d in instance.drone_ids())
            == 1,
            name=f"exclusive_service_{j}",
        )
        truck_visits = []
        drone_visits = []
        for t in instance.truck_ids():
            incoming = gp.quicksum(x_truck[(i, j, t)] for i in nodes if i != j)
            outgoing = gp.quicksum(x_truck[(j, i, t)] for i in nodes if i != j)
            truck_visits.append(incoming)
            model.addConstr(
                incoming == outgoing,
                name=f"truck_in_{t}_{j}",
            )
            model.addConstr(
                u_truck[(t, j)] <= incoming,
                name=f"truck_out_{t}_{j}",
            )
        for d in instance.drone_ids():
            incoming = gp.quicksum(x_drone[(i, j, d)] for i in nodes if i != j)
            outgoing = gp.quicksum(x_drone[(j, i, d)] for i in nodes if i != j)
            drone_visits.append(incoming)
            model.addConstr(
                incoming == outgoing,
                name=f"drone_in_{d}_{j}",
            )
            model.addConstr(
                u_drone[(d, j)] <= incoming,
                name=f"drone_out_{d}_{j}",
            )
        model.addConstr(gp.quicksum(truck_visits) <= 1, name=f"truck_visit_once_{j}")
        model.addConstr(gp.quicksum(drone_visits) <= 1, name=f"drone_visit_once_{j}")

    for t in instance.truck_ids():
        for i in nodes:
            model.addConstr(
                gp.quicksum(x_truck[(i, j, t)] for j in nodes if i != j)
                == gp.quicksum(x_truck[(j, i, t)] for j in nodes if i != j),
                name=f"truck_flow_{t}_{i}",
            )
    for d in instance.drone_ids():
        for i in nodes:
            model.addConstr(
                gp.quicksum(x_drone[(i, j, d)] for j in nodes if i != j)
                == gp.quicksum(x_drone[(j, i, d)] for j in nodes if i != j),
                name=f"drone_flow_{d}_{i}",
            )


def add_demand_and_load(
    model,
    instance: InstanceData,
    x_truck: Dict[Tuple[int, int, int], Var],
    x_drone: Dict[Tuple[int, int, int], Var],
    u_truck: Dict[Tuple[int, int], Var],
    y_loaded: Dict[Tuple[int, int], Var],
    w: Dict[Tuple[int, int], Var],
    z1: Dict[Tuple[int, int, int], Var],
    z2: Dict[Tuple[int, int, int], Var],
    u_sync: Dict[Tuple[int, int, int], Var],
    nodes: list[int],
    u_drone: Dict[Tuple[int, int], Var],
    lambda_loaded: Dict[Tuple[int, int, int], Var],
    big_m: float,
) -> None:
    """Truck inventory, drone parcel state, and synchronization links."""
    customers = _customer_nodes(nodes)
    for d in instance.drone_ids():
        for t in instance.truck_ids():
            model.addConstr(z1[(0, d, t)] == 0, name=f"no_depot_reload_{d}_{t}")
            model.addConstr(z2[(0, d, t)] == 0, name=f"no_depot_swap_{d}_{t}")
            model.addConstr(u_sync[(0, d, t)] == 0, name=f"no_depot_rendezvous_{d}_{t}")
    for t in instance.truck_ids():
        truck = instance.trucks[t - 1]
        route_use = gp.quicksum(x_truck[(0, j, t)] for j in customers)
        model.addConstr(w[(t, 0)] == truck.capacity_kg * route_use, name=f"truck_load_start_{t}")
        for j in nodes:
            model.addConstr(w[(t, j)] >= 0.0, name=f"truck_load_nonneg_{t}_{j}")
            model.addConstr(w[(t, j)] <= truck.capacity_kg, name=f"truck_load_cap_{t}_{j}")
        for i in nodes:
            for j in customers:
                if i == j:
                    continue
                demand_j = instance.customer_map()[j].demand_kg
                reload_weight = gp.quicksum(
                    instance.drones[d - 1].parcel_weight_kg * z1[(j, d, t)]
                    for d in instance.drone_ids()
                )
                inventory_change = demand_j * u_truck[(t, j)] + reload_weight
                model.addConstr(
                    w[(t, j)]
                    <= w[(t, i)] - inventory_change + big_m * (1 - x_truck[(i, j, t)]),
                    name=f"truck_load_ub_{t}_{i}_{j}",
                )
                model.addConstr(
                    w[(t, j)]
                    >= w[(t, i)] - inventory_change - big_m * (1 - x_truck[(i, j, t)]),
                    name=f"truck_load_lb_{t}_{i}_{j}",
                )

    for d in instance.drone_ids():
        for i in customers:
            model.addConstr(
                instance.customer_map()[i].demand_kg * u_drone[(d, i)] <= instance.drones[d - 1].capacity_kg,
                name=f"drone_payload_{d}_{i}",
            )
            incoming_loaded = gp.quicksum(lambda_loaded[(h, i, d)] for h in nodes if h != i)
            outgoing_loaded = gp.quicksum(lambda_loaded[(i, j, d)] for j in nodes if j != i)
            model.addConstr(y_loaded[(d, i)] == incoming_loaded, name=f"loaded_arrival_{d}_{i}")
            model.addConstr(u_drone[(d, i)] <= incoming_loaded, name=f"loaded_service_{d}_{i}")
            model.addConstr(
                outgoing_loaded == incoming_loaded - u_drone[(d, i)] + gp.quicksum(z1[(i, d, t)] for t in instance.truck_ids()),
                name=f"loaded_flow_{d}_{i}",
            )
            for t in instance.truck_ids():
                truck_visit = gp.quicksum(x_truck[(h, i, t)] for h in nodes if h != i)
                drone_visit = gp.quicksum(x_drone[(h, i, d)] for h in nodes if h != i)
                model.addConstr(u_sync[(i, d, t)] <= truck_visit, name=f"usync_truck_visit_{i}_{d}_{t}")
                model.addConstr(u_sync[(i, d, t)] <= drone_visit, name=f"usync_drone_visit_{i}_{d}_{t}")
                model.addConstr(
                    u_sync[(i, d, t)] >= truck_visit + drone_visit - 1,
                    name=f"usync_joint_visit_{i}_{d}_{t}",
                )
                model.addConstr(z1[(i, d, t)] <= u_sync[(i, d, t)], name=f"z1_usync_link_{i}_{d}_{t}")
                model.addConstr(z2[(i, d, t)] <= u_sync[(i, d, t)], name=f"z2_usync_link_{i}_{d}_{t}")
                model.addConstr(z1[(i, d, t)] <= truck_visit, name=f"z1_truck_visit_{i}_{d}_{t}")
                model.addConstr(z1[(i, d, t)] <= drone_visit, name=f"z1_drone_visit_{i}_{d}_{t}")
                model.addConstr(z2[(i, d, t)] <= truck_visit, name=f"z2_truck_visit_{i}_{d}_{t}")
                model.addConstr(z2[(i, d, t)] <= drone_visit, name=f"z2_drone_visit_{i}_{d}_{t}")
            model.addConstr(
                gp.quicksum(z1[(i, d, t)] for t in instance.truck_ids()) <= 1,
                name=f"sync_one_reload_truck_{i}_{d}",
            )
            model.addConstr(
                gp.quicksum(z2[(i, d, t)] for t in instance.truck_ids()) <= 1,
                name=f"sync_one_swap_truck_{i}_{d}",
            )

        for i in nodes:
            for j in nodes:
                if i == j:
                    continue
                model.addConstr(
                    lambda_loaded[(i, j, d)] <= x_drone[(i, j, d)],
                    name=f"loaded_arc_link_{d}_{i}_{j}",
                )


def add_energy(
    model,
    instance: InstanceData,
    x_drone: Dict[Tuple[int, int, int], Var],
    r: Dict[Tuple[int, int], Var],
    r_departure: Dict[Tuple[int, int], Var],
    t_empty: Dict[Tuple[int, int, int], float],
    lambda_loaded: Dict[Tuple[int, int, int], Var],
    z2: Dict[Tuple[int, int, int], Var],
    nodes: list[int],
    big_m: float,
) -> None:
    """Propagate arrival/departure battery states with loaded-flight energy."""
    for d in instance.drone_ids():
        drone = instance.drones[d - 1]
        model.addConstr(r[(d, 0)] == drone.max_battery_wh, name=f"drone_batt_start_{d}")
        for j in nodes:
            model.addConstr(r[(d, j)] >= 0.0, name=f"drone_batt_nonneg_{d}_{j}")
            model.addConstr(r[(d, j)] <= drone.max_battery_wh, name=f"drone_batt_cap_{d}_{j}")
            model.addConstr(r_departure[(d, j)] >= 0.0, name=f"drone_batt_dep_nonneg_{d}_{j}")
            model.addConstr(r_departure[(d, j)] <= drone.max_battery_wh, name=f"drone_batt_dep_cap_{d}_{j}")
        model.addConstr(r_departure[(d, 0)] == drone.max_battery_wh, name=f"drone_batt_start_depart_{d}")
        reserve = max(0.0, min(1.0, float(instance.metadata.get("battery_slack_ratio", 0.10)))) * drone.max_battery_wh
        for i in nodes:
            for j in nodes:
                if i == j:
                    continue
                energy = t_empty[(i, j, d)] * (
                    drone.energy_per_min_when_empty
                    + (drone.energy_per_min_when_loaded - drone.energy_per_min_when_empty) * lambda_loaded[(i, j, d)]
                )
                swap_at_i = gp.quicksum(z2[(i, d, t)] for t in instance.truck_ids()) if i != 0 else 0.0
                model.addConstr(
                    r_departure[(d, i)] <= r[(d, i)] + drone.max_battery_wh * swap_at_i + big_m * (1 - swap_at_i),
                    name=f"drone_batt_depart_ub_{d}_{i}_{j}",
                )
                model.addConstr(
                    r_departure[(d, i)] >= r[(d, i)] - big_m * swap_at_i,
                    name=f"drone_batt_depart_lb_{d}_{i}_{j}",
                )
                if j != 0:
                    model.addConstr(
                        r[(d, j)] <= r_departure[(d, i)] - energy + big_m * (1 - x_drone[(i, j, d)]),
                        name=f"drone_batt_ub_{d}_{i}_{j}",
                    )
                    model.addConstr(
                        r[(d, j)] >= r_departure[(d, i)] - energy - big_m * (1 - x_drone[(i, j, d)]),
                        name=f"drone_batt_lb_{d}_{i}_{j}",
                    )
                    model.addConstr(
                        r[(d, j)] >= reserve - big_m * (1 - x_drone[(i, j, d)]),
                        name=f"drone_batt_reserve_{d}_{i}_{j}",
                    )
                else:
                    model.addConstr(
                        r_departure[(d, i)] - energy >= reserve - big_m * (1 - x_drone[(i, j, d)]),
                        name=f"drone_batt_return_reserve_{d}_{i}",
                    )
        for i in instance.customer_ids:
            swap_at_i = gp.quicksum(z2[(i, d, t)] for t in instance.truck_ids())
            model.addConstr(r_departure[(d, i)] >= drone.max_battery_wh * swap_at_i, name=f"drone_batt_swap_lb_{d}_{i}")
            model.addConstr(r_departure[(d, i)] <= r[(d, i)] + drone.max_battery_wh * swap_at_i, name=f"drone_batt_swap_ub_{d}_{i}")


def add_timing(
    model,
    instance: InstanceData,
    x_truck: Dict[Tuple[int, int, int], Var],
    x_drone: Dict[Tuple[int, int, int], Var],
    a_truck: Dict[Tuple[int, int], Var],
    a_drone: Dict[Tuple[int, int], Var],
    ready_truck: Dict[Tuple[int, int], Var],
    ready_drone: Dict[Tuple[int, int], Var],
    l_truck: Dict[Tuple[int, int], Var],
    l_drone: Dict[Tuple[int, int], Var],
    tw_truck: Dict[Tuple[int, int], Var],
    tw_drone: Dict[Tuple[int, int], Var],
    u_truck: Dict[Tuple[int, int], Var],
    u_drone: Dict[Tuple[int, int], Var],
    z1: Dict[Tuple[int, int, int], Var],
    z2: Dict[Tuple[int, int, int], Var],
    u_sync: Dict[Tuple[int, int, int], Var],
    t_truck: Dict[Tuple[int, int, int], float],
    t_empty: Dict[Tuple[int, int, int], float],
    nodes: list[int],
    big_m: float,
    swap_time_min: float,
    reload_time_min: float,
    handling_time_min: float,
    return_time_truck: Dict[int, Var],
    return_time_drone: Dict[int, Var],
) -> None:
    customers = _customer_nodes(nodes)
    for t in instance.truck_ids():
        truck = instance.trucks[t - 1]
        model.addConstr(a_truck[(t, 0)] == 0.0, name=f"truck_start_{t}")
        model.addConstr(l_truck[(t, 0)] == 0.0, name=f"truck_start_depart_{t}")
        for i in nodes:
            service_i = instance.customer(i).service_time_min if i != 0 else 0.0
            model.addConstr(
                ready_truck[(t, i)] == a_truck[(t, i)] + service_i * u_truck.get((t, i), 0.0),
                name=f"truck_ready_{t}_{i}",
            )
            model.addConstr(
                l_truck[(t, i)]
                >= a_truck[(t, i)]
                + service_i * u_truck.get((t, i), 0.0)
                + 2.0 * handling_time_min * gp.quicksum(u_sync[(i, d, t)] for d in instance.drone_ids())
                + reload_time_min * gp.quicksum(z1[(i, d, t)] for d in instance.drone_ids())
                + swap_time_min * gp.quicksum(z2[(i, d, t)] for d in instance.drone_ids())
                + tw_truck[(t, i)],
                name=f"truck_depart_{t}_{i}",
            )
            model.addConstr(tw_truck[(t, i)] >= 0.0, name=f"truck_wait_nonneg_{t}_{i}")
            model.addConstr(tw_truck[(t, i)] <= truck.capacity_kg * 0 + big_m, name=f"truck_wait_ub_{t}_{i}")

    for d in instance.drone_ids():
        model.addConstr(a_drone[(d, 0)] == 0.0, name=f"drone_start_{d}")
        model.addConstr(l_drone[(d, 0)] == 0.0, name=f"drone_start_depart_{d}")
        for i in nodes:
            service_i = instance.customer(i).service_time_min if i != 0 else 0.0
            model.addConstr(
                ready_drone[(d, i)] == a_drone[(d, i)] + service_i * u_drone.get((d, i), 0.0),
                name=f"drone_ready_{d}_{i}",
            )
            model.addConstr(
                l_drone[(d, i)]
                >= a_drone[(d, i)]
                + service_i * u_drone.get((d, i), 0.0)
                + 2.0 * handling_time_min * gp.quicksum(u_sync[(i, d, t)] for t in instance.truck_ids())
                + reload_time_min * gp.quicksum(z1[(i, d, t)] for t in instance.truck_ids())
                + swap_time_min * gp.quicksum(z2[(i, d, t)] for t in instance.truck_ids())
                + tw_drone[(d, i)],
                name=f"drone_depart_{d}_{i}",
            )
            model.addConstr(tw_drone[(d, i)] >= 0.0, name=f"drone_wait_nonneg_{d}_{i}")

    for t in instance.truck_ids():
        for i in nodes:
            for j in nodes:
                if i == j:
                    continue
                if j == 0:
                    model.addConstr(
                        return_time_truck[t] >= l_truck[(t, i)] + t_truck[(i, j, t)] - big_m * (1 - x_truck[(i, j, t)]),
                        name=f"truck_return_lb_{t}_{i}",
                    )
                    model.addConstr(
                        return_time_truck[t] <= l_truck[(t, i)] + t_truck[(i, j, t)] + big_m * (1 - x_truck[(i, j, t)]),
                        name=f"truck_return_ub_{t}_{i}",
                    )
                else:
                    model.addConstr(
                        l_truck[(t, i)] + t_truck[(i, j, t)] - big_m * (1 - x_truck[(i, j, t)])
                        <= a_truck[(t, j)],
                        name=f"truck_arrival_lb_{t}_{i}_{j}",
                    )
                    model.addConstr(
                        a_truck[(t, j)] <= l_truck[(t, i)] + t_truck[(i, j, t)] + big_m * (1 - x_truck[(i, j, t)]),
                        name=f"truck_arrival_ub_{t}_{i}_{j}",
                    )

    for d in instance.drone_ids():
        for i in nodes:
            for j in nodes:
                if i == j:
                    continue
                if j == 0:
                    model.addConstr(
                        return_time_drone[d] >= l_drone[(d, i)] + t_empty[(i, j, d)] - big_m * (1 - x_drone[(i, j, d)]),
                        name=f"drone_return_lb_{d}_{i}",
                    )
                    model.addConstr(
                        return_time_drone[d] <= l_drone[(d, i)] + t_empty[(i, j, d)] + big_m * (1 - x_drone[(i, j, d)]),
                        name=f"drone_return_ub_{d}_{i}",
                    )
                else:
                    model.addConstr(
                        l_drone[(d, i)] + t_empty[(i, j, d)] - big_m * (1 - x_drone[(i, j, d)])
                        <= a_drone[(d, j)],
                        name=f"drone_arrival_lb_{d}_{i}_{j}",
                    )
                    model.addConstr(
                        a_drone[(d, j)] <= l_drone[(d, i)] + t_empty[(i, j, d)] + big_m * (1 - x_drone[(i, j, d)]),
                        name=f"drone_arrival_ub_{d}_{i}_{j}",
                    )

    for i in customers:
        for d in instance.drone_ids():
            for t in instance.truck_ids():
                model.addConstr(
                    tw_truck[(t, i)] >= ready_drone[(d, i)] - ready_truck[(t, i)] - big_m * (1 - u_sync[(i, d, t)]),
                    name=f"sync_wait_truck_{d}_{t}_{i}",
                )
                model.addConstr(
                    tw_drone[(d, i)] >= ready_truck[(t, i)] - ready_drone[(d, i)] - big_m * (1 - u_sync[(i, d, t)]),
                    name=f"sync_wait_drone_{d}_{t}_{i}",
                )


def add_tardiness_linearization(
    model,
    instance: InstanceData,
    a_truck: Dict[Tuple[int, int], Var],
    a_drone: Dict[Tuple[int, int], Var],
    a_bar_truck: Dict[Tuple[int, int], Var],
    a_bar_drone: Dict[Tuple[int, int], Var],
    u_truck: Dict[Tuple[int, int], Var],
    u_drone: Dict[Tuple[int, int], Var],
    tardiness: Dict[int, Var],
    nodes: list[int],
) -> None:
    customers = _customer_nodes(nodes)
    max_shift = instance.constants.max_shift_minutes
    for t in instance.truck_ids():
        for i in customers:
            model.addConstr(a_bar_truck[(t, i)] <= a_truck[(t, i)], name=f"mc_atruck_ub_{t}_{i}")
            model.addConstr(a_bar_truck[(t, i)] <= max_shift * u_truck[(t, i)], name=f"mc_atruck_active_{t}_{i}")
            model.addConstr(
                a_bar_truck[(t, i)] >= a_truck[(t, i)] - max_shift * (1 - u_truck[(t, i)]),
                name=f"mc_atruck_lb_{t}_{i}",
            )
    for d in instance.drone_ids():
        for i in customers:
            model.addConstr(a_bar_drone[(d, i)] <= a_drone[(d, i)], name=f"mc_adrone_ub_{d}_{i}")
            model.addConstr(a_bar_drone[(d, i)] <= max_shift * u_drone[(d, i)], name=f"mc_adrone_active_{d}_{i}")
            model.addConstr(
                a_bar_drone[(d, i)] >= a_drone[(d, i)] - max_shift * (1 - u_drone[(d, i)]),
                name=f"mc_adrone_lb_{d}_{i}",
            )
    for i in customers:
        model.addConstr(
            tardiness[i]
            >= gp.quicksum(a_bar_truck[(t, i)] for t in instance.truck_ids())
            + gp.quicksum(a_bar_drone[(d, i)] for d in instance.drone_ids())
            - instance.customer_map()[i].ub,
            name=f"tardiness_ge_{i}",
        )
        model.addConstr(tardiness[i] >= 0.0, name=f"tardiness_nonneg_{i}")


def add_priority_ordering(
    model,
    instance: InstanceData,
    a_bar_truck: Dict[Tuple[int, int], Var],
    a_bar_drone: Dict[Tuple[int, int], Var],
    nodes: list[int],
    big_m: float,
) -> None:
    """Priority structure constraints as precedence by class."""
    customers = _customer_nodes(nodes)
    if not customers:
        return

    precedence_pairs = {("high", "medium"), ("high", "low"), ("medium", "low")}
    for i in customers:
        for j in customers:
            if i == j:
                continue
            ci = instance.customer_map()[i].priority
            cj = instance.customer_map()[j].priority
            if (ci, cj) not in precedence_pairs:
                continue
            model.addConstr(
                gp.quicksum(a_bar_truck[(t, i)] for t in instance.truck_ids())
                + gp.quicksum(a_bar_drone[(d, i)] for d in instance.drone_ids())
                <= gp.quicksum(a_bar_truck[(t, j)] for t in instance.truck_ids())
                + gp.quicksum(a_bar_drone[(d, j)] for d in instance.drone_ids())
                + big_m,
                name=f"priority_order_{ci}_{cj}_{i}_{j}",
            )


def add_time_windows_and_shifts(
    model,
    instance: InstanceData,
    a_truck: Dict[Tuple[int, int], Var],
    a_drone: Dict[Tuple[int, int], Var],
    u_truck: Dict[Tuple[int, int], Var],
    u_drone: Dict[Tuple[int, int], Var],
    nodes: list[int],
    enforce_subtours: bool,
    x_truck: Dict[Tuple[int, int, int], Var],
    x_drone: Dict[Tuple[int, int, int], Var],
    big_m: float = 10_000.0,
    enforce_hard_upper_windows: bool = False,
    return_time_truck: Dict[int, Var] | None = None,
    return_time_drone: Dict[int, Var] | None = None,
) -> None:
    customers = _customer_nodes(nodes)
    for i in customers:
        c = instance.customer_map()[i]
        for t in instance.truck_ids():
            if enforce_hard_upper_windows:
                model.addConstr(a_truck[(t, i)] <= c.ub + big_m * (1 - u_truck[(t, i)]), name=f"tw_truck_ub_{t}_{i}")
            model.addConstr(a_truck[(t, i)] >= c.lb - big_m * (1 - u_truck[(t, i)]), name=f"tw_truck_lb_{t}_{i}")
        for d in instance.drone_ids():
            if enforce_hard_upper_windows:
                model.addConstr(a_drone[(d, i)] <= c.ub + big_m * (1 - u_drone[(d, i)]), name=f"tw_drone_ub_{d}_{i}")
            model.addConstr(a_drone[(d, i)] >= c.lb - big_m * (1 - u_drone[(d, i)]), name=f"tw_drone_lb_{d}_{i}")

    shift_limit = instance.constants.max_shift_minutes
    for t in instance.truck_ids():
        if return_time_truck is not None:
            model.addConstr(return_time_truck[t] <= shift_limit, name=f"truck_shift_{t}")
        else:
            model.addConstr(gp.quicksum(a_truck[(t, i)] for i in customers) <= shift_limit, name=f"truck_shift_{t}")
    for d in instance.drone_ids():
        if return_time_drone is not None:
            model.addConstr(return_time_drone[d] <= shift_limit, name=f"drone_shift_{d}")
        else:
            model.addConstr(gp.quicksum(a_drone[(d, i)] for i in customers) <= shift_limit, name=f"drone_shift_{d}")

    if enforce_subtours:
        _mtz_subtour(model, instance, x_truck, nodes, "T", big_m)
        _mtz_subtour(model, instance, x_drone, nodes, "D", big_m)


def _mtz_subtour(
    model,
    instance: InstanceData,
    x: Dict[Tuple[int, int, int], Var],
    nodes: list[int],
    suffix: str,
    big_m: float,
) -> None:
    customers = _customer_nodes(nodes)
    if not customers:
        return
    vehicle_ids = instance.truck_ids() if suffix == "T" else instance.drone_ids()
    n = len(customers)
    for v in vehicle_ids:
        u = {i: model.addVar(lb=0.0, ub=n, name=f"mtz_{suffix}_{v}_{i}") for i in customers}
        for i in customers:
            for j in customers:
                if i == j:
                    continue
                model.addConstr(
                    u[i] - u[j] + n * x[(i, j, v)] <= n - 1 + big_m * (1 - x[(i, j, v)]),
                    name=f"mtz_{suffix}_{v}_{i}_{j}",
                )


def _customer_nodes(nodes: list[int]) -> list[int]:
    return [i for i in nodes if i != 0]

