"""Nested Iterative Local Search (NILS) implementation used in experiments."""
from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

from ..data_models import InstanceData, SolutionData
from ..distance_utils import euclidean_distance, manhattan_distance
from ..feasibility import is_feasible
from ..timing import (
    active_rendezvous_events,
    compute_synchronization_waiting,
    synchronization_handling_minutes,
)
from .acceptance import metropolis
from .construction import _fill_solution_from_routes, build_initial_solution
from .local_search import apply_two_opt_to_solution
from .perturbation import relocate_random_customer


def _travel_time(distance_km: float, speed_kmph: float) -> float:
    if speed_kmph <= 0:
        return math.inf
    return distance_km / speed_kmph * 60.0


def _schedule_routes(
    instance: InstanceData,
    solution: SolutionData,
    coords: Dict[int, Tuple[float, float]],
) -> Tuple[
    Dict[Tuple[int, int], float],
    Dict[Tuple[int, int], float],
    Dict[Tuple[int, int], float],
    Dict[Tuple[int, int], float],
    Dict[Tuple[int, int], float],
    Dict[Tuple[int, int], float],
    Dict[Tuple[int, int], float],
    Dict[Tuple[int, int], float],
]:
    """Propagate service, window, synchronization, and handling delays."""
    truck_wait: Dict[Tuple[int, int], float] = {}
    drone_wait: Dict[Tuple[int, int], float] = {}

    def schedule_vehicle(
        route: List[int],
        speed: float,
        metric: str,
        vehicle_id: int,
        is_truck: bool,
        waits: Dict[Tuple[int, int], float],
    ) -> Tuple[
        Dict[Tuple[int, int], float],
        Dict[Tuple[int, int], float],
        Dict[Tuple[int, int], float],
    ]:
        arrivals: Dict[Tuple[int, int], float] = {}
        departures: Dict[Tuple[int, int], float] = {}
        readiness: Dict[Tuple[int, int], float] = {}
        current = 0.0
        for idx in range(len(route) - 1):
            frm = route[idx]
            if frm != 0:
                service = instance.customer_map()[frm].service_time_min
                assigned = (
                    solution.u_truck.get((vehicle_id, frm), 0) >= 0.5
                    if is_truck
                    else solution.u_drone.get((vehicle_id, frm), 0) >= 0.5
                )
                readiness[(vehicle_id, frm)] = current + (service if assigned else 0.0)
            else:
                readiness[(vehicle_id, frm)] = current
            to = route[idx + 1]
            if frm != 0:
                if is_truck:
                    current += instance.customer_map()[frm].service_time_min * solution.u_truck.get((vehicle_id, frm), 0)
                else:
                    current += instance.customer_map()[frm].service_time_min * solution.u_drone.get((vehicle_id, frm), 0)
            current += waits.get((vehicle_id, frm), 0.0)
            if is_truck:
                current += sum(
                    synchronization_handling_minutes(instance, solution, frm, drone_id, vehicle_id)
                    for node, drone_id, truck_id in active_rendezvous_events(solution)
                    if node == frm and truck_id == vehicle_id
                )
            else:
                current += sum(
                    synchronization_handling_minutes(instance, solution, frm, vehicle_id, truck_id)
                    for node, drone_id, truck_id in active_rendezvous_events(solution)
                    if node == frm and drone_id == vehicle_id
                )
            departures[(vehicle_id, frm)] = current
            if metric == "manhattan":
                distance = manhattan_distance(coords[frm], coords[to])
            else:
                distance = euclidean_distance(coords[frm], coords[to])
            current += _travel_time(distance, speed)
            if to != 0:
                assigned = (
                    solution.u_truck.get((vehicle_id, to), 0) >= 0.5
                    if is_truck
                    else solution.u_drone.get((vehicle_id, to), 0) >= 0.5
                )
                if assigned:
                    current = max(current, instance.customer_map()[to].lb)
            arrivals[(vehicle_id, to)] = current
        if route:
            last = route[-1]
            if last not in readiness:
                readiness[(vehicle_id, last)] = current
            departures[(vehicle_id, route[-1])] = current
        return arrivals, departures, readiness

    for _ in range(max(1, 2 * (instance.num_customers + 1))):
        truck_arrivals: Dict[Tuple[int, int], float] = {}
        truck_departures: Dict[Tuple[int, int], float] = {}
        truck_readiness: Dict[Tuple[int, int], float] = {}
        drone_arrivals: Dict[Tuple[int, int], float] = {}
        drone_departures: Dict[Tuple[int, int], float] = {}
        drone_readiness: Dict[Tuple[int, int], float] = {}
        for truck_id, route in solution.truck_routes.items():
            arrivals, departures, readiness = schedule_vehicle(
                route,
                instance.trucks[truck_id - 1].speed_kmph,
                "manhattan",
                truck_id,
                True,
                truck_wait,
            )
            truck_arrivals.update(arrivals)
            truck_departures.update(departures)
            truck_readiness.update(readiness)
        for drone_id, route in solution.drone_routes.items():
            arrivals, departures, readiness = schedule_vehicle(
                route,
                instance.drones[drone_id - 1].speed_kmph,
                "euclidean",
                drone_id,
                False,
                drone_wait,
            )
            drone_arrivals.update(arrivals)
            drone_departures.update(departures)
            drone_readiness.update(readiness)

        changed = False
        for node, drone_id, truck_id in active_rendezvous_events(solution):
            truck_arrival = truck_arrivals.get((truck_id, node))
            drone_arrival = drone_arrivals.get((drone_id, node))
            if truck_arrival is None or drone_arrival is None:
                continue
            truck_ready = truck_readiness.get((truck_id, node), truck_arrival)
            drone_ready = drone_readiness.get((drone_id, node), drone_arrival)
            new_truck_wait = max(truck_wait.get((truck_id, node), 0.0), max(0.0, drone_ready - truck_ready))
            new_drone_wait = max(drone_wait.get((drone_id, node), 0.0), max(0.0, truck_ready - drone_ready))
            if new_truck_wait > truck_wait.get((truck_id, node), 0.0) + 1e-9:
                truck_wait[(truck_id, node)] = new_truck_wait
                changed = True
            if new_drone_wait > drone_wait.get((drone_id, node), 0.0) + 1e-9:
                drone_wait[(drone_id, node)] = new_drone_wait
                changed = True
        if not changed:
            break

    return (
        truck_arrivals,
        truck_departures,
        drone_arrivals,
        drone_departures,
        truck_wait,
        drone_wait,
        truck_readiness,
        drone_readiness,
    )


def evaluate_solution(instance: InstanceData, solution: SolutionData) -> Tuple[float, Dict[str, float]]:
    """Evaluate a route plan under the paper's objective.

    Synchronization waiting and handling time are propagated into route timing,
    but neither is a separate objective component. The objective is
    truck operating cost + drone operating cost + priority-weighted
    tardiness, as in equations (1)--(6) of the manuscript.
    """
    coords = {0: (0.0, 0.0)}
    coords.update({c.node_id: (c.x, c.y) for c in instance.customers})

    (
        arrival_truck,
        departure_truck,
        arrival_drone,
        departure_drone,
        truck_wait,
        drone_wait,
        truck_readiness,
        drone_readiness,
    ) = _schedule_routes(instance, solution, coords)
    served_time: Dict[int, float] = {j: 0.0 for j in range(1, instance.num_customers + 1)}

    truck_cost = 0.0
    drone_cost = 0.0
    solution.waiting_truck = dict(truck_wait)
    solution.waiting_drone = dict(drone_wait)
    solution.a_truck = {}
    solution.l_truck = {}
    solution.a_drone = {}
    solution.l_drone = {}

    for t, route in solution.truck_routes.items():
        truck = instance.trucks[t - 1]
        for i in range(len(route) - 1):
            to = route[i + 1]
            frm = route[i]
            dist = manhattan_distance(coords[frm], coords[to])
            travel_time = _travel_time(dist, truck.speed_kmph)
            arrival = arrival_truck.get((t, to), 0.0)
            solution.a_truck[(t, to)] = arrival
            solution.l_truck[(t, frm)] = departure_truck.get((t, frm), arrival - travel_time)
            if to != 0 and solution.u_truck.get((t, to), 0) >= 0.5:
                served_time[to] = arrival
            truck_cost += truck.cost_per_km * dist + truck.energy_cost_per_min * travel_time

    for d, route in solution.drone_routes.items():
        drone = instance.drones[d - 1]
        service_nodes = {
            node for node in route
            if node != 0 and solution.u_drone.get((d, node), 0) >= 0.5
        }
        first_service_index = next((idx for idx, node in enumerate(route) if node in service_nodes), None)
        first_reload_index = next(
            (
                idx for idx, node in enumerate(route)
                if any(
                    value > 0.5
                    for (event_node, drone_id, _truck_id), value in solution.z1.items()
                    if event_node == node and drone_id == d
                )
            ),
            None,
        )
        loaded = first_service_index is None or first_reload_index is None or first_service_index < first_reload_index
        battery = float(drone.max_battery_wh)
        for i in range(len(route) - 1):
            frm = route[i]
            to = route[i + 1]
            dist = euclidean_distance(coords[frm], coords[to])
            travel_time = _travel_time(dist, drone.speed_kmph)
            arrival = arrival_drone.get((d, to), 0.0)
            solution.a_drone[(d, to)] = arrival
            solution.l_drone[(d, frm)] = departure_drone.get((d, frm), arrival - travel_time)
            power = drone.energy_per_min_when_loaded if loaded else drone.energy_per_min_when_empty
            energy = max(0.0, float(power)) * travel_time
            battery -= energy
            solution.battery_drone[(d, to)] = battery
            if to != 0 and solution.u_drone.get((d, to), 0) >= 0.5:
                served_time[to] = arrival
                loaded = False
            if to != 0 and any(
                value > 0.5 for (node, drone_id, _truck_id), value in solution.z1.items()
                if node == to and drone_id == d
            ):
                loaded = True
            if to != 0 and any(
                value > 0.5 for (node, drone_id, _truck_id), value in solution.z2.items()
                if node == to and drone_id == d
            ):
                battery = float(drone.max_battery_wh)
                solution.battery_drone[(d, to)] = battery
            drone_cost += drone.cost_per_km * dist + drone.energy_cost_per_min * energy

    recomputed_sync = compute_synchronization_waiting(instance, solution)
    solution.waiting_truck = dict(recomputed_sync["waiting_truck"])
    solution.waiting_drone = dict(recomputed_sync["waiting_drone"])
    solution.__dict__["ready_truck"] = dict(recomputed_sync["readiness_truck"])
    solution.__dict__["ready_drone"] = dict(recomputed_sync["readiness_drone"])

    tardiness_cost = 0.0
    for c in range(1, instance.num_customers + 1):
        delivered_at = served_time[c]
        ci = instance.customer_map()[c]
        penalty = max(0.0, delivered_at - ci.ub)
        solution.tardiness[c] = penalty
        tardiness_cost += penalty * instance.priority_penalties[ci.priority]

    sync_delay_total = float(recomputed_sync["total_waiting_minutes"])

    objective = truck_cost + drone_cost + tardiness_cost
    return objective, {
        "truck_cost": truck_cost,
        "drone_cost": drone_cost,
        "tardiness_cost": tardiness_cost,
        "waiting_cost": 0.0,
        "synchronization_wait_minutes": sync_delay_total,
        "waiting_sync_cost": 0.0,
    }


def compute_drone_operation_metrics(instance: InstanceData, solution: SolutionData) -> Dict[str, object]:
    """Reconcile drone service with loaded sorties and explicit events."""
    assigned_customers = {
        (drone_id, customer)
        for (drone_id, customer), value in solution.u_drone.items()
        if customer > 0 and value >= 0.5
    }
    loaded_service_pairs: set[tuple[int, int]] = set()
    launches = 0
    retrievals = 0
    active_drones = 0
    nonempty_routes = 0
    drone_arcs = 0
    missing_route: list[tuple[int, int]] = []
    unloaded_service: list[tuple[int, int]] = []

    for drone_id in instance.drone_ids():
        route = list(solution.drone_routes.get(drone_id, [0, 0]))
        if any(node != 0 for node in route):
            nonempty_routes += 1
            drone_arcs += max(0, len(route) - 1)
        route_services = {
            node for node in route
            if node != 0 and solution.u_drone.get((drone_id, node), 0) >= 0.5
        }
        missing_route.extend(
            (drone_id, customer)
            for pair_drone, customer in assigned_customers
            if pair_drone == drone_id and customer not in route_services
        )
        first_service = next((idx for idx, node in enumerate(route) if node in route_services), None)
        first_reload = next(
            (
                idx for idx, node in enumerate(route)
                if any(
                    value > 0.5
                    for (event_node, event_drone, _truck), value in solution.z1.items()
                    if event_node == node and event_drone == drone_id
                )
            ),
            None,
        )
        loaded = first_service is None or first_reload is None or first_service < first_reload
        drone_has_loaded_service = False
        for node in route[1:]:
            if node != 0 and solution.u_drone.get((drone_id, node), 0) >= 0.5:
                if loaded:
                    loaded_service_pairs.add((drone_id, node))
                    launches += 1
                    retrievals += 1
                    drone_has_loaded_service = True
                else:
                    unloaded_service.append((drone_id, node))
                loaded = False
            if node != 0 and any(
                value > 0.5
                for (event_node, event_drone, _truck), value in solution.z1.items()
                if event_node == node and event_drone == drone_id
            ):
                loaded = True
        if drone_has_loaded_service:
            active_drones += 1

    return {
        "drone_served_customers": len({customer for _, customer in assigned_customers}),
        "drone_arcs": int(drone_arcs),
        "reload_events": int(sum(value > 0.5 for value in solution.z1.values())),
        "battery_swaps": int(sum(value > 0.5 for value in solution.z2.values())),
        "nonempty_drone_routes": int(nonempty_routes),
        "active_drones_used": int(active_drones),
        "drone_launches": int(launches),
        "drone_retrievals": int(retrievals),
        "loaded_sorties": int(launches),
        "missing_route_service": missing_route,
        "unloaded_service": unloaded_service,
        "all_drone_service_reconciled": not missing_route and not unloaded_service,
    }


def summarize_drone_usage(solution: SolutionData, instance: InstanceData | None = None) -> Dict[str, object]:
    """Return compact, reconciled diagnostics for whether drones were used."""
    if instance is None:
        drone_served_customers = {
            c for (d, c), v in solution.u_drone.items() if c > 0 and v >= 0.5
        }
        nonempty_routes = sum(any(node != 0 for node in route) for route in solution.drone_routes.values())
        loaded_sorties = 0
        unloaded_service = 0
        active_drones = 0
        for drone_id, route in solution.drone_routes.items():
            route_services = {
                node
                for node in route
                if node != 0 and solution.u_drone.get((drone_id, node), 0) >= 0.5
            }
            first_service = next((idx for idx, node in enumerate(route) if node in route_services), None)
            first_reload = next(
                (
                    idx
                    for idx, node in enumerate(route)
                    if any(
                        value > 0.5
                        for (event_node, event_drone, _truck), value in solution.z1.items()
                        if event_node == node and event_drone == drone_id
                    )
                ),
                None,
            )
            loaded = first_service is None or first_reload is None or first_service < first_reload
            has_loaded_sortie = False
            for node in route[1:]:
                if node != 0 and solution.u_drone.get((drone_id, node), 0) >= 0.5:
                    if loaded:
                        loaded_sorties += 1
                        has_loaded_sortie = True
                    else:
                        unloaded_service += 1
                    loaded = False
                if node != 0 and any(
                    value > 0.5
                    for (event_node, event_drone, _truck), value in solution.z1.items()
                    if event_node == node and event_drone == drone_id
                ):
                    loaded = True
            if has_loaded_sortie:
                active_drones += 1
        return {
            "drone_served_customers": len(drone_served_customers),
            "drone_arcs": int(sum(max(0, len(route) - 1) for route in solution.drone_routes.values())),
            "reload_events": int(sum(value > 0.5 for value in solution.z1.values())),
            "battery_swaps": int(sum(value > 0.5 for value in solution.z2.values())),
            "nonempty_drone_routes": int(nonempty_routes),
            "active_drones_used": int(active_drones),
            "drone_launches": int(loaded_sorties),
            "drone_retrievals": int(loaded_sorties),
            "loaded_sorties": int(loaded_sorties),
            "unloaded_service": int(unloaded_service),
            "all_drone_service_reconciled": unloaded_service == 0,
        }
    return compute_drone_operation_metrics(instance, solution)


def default_drone_to_truck(instance: InstanceData) -> Dict[int, int]:
    """Deterministic round-robin default fixed-pair assignment map: drone -> truck."""
    trucks = instance.truck_ids()
    return {d: trucks[(d - 1) % len(trucks)] for d in instance.drone_ids()}


def _normalize_drone_to_truck(
    instance: InstanceData,
    drone_to_truck: Dict[int, int] | None,
) -> Dict[int, int]:
    if not drone_to_truck:
        return {}
    valid_trucks = set(instance.truck_ids())
    normalized: Dict[int, int] = {}
    for drone_id in instance.drone_ids():
        truck_id = int(drone_to_truck.get(drone_id, 0))
        if truck_id not in valid_trucks:
            return {}
        normalized[drone_id] = truck_id
    if set(drone_to_truck.keys()) - set(instance.drone_ids()):
        return {}
    return normalized


def _eligible_customer_ids(instance: InstanceData) -> set[int]:
    """Return customers eligible for drone service under the active scenario metadata."""
    raw = instance.metadata.get("drone_eligible_customers")
    if isinstance(raw, list) and raw:
        valid = {int(v) for v in raw if int(v) in set(instance.customer_ids)}
        if valid:
            return valid
    max_drone_cap = max((d.capacity_kg for d in instance.drones), default=0.0)
    return {c.node_id for c in instance.customers if c.demand_kg <= max_drone_cap}


def structural_admissibility_screen(
    instance: InstanceData,
    *,
    battery_slack_ratio: float = 0.10,
) -> tuple[bool, str]:
    """Apply the necessary resource and basic-reachability screen in the paper."""
    total_demand = sum(float(customer.demand_kg) for customer in instance.customers)
    total_truck_capacity = sum(float(truck.capacity_kg) for truck in instance.trucks)
    if total_demand > total_truck_capacity + 1e-9:
        return False, "aggregate_truck_capacity"

    coords = instance.coordinate_map()
    reserve = max(0.0, min(1.0, float(battery_slack_ratio)))
    for customer in instance.customers:
        truck_serviceable = any(customer.demand_kg <= truck.capacity_kg for truck in instance.trucks)
        drone_serviceable = False
        for drone in instance.drones:
            if customer.demand_kg > drone.capacity_kg:
                continue
            outbound = _travel_time(
                euclidean_distance(coords[0], coords[customer.node_id]), drone.speed_kmph
            )
            energy = outbound * max(drone.energy_per_min_when_loaded, 0.0)
            drone_serviceable = energy * 2.0 <= drone.max_battery_wh * (1.0 - reserve) + 1e-9
            if drone_serviceable:
                break
        truck_reachable = any(
            _route_duration_minutes(
                instance,
                [0, customer.node_id, 0],
                coords=coords,
                speed_kmph=truck.speed_kmph,
                metric="manhattan",
            )
            <= instance.constants.max_shift_minutes + 1e-9
            for truck in instance.trucks
        )
        if not truck_serviceable and not drone_serviceable:
            return False, f"customer_{customer.node_id}_capacity_or_battery"
        if not truck_reachable and not drone_serviceable:
            return False, f"customer_{customer.node_id}_basic_reachability"
    return True, "passed"


def _service_mask(solution: SolutionData) -> Dict[int, str]:
    mask: Dict[int, str] = {}
    for (t, c), v in solution.u_truck.items():
        if v == 1:
            mask[c] = f"truck_{t}"
    for (d, c), v in solution.u_drone.items():
        if v == 1:
            mask[c] = f"drone_{d}"
    return mask


def _empty_solution_clone(solution: SolutionData) -> SolutionData:
    return SolutionData(
        instance_name=solution.instance_name,
        status=solution.status,
        objective=solution.objective,
        components=dict(solution.components),
        run_time_seconds=solution.run_time_seconds,
        x_truck=dict(solution.x_truck),
        x_drone=dict(solution.x_drone),
        u_truck=dict(solution.u_truck),
        u_drone=dict(solution.u_drone),
        z1=dict(solution.z1),
        z2=dict(solution.z2),
        u_sync=dict(solution.u_sync),
        y_loaded=dict(solution.y_loaded),
        a_truck=dict(solution.a_truck),
        l_truck=dict(solution.l_truck),
        a_drone=dict(solution.a_drone),
        l_drone=dict(solution.l_drone),
        waiting_truck=dict(solution.waiting_truck),
        waiting_drone=dict(solution.waiting_drone),
        load_truck=dict(solution.load_truck),
        battery_drone=dict(solution.battery_drone),
        tardiness=dict(solution.tardiness),
        sync_events=list(solution.sync_events),
        truck_routes={k: v.copy() for k, v in solution.truck_routes.items()},
        drone_routes={k: v.copy() for k, v in solution.drone_routes.items()},
        seed=solution.seed,
        tags=solution.tags,
    )


def _sync_route_assignments(solution: SolutionData, instance: InstanceData) -> None:
    """Keep arc/assignment views consistent with the current explicit routes."""
    solution.x_truck = {}
    solution.x_drone = {}
    solution.u_sync = {}

    prior_drone_assignments = {
        key for key, value in solution.u_drone.items() if value >= 0.5
    }
    had_explicit_drone_assignments = bool(prior_drone_assignments)

    # Reset assignments and rebuild from routes.
    for c in instance.customer_ids:
        for t in instance.truck_ids():
            solution.u_truck[(t, c)] = 0
        for d in instance.drone_ids():
            solution.u_drone[(d, c)] = 0

    for t, route in solution.truck_routes.items():
        rr = list(route)
        if not rr:
            rr = [0, 0]
        if rr[0] != 0:
            rr = [0] + rr
        if rr[-1] != 0:
            rr = rr + [0]
        solution.truck_routes[t] = rr
        for idx in range(len(rr) - 1):
            i = int(rr[idx])
            j = int(rr[idx + 1])
            solution.x_truck[(i, j, t)] = 1
            if j != 0:
                solution.u_truck[(t, j)] = 1

    for d, route in solution.drone_routes.items():
        rr = list(route)
        if not rr:
            rr = [0, 0]
        if rr[0] != 0:
            rr = [0] + rr
        if rr[-1] != 0:
            rr = rr + [0]
        solution.drone_routes[d] = rr
        for idx in range(len(rr) - 1):
            i = int(rr[idx])
            j = int(rr[idx + 1])
            solution.x_drone[(i, j, d)] = 1
            if j != 0 and (
                had_explicit_drone_assignments
                and (d, j) in prior_drone_assignments
                or not had_explicit_drone_assignments
            ):
                solution.u_drone[(d, j)] = 1

    truck_nodes = {
        truck_id: {node for node in route if node != 0}
        for truck_id, route in solution.truck_routes.items()
    }
    for drone_id, route in solution.drone_routes.items():
        drone_nodes = {node for node in route if node != 0}
        for truck_id, nodes in truck_nodes.items():
            for node in drone_nodes & nodes:
                solution.u_sync[(node, drone_id, truck_id)] = 1


def _paired_trip_feasible(
    instance: InstanceData,
    drone: int,
    launch_node: int,
    customer: int,
    recovery_node: int,
    coords: Dict[int, Tuple[float, float]],
    battery_slack_ratio: float = 0.0,
) -> bool:
    drone_obj = instance.drones[drone - 1]
    lc = _travel_time(euclidean_distance(coords[launch_node], coords[customer]), drone_obj.speed_kmph)
    cr = _travel_time(euclidean_distance(coords[customer], coords[recovery_node]), drone_obj.speed_kmph)
    if lc == math.inf or cr == math.inf:
        return False
    burn = drone_obj.energy_per_min_when_loaded if drone_obj.energy_per_min_when_loaded > 0 else 20.0
    energy = (lc + cr) * burn
    if energy > drone_obj.max_battery_wh + 1e-9:
        return False
    residual = drone_obj.max_battery_wh - energy
    return residual >= max(0.0, battery_slack_ratio) * drone_obj.max_battery_wh - 1e-9


def _drone_route_energy(
    instance: InstanceData,
    drone: int,
    route: List[int],
    coords: Dict[int, Tuple[float, float]],
    *,
    service_nodes: set[int] | None = None,
    z1: Dict[Tuple[int, int, int], int] | None = None,
) -> float:
    drone_obj = instance.drones[drone - 1]
    service_nodes = set(service_nodes or ())
    z1 = z1 or {}
    first_service = next((idx for idx, node in enumerate(route) if node in service_nodes), None)
    first_reload = next(
        (
            idx for idx, node in enumerate(route)
            if any(value > 0.5 for (event_node, event_drone, _truck), value in z1.items()
                   if event_node == node and event_drone == drone)
        ),
        None,
    )
    loaded = first_service is None or first_reload is None or first_service < first_reload
    consumed = 0.0
    for idx in range(len(route) - 1):
        frm = route[idx]
        to = route[idx + 1]
        arc_time = _travel_time(euclidean_distance(coords[frm], coords[to]), drone_obj.speed_kmph)
        burn = drone_obj.energy_per_min_when_loaded if loaded else drone_obj.energy_per_min_when_empty
        if burn <= 0:
            burn = 20.0 if loaded else 15.0
        consumed += burn * arc_time
        if to in service_nodes:
            loaded = False
        if any(
            value > 0.5
            for (event_node, event_drone, _truck), value in z1.items()
            if event_node == to and event_drone == drone
        ):
            loaded = True
    return consumed


def _route_duration_minutes(
    instance: InstanceData,
    route: List[int],
    *,
    coords: Dict[int, Tuple[float, float]],
    speed_kmph: float,
    metric: str,
) -> float:
    if not route or len(route) < 2:
        return 0.0
    t_minutes = 0.0
    for idx in range(len(route) - 1):
        frm = route[idx]
        to = route[idx + 1]
        if idx > 0:
            prev = route[idx - 1]
            if prev != 0:
                t_minutes += instance.customer_map()[prev].service_time_min
        if metric == "manhattan":
            dist = manhattan_distance(coords[frm], coords[to])
        else:
            dist = euclidean_distance(coords[frm], coords[to])
        t_minutes += _travel_time(dist, speed_kmph)
    return float(t_minutes)


def _quick_local_feasible(
    instance: InstanceData,
    candidate: SolutionData,
    *,
    moved_customer: int,
    drone: int,
    coords: Dict[int, Tuple[float, float]],
    source_truck: int | None = None,
    launch_node: int | None = None,
    recovery_node: int | None = None,
    launch_truck: int | None = None,
    recovery_truck: int | None = None,
    battery_slack_ratio: float = 0.0,
) -> bool:
    if moved_customer <= 0 or drone <= 0 or drone > instance.num_drones:
        return False

    # Local assignment consistency for the moved customer.
    served_by_truck = sum(int(candidate.u_truck.get((t, moved_customer), 0) == 1) for t in instance.truck_ids())
    served_by_drone = sum(int(candidate.u_drone.get((d, moved_customer), 0) == 1) for d in instance.drone_ids())
    if served_by_truck + served_by_drone != 1:
        return False
    if candidate.u_drone.get((drone, moved_customer), 0) != 1:
        return False

    if source_truck is not None and source_truck > 0:
        truck_route = candidate.truck_routes.get(source_truck, [0, 0])
        if not truck_route or truck_route[0] != 0 or truck_route[-1] != 0:
            return False
        truck_nodes = [n for n in truck_route if n != 0]
        if len(truck_nodes) != len(set(truck_nodes)):
            return False
        if moved_customer in truck_nodes:
            return False
        if launch_truck == source_truck and launch_node not in (None, 0) and launch_node not in truck_route:
            return False
        if recovery_truck == source_truck and recovery_node not in (None, 0) and recovery_node not in truck_route:
            return False
        if (
            launch_truck == source_truck
            and recovery_truck == source_truck
            and launch_node not in (None, 0)
            and recovery_node not in (None, 0)
        ):
            try:
                if truck_route.index(launch_node) >= truck_route.index(recovery_node):
                    return False
            except ValueError:
                return False
        truck_minutes = _route_duration_minutes(
            instance,
            truck_route,
            coords=coords,
            speed_kmph=instance.trucks[source_truck - 1].speed_kmph,
            metric="manhattan",
        )
        if truck_minutes > instance.constants.max_shift_minutes + 1e-9:
            return False

    if launch_truck is not None and launch_node not in (None, 0):
        if launch_node not in candidate.truck_routes.get(launch_truck, [0, 0]):
            return False
    if recovery_truck is not None and recovery_node not in (None, 0):
        if recovery_node not in candidate.truck_routes.get(recovery_truck, [0, 0]):
            return False

    drone_route = candidate.drone_routes.get(drone, [0, 0])
    if not drone_route or drone_route[0] != 0 or drone_route[-1] != 0:
        return False
    drone_nodes = [n for n in drone_route if n != 0]
    if len(drone_nodes) != len(set(drone_nodes)):
        return False
    if moved_customer not in drone_nodes:
        return False
    if launch_node not in (None, 0) and launch_node not in drone_nodes:
        return False
    if recovery_node not in (None, 0) and recovery_node not in drone_nodes:
        return False

    drone_obj = instance.drones[drone - 1]
    drone_minutes = _route_duration_minutes(
        instance,
        drone_route,
        coords=coords,
        speed_kmph=drone_obj.speed_kmph,
        metric="euclidean",
    )
    if drone_minutes > instance.constants.max_shift_minutes + 1e-9:
        return False

    energy = _drone_route_energy(
        instance,
        drone,
        drone_route,
        coords,
        service_nodes={
            node for (drone_id, node), value in candidate.u_drone.items()
            if drone_id == drone and value >= 0.5
        },
        z1=candidate.z1,
    )
    if energy > drone_obj.max_battery_wh + 1e-9:
        return False
    residual_ratio = (drone_obj.max_battery_wh - energy) / max(drone_obj.max_battery_wh, 1e-9)
    if residual_ratio + 1e-9 < max(0.0, battery_slack_ratio):
        return False

    return True


def _select_best_strict_candidate(
    instance: InstanceData,
    ranked_candidates: List[Tuple[float, SolutionData, Tuple[int, int, int]]],
    *,
    strict_check_cap: int,
    stats: Dict[str, int] | None = None,
    deadline: float | None = None,
    preserve_best_on_deadline: bool = False,
) -> SolutionData | None:
    if not ranked_candidates:
        return None
    ranked = sorted(ranked_candidates, key=lambda x: x[0])
    max_checks = len(ranked) if strict_check_cap <= 0 else min(len(ranked), int(strict_check_cap))
    idx = 0
    while idx < max_checks:
        if deadline is not None and time.monotonic() >= deadline:
            if not preserve_best_on_deadline:
                return None
            deadline = None
            if stats is not None:
                stats["displacement_deadline_terminated"] = 1
        _, candidate, _ = ranked[idx]
        _sync_route_assignments(candidate, instance)
        if stats is not None:
            stats["strict_feasibility_checks"] += 1
        if is_feasible(instance, candidate):
            _add_stat(stats, "displacement_candidates_strict_feasible")
            candidate.components = dict(candidate.components)
            candidate.components["_strict_feasibility_checked"] = 1.0
            return candidate
        if stats is not None:
            stats["infeasible_candidate_rejections"] += 1
        idx += 1
    return None


def _add_stat(stats: Dict[str, int] | None, key: str, value: int = 1) -> None:
    if stats is not None:
        stats[key] = stats.get(key, 0) + int(value)


class _CandidateReservoir:
    def __init__(self, cap: int, rng: np.random.Generator | None) -> None:
        self.cap = max(0, int(cap))
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self.descriptors: list[tuple] = []
        self.generated = 0

    def add(self, descriptor: tuple) -> None:
        self.generated += 1
        if self.cap == 0 or len(self.descriptors) < self.cap:
            self.descriptors.append(descriptor)
            return
        replacement_index = int(self.rng.integers(self.generated))
        if replacement_index < self.cap:
            self.descriptors[replacement_index] = descriptor


def move_to_drone(
    instance: InstanceData,
    solution: SolutionData,
    customer: int,
    drone: int,
    *,
    launch_node: int | None = None,
    recovery_node: int | None = None,
    source_truck: int | None = None,
    launch_truck: int | None = None,
    recovery_truck: int | None = None,
    insertion_index: int | None = None,
    enforce_battery: bool = True,
    battery_slack_ratio: float = 0.0,
) -> SolutionData | None:
    """Generate candidate where one customer is moved from truck to a drone route."""
    service = _service_mask(solution)
    src = None
    if service.get(customer, "").startswith("truck_"):
        src = int(service[customer].split("_")[1])
    if source_truck is not None:
        src = source_truck
    if src is None or src <= 0 or src > instance.num_trucks:
        return None
    if customer <= 0 or drone <= 0 or drone > instance.num_drones:
        return None

    ci = instance.customer_map()[customer]
    if ci.demand_kg > instance.drones[drone - 1].capacity_kg:
        return None

    candidate = _empty_solution_clone(solution)

    launch_truck = int(launch_truck if launch_truck is not None else source_truck or src)
    recovery_truck = int(recovery_truck if recovery_truck is not None else source_truck or src)
    has_event_nodes = launch_node is not None or recovery_node is not None
    if has_event_nodes and (launch_node is None or recovery_node is None):
        return None
    if has_event_nodes and (
        launch_truck not in instance.truck_ids() or recovery_truck not in instance.truck_ids()
    ):
        return None

    if src in candidate.truck_routes and customer in candidate.truck_routes[src]:
        candidate.truck_routes[src] = [j for j in candidate.truck_routes[src] if j != customer]
        if len(candidate.truck_routes[src]) == 1:
            candidate.truck_routes[src] = [0, 0]
        candidate.u_truck[(src, customer)] = 0

    for d in instance.drone_ids():
        candidate.u_drone[(d, customer)] = 1 if d == drone else 0

    route = candidate.drone_routes.get(drone, [0, 0])
    if not route:
        route = [0, 0]
    if route[0] != 0:
        route = [0] + route
    if route[-1] != 0:
        route = route + [0]
    existing_non_depot = [node for node in route if node != 0]
    if customer in route:
        return None
    launch = int(launch_node or 0)
    recovery = int(recovery_node or 0)
    if existing_non_depot and launch == 0:
        # A later service on the same drone requires a new loaded parcel;
        # depot return alone is not a parcel-reload event in this model.
        return None
    if has_event_nodes:
        event_nodes = [node for node in (launch, recovery) if node != 0]
        if len(event_nodes) != len(set(event_nodes)) or any(node in existing_non_depot for node in event_nodes):
            return None
        route = (
            route[:-1]
            + ([launch] if launch != 0 else [])
            + [customer]
            + ([recovery] if recovery != 0 else [])
            + [0]
        )
    else:
        # A drone route is one loaded sortie unless a non-depot reload event
        # explicitly starts a later loaded segment.
        if existing_non_depot:
            return None
        if insertion_index is None:
            route.insert(len(route) - 1, customer)
        else:
            insert_at = max(1, min(int(insertion_index), len(route) - 1))
            route.insert(insert_at, customer)

    if len([node for node in route if node != 0]) != len(set(node for node in route if node != 0)):
        return None

    if has_event_nodes:
        if launch_node not in candidate.truck_routes.get(launch_truck, [0, 0]) and launch_node != 0:
            return None
        if recovery_node not in candidate.truck_routes.get(recovery_truck, [0, 0]) and recovery_node != 0:
            return None
        if launch_node != 0 and launch_node not in route:
            return None
        if recovery_node != 0 and recovery_node not in route:
            return None
        if launch_truck == recovery_truck and launch_node != 0 and recovery_node != 0:
            truck_route = candidate.truck_routes.get(launch_truck, [0, 0])
            if truck_route.index(launch_node) >= truck_route.index(recovery_node):
                return None
        if launch_node != 0:
            candidate.u_sync[(int(launch_node), drone, launch_truck)] = 1
            candidate.z1[(int(launch_node), drone, launch_truck)] = 1
        if recovery_node != 0:
            candidate.u_sync[(int(recovery_node), drone, recovery_truck)] = 1
            candidate.z2[(int(recovery_node), drone, recovery_truck)] = 1

    service_nodes = {
        node for (drone_id, node), value in candidate.u_drone.items()
        if drone_id == drone and value >= 0.5
    }
    if enforce_battery:
        coords = instance.coordinate_map()
        total_energy = _drone_route_energy(
            instance,
            drone,
            route,
            coords,
            service_nodes=service_nodes,
            z1=candidate.z1,
        )
        max_battery = instance.drones[drone - 1].max_battery_wh
        if total_energy > max_battery + 1e-9:
            return None
        residual_ratio = (max_battery - total_energy) / max(max_battery, 1e-9)
        if residual_ratio + 1e-9 < max(0.0, battery_slack_ratio):
            return None
    candidate.drone_routes[drone] = route
    candidate.x_drone = {}

    return candidate


def _m5_sortie_options(
    instance: InstanceData,
    solution: SolutionData,
    customer: int,
    source_truck: int,
) -> list[tuple[int, int, int, int]]:
    """Enumerate M5 launch/recovery conventions for one truck customer.

    The launch truck is the truck whose backbone currently serves the customer.
    Recovery may be at the depot or at any node visited by any truck; the
    recovery truck is retained in the event record so cross-truck recovery is
    explicit rather than inferred from a route difference.
    """
    source_route = solution.truck_routes.get(source_truck, [0, 0])
    try:
        customer_index = source_route.index(customer)
    except ValueError:
        return [(0, 0, source_truck, source_truck)]
    launch_nodes = [0] + [node for node in source_route[:customer_index] if node != 0]
    recovery_options = [(0, source_truck)]
    for truck_id in instance.truck_ids():
        route = solution.truck_routes.get(truck_id, [0, 0])
        recovery_options.extend(
            (node, truck_id)
            for node in route
            if node != 0 and node != customer
        )

    options: list[tuple[int, int, int, int]] = []
    seen: set[tuple[int, int, int, int]] = set()
    for launch_node in launch_nodes:
        for recovery_node, recovery_truck in recovery_options:
            if launch_node == customer or recovery_node == customer:
                continue
            if launch_node != 0 and recovery_node == launch_node:
                continue
            if launch_node != 0 and recovery_node != 0:
                if recovery_truck == source_truck:
                    route = solution.truck_routes.get(source_truck, [0, 0])
                    if launch_node not in route or recovery_node not in route:
                        continue
                    if route.index(launch_node) >= route.index(recovery_node):
                        continue
            option = (launch_node, recovery_node, source_truck, recovery_truck)
            if option not in seen:
                seen.add(option)
                options.append(option)
    return options


def _extract_best_displacement(
    instance: InstanceData,
    solution: SolutionData,
    *,
    coords: Dict[int, Tuple[float, float]],
    eligible_customers: set[int],
    battery_aware_screening: bool,
    battery_slack_ratio: float,
    strict_check_cap: int,
    max_candidates: int = 0,
    rng: np.random.Generator | None = None,
    tabu_moves: set[Tuple[int, int, int]] | None = None,
    stats: Dict[str, int] | None = None,
    deadline: float | None = None,
) -> SolutionData | None:
    ranked_candidates: List[Tuple[float, SolutionData, Tuple[int, int, int]]] = []
    reservoir = _CandidateReservoir(max_candidates, rng)
    deadline_reached = False

    for customer in range(1, instance.num_customers + 1):
        if deadline is not None and time.monotonic() >= deadline:
            deadline_reached = True
            break
        _add_stat(stats, "displacement_customers_scanned")
        if customer not in eligible_customers:
            continue
        truck_served = any(solution.u_truck.get((t, customer), 0) == 1 for t in instance.truck_ids())
        if not truck_served:
            continue
        for d in range(1, instance.num_drones + 1):
            if deadline is not None and time.monotonic() >= deadline:
                deadline_reached = True
                break
            source_truck = next((t for t in instance.truck_ids() if solution.u_truck.get((t, customer), 0) == 1), 0)
            move_key = (customer, d, source_truck)
            if tabu_moves is not None and move_key in tabu_moves:
                continue
            sortie_options = _m5_sortie_options(instance, solution, customer, source_truck)
            _add_stat(stats, "displacement_sortie_options_generated", len(sortie_options))
            for launch_node, recovery_node, launch_truck, recovery_truck in sortie_options:
                reservoir.add(
                    (customer, d, source_truck, launch_node, recovery_node, launch_truck, recovery_truck, move_key)
                )
                _add_stat(stats, "displacement_candidates_generated")
        if deadline_reached:
            break

    selected = reservoir.descriptors
    _add_stat(stats, "displacement_candidates_retained", len(selected))
    for customer, drone, source_truck, launch_node, recovery_node, launch_truck, recovery_truck, move_key in selected:
        if deadline is not None and time.monotonic() >= deadline:
            deadline_reached = True
            break
        _add_stat(stats, "displacement_candidates_screened")
        candidate = move_to_drone(
            instance,
            solution,
            customer,
            drone,
            launch_node=launch_node,
            recovery_node=recovery_node,
            source_truck=source_truck,
            launch_truck=launch_truck,
            recovery_truck=recovery_truck,
            enforce_battery=battery_aware_screening,
            battery_slack_ratio=battery_slack_ratio,
        )
        if candidate is None:
            if battery_aware_screening and stats is not None:
                stats["battery_infeasible_rejections"] += 1
            continue
        if not _quick_local_feasible(
            instance,
            candidate,
            moved_customer=customer,
            drone=drone,
            coords=coords,
            source_truck=source_truck if source_truck > 0 else None,
            launch_node=launch_node,
            recovery_node=recovery_node,
            launch_truck=launch_truck,
            recovery_truck=recovery_truck,
            battery_slack_ratio=battery_slack_ratio,
        ):
            if stats is not None:
                stats["local_screen_rejections"] += 1
            continue
        _add_stat(stats, "displacement_candidates_fully_evaluated")
        _add_stat(stats, "candidate_reassignments_evaluated")
        obj, comp = evaluate_solution(instance, candidate)
        candidate.objective = obj
        candidate.components = comp
        ranked_candidates.append((obj, candidate, move_key))
    if deadline_reached and stats is not None:
        stats["displacement_deadline_terminated"] = 1
    return _select_best_strict_candidate(
        instance,
        ranked_candidates,
        strict_check_cap=strict_check_cap,
        stats=stats,
        deadline=deadline,
        preserve_best_on_deadline=True,
    )


def _extract_best_paired_displacement(
    instance: InstanceData,
    solution: SolutionData,
    drone_to_truck: Dict[int, int],
    coords: Dict[int, Tuple[float, float]],
    *,
    eligible_customers: set[int],
    battery_aware_screening: bool,
    battery_slack_ratio: float,
    strict_check_cap: int,
    max_candidates: int = 0,
    rng: np.random.Generator | None = None,
    tabu_moves: set[Tuple[int, int, int]] | None = None,
    stats: Dict[str, int] | None = None,
    deadline: float | None = None,
) -> SolutionData | None:
    ranked_candidates: List[Tuple[float, SolutionData, Tuple[int, int, int]]] = []
    reservoir = _CandidateReservoir(max_candidates, rng)
    descriptor_counts: Dict[Tuple[int, int, int], int] = {}
    deadline_reached = False

    for drone, truck in drone_to_truck.items():
        if deadline is not None and time.monotonic() >= deadline:
            deadline_reached = True
            break
        route = solution.truck_routes.get(truck, [0, 0])
        if len(route) < 3:
            continue

        for customer_idx in range(1, len(route) - 1):
            if deadline is not None and time.monotonic() >= deadline:
                deadline_reached = True
                break
            customer = route[customer_idx]
            if customer == 0:
                continue
            if customer not in eligible_customers:
                continue
            if solution.u_truck.get((truck, customer), 0) != 1:
                continue
            _add_stat(stats, "displacement_customers_scanned")
            move_key = (customer, drone, truck)
            if tabu_moves is not None and move_key in tabu_moves:
                continue
            for launch_idx in range(0, customer_idx):
                if deadline is not None and time.monotonic() >= deadline:
                    deadline_reached = True
                    break
                launch_node = route[launch_idx]
                for recovery_idx in range(customer_idx + 1, len(route)):
                    if deadline is not None and time.monotonic() >= deadline:
                        deadline_reached = True
                        break
                    recovery_node = route[recovery_idx]
                    if recovery_node == launch_node or recovery_node == customer:
                        continue
                    reservoir.add((customer, drone, truck, launch_node, recovery_node, move_key))
                    descriptor_counts[move_key] = descriptor_counts.get(move_key, 0) + 1
                    _add_stat(stats, "displacement_candidates_generated")
                if deadline_reached:
                    break
            if deadline_reached:
                break
        if deadline_reached:
            break

    _add_stat(stats, "displacement_sortie_options_generated", reservoir.generated)
    selected = reservoir.descriptors
    _add_stat(stats, "displacement_candidates_retained", len(selected))
    selected_counts: Dict[Tuple[int, int, int], int] = {}
    found_screened_candidate: set[Tuple[int, int, int]] = set()
    for customer, drone, truck, launch_node, recovery_node, move_key in selected:
        if deadline is not None and time.monotonic() >= deadline:
            deadline_reached = True
            break
        _add_stat(stats, "displacement_candidates_screened")
        selected_counts[move_key] = selected_counts.get(move_key, 0) + 1
        if battery_aware_screening and not _paired_trip_feasible(
            instance,
            drone,
            launch_node,
            customer,
            recovery_node,
            coords,
            battery_slack_ratio=battery_slack_ratio,
        ):
            if stats is not None:
                stats["battery_infeasible_rejections"] += 1
                stats["battery_slack_rejections"] += 1
            continue
        candidate = move_to_drone(
            instance,
            solution,
            customer,
            drone,
            launch_node=launch_node,
            recovery_node=recovery_node,
            source_truck=truck,
            enforce_battery=battery_aware_screening,
            battery_slack_ratio=battery_slack_ratio,
        )
        if candidate is None:
            if battery_aware_screening and stats is not None:
                stats["battery_infeasible_rejections"] += 1
            continue
        if not _quick_local_feasible(
            instance,
            candidate,
            moved_customer=customer,
            drone=drone,
            coords=coords,
            source_truck=truck,
            launch_node=launch_node,
            recovery_node=recovery_node,
            battery_slack_ratio=battery_slack_ratio,
        ):
            if stats is not None:
                stats["local_screen_rejections"] += 1
            continue
        found_screened_candidate.add(move_key)
        _add_stat(stats, "displacement_candidates_fully_evaluated")
        _add_stat(stats, "candidate_reassignments_evaluated")
        obj, comp = evaluate_solution(instance, candidate)
        candidate.objective = obj
        candidate.components = comp
        ranked_candidates.append((obj, candidate, move_key))

    if tabu_moves is not None and not deadline_reached:
        for move_key, option_count in descriptor_counts.items():
            if selected_counts.get(move_key, 0) == option_count and move_key not in found_screened_candidate:
                tabu_moves.add(move_key)
    if deadline_reached and stats is not None:
        stats["displacement_deadline_terminated"] = 1
    return _select_best_strict_candidate(
        instance,
        ranked_candidates,
        strict_check_cap=strict_check_cap,
        stats=stats,
        deadline=deadline,
        preserve_best_on_deadline=True,
    )


def _extract_best_no_unpairing_displacement(
    instance: InstanceData,
    solution: SolutionData,
    coords: Dict[int, Tuple[float, float]],
    *,
    eligible_customers: set[int],
    battery_aware_screening: bool,
    battery_slack_ratio: float,
    strict_check_cap: int,
    max_candidates: int = 0,
    rng: np.random.Generator | None = None,
    tabu_moves: set[Tuple[int, int, int]] | None = None,
    stats: Dict[str, int] | None = None,
    deadline: float | None = None,
) -> SolutionData | None:
    """Best displacement with cross-truck recovery disabled.

    A drone may launch from the truck serving the customer and recover either:
    1) at a later node on that same truck route, or
    2) at the depot.
    """
    ranked_candidates: List[Tuple[float, SolutionData, Tuple[int, int, int]]] = []
    reservoir = _CandidateReservoir(max_candidates, rng)
    descriptor_counts: Dict[Tuple[int, int, int], int] = {}
    deadline_reached = False

    for truck in instance.truck_ids():
        if deadline is not None and time.monotonic() >= deadline:
            deadline_reached = True
            break
        route = solution.truck_routes.get(truck, [0, 0])
        if len(route) < 3:
            continue

        for customer_idx in range(1, len(route) - 1):
            if deadline is not None and time.monotonic() >= deadline:
                deadline_reached = True
                break
            customer = route[customer_idx]
            if customer == 0:
                continue
            if customer not in eligible_customers:
                continue
            if solution.u_truck.get((truck, customer), 0) != 1:
                continue
            _add_stat(stats, "displacement_customers_scanned")

            for drone in instance.drone_ids():
                if deadline is not None and time.monotonic() >= deadline:
                    deadline_reached = True
                    break
                move_key = (customer, drone, truck)
                if tabu_moves is not None and move_key in tabu_moves:
                    continue

                for launch_idx in range(0, customer_idx):
                    if deadline is not None and time.monotonic() >= deadline:
                        deadline_reached = True
                        break
                    launch_node = route[launch_idx]
                    recovery_candidates = [0] + [route[idx] for idx in range(customer_idx + 1, len(route))]
                    seen_recovery_nodes: set[int] = set()

                    for recovery_node in recovery_candidates:
                        if deadline is not None and time.monotonic() >= deadline:
                            deadline_reached = True
                            break
                        if recovery_node in seen_recovery_nodes:
                            continue
                        seen_recovery_nodes.add(recovery_node)

                        if recovery_node == customer:
                            continue

                        reservoir.add((customer, drone, truck, launch_node, recovery_node, move_key))
                        descriptor_counts[move_key] = descriptor_counts.get(move_key, 0) + 1
                        _add_stat(stats, "displacement_candidates_generated")

                    if deadline_reached:
                        break
                if deadline_reached:
                    break
            if deadline_reached:
                break
        if deadline_reached:
            break

    _add_stat(stats, "displacement_sortie_options_generated", reservoir.generated)
    selected = reservoir.descriptors
    _add_stat(stats, "displacement_candidates_retained", len(selected))
    selected_counts: Dict[Tuple[int, int, int], int] = {}
    found_screened_candidate: set[Tuple[int, int, int]] = set()
    for customer, drone, truck, launch_node, recovery_node, move_key in selected:
        if deadline is not None and time.monotonic() >= deadline:
            deadline_reached = True
            break
        _add_stat(stats, "displacement_candidates_screened")
        selected_counts[move_key] = selected_counts.get(move_key, 0) + 1
        if battery_aware_screening and not _paired_trip_feasible(
            instance,
            drone,
            launch_node,
            customer,
            recovery_node,
            coords,
            battery_slack_ratio=battery_slack_ratio,
        ):
            if stats is not None:
                stats["battery_infeasible_rejections"] += 1
                stats["battery_slack_rejections"] += 1
            continue
        candidate = move_to_drone(
            instance,
            solution,
            customer,
            drone,
            launch_node=launch_node,
            recovery_node=recovery_node,
            source_truck=truck,
            enforce_battery=battery_aware_screening,
            battery_slack_ratio=battery_slack_ratio,
        )
        if candidate is None:
            if battery_aware_screening and stats is not None:
                stats["battery_infeasible_rejections"] += 1
            continue
        if not _quick_local_feasible(
            instance,
            candidate,
            moved_customer=customer,
            drone=drone,
            coords=coords,
            source_truck=truck,
            launch_node=launch_node,
            recovery_node=recovery_node,
            battery_slack_ratio=battery_slack_ratio,
        ):
            if stats is not None:
                stats["local_screen_rejections"] += 1
            continue
        found_screened_candidate.add(move_key)
        _add_stat(stats, "displacement_candidates_fully_evaluated")
        _add_stat(stats, "candidate_reassignments_evaluated")
        obj, comp = evaluate_solution(instance, candidate)
        candidate.objective = obj
        candidate.components = comp
        ranked_candidates.append((obj, candidate, move_key))

    if tabu_moves is not None and not deadline_reached:
        for move_key, option_count in descriptor_counts.items():
            if selected_counts.get(move_key, 0) == option_count and move_key not in found_screened_candidate:
                tabu_moves.add(move_key)

    if deadline_reached and stats is not None:
        stats["displacement_deadline_terminated"] = 1
    return _select_best_strict_candidate(
        instance,
        ranked_candidates,
        strict_check_cap=strict_check_cap,
        stats=stats,
        deadline=deadline,
        preserve_best_on_deadline=True,
    )


def _apply_initial_paired_sorties(
    instance: InstanceData,
    solution: SolutionData,
    drone_to_truck: Dict[int, int],
    max_pairs: int,
    *,
    eligible_customers: set[int],
    battery_aware_screening: bool,
    battery_slack_ratio: float,
    strict_check_cap: int,
    max_candidates: int = 0,
    rng: np.random.Generator | None = None,
    tabu_moves: set[Tuple[int, int, int]] | None = None,
    stats: Dict[str, int] | None = None,
) -> SolutionData:
    if max_pairs <= 0:
        return solution
    coords = instance.coordinate_map()
    current = solution
    for _ in range(max_pairs):
        candidate = _extract_best_paired_displacement(
            instance,
            current,
            drone_to_truck,
            coords,
            eligible_customers=eligible_customers,
            battery_aware_screening=battery_aware_screening,
            battery_slack_ratio=battery_slack_ratio,
            strict_check_cap=strict_check_cap,
            max_candidates=max_candidates,
            rng=rng,
            tabu_moves=tabu_moves,
            stats=stats,
        )
        if candidate is None:
            break
        current = candidate
    return current


def _tabu_covers_all_candidates(
    instance: InstanceData,
    solution: SolutionData,
    eligible_customers: set[int],
    pairing_map: Dict[int, int],
    tabu_moves: set[Tuple[int, int, int]],
) -> bool:
    if not tabu_moves:
        return False

    candidate_keys: set[Tuple[int, int, int]] = set()
    if pairing_map:
        for drone, truck in pairing_map.items():
            route = solution.truck_routes.get(truck, [])
            for node in route:
                if node == 0 or node not in eligible_customers:
                    continue
                if solution.u_truck.get((truck, node), 0) == 1:
                    candidate_keys.add((node, drone, truck))
    else:
        for customer in eligible_customers:
            source_truck = next((t for t in instance.truck_ids() if solution.u_truck.get((t, customer), 0) == 1), 0)
            if source_truck <= 0:
                continue
            for drone in instance.drone_ids():
                candidate_keys.add((customer, drone, source_truck))

    if not candidate_keys:
        return True
    return candidate_keys.issubset(tabu_moves)


def _move_customer_back_to_truck(
    instance: InstanceData,
    solution: SolutionData,
    customer: int,
    *,
    coords: Dict[int, Tuple[float, float]],
    candidate_positions: list[tuple[int, int]] | None = None,
    stats: Dict[str, int] | None = None,
) -> SolutionData | None:
    """Try restoring one drone-served customer to a truck route with best local insertion."""
    best: SolutionData | None = None
    best_obj = math.inf

    positions = candidate_positions
    if positions is None:
        positions = []
        for truck in instance.truck_ids():
            route = list(solution.truck_routes.get(truck, [0, 0])) or [0, 0]
            if route[0] != 0:
                route.insert(0, 0)
            if route[-1] != 0:
                route.append(0)
            positions.extend((truck, pos) for pos in range(1, len(route)))

    for truck, pos in positions:
        base_route = list(solution.truck_routes.get(truck, [0, 0])) or [0, 0]
        if base_route[0] != 0:
            base_route.insert(0, 0)
        if base_route[-1] != 0:
            base_route.append(0)
        if pos < 1 or pos >= len(base_route):
            continue

        candidate = _empty_solution_clone(solution)
        for d in instance.drone_ids():
            dr = [n for n in candidate.drone_routes.get(d, [0, 0]) if n != customer]
            if len(dr) < 2:
                dr = [0, 0]
            if dr[0] != 0:
                dr.insert(0, 0)
            if dr[-1] != 0:
                dr.append(0)
            candidate.drone_routes[d] = dr
            candidate.u_drone[(d, customer)] = 0

        for t in instance.truck_ids():
            candidate.u_truck[(t, customer)] = 1 if t == truck else 0
        candidate.truck_routes[truck] = base_route[:pos] + [customer] + base_route[pos:]
        candidate.x_truck = {}
        candidate.x_drone = {}
        for key in list(candidate.z1.keys()):
            if key[0] == customer:
                candidate.z1[key] = 0
        for key in list(candidate.z2.keys()):
            if key[0] == customer:
                candidate.z2[key] = 0

        _add_stat(stats, "repair_candidates_screened")
        obj, comp = evaluate_solution(instance, candidate)
        _add_stat(stats, "repair_candidates_fully_evaluated")
        candidate.objective = obj
        candidate.components = comp

        if is_feasible(instance, candidate):
            _add_stat(stats, "repair_candidates_strict_feasible")
            if obj < best_obj:
                best_obj = obj
                best = candidate

    return best


def _repair_infeasible_solution(
    instance: InstanceData,
    solution: SolutionData,
    *,
    max_steps: int,
    max_candidates: int = 0,
    rng: np.random.Generator | None = None,
    stats: Dict[str, int] | None = None,
) -> SolutionData | None:
    """Greedy feasibility-recovery by moving drone-served customers back to trucks."""
    _sync_route_assignments(solution, instance)
    if is_feasible(instance, solution):
        return solution

    coords = instance.coordinate_map()
    current = _empty_solution_clone(solution)
    _sync_route_assignments(current, instance)

    for _ in range(max(1, int(max_steps))):
        drone_customers = [
            c
            for c in instance.customer_ids
            if any(current.u_drone.get((d, c), 0) == 1 for d in instance.drone_ids())
        ]
        if not drone_customers:
            break

        reservoir = _CandidateReservoir(max_candidates, rng)
        for customer in drone_customers:
            for truck in instance.truck_ids():
                route = list(current.truck_routes.get(truck, [0, 0])) or [0, 0]
                if route[0] != 0:
                    route.insert(0, 0)
                if route[-1] != 0:
                    route.append(0)
                for pos in range(1, len(route)):
                    reservoir.add((customer, truck, pos))
                    _add_stat(stats, "repair_candidates_generated")
        selected = reservoir.descriptors
        _add_stat(stats, "repair_candidates_retained", len(selected))
        selected_by_customer: Dict[int, list[tuple[int, int]]] = {}
        for customer, truck, pos in selected:
            selected_by_customer.setdefault(customer, []).append((truck, pos))

        best_step: SolutionData | None = None
        best_step_obj = math.inf
        for customer in drone_customers:
            candidate_positions = selected_by_customer.get(customer, [])
            if not candidate_positions:
                continue
            repaired = _move_customer_back_to_truck(
                instance,
                current,
                customer,
                coords=coords,
                candidate_positions=candidate_positions,
                stats=stats,
            )
            if repaired is None:
                continue
            if repaired.objective < best_step_obj:
                best_step_obj = repaired.objective
                best_step = repaired

        if best_step is None:
            break

        current = best_step
        _sync_route_assignments(current, instance)
        if is_feasible(instance, current):
            return current

    return current if is_feasible(instance, current) else None


@dataclass
class NILSResult:
    solution: SolutionData
    iterations: int
    improved: int


def run_nils(
    instance: InstanceData,
    seed: int,
    max_iter: int = 30,
    max_no_improve: int = 5,
    time_limit: int = 600,
    drone_to_truck: Dict[int, int] | None = None,
    use_paired_initialization: bool = False,
    max_initial_pairings: int = 2,
    enable_local_search: bool = True,
    enable_perturbation: bool = True,
    battery_aware_screening: bool = True,
    battery_slack_ratio: float | None = None,
    no_unpairing_mode: bool = False,
    strict_candidate_check_cap: int = 12,
    max_displacement_candidates: int = 0,
    max_repair_candidates: int = 0,
    full_feasibility_check_interval: int = 6,
    require_feasible_incumbent: bool = True,
    repair_infeasible_candidates: bool = True,
    repair_max_steps: int = 3,
    initialization_mode: str = "greedy",
    initialization_time_limit_seconds: int = 3,
    initial_truck_routes: Dict[int, List[int]] | None = None,
    strict_improving_acceptance: bool = False,
    stagnation_perturb_from_best: bool = False,
    stagnation_perturb_attempts: int = 1,
    metropolis_temperature_scale: float = 1.0,
    risk_rollback: bool = False,
    risk_rollback_reconstruction_max: int = 1,
    risk_rollback_stagnation: int | None = None,
    risk_rollback_size_cap: int = 3,
    risk_rollback_epsilon_z: float = 1e-6,
    risk_rollback_epsilon_h: float = 1e-6,
    risk_rollback_epsilon_tail: float = 1e-6,
    risk_rollback_logs: List[Dict[str, object]] | None = None,
    risk_rollback_move_logs: List[Dict[str, object]] | None = None,
) -> SolutionData:
    """Run the NILS routine.

    If `drone_to_truck` is provided, the algorithm uses fixed-pair neighborhoods where
    each drone can only launch and recover on its assigned truck.
    If 
o_unpairing_mode` is True, cross-truck recovery is disabled and a drone may
    only recover at the depot or on the launch truck route.
    """
    start = time.monotonic()
    deadline = start + max(0.0, float(time_limit))
    rng = np.random.default_rng(seed)
    candidate_rng = np.random.default_rng(int(seed) + 104729)
    rnd = random.Random(seed)
    max_displacement_candidates = int(max_displacement_candidates)
    max_repair_candidates = int(max_repair_candidates)
    if max_displacement_candidates < 0 or max_repair_candidates < 0:
        raise ValueError("NILS candidate caps must be non-negative; use 0 for unlimited")
    if battery_slack_ratio is None:
        battery_slack_ratio = float(instance.metadata.get("battery_slack_ratio", 0.10))
    battery_slack_ratio = float(max(0.0, battery_slack_ratio))
    structurally_admissible, screen_reason = structural_admissibility_screen(
        instance,
        battery_slack_ratio=battery_slack_ratio,
    )
    if not structurally_admissible:
        return SolutionData(
            instance_name=instance.name,
            status="STRUCTURAL-INADMISSIBLE",
            objective=float("nan"),
            components={"structural_screen_passed": 0.0},
            seed=seed,
            tags=(f"structural_screen_reason:{screen_reason}",),
        )
    stagnation_perturb_attempts = max(1, int(stagnation_perturb_attempts))
    metropolis_temperature_scale = max(1e-6, float(metropolis_temperature_scale))
    risk_rollback_reconstruction_max = max(0, int(risk_rollback_reconstruction_max))
    risk_rollback_stagnation = max(1, int(max_no_improve if risk_rollback_stagnation is None else risk_rollback_stagnation))
    risk_logs = risk_rollback_logs if risk_rollback_logs is not None else []
    risk_move_logs = risk_rollback_move_logs if risk_rollback_move_logs is not None else []
    risk_rollback_attempts = 0
    risk_admission_guard = False

    stats = {
        "iterations": 0,
        "improving_moves_accepted": 0,
        "candidate_reassignments_evaluated": 0,
        "battery_infeasible_rejections": 0,
        "battery_slack_rejections": 0,
        "infeasible_candidate_rejections": 0,
        "tabu_reverts": 0,
        "perturbation_moves": 0,
        "local_search_calls": 0,
        "local_screen_rejections": 0,
        "strict_feasibility_checks": 0,
        "displacement_customers_scanned": 0,
        "displacement_sortie_options_generated": 0,
        "displacement_candidates_generated": 0,
        "displacement_candidates_retained": 0,
        "displacement_candidates_screened": 0,
        "displacement_candidates_fully_evaluated": 0,
        "displacement_candidates_strict_feasible": 0,
        "displacement_deadline_terminated": 0,
        "repair_candidates_generated": 0,
        "repair_candidates_retained": 0,
        "repair_candidates_screened": 0,
        "repair_candidates_fully_evaluated": 0,
        "repair_candidates_strict_feasible": 0,
        "repair_attempts": 0,
        "repair_successes": 0,
        "repair_failures": 0,
    }
    initial_backbone_reused = initial_truck_routes is not None

    if initial_truck_routes is None:
        current = build_initial_solution(
            instance,
            seed,
            initializer=initialization_mode,
            ortools_time_limit_seconds=max(1, int(initialization_time_limit_seconds)),
        )
    else:
        normalized_routes = {
            int(truck): [int(node) for node in route]
            for truck, route in initial_truck_routes.items()
        }
        if set(normalized_routes) != set(instance.truck_ids()):
            raise ValueError("initial_truck_routes must contain one route for every truck")
        assigned_customers = [node for route in normalized_routes.values() for node in route if node != 0]
        if sorted(assigned_customers) != sorted(instance.customer_ids):
            raise ValueError("initial_truck_routes must assign every customer exactly once")
        current = _fill_solution_from_routes(instance, normalized_routes)
    current.objective, current.components = evaluate_solution(instance, current)
    _sync_route_assignments(current, instance)
    if require_feasible_incumbent and not is_feasible(instance, current):
        repaired = _repair_infeasible_solution(
            instance,
            current,
            max_steps=max(1, int(repair_max_steps)),
            max_candidates=max_repair_candidates,
            rng=candidate_rng,
            stats=stats,
        )
        if repaired is not None:
            current = repaired
            current.objective, current.components = evaluate_solution(instance, current)
            _sync_route_assignments(current, instance)
    if require_feasible_incumbent and not is_feasible(instance, current):
        fallback_current: SolutionData | None = None
        fallback_attempts = [
            ("ortools", int(seed)),
            ("greedy", int(seed) + 1),
            ("greedy", int(seed) + 2),
        ]
        for init_mode, init_seed in fallback_attempts:
            trial = build_initial_solution(
                instance,
                init_seed,
                initializer=init_mode,
                ortools_time_limit_seconds=max(1, int(initialization_time_limit_seconds)),
            )
            trial.objective, trial.components = evaluate_solution(instance, trial)
            _sync_route_assignments(trial, instance)
            if is_feasible(instance, trial):
                fallback_current = trial
                break
            repaired_trial = _repair_infeasible_solution(
                instance,
                trial,
                max_steps=max(1, int(repair_max_steps)),
                max_candidates=max_repair_candidates,
                rng=candidate_rng,
                stats=stats,
            )
            if repaired_trial is not None:
                repaired_trial.objective, repaired_trial.components = evaluate_solution(instance, repaired_trial)
                _sync_route_assignments(repaired_trial, instance)
                if is_feasible(instance, repaired_trial):
                    fallback_current = repaired_trial
                    break
        if fallback_current is not None:
            current = fallback_current
        else:
            current.status = "CONSTRUCTION-FAILED"
            current.components = dict(current.components)
            current.components["structural_screen_passed"] = 1.0
            current.components["max_displacement_candidates"] = float(max_displacement_candidates)
            current.components["max_repair_candidates"] = float(max_repair_candidates)
            current.components["initial_backbone_reused"] = 1.0 if initial_backbone_reused else 0.0
            for metric in (
                "displacement_candidates_generated",
                "displacement_candidates_retained",
                "displacement_candidates_screened",
                "displacement_candidates_fully_evaluated",
                "displacement_candidates_strict_feasible",
                "repair_candidates_generated",
                "repair_candidates_retained",
                "repair_candidates_screened",
                "repair_candidates_fully_evaluated",
                "repair_candidates_strict_feasible",
            ):
                current.components[metric] = float(stats[metric])
            current.tags = tuple(current.tags) + ("construction_failure_reason:bounded_restarts_and_repairs_exhausted",)
            return current

    pairing_map = _normalize_drone_to_truck(instance, drone_to_truck)
    if pairing_map and len(pairing_map) != instance.num_drones:
        pairing_map = {}
    if pairing_map:
        pairing_map = dict(pairing_map)
    else:
        pairing_map = {}

    eligible_customers = _eligible_customer_ids(instance)
    stats["risk_rollback_attempts"] = 0
    stats["risk_rollback_candidates_evaluated"] = 0
    stats["risk_rollback_accepted"] = 0
    stats["risk_rollback_guard_rejections"] = 0
    tabu_moves: set[Tuple[int, int, int]] = set()

    if use_paired_initialization and pairing_map:
        current = _apply_initial_paired_sorties(
            instance,
            current,
            pairing_map,
            max_pairs=max_initial_pairings,
            eligible_customers=eligible_customers,
            battery_aware_screening=battery_aware_screening,
            battery_slack_ratio=battery_slack_ratio,
            strict_check_cap=strict_candidate_check_cap,
            max_candidates=max_displacement_candidates,
            rng=candidate_rng,
            tabu_moves=tabu_moves,
            stats=stats,
        )
        current.objective, current.components = evaluate_solution(instance, current)

    best = _empty_solution_clone(current)
    best.status = "nils_best"
    no_improve = 0

    def _record_risk_move(before: SolutionData, after: SolutionData, iteration: int) -> None:
        if not risk_rollback:
            return
        before_truck = before.served_by_truck()
        before_drone = before.served_by_drone()
        after_truck = after.served_by_truck()
        after_drone = after.served_by_drone()
        all_customers = (before_truck | before_drone | after_truck | after_drone)
        changed_customers = sorted(
            customer_id for customer_id in all_customers
            if (customer_id in before_drone) != (customer_id in after_drone)
        )
        for customer_id in changed_customers:
            from_mode = "drone" if customer_id in before_drone else "truck"
            to_mode = "drone" if customer_id in after_drone else "truck"
            if from_mode == to_mode:
                continue
            drone_id = next(
                (drone for (drone, customer), value in after.u_drone.items() if customer == customer_id and value >= 0.5),
                next((drone for (drone, customer), value in before.u_drone.items() if customer == customer_id and value >= 0.5), None),
            )
            route = list(after.drone_routes.get(drone_id, [])) if drone_id is not None else []
            launch_node = route[0] if route else ""
            recovery_node = route[-1] if route else ""
            sync_truck_id = ""
            recovery_type = ""
            if drone_id is not None:
                sync_truck_id = next(
                    (
                        truck
                        for _node, drone, truck in active_rendezvous_events(after)
                        if drone == drone_id
                    ),
                    "",
                )
                recovery_type = "depot" if recovery_node == 0 else "truck"
            from .nils_r import compute_priority_metrics

            before_components = before.components or {}
            after_components = after.components or {}
            before_metrics = compute_priority_metrics(instance, before)
            after_metrics = compute_priority_metrics(instance, after)
            battery_slack = float("nan")
            if drone_id is not None and route:
                drone = instance.drones[int(drone_id) - 1]
                energy = 0.0
                for left, right in zip(route, route[1:]):
                    duration = _travel_time(
                        euclidean_distance(instance.coordinate_map()[left], instance.coordinate_map()[right]),
                        drone.speed_kmph,
                    )
                    burn = drone.energy_per_min_when_loaded if right != 0 else drone.energy_per_min_when_empty
                    energy += duration * burn
                battery_value = after.battery_drone.get((drone_id, recovery_node))
                battery_slack = float(battery_value) if battery_value is not None else float(drone.max_battery_wh - energy)
            risk_move_logs.append(
                {
                    "iteration": int(iteration),
                    "move_type": "truck_to_drone" if to_mode == "drone" else "drone_to_truck",
                    "customer_id": int(customer_id),
                    "action": "admit_drone" if to_mode == "drone" else "rollback_to_truck",
                    "from_mode": from_mode,
                    "to_mode": to_mode,
                    "launch_node": launch_node,
                    "recovery_node": recovery_node,
                    "recovery_type": recovery_type,
                    "sync_truck_id": sync_truck_id,
                    "delta_Z": float(after.objective - before.objective),
                    "delta_priority_tardiness_cost": float(
                        after_components.get("tardiness_cost", 0.0) - before_components.get("tardiness_cost", 0.0)
                    ),
                    "delta_high_priority_tardiness": float(
                        after_metrics["high_priority_tardiness"] - before_metrics["high_priority_tardiness"]
                    ),
                    "delta_max_high_priority_tardiness": float(
                        after_metrics["max_high_priority_tardiness"] - before_metrics["max_high_priority_tardiness"]
                    ),
                    "delta_truck_cost": float(
                        after_components.get("truck_cost", 0.0) - before_components.get("truck_cost", 0.0)
                    ),
                    "delta_drone_cost": float(
                        after_components.get("drone_cost", 0.0) - before_components.get("drone_cost", 0.0)
                    ),
                    "delta_sync_wait": float(
                        after_components.get("waiting_cost", 0.0) - before_components.get("waiting_cost", 0.0)
                    ),
                    "battery_slack_after_move": battery_slack,
                    "accepted": True,
                    "rejection_reason": "",
                }
            )

    def _try_risk_rollback() -> bool:
        nonlocal current, best, no_improve, risk_rollback_attempts, risk_admission_guard
        if time.monotonic() >= deadline:
            return False
        if not risk_rollback or risk_rollback_attempts >= risk_rollback_reconstruction_max:
            return False
        from .nils_r import _attempt_priority_risk_rollback, compute_priority_metrics

        metrics = compute_priority_metrics(instance, best)
        trigger_reasons = []
        if metrics["high_priority_tardiness"] > risk_rollback_epsilon_h:
            trigger_reasons.append("high_priority_tardiness")
        if metrics["max_high_priority_tardiness"] > risk_rollback_epsilon_tail:
            trigger_reasons.append("max_high_priority_tardiness")
        if no_improve >= risk_rollback_stagnation:
            trigger_reasons.append("objective_stagnation")
        if not trigger_reasons:
            return False

        result = _attempt_priority_risk_rollback(
            instance,
            current,
            best,
            trigger_reason=";".join(trigger_reasons),
            reconstruction_attempt=risk_rollback_attempts + 1,
            size_cap=risk_rollback_size_cap,
            epsilon_z=risk_rollback_epsilon_z,
            epsilon_h=risk_rollback_epsilon_h,
            epsilon_tail=risk_rollback_epsilon_tail,
            deadline=deadline,
        )
        risk_rollback_attempts += 1
        stats["risk_rollback_attempts"] = risk_rollback_attempts
        stats["risk_rollback_candidates_evaluated"] += result.candidates_evaluated
        risk_logs.append(result.log)
        if not result.accepted or result.solution is None:
            return False

        current = result.solution
        best = _empty_solution_clone(current)
        best.status = "nils_r_best"
        no_improve = 0
        risk_admission_guard = True
        stats["risk_rollback_accepted"] += 1
        return True

    coords = instance.coordinate_map()
    for it in range(1, max_iter + 1):
        if time.monotonic() >= deadline:
            break
        stats["iterations"] = it

        if pairing_map:
            candidate = _extract_best_paired_displacement(
                instance,
                current,
                pairing_map,
                coords,
                eligible_customers=eligible_customers,
                battery_aware_screening=battery_aware_screening,
                battery_slack_ratio=battery_slack_ratio,
                strict_check_cap=strict_candidate_check_cap,
                max_candidates=max_displacement_candidates,
                rng=candidate_rng,
                tabu_moves=tabu_moves,
                stats=stats,
                deadline=deadline,
            )
        elif no_unpairing_mode:
            candidate = _extract_best_no_unpairing_displacement(
                instance,
                current,
                coords,
                eligible_customers=eligible_customers,
                battery_aware_screening=battery_aware_screening,
                battery_slack_ratio=battery_slack_ratio,
                strict_check_cap=strict_candidate_check_cap,
                max_candidates=max_displacement_candidates,
                rng=candidate_rng,
                tabu_moves=tabu_moves,
                stats=stats,
                deadline=deadline,
            )
        else:
            candidate = _extract_best_displacement(
                instance,
                current,
                coords=coords,
                eligible_customers=eligible_customers,
                battery_aware_screening=battery_aware_screening,
                battery_slack_ratio=battery_slack_ratio,
                strict_check_cap=strict_candidate_check_cap,
                max_candidates=max_displacement_candidates,
                rng=candidate_rng,
                tabu_moves=tabu_moves,
                stats=stats,
                deadline=deadline,
            )

        deadline_reached = time.monotonic() >= deadline
        if deadline_reached and candidate is None:
            break

        if candidate is not None and risk_admission_guard:
            from .nils_r import compute_priority_metrics

            before_risk = compute_priority_metrics(instance, current)
            after_risk = compute_priority_metrics(instance, candidate)
            guarded = (
                candidate.objective >= current.objective - risk_rollback_epsilon_z
                or after_risk["high_priority_tardiness"]
                > before_risk["high_priority_tardiness"] + risk_rollback_epsilon_h
                or after_risk["max_high_priority_tardiness"]
                > before_risk["max_high_priority_tardiness"] + risk_rollback_epsilon_tail
            )
            if guarded:
                stats["risk_rollback_guard_rejections"] += 1
                candidate = None
                no_improve += 1
                if _try_risk_rollback():
                    continue
                if no_improve >= max_no_improve:
                    break
                continue

        if candidate is None:
            if deadline_reached:
                break
            if _try_risk_rollback():
                continue
            if enable_perturbation:
                perturb_source = best if stagnation_perturb_from_best else current
                best_perturbed = None
                for _ in range(stagnation_perturb_attempts):
                    trial = relocate_random_customer(perturb_source, instance, rng)
                    stats["perturbation_moves"] += 1
                    if trial is None:
                        continue
                    trial.objective, trial.components = evaluate_solution(instance, trial)
                    _sync_route_assignments(trial, instance)
                    if not is_feasible(instance, trial):
                        continue
                    if best_perturbed is None or trial.objective < best_perturbed.objective:
                        best_perturbed = trial
                candidate = best_perturbed
                if candidate is None:
                    if _tabu_covers_all_candidates(instance, current, eligible_customers, pairing_map, tabu_moves):
                        no_improve += 1
                    continue
            else:
                if not _tabu_covers_all_candidates(instance, current, eligible_customers, pairing_map, tabu_moves):
                    continue
                no_improve += 1
                if no_improve >= max_no_improve:
                    break
                continue
        strict_checked = bool(float(candidate.components.get("_strict_feasibility_checked", 0.0)) >= 0.5)
        if not strict_checked:
            candidate.objective, candidate.components = evaluate_solution(instance, candidate)
        _sync_route_assignments(candidate, instance)

        need_global_check = require_feasible_incumbent or (not strict_checked) or (
            full_feasibility_check_interval > 0 and it % int(full_feasibility_check_interval) == 0
        )
        if deadline_reached and strict_checked:
            need_global_check = False
        if need_global_check:
            stats["strict_feasibility_checks"] += 1
            if not is_feasible(instance, candidate):
                repaired = None
                if repair_infeasible_candidates:
                    stats["repair_attempts"] += 1
                    repaired = _repair_infeasible_solution(
                        instance,
                        candidate,
                        max_steps=max(1, int(repair_max_steps)),
                        max_candidates=max_repair_candidates,
                        rng=candidate_rng,
                        stats=stats,
                    )
                if repaired is not None:
                    stats["repair_successes"] += 1
                    candidate = repaired
                    candidate.objective, candidate.components = evaluate_solution(instance, candidate)
                    _sync_route_assignments(candidate, instance)
                else:
                    stats["repair_failures"] += 1
                    stats["infeasible_candidate_rejections"] += 1
                    stats["tabu_reverts"] += 1
                    no_improve += 1
                    if no_improve >= max_no_improve:
                        break
                    continue
        if enable_perturbation and it % 4 == 0 and not deadline_reached:
            perturbed = relocate_random_customer(candidate, instance, rng)
            stats["perturbation_moves"] += 1
            perturbed.objective, perturbed.components = evaluate_solution(instance, perturbed)
            if perturbed.objective < candidate.objective or rnd.random() < 0.2:
                candidate = perturbed
            _sync_route_assignments(candidate, instance)

        if require_feasible_incumbent and not (deadline_reached and strict_checked):
            stats["strict_feasibility_checks"] += 1
            if not is_feasible(instance, candidate):
                repaired = None
                if repair_infeasible_candidates:
                    stats["repair_attempts"] += 1
                    repaired = _repair_infeasible_solution(
                        instance,
                        candidate,
                        max_steps=max(1, int(repair_max_steps)),
                        max_candidates=max_repair_candidates,
                        rng=candidate_rng,
                        stats=stats,
                    )
                if repaired is not None:
                    stats["repair_successes"] += 1
                    candidate = repaired
                    candidate.objective, candidate.components = evaluate_solution(instance, candidate)
                    _sync_route_assignments(candidate, instance)
                else:
                    stats["repair_failures"] += 1
                    stats["infeasible_candidate_rejections"] += 1
                    stats["tabu_reverts"] += 1
                    no_improve += 1
                    if no_improve >= max_no_improve:
                        break
                    continue

        temp = max(1e-6, float(max_iter - it + 1) * metropolis_temperature_scale)
        accept = candidate.objective <= current.objective + 1e-12
        if not strict_improving_acceptance and not accept:
            accept = metropolis(current.objective, candidate.objective, temp, rnd)
        if accept:
            previous_current = _empty_solution_clone(current)
            if enable_local_search and not deadline_reached:
                candidate = apply_two_opt_to_solution(candidate, instance)
                candidate.objective, candidate.components = evaluate_solution(instance, candidate)
                _sync_route_assignments(candidate, instance)
                stats["local_search_calls"] += 1
            _record_risk_move(previous_current, candidate, it)
            current = candidate
            no_improve = 0
            if candidate.objective < best.objective - 1e-12:
                best = _empty_solution_clone(candidate)
                stats["improving_moves_accepted"] += 1
        else:
            no_improve += 1

        if risk_rollback and no_improve >= risk_rollback_stagnation:
            if _try_risk_rollback():
                continue

        if no_improve >= max_no_improve:
            break

    best.status = "nils_complete"
    best.run_time_seconds = time.monotonic() - start
    _sync_route_assignments(best, instance)
    best.components = dict(best.components)
    best.components["iterations"] = float(stats["iterations"])
    best.components["improving_moves_accepted"] = float(stats["improving_moves_accepted"])
    best.components["candidate_reassignments_evaluated"] = float(stats["candidate_reassignments_evaluated"])
    best.components["battery_infeasible_rejections"] = float(stats["battery_infeasible_rejections"])
    best.components["battery_slack_rejections"] = float(stats["battery_slack_rejections"])
    best.components["infeasible_candidate_rejections"] = float(stats["infeasible_candidate_rejections"])
    best.components["tabu_reverts"] = float(stats["tabu_reverts"])
    best.components["perturbation_moves"] = float(stats["perturbation_moves"])
    best.components["local_search_calls"] = float(stats["local_search_calls"])
    best.components["local_screen_rejections"] = float(stats["local_screen_rejections"])
    best.components["strict_feasibility_checks"] = float(stats["strict_feasibility_checks"])
    best.components["displacement_customers_scanned"] = float(stats["displacement_customers_scanned"])
    best.components["displacement_sortie_options_generated"] = float(
        stats["displacement_sortie_options_generated"]
    )
    for metric in (
        "displacement_candidates_generated",
        "displacement_candidates_retained",
        "displacement_candidates_screened",
        "displacement_candidates_fully_evaluated",
        "displacement_candidates_strict_feasible",
        "repair_candidates_generated",
        "repair_candidates_retained",
        "repair_candidates_screened",
        "repair_candidates_fully_evaluated",
        "repair_candidates_strict_feasible",
    ):
        best.components[metric] = float(stats[metric])
    best.components["max_displacement_candidates"] = float(max_displacement_candidates)
    best.components["max_repair_candidates"] = float(max_repair_candidates)
    best.components["initial_backbone_reused"] = 1.0 if initial_backbone_reused else 0.0
    best.components["displacement_deadline_terminated"] = float(stats["displacement_deadline_terminated"])
    best.components["repair_attempts"] = float(stats["repair_attempts"])
    best.components["repair_successes"] = float(stats["repair_successes"])
    best.components["repair_failures"] = float(stats["repair_failures"])
    best.components["battery_screening_enabled"] = 1.0 if battery_aware_screening else 0.0
    best.components["structural_screen_passed"] = 1.0
    best.components["battery_slack_ratio"] = float(max(0.0, battery_slack_ratio))
    best.components["tabu_size_final"] = float(len(tabu_moves))
    best.components["local_search_enabled"] = 1.0 if enable_local_search else 0.0
    best.components["perturbation_enabled"] = 1.0 if enable_perturbation else 0.0
    best.components["no_unpairing_mode"] = 1.0 if no_unpairing_mode else 0.0
    best.components["fixed_pair_mode"] = 1.0 if pairing_map else 0.0
    best.components["require_feasible_incumbent"] = 1.0 if require_feasible_incumbent else 0.0
    best.components["repair_infeasible_candidates"] = 1.0 if repair_infeasible_candidates else 0.0
    best.components["repair_max_steps"] = float(max(1, int(repair_max_steps)))
    best.components["strict_improving_acceptance"] = 1.0 if strict_improving_acceptance else 0.0
    best.components["stagnation_perturb_from_best"] = 1.0 if stagnation_perturb_from_best else 0.0
    best.components["stagnation_perturb_attempts"] = float(stagnation_perturb_attempts)
    best.components["metropolis_temperature_scale"] = float(metropolis_temperature_scale)
    if risk_rollback:
        best.components["nils_r_enabled"] = 1.0
        best.components["nils_r_reconstruction_attempts"] = float(risk_rollback_attempts)
        best.components["nils_r_reconstruction_accepted"] = float(stats["risk_rollback_accepted"])
        best.components["nils_r_reconstruction_candidates_evaluated"] = float(
            stats["risk_rollback_candidates_evaluated"]
        )
        best.components["nils_r_drone_guard_rejections"] = float(stats["risk_rollback_guard_rejections"])
        best.components["nils_r_reconstruction_max_attempts"] = float(risk_rollback_reconstruction_max)
        best.components["nils_r_reconstruction_size_cap"] = float(risk_rollback_size_cap)
        best.components["nils_r_reconstruction_stagnation"] = float(risk_rollback_stagnation)
        best.__dict__["nils_r_reconstruction_logs"] = list(risk_logs)
        best.__dict__["nils_r_drone_decision_logs"] = list(risk_move_logs)
    return best
