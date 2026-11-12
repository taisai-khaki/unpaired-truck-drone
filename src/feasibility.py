"""Strict solution validator for UTDRP-DP solution objects."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np

from .data_models import InstanceData, SolutionData
from .distance_utils import euclidean_distance, manhattan_distance, travel_time_from_distance
from .drone_state import (
    analyze_drone_states,
    route_intersection_events,
    simulate_drone_states,
    support_state_mismatches,
)
from .timing import (
    active_rendezvous_events,
    compute_synchronization_waiting,
    compute_readiness_times,
)


@dataclass
class ValidationFinding:
    code: str
    severity: str
    message: str
    vehicle: str
    node: int | None = None


def _route_from_arcs(
    start: int,
    arcs: Dict[Tuple[int, int], float],
    tolerance: float = 1e-8,
) -> List[int]:
    outgoing = {}
    incoming_count = {i: 0 for i in range(0, 1_000)}
    for (i, j), v in arcs.items():
        if v >= 0.5:
            outgoing.setdefault(i, []).append(j)
            incoming_count[j] = incoming_count.get(j, 0) + 1
    current = start
    visited = {start}
    route = [start]
    while True:
        next_nodes = outgoing.get(current, [])
        if not next_nodes:
            break
        if len(next_nodes) != 1:
            raise ValueError(f"Non-functional route: node {current} has {len(next_nodes)} outgoing arcs")
        nxt = next_nodes[0]
        route.append(nxt)
        if nxt == start:
            break
        if nxt in visited:
            raise ValueError(f"Cycle detected at node {nxt}")
        visited.add(nxt)
        current = nxt
        if len(route) > 10_000:
            raise ValueError("Route reconstruction timeout")
    return route


def _infer_arrival(route: Sequence[int], distances: np.ndarray, speed_kmph: float, service_times: Dict[int, float]) -> Dict[int, float]:
    arr: Dict[int, float] = {route[0]: 0.0}
    current = route[0]
    time = 0.0
    for nxt in route[1:]:
        time += travel_time_from_distance(distances[current][nxt], speed_kmph)
        time += service_times.get(current, 0.0)
        arr[nxt] = time
        current = nxt
    return arr


def _node_positions(instance: InstanceData) -> Dict[int, Tuple[float, float]]:
    coords: Dict[int, Tuple[float, float]] = {0: (0.0, 0.0)}
    for c in instance.customers:
        coords[c.node_id] = (c.x, c.y)
    return coords


def validate_solution(instance: InstanceData, solution: SolutionData, *, tol: float = 1e-5) -> List[ValidationFinding]:
    """Validate a complete solution; return findings (ERROR/WARN)."""
    findings: List[ValidationFinding] = []
    customers = instance.customer_ids
    nodes = instance.all_nodes()

    served_by_truck = set(solution.served_by_truck())
    served_by_drone = set(solution.served_by_drone())
    if not solution.u_truck:
        for route in solution.truck_routes.values():
            served_by_truck.update(node for node in route if node != 0)
    if not solution.u_drone:
        for route in solution.drone_routes.values():
            served_by_drone.update(node for node in route if node != 0)
    if not served_by_truck and not served_by_drone and not solution.x_truck and not solution.x_drone:
        findings.append(ValidationFinding("EMPTY", "ERROR", "Solution has no routing or assignment data", "global"))

    for c in customers:
        in_truck = c in served_by_truck
        in_drone = c in served_by_drone
        if not (in_truck or in_drone):
            findings.append(ValidationFinding("UNSERVED", "ERROR", f"Customer {c} is not served", "global", c))
        if in_truck and in_drone:
            findings.append(ValidationFinding("DUPLICATE", "ERROR", f"Customer {c} assigned to both truck and drone", "global", c))

    # route reconstruction from arc binaries
    truck_routes: Dict[int, List[int]] = {}
    for t in instance.truck_ids():
        arc_map = {(i, j): 1 for (i, j, tt), v in solution.x_truck.items() if tt == t and v >= 0.5}
        if not arc_map and t in solution.truck_routes and len(solution.truck_routes[t]) >= 2:
            route = list(solution.truck_routes[t])
            truck_routes[t] = route
        elif not arc_map:
            truck_routes[t] = [0, 0]
            continue
        else:
            try:
                route = _route_from_arcs(0, arc_map)
                truck_routes[t] = route
            except Exception as ex:
                findings.append(ValidationFinding("ROUTE", "ERROR", str(ex), "truck", t))
                truck_routes[t] = [0, 0]
                continue
        route = truck_routes[t]
        if route == [0]:
            route = [0, 0]
            truck_routes[t] = route
        if route[0] != 0:
            findings.append(ValidationFinding("ROUTE", "ERROR", f"Truck {t} route does not start at depot", "truck", t))
        if route[-1] != 0:
            findings.append(ValidationFinding("ROUTE", "ERROR", f"Truck {t} route does not return to depot", "truck", t))

    drone_routes: Dict[int, List[int]] = {}
    for d in instance.drone_ids():
        arc_map = {(i, j): 1 for (i, j, dd), v in solution.x_drone.items() if dd == d and v >= 0.5}
        if not arc_map and d in solution.drone_routes and len(solution.drone_routes[d]) >= 2:
            route = list(solution.drone_routes[d])
            drone_routes[d] = route
        elif not arc_map:
            drone_routes[d] = [0, 0]
            continue
        else:
            try:
                route = _route_from_arcs(0, arc_map)
                drone_routes[d] = route
            except Exception as ex:
                findings.append(ValidationFinding("ROUTE", "ERROR", str(ex), "drone", d))
                drone_routes[d] = [0, 0]
                continue
        route = drone_routes[d]
        if route == [0]:
            route = [0, 0]
            drone_routes[d] = route
        if route[0] != 0:
            findings.append(ValidationFinding("ROUTE", "ERROR", f"Drone {d} route does not start at depot", "drone", d))
        if route[-1] != 0:
            findings.append(ValidationFinding("ROUTE", "ERROR", f"Drone {d} route does not return to depot", "drone", d))

    solution.truck_routes = truck_routes
    solution.drone_routes = drone_routes

    dist_manh, dist_euc = instance.distance_matrices()

    # Truck and drone timing consistency
    truck_arrivals: Dict[Tuple[int, int], float] = {}
    drone_arrivals: Dict[Tuple[int, int], float] = {}
    for t, route in truck_routes.items():
        if len(route) < 2:
            continue
        speed = instance.trucks[t - 1].speed_kmph
        service_times = {0: 0.0}
        for c in instance.customers:
            service_times[c.node_id] = (
                c.service_time_min if solution.u_truck.get((t, c.node_id), 0) >= 0.5 else 0.0
            )
        inferred = _infer_arrival(route, dist_manh, speed, service_times)
        arrivals = {
            (t, node): solution.a_truck.get((t, node), value)
            for node, value in inferred.items()
        }
        for (vehicle, node), arr in arrivals.items():
            truck_arrivals[(vehicle, node)] = arr
            if node != 0:
                c = instance.customer_map()[node]
                if solution.u_truck.get((t, node), 0) >= 0.5 and arr < c.lb - tol:
                    findings.append(ValidationFinding("TW_LOWER_BOUND", "ERROR", f"Truck {t} arrives before lb at customer {node}", "truck", node))
                if solution.u_truck.get((t, node), 0) >= 0.5 and arr > c.ub + tol:
                    findings.append(ValidationFinding("TW_TARDINESS", "WARN", f"Truck {t} arrives after ub at customer {node}", "truck", node))
        return_time = solution.__dict__.get("return_time_truck", {}).get(t, arrivals.get((t, 0), 0.0))
        if return_time > instance.constants.max_shift_minutes + tol:
            findings.append(ValidationFinding("SHIFT", "ERROR", f"Truck {t} exceeds shift", "truck", t))

    for d, route in drone_routes.items():
        if len(route) < 2:
            continue
        speed = instance.drones[d - 1].speed_kmph
        service_times = {0: 0.0}
        for c in instance.customers:
            service_times[c.node_id] = (
                c.service_time_min if solution.u_drone.get((d, c.node_id), 0) >= 0.5 else 0.0
            )
        inferred = _infer_arrival(route, dist_euc, speed, service_times)
        arrivals = {
            (d, node): solution.a_drone.get((d, node), value)
            for node, value in inferred.items()
        }
        for (vehicle, node), arr in arrivals.items():
            drone_arrivals[(vehicle, node)] = arr
            if node != 0:
                c = instance.customer_map()[node]
                if solution.u_drone.get((d, node), 0) >= 0.5 and arr < c.lb - tol:
                    findings.append(ValidationFinding("TW_LOWER_BOUND", "ERROR", f"Drone {d} arrives before lb at customer {node}", "drone", node))
                if solution.u_drone.get((d, node), 0) >= 0.5 and arr > c.ub + tol:
                    findings.append(ValidationFinding("TW_TARDINESS", "WARN", f"Drone {d} arrives after ub at customer {node}", "drone", node))
        return_time = solution.__dict__.get("return_time_drone", {}).get(d, arrivals.get((d, 0), 0.0))
        if return_time > instance.constants.max_shift_minutes + tol:
            findings.append(ValidationFinding("SHIFT", "ERROR", f"Drone {d} exceeds shift", "drone", d))

    # Load feasibility
    for t in instance.truck_ids():
        cap = instance.trucks[t - 1].capacity_kg
        load = cap
        visited = set()
        for node in truck_routes[t]:
            if node == 0:
                continue
            if node in visited:
                break
            visited.add(node)
            if node not in served_by_truck:
                continue
            c = instance.customer_map()[node]
            if solution.u_truck.get((t, node), 0) >= 0.5:
                load -= c.demand_kg
            reload_weight = sum(
                instance.drones[d - 1].parcel_weight_kg
                for d in instance.drone_ids()
                if solution.z1.get((node, d, t), 0) > 0.5
            )
            load -= reload_weight
            if load < -tol:
                findings.append(ValidationFinding("LOAD", "ERROR", f"Truck {t} overload after {node}", "truck", node))
        if load < -tol:
            findings.append(ValidationFinding("LOAD", "ERROR", f"Truck {t} negative final load", "truck", t))

    # Route intersections define rendezvous; parcel and battery support actions
    # are validated from one shared state transition model.
    expected_events = route_intersection_events(solution)
    explicit_events = {
        (int(node), int(drone), int(truck))
        for (node, drone, truck), value in solution.u_sync.items()
        if value > 0.5
    }
    for node, drone, truck in sorted(expected_events - explicit_events):
        findings.append(
            ValidationFinding(
                "SYNC_MISSING",
                "ERROR",
                f"Drone {drone} and truck {truck} share node {node} without u_sync",
                "drone",
                node,
            )
        )
    for node, drone, truck in sorted(explicit_events - expected_events):
        findings.append(
            ValidationFinding(
                "SYNC_EXTRA",
                "ERROR",
                f"u_sync at node {node} for drone {drone} and truck {truck} lacks a joint route visit",
                "drone",
                node,
            )
        )

    state_analysis = analyze_drone_states(instance, solution, events=expected_events)
    for issue in state_analysis.issues:
        findings.append(
            ValidationFinding(issue.code, "ERROR", issue.message, "drone", issue.node)
        )
    for issue in support_state_mismatches(solution, state_analysis):
        findings.append(
            ValidationFinding(issue.code, "ERROR", issue.message, "drone", issue.node)
        )
    actual_state = simulate_drone_states(instance, solution)
    seen_state_errors = {
        (item.code, item.message)
        for item in findings
        if item.vehicle == "drone" and item.severity == "ERROR"
    }
    for issue in actual_state.issues:
        if (issue.code, issue.message) not in seen_state_errors:
            findings.append(
                ValidationFinding(issue.code, "ERROR", issue.message, "drone", issue.node)
            )

    # Delivery time is the arrival time of the vehicle assigned to serve the
    # customer; an unrelated synchronization visit must not determine L_i.
    served_time: Dict[int, float] = {c: 0.0 for c in customers}
    for c in customers:
        assigned_truck = [t for t in instance.truck_ids() if solution.u_truck.get((t, c), 0) >= 0.5]
        assigned_drone = [d for d in instance.drone_ids() if solution.u_drone.get((d, c), 0) >= 0.5]
        if assigned_truck:
            served_time[c] = truck_arrivals.get((assigned_truck[0], c), solution.a_truck.get((assigned_truck[0], c), 0.0))
        elif assigned_drone:
            served_time[c] = drone_arrivals.get((assigned_drone[0], c), solution.a_drone.get((assigned_drone[0], c), 0.0))
        expected = max(0.0, served_time[c] - instance.customer_map()[c].ub)
        observed = solution.tardiness.get(c, expected)
        if abs(observed - expected) > max(1e-3, 1e-6 * expected + 1e-3):
            findings.append(ValidationFinding("TARDINESS", "WARN", f"Stored tardiness mismatch at customer {c}", "global", c))
            solution.tardiness[c] = expected

    # Synchronization consistency is evaluated from post-service readiness,
    # not from stored lower-bound waiting variables or raw arrival gaps.
    readiness_truck, readiness_drone = compute_readiness_times(
        instance,
        solution,
        truck_arrivals=truck_arrivals,
        drone_arrivals=drone_arrivals,
    )
    sync_diagnostics = compute_synchronization_waiting(
        instance,
        solution,
        truck_arrivals=truck_arrivals,
        drone_arrivals=drone_arrivals,
    )
    solution.__dict__["ready_truck"] = readiness_truck
    solution.__dict__["ready_drone"] = readiness_drone
    solution.waiting_truck = dict(sync_diagnostics["waiting_truck"])
    solution.waiting_drone = dict(sync_diagnostics["waiting_drone"])

    truck_route_nodes = {t: set(route) for t, route in truck_routes.items()}
    drone_route_nodes = {d: set(route) for d, route in drone_routes.items()}
    explicit_rendezvous = any(value > 0.5 for value in solution.u_sync.values())

    for (i, d, t), val in solution.u_sync.items():
        if val <= 0.5:
            continue
        if i == 0:
            findings.append(ValidationFinding("SYNC_DEPOT_EVENT", "ERROR", "Truck-based rendezvous cannot be encoded at the depot", "sync", i))
        if i not in truck_route_nodes.get(t, set()) or i not in drone_route_nodes.get(d, set()):
            findings.append(ValidationFinding("SYNC_ROUTE", "ERROR", f"Rendezvous at node {i} is not visited by both truck {t} and drone {d}", "sync", i))

    # Sync consistency checks
    for (i, d, t), val in solution.z1.items():
        if val <= 0.5:
            continue
        if i == 0:
            findings.append(ValidationFinding("SYNC_DEPOT_EVENT", "ERROR", "Parcel reload cannot be encoded as a depot synchronization event", "sync", i))
        if i not in truck_route_nodes.get(t, set()) or i not in drone_route_nodes.get(d, set()):
            findings.append(ValidationFinding("SYNC_ROUTE", "ERROR", f"Parcel reload at node {i} is not visited by both truck {t} and drone {d}", "sync", i))
        if explicit_rendezvous and solution.u_sync.get((i, d, t), 0) <= 0.5:
            findings.append(ValidationFinding("SYNC_U_LINK", "ERROR", f"Parcel reload at node {i} is not linked to physical rendezvous u", "sync", i))

    for (i, d, t), val in solution.z2.items():
        if val <= 0.5:
            continue
        if i == 0:
            findings.append(ValidationFinding("SYNC_DEPOT_EVENT", "ERROR", "Battery swap cannot be encoded as a depot synchronization event", "sync", i))
        if i not in truck_route_nodes.get(t, set()) or i not in drone_route_nodes.get(d, set()):
            findings.append(ValidationFinding("SYNC_ROUTE", "ERROR", f"Battery swap at node {i} is not visited by both truck {t} and drone {d}", "sync", i))
        if explicit_rendezvous and solution.u_sync.get((i, d, t), 0) <= 0.5:
            findings.append(ValidationFinding("SYNC_U_LINK", "ERROR", f"Battery swap at node {i} is not linked to physical rendezvous u", "sync", i))

    # If a complete schedule is available, enforce the departure inequalities
    # directly.  Waiting consumes no battery here: the battery validator above
    # charges only flight arcs, matching the landed-waiting assumption.
    for event in sync_diagnostics["events"]:
        node = int(event["node"])
        truck_id = int(event["truck_id"])
        drone_id = int(event["drone_id"])
        handling_min = float(event.get("handling_minutes", 0.0))
        required_truck_departure = (
            float(event["truck_readiness"])
            + handling_min
            + float(event["truck_wait"])
        )
        required_drone_departure = (
            float(event["drone_readiness"])
            + handling_min
            + float(event["drone_wait"])
        )
        observed_truck_departure = solution.l_truck.get((truck_id, node))
        observed_drone_departure = solution.l_drone.get((drone_id, node))
        if observed_truck_departure is not None and observed_truck_departure + tol < required_truck_departure:
            findings.append(ValidationFinding("SYNC_DEPARTURE", "ERROR", f"Truck {truck_id} departs synchronization node {node} before readiness, waiting, and handling are complete", "truck", node))
        if observed_drone_departure is not None and observed_drone_departure + tol < required_drone_departure:
            findings.append(ValidationFinding("SYNC_DEPARTURE", "ERROR", f"Drone {drone_id} departs synchronization node {node} before readiness, waiting, and handling are complete", "drone", node))

    return findings


def is_feasible(instance: InstanceData, solution: SolutionData) -> bool:
    findings = validate_solution(instance, solution)
    return all(item.severity != "ERROR" for item in findings)
