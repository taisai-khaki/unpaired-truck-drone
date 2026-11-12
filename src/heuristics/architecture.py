from __future__ import annotations

from collections import defaultdict
from typing import Any

from ..data_models import InstanceData, SolutionData
from ..drone_state import analyze_drone_states, route_intersection_events, support_state_mismatches
from ..timing import active_rendezvous_events


def sortie_associations(solution: SolutionData) -> list[dict[str, Any]]:
    associations: list[dict[str, Any]] = []
    rendezvous = active_rendezvous_events(solution)
    for drone_id, raw_route in solution.drone_routes.items():
        route = list(raw_route)
        positions = {int(node): index for index, node in enumerate(route) if int(node) != 0}
        trucks_by_position: dict[int, set[int]] = defaultdict(set)
        for node, event_drone, truck_id in rendezvous:
            if event_drone == int(drone_id) and node in positions:
                trucks_by_position[positions[node]].add(int(truck_id))
        event_positions = sorted(trucks_by_position)
        service_positions = [
            (positions[int(customer_id)], int(customer_id))
            for (event_drone, customer_id), value in solution.u_drone.items()
            if int(event_drone) == int(drone_id)
            and value >= 0.5
            and int(customer_id) in positions
        ]
        for service_position, customer_id in sorted(service_positions):
            prior_positions = [position for position in event_positions if position < service_position]
            next_positions = [position for position in event_positions if position >= service_position]
            launch_position = prior_positions[-1] if prior_positions else None
            recovery_position = next_positions[0] if next_positions else None
            launch_node = int(route[launch_position]) if launch_position is not None else 0
            recovery_node = int(route[recovery_position]) if recovery_position is not None else 0
            launch_trucks = sorted(trucks_by_position[launch_position]) if launch_position is not None else []
            recovery_trucks = sorted(trucks_by_position[recovery_position]) if recovery_position is not None else []
            launch_truck = launch_trucks[0] if len(launch_trucks) == 1 else None
            recovery_truck = recovery_trucks[0] if len(recovery_trucks) == 1 else None
            associations.append(
                {
                    "drone_id": int(drone_id),
                    "customer_id": customer_id,
                    "service_position": service_position,
                    "launch_node": launch_node,
                    "launch_truck": launch_truck,
                    "launch_trucks": launch_trucks,
                    "recovery_node": recovery_node,
                    "recovery_truck": recovery_truck,
                    "recovery_trucks": recovery_trucks,
                    "cross_truck_recovery": bool(
                        launch_truck is not None
                        and recovery_truck is not None
                        and launch_truck != recovery_truck
                    ),
                }
            )
    return associations


def count_cross_truck_recoveries(solution: SolutionData) -> int:
    return sum(association["cross_truck_recovery"] for association in sortie_associations(solution))


def validate_drone_architecture(
    instance: InstanceData,
    solution: SolutionData,
    architecture: str,
    drone_to_truck: dict[int, int] | None = None,
) -> list[str]:
    architecture = architecture.upper()
    if architecture not in {"M3", "M4", "M5"}:
        raise ValueError(f"Unknown truck-drone architecture: {architecture}")
    pairing = {int(drone): int(truck) for drone, truck in (drone_to_truck or {}).items()}
    violations: list[str] = []
    rendezvous = active_rendezvous_events(solution)
    route_events = route_intersection_events(solution)
    explicit_events = {
        (int(node), int(drone), int(truck))
        for (node, drone, truck), value in solution.u_sync.items()
        if value > 0.5
    }

    for node, drone_id, truck_id in sorted(route_events - explicit_events):
        violations.append(
            f"Drone {drone_id} and truck {truck_id} share node {node} without u_sync"
        )
    for node, drone_id, truck_id in sorted(explicit_events - route_events):
        violations.append(
            f"u_sync at node {node} for drone {drone_id} and truck {truck_id} lacks a joint route visit"
        )

    state_analysis = analyze_drone_states(instance, solution, events=route_events)
    for issue in state_analysis.issues:
        violations.append(issue.message)
    for issue in support_state_mismatches(solution, state_analysis):
        violations.append(issue.message)

    if architecture == "M3" and any(drone not in pairing for drone in instance.drone_ids()):
        violations.append("M3 requires a fixed truck assignment for every drone")

    for node, drone_id, truck_id in sorted(rendezvous):
        if node not in solution.drone_routes.get(drone_id, []):
            violations.append(f"M{architecture[1]} drone {drone_id} rendezvous node {node} is absent from its route")
        if node not in solution.truck_routes.get(truck_id, []):
            violations.append(f"M{architecture[1]} truck {truck_id} rendezvous node {node} is absent from its route")
        if architecture == "M3" and pairing.get(drone_id) != truck_id:
            violations.append(f"M3 drone {drone_id} uses truck {truck_id}, not paired truck {pairing.get(drone_id)}")

    associations = sortie_associations(solution)
    by_drone: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for association in associations:
        drone_id = association["drone_id"]
        by_drone[drone_id].append(association)
        customer_id = association["customer_id"]
        launch_node = association["launch_node"]
        recovery_node = association["recovery_node"]
        launch_trucks = association["launch_trucks"]
        recovery_trucks = association["recovery_trucks"]

        if len(launch_trucks) > 1:
            violations.append(f"Drone {drone_id} has ambiguous launch trucks at node {launch_node}")
        if len(recovery_trucks) > 1:
            violations.append(f"Drone {drone_id} has ambiguous recovery trucks at node {recovery_node}")

        if recovery_node != 0 and len(recovery_trucks) == 1:
            recovery_truck = recovery_trucks[0]
            if architecture == "M3" and pairing.get(drone_id) != recovery_truck:
                violations.append(f"M3 drone {drone_id} recovers with unpaired truck {recovery_truck}")
            if architecture == "M4" and launch_trucks and launch_trucks[0] != recovery_truck:
                violations.append(
                    f"M4 drone {drone_id} customer {customer_id} recovers with truck {recovery_truck} "
                    f"after launching with truck {launch_trucks[0]}"
                )
        if architecture == "M3" and launch_trucks and pairing.get(drone_id) != launch_trucks[0]:
            violations.append(f"M3 drone {drone_id} launches with unpaired truck {launch_trucks[0]}")

    for drone_id, drone_associations in by_drone.items():
        drone_associations.sort(key=lambda item: item["service_position"])
        for previous, current in zip(drone_associations, drone_associations[1:]):
            previous_recovery = (previous["recovery_node"], previous["recovery_truck"])
            next_launch = (current["launch_node"], current["launch_truck"])
            if previous_recovery[0] == 0:
                violations.append(
                    f"Drone {drone_id} returns to depot after customer {previous['customer_id']} "
                    f"before later customer {current['customer_id']}"
                )
                continue
            if previous_recovery != next_launch:
                violations.append(
                    f"Drone {drone_id} jumps from recovery {previous_recovery} to launch {next_launch} "
                    f"between customers {previous['customer_id']} and {current['customer_id']}"
                )
                continue
            launch_node, launch_truck = next_launch
            if launch_truck is None:
                violations.append(
                    f"Drone {drone_id} has no unique truck at continuous rendezvous {launch_node} "
                    f"before customer {current['customer_id']}"
                )

    for support_name, support_map in (("z1", solution.z1), ("z2", solution.z2)):
        for (node, drone_id, truck_id), value in support_map.items():
            if value <= 0.5:
                continue
            if (int(node), int(drone_id), int(truck_id)) not in route_events:
                violations.append(
                    f"Drone {drone_id} {support_name} event at node {node} is not linked to a physical rendezvous"
                )
            if architecture == "M3" and pairing.get(int(drone_id)) != int(truck_id):
                violations.append(
                    f"M3 drone {drone_id} {support_name} uses unpaired truck {truck_id}"
                )

    return list(dict.fromkeys(violations))
