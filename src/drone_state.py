from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from .data_models import InstanceData, SolutionData
from .distance_utils import euclidean_distance, travel_time_from_distance

RendezvousKey = Tuple[int, int, int]


@dataclass
class DroneStateIssue:
    code: str
    message: str
    drone_id: int
    node: int | None = None


@dataclass
class DroneLegState:
    drone_id: int
    leg_index: int
    from_node: int
    to_node: int
    loaded: bool
    energy_wh: float
    battery_before_wh: float
    battery_after_arrival_wh: float


@dataclass
class DroneRendezvousState:
    event_key: RendezvousKey
    parcel_loaded_before_support: bool
    parcel_loaded_after_support: bool
    z1_required: bool
    battery_before_support_wh: float
    battery_after_support_wh: float
    z2_required: bool
    energy_to_next_event_or_depot_wh: float


@dataclass
class DroneStateAnalysis:
    route_events: set[RendezvousKey] = field(default_factory=set)
    required_z1: Dict[RendezvousKey, int] = field(default_factory=dict)
    required_z2: Dict[RendezvousKey, int] = field(default_factory=dict)
    rendezvous: List[DroneRendezvousState] = field(default_factory=list)
    legs: List[DroneLegState] = field(default_factory=list)
    issues: List[DroneStateIssue] = field(default_factory=list)
    battery_after_node_wh: Dict[Tuple[int, int], float] = field(default_factory=dict)


def route_intersection_events(
    solution: SolutionData,
    *,
    truck_routes: Dict[int, List[int]] | None = None,
    drone_routes: Dict[int, List[int]] | None = None,
) -> set[RendezvousKey]:
    trucks = truck_routes if truck_routes is not None else solution.truck_routes
    drones = drone_routes if drone_routes is not None else solution.drone_routes
    truck_nodes = {
        int(truck_id): {int(node) for node in route if int(node) != 0}
        for truck_id, route in trucks.items()
    }
    events: set[RendezvousKey] = set()
    for drone_id, route in drones.items():
        drone_nodes = {int(node) for node in route if int(node) != 0}
        for truck_id, nodes in truck_nodes.items():
            events.update(
                (node, int(drone_id), int(truck_id))
                for node in drone_nodes.intersection(nodes)
            )
    return events


def _drone_power_per_min(instance: InstanceData, drone_id: int, loaded: bool) -> float:
    drone = instance.drones[drone_id - 1]
    return float(
        drone.energy_per_min_when_loaded if loaded else drone.energy_per_min_when_empty
    )


def _leg_energy(
    instance: InstanceData,
    drone_id: int,
    frm: int,
    to: int,
    loaded: bool,
) -> float:
    drone = instance.drones[drone_id - 1]
    coordinates = instance.coordinate_map()
    travel_minutes = travel_time_from_distance(
        euclidean_distance(coordinates[int(frm)], coordinates[int(to)]),
        float(drone.speed_kmph),
    )
    return _drone_power_per_min(instance, drone_id, loaded) * travel_minutes


def analyze_drone_states(
    instance: InstanceData,
    solution: SolutionData,
    *,
    events: set[RendezvousKey] | None = None,
    truck_routes: Dict[int, List[int]] | None = None,
    drone_routes: Dict[int, List[int]] | None = None,
    tolerance: float = 1e-9,
    reserve_ratio: float | None = None,
) -> DroneStateAnalysis:
    trucks = truck_routes if truck_routes is not None else solution.truck_routes
    drones = drone_routes if drone_routes is not None else solution.drone_routes
    route_events = set(
        events
        if events is not None
        else route_intersection_events(
            solution, truck_routes=trucks, drone_routes=drones
        )
    )
    analysis = DroneStateAnalysis(route_events=route_events)
    coordinates = instance.coordinate_map()
    reserve_ratio = float(
        instance.metadata.get("battery_slack_ratio", 0.10)
        if reserve_ratio is None
        else reserve_ratio
    )
    reserve_ratio = max(0.0, min(1.0, reserve_ratio))

    for drone_id in instance.drone_ids():
        route = [int(node) for node in drones.get(drone_id, [0, 0])]
        if not route:
            route = [0, 0]
        services = {
            int(customer)
            for (assigned_drone, customer), value in solution.u_drone.items()
            if int(assigned_drone) == drone_id and value >= 0.5
        }
        route_positions: Dict[int, int] = {}
        for position, node in enumerate(route):
            if node != 0:
                if node in route_positions:
                    analysis.issues.append(
                        DroneStateIssue(
                            "DRONE_ROUTE_DUPLICATE",
                            f"Drone {drone_id} visits node {node} more than once",
                            drone_id,
                            node,
                        )
                    )
                route_positions[node] = position
        for customer in sorted(services - set(route_positions)):
            analysis.issues.append(
                DroneStateIssue(
                    "DRONE_SERVICE_ROUTE",
                    f"Drone {drone_id} serves customer {customer} but the customer is absent from its route",
                    drone_id,
                    customer,
                )
            )

        events_at_position: Dict[int, List[RendezvousKey]] = {}
        for key in sorted(route_events):
            node, event_drone, _truck_id = key
            if event_drone != drone_id:
                continue
            if node not in route_positions:
                analysis.issues.append(
                    DroneStateIssue(
                        "SYNC_ROUTE",
                        f"Rendezvous at node {node} is absent from drone {drone_id}'s route",
                        drone_id,
                        node,
                    )
                )
                continue
            events_at_position.setdefault(route_positions[node], []).append(key)

        for position, keys in events_at_position.items():
            truck_ids = {key[2] for key in keys}
            if len(truck_ids) > 1:
                analysis.issues.append(
                    DroneStateIssue(
                        "SYNC_AMBIGUOUS_TRUCK",
                        f"Drone {drone_id} has multiple rendezvous trucks at node {route[position]}",
                        drone_id,
                        route[position],
                    )
                )

        event_positions = sorted(events_at_position)
        loaded = True
        loaded_by_leg: List[bool] = []
        parcel_event_values: Dict[RendezvousKey, Tuple[bool, bool, bool]] = {}
        service_positions = {
            position
            for customer, position in route_positions.items()
            if customer in services
        }
        for position, node in enumerate(route):
            if position > 0 and node in services:
                if not loaded:
                    analysis.issues.append(
                        DroneStateIssue(
                            "PARCEL_STATE",
                            f"Drone {drone_id} reaches customer {node} without a loaded parcel",
                            drone_id,
                            node,
                        )
                    )
                loaded = False
            keys = events_at_position.get(position, [])
            if keys:
                next_event_position = next(
                    (event_position for event_position in event_positions if event_position > position),
                    len(route) - 1,
                )
                future_service = any(
                    position < service_position <= next_event_position
                    for service_position in service_positions
                )
                parcel_before = loaded
                reload_required = not loaded and future_service
                if reload_required:
                    if len(keys) == 1:
                        analysis.required_z1[keys[0]] = 1
                    else:
                        analysis.issues.append(
                            DroneStateIssue(
                                "PARCEL_RELOAD_TRUCK_AMBIGUOUS",
                                f"Drone {drone_id} requires a reload at node {node}, but the rendezvous truck is ambiguous",
                                drone_id,
                                node,
                            )
                        )
                    loaded = True
                for key in keys:
                    parcel_event_values[key] = (parcel_before, loaded, reload_required)
            if position < len(route) - 1:
                loaded_by_leg.append(loaded)

        leg_energy = [
            _leg_energy(instance, drone_id, route[index], route[index + 1], loaded_by_leg[index])
            for index in range(max(0, len(route) - 1))
        ]
        drone = instance.drones[drone_id - 1]
        max_battery = float(drone.max_battery_wh)
        reserve = reserve_ratio * max_battery
        battery = max_battery
        rendezvous_by_position: Dict[int, List[DroneRendezvousState]] = {}
        leg_records_by_index: Dict[int, DroneLegState] = {}

        for position, node in enumerate(route):
            battery_before_arrival = battery
            if position > 0:
                energy = leg_energy[position - 1]
                battery -= energy
                if battery < -tolerance:
                    analysis.issues.append(
                        DroneStateIssue(
                            "ENERGY",
                            f"Drone {drone_id} battery infeasible on {route[position - 1]}->{node}",
                            drone_id,
                            node,
                        )
                    )
                if battery < reserve - tolerance:
                    analysis.issues.append(
                        DroneStateIssue(
                            "BATTERY_RESERVE",
                            f"Drone {drone_id} reaches node {node} below residual battery reserve",
                            drone_id,
                            node,
                        )
                    )
                leg_records_by_index[position - 1] = DroneLegState(
                    drone_id=drone_id,
                    leg_index=position - 1,
                    from_node=int(route[position - 1]),
                    to_node=int(node),
                    loaded=loaded_by_leg[position - 1],
                    energy_wh=energy,
                    battery_before_wh=battery_before_arrival,
                    battery_after_arrival_wh=battery,
                )

            keys = events_at_position.get(position, [])
            if not keys:
                analysis.battery_after_node_wh[(drone_id, int(node))] = battery
                continue
            next_event_position = next(
                (event_position for event_position in event_positions if event_position > position),
                len(route) - 1,
            )
            energy_to_next = sum(leg_energy[position:next_event_position])
            swap_required = battery - energy_to_next < reserve - tolerance
            swap_possible = max_battery - energy_to_next >= reserve - tolerance
            if swap_required and not swap_possible:
                analysis.issues.append(
                    DroneStateIssue(
                        "BATTERY_SEGMENT",
                        f"Drone {drone_id} cannot complete the flight segment after rendezvous {node} even with a full battery",
                        drone_id,
                        node,
                    )
                )
            if swap_required and len(keys) == 1 and swap_possible:
                analysis.required_z2[keys[0]] = 1
            battery_before_support = battery
            if swap_required and swap_possible:
                battery = max_battery
            for key in keys:
                parcel_before, parcel_after, reload_required = parcel_event_values.get(
                    key, (True, True, False)
                )
                state = DroneRendezvousState(
                    event_key=key,
                    parcel_loaded_before_support=parcel_before,
                    parcel_loaded_after_support=parcel_after,
                    z1_required=reload_required,
                    battery_before_support_wh=battery_before_support,
                    battery_after_support_wh=battery,
                    z2_required=swap_required and swap_possible,
                    energy_to_next_event_or_depot_wh=energy_to_next,
                )
                rendezvous_by_position.setdefault(position, []).append(state)
            analysis.battery_after_node_wh[(drone_id, int(node))] = battery

        analysis.legs.extend(leg_records_by_index[index] for index in sorted(leg_records_by_index))
        for position in sorted(rendezvous_by_position):
            analysis.rendezvous.extend(rendezvous_by_position[position])

    return analysis


def simulate_drone_states(
    instance: InstanceData,
    solution: SolutionData,
    *,
    tolerance: float = 1e-9,
) -> DroneStateAnalysis:
    """Simulate parcel and battery transitions using the solution's support actions."""
    route_events = route_intersection_events(solution)
    analysis = DroneStateAnalysis(route_events=route_events)
    event_keys = set(route_events)
    for support_map, code, label in (
        (solution.z1, "UNMATCHED_RELOAD", "parcel reload"),
        (solution.z2, "UNMATCHED_BATTERY_SWAP", "battery swap"),
    ):
        for (node, drone_id, truck_id), value in support_map.items():
            key = (int(node), int(drone_id), int(truck_id))
            if value > 0.5 and key not in event_keys:
                analysis.issues.append(
                    DroneStateIssue(
                        code,
                        f"Drone {drone_id} has a {label} without a physical rendezvous at node {node}",
                        int(drone_id),
                        int(node),
                    )
                )

    reserve_ratio = max(
        0.0,
        min(1.0, float(instance.metadata.get("battery_slack_ratio", 0.10))),
    )
    for drone_id in instance.drone_ids():
        route = [int(node) for node in solution.drone_routes.get(drone_id, [0, 0])]
        if len(route) < 2:
            route = [0, 0]
        services = {
            int(customer)
            for (assigned_drone, customer), value in solution.u_drone.items()
            if int(assigned_drone) == drone_id and value >= 0.5
        }
        positions: Dict[int, int] = {}
        for index, node in enumerate(route):
            if node != 0:
                if node in positions:
                    analysis.issues.append(
                        DroneStateIssue(
                            "DRONE_ROUTE_DUPLICATE",
                            f"Drone {drone_id} visits node {node} more than once",
                            drone_id,
                            node,
                        )
                    )
                positions[node] = index
        for customer in sorted(services - set(positions)):
            analysis.issues.append(
                DroneStateIssue(
                    "DRONE_SERVICE_ROUTE",
                    f"Drone {drone_id} serves customer {customer} but the customer is absent from its route",
                    drone_id,
                    customer,
                )
            )

        events_at_position: Dict[int, List[RendezvousKey]] = {}
        for key in sorted(route_events):
            node, event_drone, _truck_id = key
            if event_drone == drone_id and node in positions:
                events_at_position.setdefault(positions[node], []).append(key)
        for position, keys in events_at_position.items():
            if len({key[2] for key in keys}) > 1:
                analysis.issues.append(
                    DroneStateIssue(
                        "SYNC_AMBIGUOUS_TRUCK",
                        f"Drone {drone_id} has multiple rendezvous trucks at node {route[position]}",
                        drone_id,
                        route[position],
                    )
                )

        drone = instance.drones[drone_id - 1]
        battery = float(drone.max_battery_wh)
        reserve = reserve_ratio * battery
        loaded = True
        for index, node in enumerate(route):
            if index > 0:
                previous = route[index - 1]
                energy = _leg_energy(instance, drone_id, previous, node, loaded)
                before = battery
                battery -= energy
                analysis.legs.append(
                    DroneLegState(
                        drone_id=drone_id,
                        leg_index=index - 1,
                        from_node=previous,
                        to_node=node,
                        loaded=loaded,
                        energy_wh=energy,
                        battery_before_wh=before,
                        battery_after_arrival_wh=battery,
                    )
                )
                if battery < -tolerance:
                    analysis.issues.append(
                        DroneStateIssue(
                            "ENERGY",
                            f"Drone {drone_id} battery infeasible on {previous}->{node}",
                            drone_id,
                            node,
                        )
                    )
                if node != 0 and battery < reserve - tolerance:
                    analysis.issues.append(
                        DroneStateIssue(
                            "BATTERY_RESERVE",
                            f"Drone {drone_id} reaches node {node} below residual battery reserve",
                            drone_id,
                            node,
                        )
                    )

            if node in services:
                if not loaded:
                    analysis.issues.append(
                        DroneStateIssue(
                            "PARCEL_STATE",
                            f"Drone {drone_id} reaches customer {node} without a loaded parcel",
                            drone_id,
                            node,
                        )
                    )
                loaded = False

            keys = events_at_position.get(index, [])
            if keys:
                reload = any(solution.z1.get(key, 0) > 0.5 for key in keys)
                swap = any(solution.z2.get(key, 0) > 0.5 for key in keys)
                before_support = battery
                loaded_before = loaded
                if reload:
                    loaded = True
                if swap:
                    battery = float(drone.max_battery_wh)
                analysis.rendezvous.extend(
                    DroneRendezvousState(
                        event_key=key,
                        parcel_loaded_before_support=loaded_before,
                        parcel_loaded_after_support=loaded,
                        z1_required=reload,
                        battery_before_support_wh=before_support,
                        battery_after_support_wh=battery,
                        z2_required=swap,
                        energy_to_next_event_or_depot_wh=0.0,
                    )
                    for key in keys
                )
            analysis.battery_after_node_wh[(drone_id, int(node))] = battery
    return analysis


def reconstruct_support_events(
    instance: InstanceData,
    solution: SolutionData,
    *,
    reserve_ratio: float | None = None,
) -> DroneStateAnalysis:
    events = route_intersection_events(solution)
    solution.u_sync = {key: 1 for key in sorted(events)}
    analysis = analyze_drone_states(
        instance,
        solution,
        events=events,
        reserve_ratio=reserve_ratio,
    )
    solution.z1 = dict(analysis.required_z1)
    solution.z2 = dict(analysis.required_z2)
    return analysis


def support_state_mismatches(
    solution: SolutionData,
    analysis: DroneStateAnalysis,
) -> List[DroneStateIssue]:
    issues: List[DroneStateIssue] = []
    actual_z1 = {key for key, value in solution.z1.items() if value > 0.5}
    actual_z2 = {key for key, value in solution.z2.items() if value > 0.5}
    required_z1 = set(analysis.required_z1)
    required_z2 = set(analysis.required_z2)
    for key in sorted(actual_z1 - analysis.route_events):
        issues.append(
            DroneStateIssue(
                "UNMATCHED_RELOAD",
                f"Drone {key[1]} has a parcel reload without a rendezvous at node {key[0]}",
                key[1],
                key[0],
            )
        )
    for key in sorted(actual_z2 - analysis.route_events):
        issues.append(
            DroneStateIssue(
                "UNMATCHED_BATTERY_SWAP",
                f"Drone {key[1]} has a battery swap without a rendezvous at node {key[0]}",
                key[1],
                key[0],
            )
        )
    for key in sorted(required_z1 - actual_z1):
        issues.append(
            DroneStateIssue(
                "PARCEL_STATE",
                f"Drone {key[1]} requires parcel reload at rendezvous node {key[0]}",
                key[1],
                key[0],
            )
        )
    for key in sorted(actual_z1 - required_z1):
        issues.append(
            DroneStateIssue(
                "UNNECESSARY_RELOAD",
                f"Drone {key[1]} has an unnecessary parcel reload at rendezvous node {key[0]}",
                key[1],
                key[0],
            )
        )
    for key in sorted(required_z2 - actual_z2):
        issues.append(
            DroneStateIssue(
                "BATTERY_RESERVE",
                f"Drone {key[1]} requires battery restoration at rendezvous node {key[0]}",
                key[1],
                key[0],
            )
        )
    for key in sorted(actual_z2 - required_z2):
        issues.append(
            DroneStateIssue(
                "UNNECESSARY_BATTERY_SWAP",
                f"Drone {key[1]} has an unnecessary battery swap at rendezvous node {key[0]}",
                key[1],
                key[0],
            )
        )
    return issues
