"""Timing diagnostics derived from the final service schedule."""
from __future__ import annotations

from typing import Dict, Mapping, Tuple

from .data_models import InstanceData, SolutionData


def active_rendezvous_events(solution: SolutionData) -> set[Tuple[int, int, int]]:
    """Return physical truck-drone rendezvous events.

    New solutions use ``u_sync`` explicitly.  The fallback preserves the
    interpretation of legacy solution files that only contain reload or swap
    flags.
    """
    explicit = {
        (int(node), int(drone_id), int(truck_id))
        for (node, drone_id, truck_id), value in solution.u_sync.items()
        if value > 0.5 and int(node) != 0
    }
    if explicit:
        return explicit
    return {
        (int(node), int(drone_id), int(truck_id))
        for event_map in (solution.z1, solution.z2)
        for (node, drone_id, truck_id), value in event_map.items()
        if value > 0.5 and int(node) != 0
    }


def synchronization_handling_minutes(
    instance: InstanceData,
    solution: SolutionData,
    node: int,
    drone_id: int,
    truck_id: int,
) -> float:
    """Return additive handling time for one physical rendezvous."""
    rendezvous = 1.0 if (node, drone_id, truck_id) in active_rendezvous_events(solution) else 0.0
    reload = 1.0 if solution.z1.get((node, drone_id, truck_id), 0) > 0.5 else 0.0
    swap = 1.0 if solution.z2.get((node, drone_id, truck_id), 0) > 0.5 else 0.0
    handling_seconds = (
        2.0 * instance.constants.handling_time_s * rendezvous
        + instance.constants.reload_time_s * reload
        + instance.constants.swap_time_s * swap
    )
    return handling_seconds / 60.0


def compute_readiness_times(
    instance: InstanceData,
    solution: SolutionData,
    *,
    truck_arrivals: Mapping[Tuple[int, int], float] | None = None,
    drone_arrivals: Mapping[Tuple[int, int], float] | None = None,
) -> tuple[Dict[Tuple[int, int], float], Dict[Tuple[int, int], float]]:
    """Return post-service readiness times for the final schedule.

    A vehicle is ready to synchronize only after it reaches the node and
    completes the service assigned to that vehicle.  Stored lower-bound
    waiting variables are deliberately not used here.
    """
    truck_source = truck_arrivals if truck_arrivals is not None else solution.a_truck
    drone_source = drone_arrivals if drone_arrivals is not None else solution.a_drone
    readiness_truck: Dict[Tuple[int, int], float] = {}
    readiness_drone: Dict[Tuple[int, int], float] = {}
    for key, arrival in truck_source.items():
        truck_id, node = key
        service = instance.customer_map()[node].service_time_min if node != 0 else 0.0
        assigned = solution.u_truck.get(key, 0) >= 0.5
        readiness_truck[key] = float(arrival) + (service if assigned else 0.0)
    for key, arrival in drone_source.items():
        drone_id, node = key
        service = instance.customer_map()[node].service_time_min if node != 0 else 0.0
        assigned = solution.u_drone.get(key, 0) >= 0.5
        readiness_drone[key] = float(arrival) + (service if assigned else 0.0)
    return readiness_truck, readiness_drone


def compute_synchronization_waiting(
    instance: InstanceData,
    solution: SolutionData,
    *,
    truck_arrivals: Mapping[Tuple[int, int], float] | None = None,
    drone_arrivals: Mapping[Tuple[int, int], float] | None = None,
) -> dict[str, object]:
    """Recompute synchronization waiting from readiness times.

    The returned maps are per-vehicle/per-node diagnostics.  ``events`` keeps
    the directional waiting for each active parcel-reload or battery-swap
    event, which avoids confusing a stored lower bound with the realized
    schedule.
    """
    readiness_truck, readiness_drone = compute_readiness_times(
        instance,
        solution,
        truck_arrivals=truck_arrivals,
        drone_arrivals=drone_arrivals,
    )
    waiting_truck: Dict[Tuple[int, int], float] = {}
    waiting_drone: Dict[Tuple[int, int], float] = {}
    events: list[dict[str, float | int | str]] = []
    for node, drone_id, truck_id in active_rendezvous_events(solution):
        truck_ready = readiness_truck.get((truck_id, node))
        drone_ready = readiness_drone.get((drone_id, node))
        if truck_ready is None or drone_ready is None:
            continue
        truck_wait = max(0.0, float(drone_ready) - float(truck_ready))
        drone_wait = max(0.0, float(truck_ready) - float(drone_ready))
        waiting_truck[(truck_id, node)] = max(
            waiting_truck.get((truck_id, node), 0.0), truck_wait
        )
        waiting_drone[(drone_id, node)] = max(
            waiting_drone.get((drone_id, node), 0.0), drone_wait
        )
        events.append(
            {
                "event_type": "physical_rendezvous",
                "node": int(node),
                "truck_id": int(truck_id),
                "drone_id": int(drone_id),
                "truck_readiness": float(truck_ready),
                "drone_readiness": float(drone_ready),
                "truck_wait": truck_wait,
                "drone_wait": drone_wait,
                "handling_minutes": synchronization_handling_minutes(
                    instance, solution, node, drone_id, truck_id
                ),
                "total_wait": truck_wait + drone_wait,
            }
        )
    return {
        "readiness_truck": readiness_truck,
        "readiness_drone": readiness_drone,
        "waiting_truck": waiting_truck,
        "waiting_drone": waiting_drone,
        "events": events,
        "total_waiting_minutes": float(
            sum(float(event["total_wait"]) for event in events)
        ),
    }
