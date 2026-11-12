from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import Counter, defaultdict
from copy import deepcopy
from pathlib import Path
from typing import Any

from ..data_models import InstanceData, SolutionData
from ..drone_state import (
    analyze_drone_states,
    route_intersection_events,
    simulate_drone_states,
)
from ..feasibility import validate_solution
from ..heuristics.architecture import sortie_associations, validate_drone_architecture
from ..heuristics.nils import (
    _sync_route_assignments,
    default_drone_to_truck,
    evaluate_solution,
)
from ..instance_generator import load_instance_json
from ..parameters import build_search_config, load_yaml_config
from ..timing import (
    active_rendezvous_events,
    compute_synchronization_waiting,
    synchronization_handling_minutes,
)
from .heuristic_study import _as_levels, _build_scenarios, _run_method, _scenario_config


SAMPLE_INSTANCE_IDS = [
    "S0002_R07",
    "S0021_R06",
    "S0025_R02",
    "S0041_R02",
    "S0048_R01",
    "S0053_R05",
    "S0054_R04",
    "S0068_R08",
]
METHODS = {
    "paired_baseline": "M3",
    "no_unpairing": "M4",
    "nils": "M5",
    "feasibility_aware_alns": "M5",
}


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "item"):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    return value


def _read_backbone(path: Path) -> dict[int, list[int]]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    routes = {int(truck): [int(node) for node in route] for truck, route in raw["truck_routes"].items()}
    encoded = json.dumps(
        {str(truck): route for truck, route in sorted(routes.items())},
        sort_keys=True,
        separators=(",", ":"),
    )
    digest = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
    if raw.get("truck_routes_sha256") != digest:
        raise RuntimeError(f"Backbone hash mismatch: {path}")
    if raw.get("initialization_feasible") is not True:
        raise RuntimeError(f"Persisted backbone was not marked feasible: {path}")
    return routes


def _routes_sha256(routes: dict[int, list[int]]) -> str:
    encoded = json.dumps(
        {str(truck): route for truck, route in sorted(routes.items())},
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _route_intersection_events(
    instance: InstanceData,
    solution: SolutionData,
) -> set[tuple[int, int, int]]:
    return route_intersection_events(solution)


def _events_by_drone_node(events: set[tuple[int, int, int]]) -> dict[int, dict[int, list[tuple[int, int, int]]]]:
    result: dict[int, dict[int, list[tuple[int, int, int]]]] = defaultdict(lambda: defaultdict(list))
    for event in sorted(events):
        node, drone_id, _truck_id = event
        result[drone_id][node].append(event)
    return result


def _service_nodes(solution: SolutionData, drone_id: int) -> set[int]:
    return {
        int(customer)
        for (drone, customer), value in solution.u_drone.items()
        if int(drone) == drone_id and int(customer) != 0 and value >= 0.5
    }


def _truck_for_node(events_at_node: list[tuple[int, int, int]]) -> int | None:
    truck_ids = {event[2] for event in events_at_node}
    return next(iter(truck_ids)) if len(truck_ids) == 1 else None


def _derive_state_support_flags(
    instance: InstanceData,
    solution: SolutionData,
    events: set[tuple[int, int, int]],
) -> tuple[dict[tuple[int, int, int], int], dict[tuple[int, int, int], int], dict[str, list[str]]]:
    analysis = analyze_drone_states(instance, solution, events=events)
    findings: dict[str, list[str]] = {"parcel": [], "battery": [], "other": []}
    for issue in analysis.issues:
        category = "parcel" if issue.code.startswith("PARCEL") else "battery" if issue.code in {
            "ENERGY", "BATTERY_RESERVE", "BATTERY_SEGMENT"
        } else "other"
        findings[category].append(issue.message)
    return analysis.required_z1, analysis.required_z2, findings


def _support_trace(
    instance: InstanceData,
    solution: SolutionData,
    drone_id: int,
    events: set[tuple[int, int, int]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, list[str]]]:
    route = [int(node) for node in solution.drone_routes.get(drone_id, [0, 0])]
    services = _service_nodes(solution, drone_id)
    expected = analyze_drone_states(instance, solution, events=events)
    actual = simulate_drone_states(instance, solution)
    events_by_drone = _events_by_drone_node(events).get(drone_id, {})
    legs: list[dict[str, Any]] = []
    event_rows: list[dict[str, Any]] = []
    violations: dict[str, list[str]] = {"parcel": [], "battery": [], "orphan_support": []}
    for issue in expected.issues + actual.issues:
        group = (
            "parcel"
            if issue.code.startswith("PARCEL")
            else "orphan_support"
            if issue.code.startswith("UNMATCHED")
            else "battery"
        )
        if issue.drone_id == drone_id:
            violations[group].append(issue.message)

    for leg in actual.legs:
        if leg.drone_id != drone_id:
            continue
        legs.append(
            {
                "drone_id": int(drone_id),
                "leg_index": int(leg.leg_index),
                "from_node": int(leg.from_node),
                "to_node": int(leg.to_node),
                "cargo_state": "loaded" if leg.loaded else "empty",
                "battery_before_leg_wh": float(leg.battery_before_wh),
                "energy_wh": float(leg.energy_wh),
                "battery_after_arrival_wh": float(leg.battery_after_arrival_wh),
                "is_service_arrival": int(leg.to_node) in services,
                "is_rendezvous_arrival": int(leg.to_node) in events_by_drone,
            }
        )

    actual_events = {state.event_key: state for state in actual.rendezvous}
    for state in expected.rendezvous:
        event_key = state.event_key
        if event_key[1] != drone_id:
            continue
        actual_event = actual_events.get(event_key)
        actual_z1 = bool(solution.z1.get(event_key, 0) > 0.5)
        actual_z2 = bool(solution.z2.get(event_key, 0) > 0.5)
        event_truck = int(event_key[2])
        event_rows.append(
            {
                "node": int(event_key[0]),
                "drone_id": int(drone_id),
                "truck_id": event_truck,
                "event_key": list(event_key),
                "z1": actual_z1,
                "z1_required": bool(state.z1_required),
                "z1_reason": (
                    "required by parcel-state conservation"
                    if state.z1_required
                    else "not required by parcel state"
                ),
                "z2": actual_z2,
                "z2_required": bool(state.z2_required),
                "z2_reason": (
                    "required to cover the next rendezvous/depot segment"
                    if state.z2_required
                    else "not required: current battery covers the next segment"
                ),
                "battery_before_support_wh": float(
                    actual_event.battery_before_support_wh
                    if actual_event is not None
                    else state.battery_before_support_wh
                ),
                "battery_after_support_wh": float(
                    actual_event.battery_after_support_wh
                    if actual_event is not None
                    else state.battery_after_support_wh
                ),
                "energy_to_next_event_or_depot_wh": float(state.energy_to_next_event_or_depot_wh),
                "handling_minutes": synchronization_handling_minutes(
                    instance, solution, int(event_key[0]), int(drone_id), event_truck
                ),
            }
        )
    return legs, event_rows, violations


def _sortie_rows(solution: SolutionData, events: set[tuple[int, int, int]]) -> list[dict[str, Any]]:
    events_by_drone = _events_by_drone_node(events)
    rows: list[dict[str, Any]] = []
    for drone_id, raw_route in sorted(solution.drone_routes.items()):
        route = [int(node) for node in raw_route]
        services = _service_nodes(solution, int(drone_id))
        event_positions: dict[int, list[tuple[int, int, int]]] = defaultdict(list)
        for node, event_drone, truck_id in events:
            if event_drone == int(drone_id) and node in route:
                event_positions[route.index(node)].append((node, event_drone, truck_id))
        launch_node = 0
        launch_truck: int | None = None
        segment_start = 0
        segment_services: list[int] = []
        previous_recovery: tuple[int, int | None] | None = None
        sortie_index = 0

        def close_sortie(recovery_node: int, recovery_truck: int | None, end_pos: int) -> None:
            nonlocal sortie_index, segment_services, previous_recovery, launch_node, launch_truck, segment_start
            if not segment_services:
                return
            sortie_index += 1
            if launch_node == 0 and recovery_node == 0:
                sortie_type = "depot-depot"
            elif launch_node == 0:
                sortie_type = "depot-truck"
            elif recovery_node == 0:
                sortie_type = "truck-depot"
            elif launch_truck == recovery_truck:
                sortie_type = "same-truck"
            else:
                sortie_type = "cross-truck"
            launch = (launch_node, launch_truck)
            recovery = (recovery_node, recovery_truck)
            rows.append(
                {
                    "drone_id": int(drone_id),
                    "sortie_index": sortie_index,
                    "sortie_type": sortie_type,
                    "launch_node": int(launch_node),
                    "launch_truck": launch_truck,
                    "recovery_node": int(recovery_node),
                    "recovery_truck": recovery_truck,
                    "service_customers": list(segment_services),
                    "route_segment": route[segment_start : end_pos + 1],
                    "continuous_from_previous": previous_recovery is None or previous_recovery == launch,
                    "ambiguous_event_truck": any(
                        len({key[2] for key in keys}) > 1 for position, keys in event_positions.items()
                        if segment_start <= position <= end_pos
                    ),
                }
            )
            previous_recovery = recovery
            segment_services = []
            launch_node = int(recovery_node)
            launch_truck = recovery_truck
            segment_start = end_pos

        for position, node in enumerate(route[1:], start=1):
            if node in services:
                segment_services.append(int(node))
            if position in event_positions:
                keys = event_positions[position]
                event_truck = _truck_for_node(keys)
                close_sortie(int(node), event_truck, position)
                launch_node = int(node)
                launch_truck = event_truck
                segment_start = position
            elif node == 0:
                close_sortie(0, None, position)
                if position < len(route) - 1:
                    launch_node = 0
                    launch_truck = None
                    segment_start = position
        if segment_services:
            close_sortie(0, None, len(route) - 1)
    return rows


def _evaluate_solution_snapshot(
    instance: InstanceData,
    solution: SolutionData,
    architecture: str,
    pairing: dict[int, int] | None,
) -> dict[str, Any]:
    evaluated = deepcopy(solution)
    objective, components = evaluate_solution(instance, evaluated)
    findings = validate_solution(instance, deepcopy(evaluated))
    return {
        "objective": float(objective),
        "objective_components": _jsonable(components),
        "full_feasible": all(item.severity != "ERROR" for item in findings),
        "feasibility_findings": [
            {
                "severity": item.severity,
                "code": item.code,
                "message": item.message,
                "vehicle": item.vehicle,
                "node": item.node,
            }
            for item in findings
        ],
        "architecture": architecture,
        "architecture_violations": validate_drone_architecture(
            instance, evaluated, architecture, pairing
        ),
    }


def _support_counterfactual(
    instance: InstanceData,
    solution: SolutionData,
    events: set[tuple[int, int, int]],
    architecture: str,
    pairing: dict[int, int] | None,
) -> dict[str, Any]:
    z1, z2, state_findings = _derive_state_support_flags(instance, solution, events)
    variant = deepcopy(solution)
    variant.u_sync = {event: 1 for event in events}
    drone_ids = set(instance.drone_ids())
    old_z1 = {key for key, value in solution.z1.items() if value > 0.5 and key[1] in drone_ids}
    old_z2 = {key for key, value in solution.z2.items() if value > 0.5 and key[1] in drone_ids}
    variant.z1 = {key: value for key, value in variant.z1.items() if key[1] not in drone_ids}
    variant.z2 = {key: value for key, value in variant.z2.items() if key[1] not in drone_ids}
    variant.z1.update(z1)
    variant.z2.update(z2)
    objective, components = evaluate_solution(instance, variant)
    findings = validate_solution(instance, deepcopy(variant))
    violations = validate_drone_architecture(instance, variant, architecture, pairing)
    new_z1 = {key for key, value in z1.items() if value > 0.5}
    new_z2 = {key for key, value in z2.items() if value > 0.5}
    return {
        "objective": float(objective),
        "objective_components": _jsonable(components),
        "objective_delta": float(objective - solution.objective),
        "full_feasible": all(item.severity != "ERROR" for item in findings),
        "feasibility_errors": [
            f"{item.code}: {item.message}" for item in findings if item.severity == "ERROR"
        ],
        "architecture_violations": violations,
        "frozen_semantics_architecture_violations": _frozen_architecture_violations(
            instance, variant, events, architecture, pairing
        ),
        "z1_events": [list(key) for key in sorted(new_z1)],
        "z2_events": [list(key) for key in sorted(new_z2)],
        "z1_removed_count": len(old_z1 - new_z1),
        "z1_added_count": len(new_z1 - old_z1),
        "z2_removed_count": len(old_z2 - new_z2),
        "z2_added_count": len(new_z2 - old_z2),
        "state_findings": state_findings,
    }


def _frozen_architecture_violations(
    instance: InstanceData,
    solution: SolutionData,
    events: set[tuple[int, int, int]],
    architecture: str,
    pairing: dict[int, int] | None,
) -> list[str]:
    violations: list[str] = []
    pairing = pairing or {}
    event_set = set(events)
    for node, drone_id, truck_id in sorted(event_set):
        if node not in solution.drone_routes.get(drone_id, []):
            violations.append(f"rendezvous {node}/{drone_id}/{truck_id} is absent from the drone route")
        if node not in solution.truck_routes.get(truck_id, []):
            violations.append(f"rendezvous {node}/{drone_id}/{truck_id} is absent from the truck route")
        if architecture == "M3" and pairing.get(drone_id) != truck_id:
            violations.append(f"M3 rendezvous uses truck {truck_id}, not paired truck {pairing.get(drone_id)}")

    for name, state in (("z1", solution.z1), ("z2", solution.z2)):
        for (node, drone_id, truck_id), value in state.items():
            if value <= 0.5:
                continue
            key = (int(node), int(drone_id), int(truck_id))
            if key not in event_set:
                violations.append(f"{name} event {key} is not linked to a physical rendezvous")
            if architecture == "M3" and pairing.get(int(drone_id)) != int(truck_id):
                violations.append(
                    f"M3 {name} event uses truck {truck_id}, not paired truck {pairing.get(int(drone_id))}"
                )

    associations = sortie_associations(solution)
    by_drone: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for association in associations:
        by_drone[int(association["drone_id"])].append(association)
        launch_trucks = association["launch_trucks"]
        recovery_trucks = association["recovery_trucks"]
        if len(launch_trucks) > 1 or len(recovery_trucks) > 1:
            violations.append(
                f"drone {association['drone_id']} has ambiguous truck identity for customer {association['customer_id']}"
            )
        if architecture == "M3":
            for truck_id in (*launch_trucks, *recovery_trucks):
                if pairing.get(int(association["drone_id"])) != int(truck_id):
                    violations.append(
                        f"M3 sortie for customer {association['customer_id']} uses unpaired truck {truck_id}"
                    )
        if architecture == "M4" and launch_trucks and recovery_trucks:
            if int(launch_trucks[0]) != int(recovery_trucks[0]):
                violations.append(
                    f"M4 sortie for customer {association['customer_id']} recovers with a different truck"
                )

    for drone_id, drone_associations in by_drone.items():
        drone_associations.sort(key=lambda row: int(row["service_position"]))
        for previous, current in zip(drone_associations, drone_associations[1:]):
            previous_recovery = (int(previous["recovery_node"]), previous["recovery_truck"])
            next_launch = (int(current["launch_node"]), current["launch_truck"])
            if previous_recovery[0] == 0 or previous_recovery != next_launch:
                violations.append(
                    f"drone {drone_id} has discontinuous sortie sequence between customers "
                    f"{previous['customer_id']} and {current['customer_id']}"
                )
    return sorted(set(violations))


def _build_truck_only_reference(
    instance: InstanceData,
    routes: dict[int, list[int]],
) -> SolutionData:
    solution = SolutionData(
        instance_name=instance.name,
        status="truck_only_shared_backbone_reference",
        objective=0.0,
        components={},
        truck_routes=deepcopy(routes),
        drone_routes={drone_id: [0, 0] for drone_id in instance.drone_ids()},
    )
    _sync_route_assignments(solution, instance)
    solution.objective, solution.components = evaluate_solution(instance, solution)
    return solution


def _method_diagnostic(
    instance: InstanceData,
    method_name: str,
    architecture: str,
    pairing: dict[int, int] | None,
    solution: SolutionData,
) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    original = deepcopy(solution)
    original.objective, original.components = evaluate_solution(instance, original)
    exact_events = {
        event for event in active_rendezvous_events(original)
        if int(event[0]) != 0
    }
    intersection_events = _route_intersection_events(instance, original)
    waiting = compute_synchronization_waiting(instance, original)
    sorties = _sortie_rows(original, exact_events)
    legs: list[dict[str, Any]] = []
    rendezvous: list[dict[str, Any]] = []
    support_errors: dict[str, list[str]] = {"parcel": [], "battery": [], "orphan_support": []}
    for drone_id in instance.drone_ids():
        drone_legs, drone_events, issues = _support_trace(instance, original, drone_id, exact_events)
        legs.extend(drone_legs)
        rendezvous.extend(drone_events)
        for group, messages in issues.items():
            support_errors.setdefault(group, []).extend(messages)
    current_findings = validate_solution(instance, deepcopy(original))
    current_architecture_violations = validate_drone_architecture(
        instance, original, architecture, pairing
    )
    frozen_architecture_violations = _frozen_architecture_violations(
        instance, original, exact_events, architecture, pairing
    )
    support_only = _support_counterfactual(
        instance, original, exact_events, architecture, pairing
    )
    reconstructed_support = _support_counterfactual(
        instance, original, intersection_events, architecture, pairing
    )

    waiting_by_key = {
        (int(row["node"]), int(row["drone_id"]), int(row["truck_id"])): row
        for row in waiting["events"]
    }
    for event_row in rendezvous:
        event_key = tuple(int(part) for part in event_row["event_key"])
        matched = waiting_by_key.get(event_key)
        if matched:
            event_row["truck_readiness_minutes"] = matched["truck_readiness"]
            event_row["drone_readiness_minutes"] = matched["drone_readiness"]
            event_row["truck_wait_minutes"] = matched["truck_wait"]
            event_row["drone_wait_minutes"] = matched["drone_wait"]
            event_row["total_wait_minutes"] = matched["total_wait"]

    association_rows = sortie_associations(original)
    launches = {
        (int(row["launch_node"]), int(row["drone_id"]), int(row["launch_truck"]))
        for row in association_rows
        if int(row["launch_node"]) != 0 and row["launch_truck"] is not None
    }
    recoveries = {
        (int(row["recovery_node"]), int(row["drone_id"]), int(row["recovery_truck"]))
        for row in association_rows
        if int(row["recovery_node"]) != 0 and row["recovery_truck"] is not None
    }
    state_z1, state_z2, _ = _derive_state_support_flags(instance, original, exact_events)
    state_support_records = {
        "z1": [],
        "z2": [],
    }
    for label, existing, required in (
        ("z1", original.z1, state_z1),
        ("z2", original.z2, state_z2),
    ):
        for key, value in sorted(existing.items()):
            if value <= 0.5:
                continue
            normalized = tuple(int(part) for part in key)
            state_support_records[label].append(
                {
                    "event_key": list(normalized),
                    "active_rendezvous": normalized in exact_events,
                    "required_by_state": normalized in required,
                    "state_reason": (
                        "required by parcel conservation"
                        if label == "z1" and normalized in required
                        else "not required by parcel state"
                        if label == "z1"
                        else "required by battery reserve state"
                        if normalized in required
                        else "not required by battery state"
                    ),
                }
            )
    forced_z1 = []
    for key, value in original.z1.items():
        if value <= 0.5 or int(key[1]) not in instance.drone_ids():
            continue
        if key in launches and key not in state_z1:
            forced_z1.append([int(part) for part in key])
    forced_z2 = []
    for key, value in original.z2.items():
        if value <= 0.5 or int(key[1]) not in instance.drone_ids():
            continue
        if key in recoveries and key not in state_z2:
            forced_z2.append([int(part) for part in key])

    common_without_event = sorted(intersection_events - exact_events)
    explicit_without_common = sorted(exact_events - intersection_events)
    summary = {
        "instance_id": instance.name,
        "method": method_name,
        "architecture": architecture,
        "status": str(original.status),
        "reported_objective": float(solution.objective),
        "recomputed_objective": float(original.objective),
        "objective_components": _jsonable(original.components),
        "full_feasible": all(item.severity != "ERROR" for item in current_findings),
        "feasibility_errors": [
            f"{item.code}: {item.message}" for item in current_findings if item.severity == "ERROR"
        ],
        "architecture_violations": current_architecture_violations,
        "frozen_semantics_architecture_violations": frozen_architecture_violations,
        "run_time_seconds": float(original.run_time_seconds),
        "iterations": float(original.components.get("iterations", 0.0)),
        "drone_assignments": len(original.served_by_drone()),
        "explicit_u_sync_events": [list(event) for event in sorted(exact_events)],
        "route_intersection_u_sync_events": [list(event) for event in sorted(intersection_events)],
        "intersection_events_without_explicit_u_sync": [list(event) for event in common_without_event],
        "explicit_u_sync_without_route_intersection": [list(event) for event in explicit_without_common],
        "num_z1_events": sum(value > 0.5 for value in original.z1.values()),
        "num_z2_events": sum(value > 0.5 for value in original.z2.values()),
        "forced_launch_label_z1_count": len(forced_z1),
        "forced_recovery_label_z2_count": len(forced_z2),
        "forced_z1_keys": forced_z1,
        "forced_z2_keys": forced_z2,
        "support_event_state_audit": state_support_records,
        "state_correction": {
            "support_only_counterfactual": support_only,
            "reconstructed_u_sync_counterfactual": reconstructed_support,
        },
        "total_sync_wait_minutes": float(waiting["total_waiting_minutes"]),
        "rendezvous_events": rendezvous,
        "sorties": sorties,
        "drone_legs": legs,
        "support_state_findings": support_errors,
    }
    return summary, rendezvous, legs


def run_diagnostic(
    repo: Path,
    config_path: Path,
    output_dir: Path,
    instance_ids: list[str],
) -> dict[str, Any]:
    source_output = config_path.parent
    raw_dir = source_output / "raw"
    backbone_dir = raw_dir / "initial_truck_backbones"
    config = build_search_config(load_yaml_config(config_path))
    scenarios = _build_scenarios(_as_levels(config), config)
    scenario_by_id = {str(row["scenario_id"]): row for row in scenarios}
    output_dir.mkdir(parents=True, exist_ok=True)
    checkpoint_path = output_dir / "diagnostic_checkpoint.json"
    if checkpoint_path.is_file():
        checkpoint = json.loads(checkpoint_path.read_text(encoding="utf-8"))
    else:
        checkpoint = {
            "method_summaries": [],
            "sortie_records": [],
            "rendezvous_records": [],
            "support_event_records": [],
            "drone_leg_records": [],
            "truck_only_references": [],
            "truck_only_constructor_audits": [],
        }
    summaries = checkpoint["method_summaries"]
    all_sorties = checkpoint["sortie_records"]
    all_events = checkpoint["rendezvous_records"]
    all_support_events = checkpoint.setdefault("support_event_records", [])
    all_legs = checkpoint["drone_leg_records"]
    truck_only_rows = checkpoint["truck_only_references"]
    truck_only_constructor_rows = checkpoint.setdefault("truck_only_constructor_audits", [])
    completed_runs = {
        (row["instance_id"], row["method"]) for row in summaries
    }
    completed_truck_refs = {row["instance_id"] for row in truck_only_rows}
    completed_constructor_audits = {row["instance_id"] for row in truck_only_constructor_rows}

    historical_path = repo / "outputs/full_revised_usync_20261101_paper_ready/paper_method_rows_partial.csv"
    historical_rows = {}
    if historical_path.exists():
        with historical_path.open(newline="", encoding="utf-8-sig") as handle:
            historical_rows = {
                row["instance_id"]: row
                for row in csv.DictReader(handle)
                if row.get("method_name") == "truck_only"
            }

    for instance_id in instance_ids:
        instance_path = raw_dir / f"{instance_id}.json"
        backbone_path = backbone_dir / f"{instance_id}.json"
        if not instance_path.is_file() or not backbone_path.is_file():
            raise FileNotFoundError(f"Missing frozen instance or backbone for {instance_id}")
        instance = load_instance_json(instance_path)
        backbone = _read_backbone(backbone_path)
        scenario_id = str(instance.metadata.get("scenario_id", ""))
        if scenario_id not in scenario_by_id:
            raise RuntimeError(f"Scenario {scenario_id} for {instance_id} is absent from the frozen config")
        scenario_config = _scenario_config(config, scenario_by_id[scenario_id])
        truck_reference = _build_truck_only_reference(instance, backbone)
        truck_findings = validate_solution(instance, deepcopy(truck_reference))
        old_row = historical_rows.get(instance_id, {})
        if instance_id not in completed_truck_refs:
            truck_only_rows.append(
            {
                "instance_id": instance_id,
                "n": instance.num_customers,
                "backbone_sha256": json.loads(backbone_path.read_text(encoding="utf-8"))["truck_routes_sha256"],
                "current_backbone_objective": float(truck_reference.objective),
                "current_backbone_components": _jsonable(truck_reference.components),
                "current_backbone_feasible": all(item.severity != "ERROR" for item in truck_findings),
                "current_backbone_errors": [
                    f"{item.code}: {item.message}" for item in truck_findings if item.severity == "ERROR"
                ],
                "historical_truck_only_objective": float(old_row["objective_total"]) if old_row.get("objective_total") else None,
                "historical_truck_only_status": old_row.get("status"),
                "historical_truck_only_feasible": old_row.get("final_solution_feasibility_flag"),
                "backbone_minus_historical_objective": (
                    float(truck_reference.objective) - float(old_row["objective_total"])
                    if old_row.get("objective_total") else None
                ),
            }
            )
            completed_truck_refs.add(instance_id)
            checkpoint_path.write_text(json.dumps(_jsonable(checkpoint), indent=2) + "\n", encoding="utf-8")
        if instance_id not in completed_constructor_audits:
            constructor_solution = _run_method(instance, scenario_config, "truck_only")
            constructor_findings = validate_solution(instance, deepcopy(constructor_solution))
            constructor_routes_sha256 = _routes_sha256(constructor_solution.truck_routes)
            truck_only_constructor_rows.append(
                {
                    "instance_id": instance_id,
                    "n": instance.num_customers,
                    "current_constructor_objective": float(constructor_solution.objective),
                    "current_constructor_feasible": all(
                        item.severity != "ERROR" for item in constructor_findings
                    ),
                    "current_constructor_status": str(constructor_solution.status),
                    "current_constructor_routes_sha256": constructor_routes_sha256,
                    "matches_shared_initial_backbone": constructor_routes_sha256
                    == json.loads(backbone_path.read_text(encoding="utf-8"))["truck_routes_sha256"],
                    "historical_truck_only_objective": float(old_row["objective_total"])
                    if old_row.get("objective_total")
                    else None,
                    "historical_truck_only_feasible": old_row.get(
                        "final_solution_feasibility_flag"
                    ),
                    "constructor_minus_historical_objective": (
                        float(constructor_solution.objective) - float(old_row["objective_total"])
                        if old_row.get("objective_total")
                        else None
                    ),
                }
            )
            completed_constructor_audits.add(instance_id)
            checkpoint_path.write_text(json.dumps(_jsonable(checkpoint), indent=2) + "\n", encoding="utf-8")
        for method_name, architecture in METHODS.items():
            if (instance_id, method_name) in completed_runs:
                continue
            shared_backbone = backbone if method_name in {"paired_baseline", "no_unpairing", "nils"} else None
            solution = _run_method(
                instance,
                scenario_config,
                method_name,
                initial_truck_routes=shared_backbone,
            )
            pairing = default_drone_to_truck(instance) if architecture == "M3" else None
            summary, events, legs = _method_diagnostic(
                instance, method_name, architecture, pairing, solution
            )
            summary["shared_backbone_hash"] = json.loads(
                backbone_path.read_text(encoding="utf-8")
            )["truck_routes_sha256"] if shared_backbone is not None else None
            summary["initial_backbone_reused"] = bool(
                solution.components.get("initial_backbone_reused", 0.0) >= 0.5
            ) if shared_backbone is not None else False
            summaries.append(summary)
            for row in summary["sorties"]:
                all_sorties.append({"instance_id": instance_id, "method": method_name, **row})
            for row in events:
                all_events.append({"instance_id": instance_id, "method": method_name, **row})
            for event_type, records in summary["support_event_state_audit"].items():
                for row in records:
                    all_support_events.append(
                        {"instance_id": instance_id, "method": method_name, "event_type": event_type, **row}
                    )
            for row in legs:
                all_legs.append({"instance_id": instance_id, "method": method_name, **row})
            completed_runs.add((instance_id, method_name))
            checkpoint_path.write_text(json.dumps(_jsonable(checkpoint), indent=2) + "\n", encoding="utf-8")
            print(f"completed {instance_id} {method_name}", flush=True)

    alns_summaries = [row for row in summaries if row["method"] == "feasibility_aware_alns"]
    alns_sortie_types = Counter(
        row["sortie_type"]
        for row in all_sorties
        if row["method"] == "feasibility_aware_alns"
    )
    reconstructed_validation_failures = [
        row
        for row in summaries
        if row["state_correction"]["reconstructed_u_sync_counterfactual"]["frozen_semantics_architecture_violations"]
        or not row["state_correction"]["reconstructed_u_sync_counterfactual"]["full_feasible"]
    ]
    source_commit = __import__("subprocess").check_output(
        ["git", "rev-parse", "HEAD"], cwd=repo, text=True
    ).strip()
    config_sha256 = hashlib.sha256(config_path.read_bytes()).hexdigest()
    script_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    input_hashes = []
    for instance_id in instance_ids:
        instance_path = raw_dir / f"{instance_id}.json"
        backbone_path = backbone_dir / f"{instance_id}.json"
        input_hashes.append(
            {
                "instance_id": instance_id,
                "instance_sha256": hashlib.sha256(instance_path.read_bytes()).hexdigest(),
                "backbone_file_sha256": hashlib.sha256(backbone_path.read_bytes()).hexdigest(),
                "backbone_routes_sha256": json.loads(
                    backbone_path.read_text(encoding="utf-8")
                )["truck_routes_sha256"],
            }
        )
    nils_summaries = [row for row in summaries if row["method"] in {"paired_baseline", "no_unpairing", "nils"}]
    support_deltas = [
        row["state_correction"]["support_only_counterfactual"]["objective_delta"]
        for row in summaries
    ]
    support_feasibility_changes = [
        row for row in summaries
        if row["full_feasible"] != row["state_correction"]["support_only_counterfactual"]["full_feasible"]
    ]
    total_alns_sorties = sum(alns_sortie_types.values())
    support_counterfactual_rows = []
    for row in summaries:
        counterfactual = row["state_correction"]["support_only_counterfactual"]
        support_counterfactual_rows.append(
            {
                "instance_id": row["instance_id"],
                "method": row["method"],
                "objective_before": row["recomputed_objective"],
                "objective_after_state_support": counterfactual["objective"],
                "objective_delta": counterfactual["objective_delta"],
                "feasible_before": row["full_feasible"],
                "feasible_after": counterfactual["full_feasible"],
                "forced_launch_label_z1_removed": row["forced_launch_label_z1_count"],
                "forced_recovery_label_z2_removed": row["forced_recovery_label_z2_count"],
                "current_validator_counterfactual_violations": len(counterfactual["architecture_violations"]),
                "frozen_semantics_counterfactual_violations": len(
                    counterfactual["frozen_semantics_architecture_violations"]
                ),
            }
        )

    output_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "execution_code_commit": source_commit,
        "diagnostic_source_commit": __import__("subprocess").check_output(
            ["git", "rev-parse", "HEAD"], cwd=repo, text=True
        ).strip(),
        "configuration": str(config_path),
        "configuration_sha256": config_sha256,
        "diagnostic_script_sha256": script_sha256,
        "run_limits": {
            "nils_seconds": config.heuristics.time_limit_seconds,
            "nils_max_iter": config.heuristics.max_outer_iter,
            "nils_max_no_improve": config.heuristics.max_no_improve,
            "max_displacement_candidates": config.heuristics.max_displacement_candidates,
            "max_repair_candidates": config.heuristics.max_repair_candidates,
            "alns_seconds": config.experiment.get("alns_time_limit_seconds"),
            "alns_iterations": config.experiment.get("alns_max_iter"),
        },
        "selected_instances": instance_ids,
        "input_hashes": input_hashes,
        "selection_note": "Two event-rich frozen instances per customer size, stratified by eligible share (0.25/0.75); purposive for operational-state diagnostics, not a performance sample.",
        "method_summaries": summaries,
        "alns_architecture_validator_passes": sum(not row["architecture_violations"] for row in alns_summaries),
        "alns_solutions_tested": len(alns_summaries),
        "every_alns_solution_passes_current_m5_architecture_validator": (
            len(alns_summaries) > 0
            and all(not row["architecture_violations"] for row in alns_summaries)
        ),
        "alns_sortie_type_distribution": dict(sorted(alns_sortie_types.items())),
        "alns_sortie_type_share": {
            key: value / total_alns_sorties for key, value in sorted(alns_sortie_types.items())
        } if total_alns_sorties else {},
        "state_support_counterfactual": {
            "objective_changed_runs": sum(abs(float(value)) > 1e-8 for value in support_deltas),
            "objective_delta_min": min(support_deltas, default=0.0),
            "objective_delta_max": max(support_deltas, default=0.0),
            "objective_delta_mean": sum(support_deltas) / len(support_deltas) if support_deltas else 0.0,
            "feasibility_changed_runs": len(support_feasibility_changes),
            "feasibility_changed_instances": [
                {"instance_id": row["instance_id"], "method": row["method"]}
                for row in support_feasibility_changes
            ],
            "nils_family_forced_z1_events": sum(row["forced_launch_label_z1_count"] for row in nils_summaries),
            "nils_family_forced_z2_events": sum(row["forced_recovery_label_z2_count"] for row in nils_summaries),
        },
        "reconstructed_u_sync_invalid_or_discontinuous_runs": len(reconstructed_validation_failures),
        "reconstructed_u_sync_invalid_or_discontinuous_examples": [
            {
                "instance_id": row["instance_id"],
                "method": row["method"],
                "architecture_violations": row["state_correction"]["reconstructed_u_sync_counterfactual"]["frozen_semantics_architecture_violations"],
                "feasibility_errors": row["state_correction"]["reconstructed_u_sync_counterfactual"]["feasibility_errors"],
            }
            for row in reconstructed_validation_failures
        ],
        "truck_only_references": truck_only_rows,
        "truck_only_constructor_audits": truck_only_constructor_rows,
        "sortie_records": all_sorties,
        "rendezvous_records": all_events,
        "support_event_records": all_support_events,
        "drone_leg_records": all_legs,
    }
    json_path = output_dir / "event_semantics_diagnostic.json"
    json_path.write_text(json.dumps(_jsonable(payload), indent=2) + "\n", encoding="utf-8")
    for filename, rows in [
        ("method_summary.csv", summaries),
        ("sorties.csv", all_sorties),
        ("rendezvous_events.csv", all_events),
        ("support_events.csv", all_support_events),
        ("support_counterfactuals.csv", support_counterfactual_rows),
        ("drone_legs.csv", all_legs),
        ("truck_only_backbone_reference.csv", truck_only_rows),
        ("truck_only_constructor_audit.csv", truck_only_constructor_rows),
    ]:
        if not rows:
            continue
        columns = list(dict.fromkeys(key for row in rows for key in row))
        with (output_dir / filename).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(_jsonable(rows))
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--repo",
        type=Path,
        default=Path(__file__).resolve().parents[2],
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("outputs/full_m3_m4_m5_alns_720_frozen_20261106/run_config.yaml"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("outputs/targeted_event_semantics_diagnostic_20261112"),
    )
    parser.add_argument("--instances", nargs="*", default=SAMPLE_INSTANCE_IDS)
    args = parser.parse_args()
    repo = args.repo.resolve()
    config_path = args.config if args.config.is_absolute() else repo / args.config
    output_dir = args.output if args.output.is_absolute() else repo / args.output
    payload = run_diagnostic(repo, config_path, output_dir, list(args.instances))
    failing = [
        row for row in payload["method_summaries"]
        if not row["full_feasible"] or row["architecture_violations"]
    ]
    nils_rows = [row for row in payload["method_summaries"] if row["method"] != "feasibility_aware_alns"]
    forced_z1 = sum(row["forced_launch_label_z1_count"] for row in nils_rows)
    forced_z2 = sum(row["forced_recovery_label_z2_count"] for row in nils_rows)
    print(
        json.dumps(
            {
                "instances": len(payload["selected_instances"]),
                "method_runs": len(payload["method_summaries"]),
                "runs_with_feasibility_or_architecture_findings": len(failing),
                "forced_z1_launch_events": forced_z1,
                "forced_z2_recovery_events": forced_z2,
                "output": str(args.output),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
