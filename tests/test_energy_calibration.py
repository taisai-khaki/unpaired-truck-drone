from __future__ import annotations

import math

from src.energy import (
    AerodynamicCalibration,
    battery_for_range_wh,
    derive_drone_energy_rates,
    kmh_to_mps,
)
from src.instance_generator import InstanceGenerator
from src.parameters import DroneSpec, GenerationSettings, SearchConfig, VehicleCatalog


def test_aerodynamic_preprocessing_converts_speed_and_units() -> None:
    assert math.isclose(kmh_to_mps(65.0), 65.0 / 3.6)
    empty_rate, loaded_rate, provenance = derive_drone_energy_rates(65.0, 3.0, 2.0)
    assert math.isclose(provenance["empty_power_w"], 512.4642207847554, rel_tol=1e-12)
    assert math.isclose(provenance["loaded_power_w"], 1003.3374567964224, rel_tol=1e-12)
    assert math.isclose(empty_rate, 8.54107034641259, rel_tol=1e-12)
    assert math.isclose(loaded_rate, 16.72229094660704, rel_tol=1e-12)
    assert provenance["power_unit"] == "W"
    assert provenance["energy_rate_unit"] == "Wh/min"


def test_range_calibration_is_explicit_and_not_used_as_battery_default() -> None:
    _, loaded_rate, provenance = derive_drone_energy_rates(65.0, 3.0, 2.0)
    loaded_power = float(provenance["loaded_power_w"])
    assert math.isclose(
        battery_for_range_wh(20.0, 65.0, loaded_power),
        308.7192174755244,
        rel_tol=1e-12,
    )
    assert loaded_rate > 0.0


def test_generated_instances_persist_aerodynamic_provenance() -> None:
    config = SearchConfig(
        generation=GenerationSettings(
            num_customers=4,
            num_trucks=2,
            num_drones=1,
        ),
        vehicles=VehicleCatalog(
            drone=DroneSpec(
                speed_kmph=65.0,
                capacity_kg=5.0,
                max_battery_wh=1200.0,
                curb_weight_kg=3.0,
                parcel_weight_kg=2.0,
            )
        ),
    )
    instance = InstanceGenerator.from_search_config(config).generate_single(11, "energy_test")
    drone = instance.drones[0]
    assert math.isclose(drone.energy_per_min_when_empty, 8.54107034641259, rel_tol=1e-12)
    assert math.isclose(drone.energy_per_min_when_loaded, 16.72229094660704, rel_tol=1e-12)
    assert instance.metadata["model"] == "appendix_a3_aerodynamic_preprocessing"
    assert instance.metadata["speed_formula_unit"] == "m/s"
