"""Aerodynamic drone-energy preprocessing and unit conversions."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class AerodynamicCalibration:
    """Fixed coefficients used to derive empty and loaded flight power."""

    c1: float = 2.8037
    c2: float = 0.3177
    c4: float = 0.0296
    c5: float = 0.0279
    gravity_m_per_s2: float = 9.8
    alpha_deg: float = 10.0

    def __post_init__(self) -> None:
        for name in ("c1", "c2", "c4", "c5", "gravity_m_per_s2"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name} must be non-negative")
        if not 0.0 <= self.alpha_deg < 90.0:
            raise ValueError("alpha_deg must be in [0, 90)")


def kmh_to_mps(speed_kmph: float) -> float:
    """Convert speed from km/h to m/s for the aerodynamic formula."""
    if speed_kmph <= 0:
        raise ValueError("speed_kmph must be positive")
    return float(speed_kmph) / 3.6


def aerodynamic_power_w(
    speed_kmph: float,
    mass_kg: float,
    calibration: AerodynamicCalibration | None = None,
) -> float:
    """Return aerodynamic flight power in watts.

    This is the preprocessing expression used by Appendix A.3.  The supplied
    speed is converted to m/s before it is substituted into the expression.
    """
    if mass_kg <= 0:
        raise ValueError("mass_kg must be positive")
    calibration = calibration or AerodynamicCalibration()
    speed_mps = kmh_to_mps(speed_kmph)
    alpha = math.radians(calibration.alpha_deg)
    lift_term = (
        calibration.gravity_m_per_s2 * mass_kg
        - calibration.c5 * (speed_mps * math.cos(alpha)) ** 2
    )
    transverse_term = calibration.c4 * speed_mps**2
    induced_power = (calibration.c1 + calibration.c2) * (
        lift_term**2 + transverse_term**2
    ) ** 0.75
    profile_power = calibration.c4 * speed_mps**3
    return float(induced_power + profile_power)


def power_w_to_energy_wh_per_min(power_w: float) -> float:
    """Convert watts (joules/second) to watt-hours per minute."""
    if power_w < 0:
        raise ValueError("power_w must be non-negative")
    return float(power_w) / 60.0


def derive_drone_energy_rates(
    speed_kmph: float,
    curb_weight_kg: float,
    parcel_weight_kg: float,
    calibration: AerodynamicCalibration | None = None,
) -> tuple[float, float, dict[str, float | str]]:
    """Return empty/loaded Wh-per-minute rates and calibration provenance."""
    calibration = calibration or AerodynamicCalibration()
    empty_power_w = aerodynamic_power_w(speed_kmph, curb_weight_kg, calibration)
    loaded_power_w = aerodynamic_power_w(
        speed_kmph,
        curb_weight_kg + parcel_weight_kg,
        calibration,
    )
    empty_rate = power_w_to_energy_wh_per_min(empty_power_w)
    loaded_rate = power_w_to_energy_wh_per_min(loaded_power_w)
    provenance: dict[str, float | str] = {
        "model": "appendix_a3_aerodynamic_preprocessing",
        "speed_input_unit": "km/h",
        "speed_formula_unit": "m/s",
        "power_unit": "W",
        "energy_rate_unit": "Wh/min",
        "c1": calibration.c1,
        "c2": calibration.c2,
        "c4": calibration.c4,
        "c5": calibration.c5,
        "gravity_m_per_s2": calibration.gravity_m_per_s2,
        "alpha_deg": calibration.alpha_deg,
        "speed_kmph": float(speed_kmph),
        "empty_mass_kg": float(curb_weight_kg),
        "loaded_mass_kg": float(curb_weight_kg + parcel_weight_kg),
        "empty_power_w": empty_power_w,
        "loaded_power_w": loaded_power_w,
        "empty_energy_wh_per_min": empty_rate,
        "loaded_energy_wh_per_min": loaded_rate,
    }
    return empty_rate, loaded_rate, provenance


def battery_for_range_wh(
    range_km: float,
    speed_kmph: float,
    power_w: float,
) -> float:
    """Return the battery energy for a stated one-mode flight range."""
    if range_km < 0:
        raise ValueError("range_km must be non-negative")
    if speed_kmph <= 0:
        raise ValueError("speed_kmph must be positive")
    if power_w < 0:
        raise ValueError("power_w must be non-negative")
    flight_hours = float(range_km) / float(speed_kmph)
    return float(power_w * flight_hours)
