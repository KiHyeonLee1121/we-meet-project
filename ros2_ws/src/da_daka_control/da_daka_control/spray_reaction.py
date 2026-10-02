"""Pure spray-reaction feedforward physics and shaping helpers."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class OperatingPoint:
    """Steady-state pump/nozzle intersection point."""

    pressure_pa: float
    flow_m3s: float
    velocity_mps: float


def nozzle_area_m2(diameter_m: float) -> float:
    """Return the circular nozzle exit area."""
    if not math.isfinite(diameter_m) or diameter_m <= 0.0:
        raise ValueError('diameter_m must be finite and greater than zero')
    return math.pi / 4.0 * diameter_m * diameter_m


def solve_operating_point(
    pump_open_flow_m3s: float,
    pump_shutoff_pa: float,
    nozzle_area_m2_: float,
    discharge_coefficient: float,
    water_density_kgm3: float = 1000.0,
) -> OperatingPoint:
    """Solve the intersection of a linear pump curve and nozzle orifice."""
    values = (
        pump_open_flow_m3s,
        pump_shutoff_pa,
        nozzle_area_m2_,
        discharge_coefficient,
        water_density_kgm3,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError('operating-point inputs must be finite and positive')

    q_max = pump_open_flow_m3s
    p_max = pump_shutoff_pa
    nozzle_coefficient = (
        discharge_coefficient
        * nozzle_area_m2_
        * math.sqrt(2.0 * p_max / water_density_kgm3)
    )
    root_pressure_ratio = (
        -nozzle_coefficient
        + math.sqrt(
            nozzle_coefficient * nozzle_coefficient
            + 4.0 * q_max * q_max
        )
    ) / (2.0 * q_max)
    pressure_ratio = root_pressure_ratio * root_pressure_ratio
    pressure_pa = pressure_ratio * p_max
    flow_m3s = q_max * (1.0 - pressure_ratio)
    velocity_mps = flow_m3s / nozzle_area_m2_
    return OperatingPoint(pressure_pa, flow_m3s, velocity_mps)


def reaction_force_n(
    flow_m3s: float,
    velocity_mps: float,
    water_density_kgm3: float = 1000.0,
) -> float:
    """Return the momentum-flux reaction force ``rho * Q * velocity``."""
    values = (flow_m3s, velocity_mps, water_density_kgm3)
    if not all(math.isfinite(value) and value >= 0.0 for value in values):
        raise ValueError(
            'reaction-force inputs must be finite and non-negative'
        )
    if water_density_kgm3 <= 0.0:
        raise ValueError('water density must be greater than zero')
    return water_density_kgm3 * flow_m3s * velocity_mps


def apply_vertical_feedforward(
    base_speed_mps: float,
    feedforward_speed_mps: float,
    maximum_total_speed_mps: float,
) -> float:
    """Add feedforward while retaining an explicit total-speed bound."""
    values = (
        base_speed_mps,
        feedforward_speed_mps,
        maximum_total_speed_mps,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('feedforward inputs must be finite')
    if maximum_total_speed_mps <= 0.0:
        raise ValueError('maximum_total_speed_mps must be positive')
    combined = base_speed_mps + feedforward_speed_mps
    return max(
        -maximum_total_speed_mps,
        min(maximum_total_speed_mps, combined),
    )


class RampShaper:
    """Linearly ramp a zero-to-one level toward an on/off target."""

    def __init__(self, ramp_time_s: float) -> None:
        if not math.isfinite(ramp_time_s) or ramp_time_s < 0.0:
            raise ValueError('ramp_time_s must be finite and non-negative')
        self.ramp_time_s = ramp_time_s
        self.level = 0.0

    def reset(self) -> None:
        """Drop the shaped level back to zero."""
        self.level = 0.0

    def update(self, target_on: bool, dt_s: float) -> float:
        """Advance the shaped level and return its new value."""
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError('dt_s must be finite and greater than zero')
        target = 1.0 if target_on else 0.0
        if self.ramp_time_s <= 0.0:
            self.level = target
        else:
            maximum_step = dt_s / self.ramp_time_s
            if self.level < target:
                self.level = min(target, self.level + maximum_step)
            elif self.level > target:
                self.level = max(target, self.level - maximum_step)
        return self.level
