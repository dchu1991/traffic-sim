from __future__ import annotations

import tomllib
from dataclasses import dataclass, field


@dataclass
class LaneChangeConfig:
    cooldown_s: float = 3.0       # minimum seconds between lane changes
    incentive_m: float = 8.0      # gap improvement required to move LEFT (overtake)
    safety_gap_m: float = 6.0     # minimum gap behind in target lane
    keep_right_gap_m: float = 25.0  # gap needed to merge RIGHT; 0 = disabled
    duration_s: float = 1.2       # visual transition duration (cosmetic)


@dataclass
class RampConfig:
    onramp_position: float = 0.10   # fraction of road length
    offramp_position: float = 0.80
    onramp_rate: float = 0.5        # cars / second
    offramp_prob: float = 0.30      # probability a passing car exits
    min_gap_m: float = 30.0         # safety gap (m) required in front and behind to merge
    merge_window_m: float = 100.0   # how far back from the tip a car may attempt to merge
    zipper_speed_kmh: float = 30.0  # activate zipper mode when rightmost lane avg speed < this
    zipper_gap_m: float = 8.0       # gap required in zipper mode (~1 car length + buffer)
    max_queue: int = 10             # max cars waiting on ramp (0 = no limit)
    ramp_length_m: float = 200.0    # physical ramp length in metres
    target_cars: int = 0            # 0 = disabled; controller adjusts offramp_prob to hit this count
    offramp_control_gain: float = 0.001  # prob per excess car per second (proportional gain)
    onramp_control_gain: float = 0.001   # rate (cars/s) per excess car per second


@dataclass
class CarSpawnConfig:
    desired_v_mean_ms: float = 33.0
    desired_v_std_ms: float = 4.0
    desired_v_min_ms: float = 22.0
    desired_v_max_ms: float = 44.0
    time_headway_mean: float = 1.5
    time_headway_std: float = 0.3
    min_gap_mean_m: float = 2.0
    max_accel_mean: float = 1.5
    comfortable_decel_mean: float = 2.0


@dataclass
class TruckSpawnConfig:
    desired_v_mean_ms: float = 22.0
    desired_v_std_ms: float = 2.0
    desired_v_min_ms: float = 15.0
    desired_v_max_ms: float = 27.0
    length_min_m: float = 10.0
    length_max_m: float = 16.0


@dataclass
class DestinationConfig:
    enabled: bool = False           # False = classic probabilistic offramp_prob behavior
    min_loops: int = 5              # every car completes at least this many laps
    loops_lambda: float = 3.0       # Poisson(λ) extra laps; mean destination = min_loops + λ
    exit_lookahead_m: float = 300.0  # metres before off-ramp where car commits to right lane


@dataclass
class SimConfig:
    # Speed limits in km/h, index 0 = leftmost (fast) lane
    lane_speed_limits_kmh: list[float] = field(
        default_factory=lambda: [130.0, 110.0, 90.0]
    )
    lane_change: LaneChangeConfig = field(default_factory=LaneChangeConfig)
    ramp: RampConfig = field(default_factory=RampConfig)
    cars: CarSpawnConfig = field(default_factory=CarSpawnConfig)
    trucks: TruckSpawnConfig = field(default_factory=TruckSpawnConfig)
    destination: DestinationConfig = field(default_factory=DestinationConfig)

    # ------------------------------------------------------------------

    @classmethod
    def from_toml(cls, path: str) -> SimConfig:
        with open(path, "rb") as f:
            data = tomllib.load(f)
        cfg = cls()
        if "road" in data:
            cfg.lane_speed_limits_kmh = data["road"].get(
                "lane_speed_limits_kmh", cfg.lane_speed_limits_kmh
            )
        if "lane_change" in data:
            cfg.lane_change = LaneChangeConfig(**data["lane_change"])
        if "ramp" in data:
            cfg.ramp = RampConfig(**data["ramp"])
        if "cars" in data:
            cfg.cars = CarSpawnConfig(**data["cars"])
        if "trucks" in data:
            cfg.trucks = TruckSpawnConfig(**data["trucks"])
        if "destination" in data:
            cfg.destination = DestinationConfig(**data["destination"])
        return cfg

    def speed_limits_ms(self, num_lanes: int) -> list[float]:
        """Return per-lane speed limits in m/s, adjusted for actual lane count."""
        limits_kmh = self.lane_speed_limits_kmh
        if len(limits_kmh) >= num_lanes:
            return [v / 3.6 for v in limits_kmh[:num_lanes]]
        # Extend by repeating the slowest limit for any extra lanes
        base, step = limits_kmh[0], (limits_kmh[0] - limits_kmh[-1]) / max(len(limits_kmh) - 1, 1)
        extended = list(limits_kmh)
        while len(extended) < num_lanes:
            extended.append(extended[-1] - step)
        return [v / 3.6 for v in extended]
