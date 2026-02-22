from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class Car:
    car_id: int
    lane: int
    position: float          # meters along road
    velocity: float          # m/s
    color: tuple[int, int, int]

    # IDM parameters — randomized per car for driver heterogeneity
    desired_velocity: float  # m/s
    time_headway: float      # s
    min_gap: float           # m
    max_accel: float         # m/s²
    comfortable_decel: float # m/s²
    accel_exponent: float = 4.0

    length: float = 4.5      # m (trucks are longer, set at spawn)

    # Runtime state
    acceleration: float = 0.0
    lane_change_timer: float = 0.0  # cooldown before next lane change
    exiting: bool = False           # set when car is committed to an off-ramp
    _passed_ramps: set = field(default_factory=set)

    def idm_acceleration(self, gap: float, lead_velocity: float,
                         effective_v0: float | None = None) -> float:
        """Intelligent Driver Model — returns acceleration given gap to leader.

        Pass effective_v0 to override desired_velocity (e.g. to enforce a speed limit).
        """
        v = self.velocity
        v0 = effective_v0 if effective_v0 is not None else self.desired_velocity
        delta_v = v - lead_velocity

        # Desired minimum gap
        s_star = (
            self.min_gap
            + max(0.0, v * self.time_headway
                  + v * delta_v / (2.0 * math.sqrt(self.max_accel * self.comfortable_decel)))
        )

        accel = self.max_accel * (
            1.0
            - (v / v0) ** self.accel_exponent
            - (s_star / max(gap, 0.1)) ** 2
        )
        return max(-9.0, min(self.max_accel, accel))

    def update(self, dt: float, gap: float, lead_velocity: float,
               road_length: float, speed_limit: float = float('inf')) -> None:
        effective_v0 = min(self.desired_velocity, speed_limit)
        self.acceleration = self.idm_acceleration(gap, lead_velocity, effective_v0)
        self.velocity = max(0.0, self.velocity + self.acceleration * dt)
        self.position = (self.position + self.velocity * dt) % road_length
        if self.lane_change_timer > 0.0:
            self.lane_change_timer = max(0.0, self.lane_change_timer - dt)
