from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .car import Car

LARGE_GAP = 1e6  # sentinel when no leader / follower exists


@dataclass
class Ramp:
    position: float   # m along road
    lane: int         # which lane the ramp connects to
    is_onramp: bool
    # on-ramp: spawn rate in cars/second
    # off-ramp: probability per meter that a passing car exits (≈ fraction of cars exiting)
    rate: float
    _timer: float = 0.0  # accumulates time between spawns (on-ramps only)
    queue: list = field(default_factory=list)  # cars waiting to merge (on-ramp only)


class Road:
    def __init__(self, length: float = 1000.0, num_lanes: int = 3,
                 lane_speed_limits: list[float] | None = None):
        self.length = length
        self.num_lanes = num_lanes
        self.lanes: list[list[Car]] = [[] for _ in range(num_lanes)]
        self.ramps: list[Ramp] = []
        # Speed limits in m/s, index 0 = leftmost (fast) lane.
        # Caller is responsible for supplying the right length; defaults to no limit.
        self.lane_speed_limits: list[float] = (
            lane_speed_limits if lane_speed_limits is not None
            else [float('inf')] * num_lanes
        )

    # ------------------------------------------------------------------
    # Car management
    # ------------------------------------------------------------------

    def add_car(self, car: Car) -> None:
        self.lanes[car.lane].append(car)

    def remove_car(self, car: Car) -> None:
        self.lanes[car.lane].remove(car)

    def sorted_lane(self, lane: int) -> list[Car]:
        return sorted(self.lanes[lane], key=lambda c: c.position)

    # ------------------------------------------------------------------
    # Gap queries (all handle circular/wraparound road)
    # ------------------------------------------------------------------

    def find_leader(self, car: Car) -> tuple[float, float]:
        """Return (gap_to_leader, leader_velocity) in the car's current lane."""
        lane_cars = self.sorted_lane(car.lane)
        others = [c for c in lane_cars if c is not car]
        if not others:
            return LARGE_GAP, car.desired_velocity

        # Cars strictly ahead (higher position)
        ahead = [c for c in others if c.position > car.position]
        if ahead:
            leader = ahead[0]
            gap = leader.position - car.position - leader.length
        else:
            # Wrap-around: nearest car at the beginning of the road
            leader = others[0]  # sorted, so smallest position
            gap = (self.length - car.position) + leader.position - leader.length

        return max(0.0, gap), leader.velocity

    def find_lane_neighbors(
        self, car: Car, lane: int
    ) -> tuple[Car | None, Car | None, float, float]:
        """Return (leader, follower, gap_ahead, gap_behind) as if car were in `lane`.

        `car` is excluded from the search so this is safe to call with lane == car.lane.
        leader / follower are None when the lane has no other cars in that direction.
        """
        others = [c for c in self.sorted_lane(lane) if c is not car]
        if not others:
            return None, None, LARGE_GAP, LARGE_GAP

        ahead  = [c for c in others if c.position > car.position]
        behind = [c for c in others if c.position <= car.position]

        if ahead:
            leader   = ahead[0]
            gap_ahead = leader.position - car.position - leader.length
        else:
            leader   = others[0]   # wrap-around: smallest position
            gap_ahead = (self.length - car.position) + leader.position - leader.length

        if behind:
            follower   = behind[-1]
            gap_behind = car.position - follower.position - car.length
        else:
            follower   = others[-1]  # wrap-around: largest position
            gap_behind = (car.position + self.length - follower.position) - car.length

        return leader, follower, max(0.0, gap_ahead), max(0.0, gap_behind)

    def find_gap_in_lane(self, car: Car, target_lane: int) -> tuple[float, float]:
        """Return (gap_ahead, gap_behind) if car were in target_lane at its current position."""
        _, _, gap_ahead, gap_behind = self.find_lane_neighbors(car, target_lane)
        return gap_ahead, gap_behind
