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


class Road:
    def __init__(self, length: float = 1000.0, num_lanes: int = 3):
        self.length = length
        self.num_lanes = num_lanes
        self.lanes: list[list[Car]] = [[] for _ in range(num_lanes)]
        self.ramps: list[Ramp] = []

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

    def find_gap_in_lane(self, car: Car, target_lane: int) -> tuple[float, float]:
        """Return (gap_ahead, gap_behind) if car were in target_lane at its current position."""
        lane_cars = self.sorted_lane(target_lane)
        if not lane_cars:
            return LARGE_GAP, LARGE_GAP

        ahead = [c for c in lane_cars if c.position > car.position]
        behind = [c for c in lane_cars if c.position <= car.position]

        if ahead:
            gap_ahead = ahead[0].position - car.position - ahead[0].length
        else:
            # wrap-around
            leader = lane_cars[0]
            gap_ahead = (self.length - car.position) + leader.position - leader.length

        if behind:
            follower = behind[-1]
            gap_behind = car.position - follower.position - car.length
        else:
            # wrap-around
            follower = lane_cars[-1]
            gap_behind = (car.position + self.length - follower.position) - car.length

        return max(0.0, gap_ahead), max(0.0, gap_behind)
