from __future__ import annotations

import colorsys
import random

import numpy as np

from .car import Car
from .road import LARGE_GAP, Ramp, Road

# Lane-change thresholds
LANE_CHANGE_COOLDOWN = 3.0   # s — minimum time between lane changes
LANE_CHANGE_INCENTIVE = 8.0  # m — gap_ahead must improve by this much to bother
SAFETY_GAP = 6.0             # m — minimum gap behind in target lane

# On-ramp spawning
ONRAMP_MIN_GAP = 20.0        # m — minimum gap needed to merge onto road


def _random_color() -> tuple[int, int, int]:
    h = random.random()
    r, g, b = colorsys.hsv_to_rgb(h, 0.75, 0.95)
    return (int(r * 255), int(g * 255), int(b * 255))


def _make_car(car_id: int, lane: int, position: float, is_truck: bool = False) -> Car:
    """Spawn a new car (or truck) with randomised IDM parameters."""
    if is_truck:
        v0 = float(np.clip(random.gauss(22.0, 2.0), 15.0, 27.0))  # ~80 km/h max
        length = random.uniform(10.0, 16.0)
        color = (180, 140, 80)
    else:
        v0 = float(np.clip(random.gauss(33.0, 4.0), 22.0, 44.0))  # ~120 km/h max
        length = random.uniform(4.0, 5.5)
        color = _random_color()

    return Car(
        car_id=car_id,
        lane=lane,
        position=position,
        velocity=v0 * 0.7,  # start a bit below desired speed
        color=color,
        desired_velocity=v0,
        time_headway=float(np.clip(random.gauss(1.5, 0.3), 0.8, 2.5)),
        min_gap=float(np.clip(random.gauss(2.0, 0.5), 1.0, 4.0)),
        max_accel=float(np.clip(random.gauss(1.5, 0.3), 0.8, 2.5)),
        comfortable_decel=float(np.clip(random.gauss(2.0, 0.4), 1.0, 3.5)),
        length=length,
    )


class Simulation:
    def __init__(
        self,
        road_length: float = 1000.0,
        num_lanes: int = 3,
        num_cars: int = 50,
        truck_fraction: float = 0.15,
    ):
        self.road = Road(length=road_length, num_lanes=num_lanes)
        self.cars: list[Car] = []
        self.time = 0.0
        self._next_id = 0

        # Default ramps: one on-ramp near start, one off-ramp near end (rightmost lane)
        rightmost = num_lanes - 1
        self.road.ramps = [
            Ramp(position=road_length * 0.10, lane=rightmost, is_onramp=True,  rate=0.5),
            Ramp(position=road_length * 0.80, lane=rightmost, is_onramp=False, rate=0.30),
        ]

        self._spawn_initial_cars(num_cars, truck_fraction)

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def _spawn_initial_cars(self, num_cars: int, truck_fraction: float) -> None:
        per_lane = num_cars // self.road.num_lanes
        for lane in range(self.road.num_lanes):
            count = per_lane + (1 if lane < num_cars % self.road.num_lanes else 0)
            positions = sorted(np.random.uniform(0.0, self.road.length, count))
            for pos in positions:
                is_truck = random.random() < truck_fraction
                car = _make_car(self._next_id, lane, float(pos), is_truck)
                self._next_id += 1
                self.cars.append(car)
                self.road.add_car(car)

    # ------------------------------------------------------------------
    # Lane changing (MOBIL-lite)
    # ------------------------------------------------------------------

    def _try_lane_change(self, car: Car) -> None:
        if car.lane_change_timer > 0.0:
            return

        current_gap, _ = self.road.find_leader(car)

        # Prefer left (faster) lane first, then right (slower)
        candidates = []
        if car.lane > 0:
            candidates.append(car.lane - 1)
        if car.lane < self.road.num_lanes - 1:
            candidates.append(car.lane + 1)

        for target_lane in candidates:
            gap_ahead, gap_behind = self.road.find_gap_in_lane(car, target_lane)

            # Safety: don't cut off the car behind in the target lane
            if gap_behind < SAFETY_GAP:
                continue

            # Incentive: must gain meaningfully
            if gap_ahead > current_gap + LANE_CHANGE_INCENTIVE:
                self.road.lanes[car.lane].remove(car)
                car.lane = target_lane
                car.lane_change_timer = LANE_CHANGE_COOLDOWN
                car.exiting = False  # reset — will re-evaluate off-ramp commitment
                self.road.lanes[target_lane].append(car)
                return

    # ------------------------------------------------------------------
    # Ramp logic
    # ------------------------------------------------------------------

    def _process_offramp(self, car: Car, ramp: Ramp) -> bool:
        """Return True if the car should be removed from the simulation."""
        if car.lane != ramp.lane:
            return False
        # Each meter past the ramp, the car has rate% chance to exit
        # We approximate: prob = rate (a flat probability per passing event)
        if random.random() < ramp.rate and ramp.position not in car._passed_ramps:
            car._passed_ramps.add(ramp.position)
            return True
        car._passed_ramps.add(ramp.position)
        return False

    def _process_onramp(self, ramp: Ramp, dt: float) -> None:
        """Attempt to spawn a car from the on-ramp."""
        ramp._timer += dt
        interval = 1.0 / ramp.rate  # seconds between spawns
        if ramp._timer < interval:
            return
        ramp._timer -= interval

        # Check gap at the ramp position in the target lane
        test_car_len = 5.0
        lane_cars = self.road.sorted_lane(ramp.lane)
        ahead = [c for c in lane_cars if c.position > ramp.position]
        behind = [c for c in lane_cars if c.position <= ramp.position]

        gap_ahead = (ahead[0].position - ramp.position - ahead[0].length) if ahead else LARGE_GAP
        gap_behind = (ramp.position - behind[-1].position - test_car_len) if behind else LARGE_GAP

        if gap_ahead >= ONRAMP_MIN_GAP and gap_behind >= ONRAMP_MIN_GAP * 0.5:
            car = _make_car(self._next_id, ramp.lane, ramp.position)
            self._next_id += 1
            self.cars.append(car)
            self.road.add_car(car)

    # ------------------------------------------------------------------
    # Main step
    # ------------------------------------------------------------------

    def step(self, dt: float) -> None:
        # 1. Lane changes
        for car in list(self.cars):
            self._try_lane_change(car)

        # 2. IDM update
        for car in self.cars:
            gap, lead_v = self.road.find_leader(car)
            car.update(dt, gap, lead_v, self.road.length)

        # 3. Off-ramp processing — check which cars crossed a ramp this tick
        to_remove: list[Car] = []
        for car in self.cars:
            for ramp in self.road.ramps:
                if ramp.is_onramp:
                    continue
                # Detect crossing: car's position is within a small window past the ramp
                dist = (car.position - ramp.position) % self.road.length
                if dist < car.velocity * dt * 2 + 2.0:  # within ~2 frames
                    if self._process_offramp(car, ramp):
                        to_remove.append(car)
                        break

        for car in to_remove:
            self.road.remove_car(car)
            self.cars.remove(car)

        # 4. On-ramp spawning
        for ramp in self.road.ramps:
            if ramp.is_onramp:
                self._process_onramp(ramp, dt)

        self.time += dt

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------

    @property
    def avg_speed_kmh(self) -> float:
        if not self.cars:
            return 0.0
        return float(np.mean([c.velocity for c in self.cars])) * 3.6

    @property
    def car_count(self) -> int:
        return len(self.cars)
