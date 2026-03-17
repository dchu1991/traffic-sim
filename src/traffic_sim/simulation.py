from __future__ import annotations

import colorsys
import random
import warnings

import numpy as np

from .car import Car
from .config import SimConfig
from .road import LARGE_GAP, Ramp, Road


def _poisson_sample(lam: float) -> int:
    """Sample from Poisson(lam). numpy is a runtime dep so this is always available."""
    return np.random.poisson(lam)


class Simulation:
    def __init__(
        self,
        road_length: float = 1000.0,
        num_lanes: int = 3,
        num_cars: int = 50,
        truck_fraction: float = 0.15,
        config: SimConfig | None = None,
    ):
        self.cfg = config or SimConfig()
        lc = self.cfg.lane_change

        self.lane_change_cooldown = lc.cooldown_s
        self.lane_change_incentive = lc.incentive_m  # kept for reference; unused for left moves
        self.safety_gap = lc.safety_gap_m
        self.keep_right_gap = lc.keep_right_gap_m
        self.lane_change_duration = lc.duration_s
        self.politeness = lc.politeness
        self.delta_a_threshold = lc.delta_a_threshold_ms2

        speed_limits = self.cfg.speed_limits_ms(num_lanes)
        self.road = Road(
            length=road_length, num_lanes=num_lanes, lane_speed_limits=speed_limits
        )

        self.cars: list[Car] = []
        self.time = 0.0
        self._next_id = 0

        rc = self.cfg.ramp
        self._onramp_rate_max: float = rc.onramp_rate   # free-flow ceiling for ramp metering
        rightmost = num_lanes - 1
        self.road.ramps = [
            Ramp(
                position=road_length * rc.onramp_position,
                lane=rightmost,
                is_onramp=True,
                rate=rc.onramp_rate,
            ),
            Ramp(
                position=road_length * rc.offramp_position,
                lane=rightmost,
                is_onramp=False,
                rate=rc.offramp_prob,
            ),
        ]

        # car_id → (from_lane, progress)  where progress runs 0 → 1
        self._lane_transitions: dict[int, tuple[int, float]] = {}

        self._spawn_initial_cars(num_cars, truck_fraction)

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def _make_car(
        self, car_id: int, lane: int, position: float, is_truck: bool = False
    ) -> Car:
        """Spawn a new car (or truck) with randomised IDM parameters from config."""
        if is_truck:
            tc = self.cfg.trucks
            v0 = max(tc.desired_v_min_ms, min(tc.desired_v_max_ms,
                     random.gauss(tc.desired_v_mean_ms, tc.desired_v_std_ms)))
            length = random.uniform(tc.length_min_m, tc.length_max_m)
            color: tuple[int, int, int] = (180, 140, 80)
        else:
            cc = self.cfg.cars
            v0 = max(cc.desired_v_min_ms, min(cc.desired_v_max_ms,
                     random.gauss(cc.desired_v_mean_ms, cc.desired_v_std_ms)))
            length = random.uniform(4.0, 5.5)
            h = random.random()
            r, g, b = colorsys.hsv_to_rgb(h, 0.75, 0.95)
            color = (int(r * 255), int(g * 255), int(b * 255))

        cc = self.cfg.cars
        car = Car(
            car_id=car_id,
            lane=lane,
            position=position,
            velocity=v0 * 0.7,
            color=color,
            desired_velocity=v0,
            time_headway=max(0.8, min(2.5, random.gauss(cc.time_headway_mean, cc.time_headway_std))),
            min_gap=max(1.0, min(4.0, random.gauss(cc.min_gap_mean_m, 0.5))),
            max_accel=max(0.8, min(2.5, random.gauss(cc.max_accel_mean, 0.3))),
            comfortable_decel=max(1.0, min(3.5, random.gauss(cc.comfortable_decel_mean, 0.4))),
            length=length,
        )
        if self.cfg.destination.enabled:
            dc = self.cfg.destination
            car.destination_laps = dc.min_loops + _poisson_sample(dc.loops_lambda)
            if car.destination_laps == 0:
                warnings.warn(
                    "Destination mode: car spawned with destination_laps=0 — car will never exit."
                    " Set min_loops >= 1.",
                    stacklevel=2,
                )
        return car

    def _spawn_initial_cars(self, num_cars: int, truck_fraction: float) -> None:
        per_lane = num_cars // self.road.num_lanes
        min_space = self.cfg.cars.min_gap_mean_m + 5.0  # min gap + typical car length
        for lane in range(self.road.num_lanes):
            count = per_lane + (1 if lane < num_cars % self.road.num_lanes else 0)
            # Guarantee minimum spacing: map uniform samples onto positions with
            # at least min_space between each car.
            available = max(0.0, self.road.length - count * min_space)
            raw = sorted(random.uniform(0.0, available) for _ in range(count))
            positions = [r + i * min_space for i, r in enumerate(raw)]
            for pos in positions:
                is_truck = random.random() < truck_fraction
                car = self._make_car(self._next_id, lane, pos, is_truck)
                self._next_id += 1
                self.cars.append(car)
                self.road.add_car(car)

    # ------------------------------------------------------------------
    # Lane changing (MOBIL + keep-right)
    # ------------------------------------------------------------------

    def _do_lane_change(self, car: Car, target_lane: int) -> None:
        prev_lane = car.lane
        self.road.lanes[car.lane].remove(car)
        car.lane = target_lane
        car.lane_change_timer = self.lane_change_cooldown
        car.exiting = False
        self.road.lanes[target_lane].append(car)
        self._lane_transitions[car.car_id] = (prev_lane, 0.0)

    def _try_lane_change(self, car: Car) -> None:
        if car.lane_change_timer > 0.0:
            return

        current_gap, current_leader_vel = self.road.find_leader(car)

        # Build candidate lanes: left first (overtaking), then right (keep-right)
        # Exiting cars skip the left candidate — they need to reach the rightmost lane.
        candidates: list[int] = []
        if car.lane > 0 and not car.exiting:
            candidates.append(car.lane - 1)
        if car.lane < self.road.num_lanes - 1:
            candidates.append(car.lane + 1)

        for target_lane in candidates:
            target_leader, new_follower, gap_ahead, gap_behind = \
                self.road.find_lane_neighbors(car, target_lane)

            # Safety check (always required)
            if gap_behind < self.safety_gap:
                continue

            if target_lane < car.lane:
                # Moving LEFT — MOBIL acceleration-based criterion
                target_leader_vel = (
                    target_leader.velocity if target_leader is not None
                    else car.desired_velocity
                )
                cur_v0 = min(car.desired_velocity, self.road.lane_speed_limits[car.lane])
                tgt_v0 = min(car.desired_velocity, self.road.lane_speed_limits[target_lane])

                a_self_before = car.idm_acceleration(current_gap, current_leader_vel, cur_v0)
                a_self_after  = car.idm_acceleration(gap_ahead,   target_leader_vel,  tgt_v0)
                gain = a_self_after - a_self_before

                if self.politeness > 0.0:
                    # New follower in target lane: braking they must do to accommodate us
                    if new_follower is not None:
                        nf_v0 = min(
                            new_follower.desired_velocity,
                            self.road.lane_speed_limits[target_lane],
                        )
                        nf_gap_before = gap_behind + car.length + gap_ahead
                        gain += self.politeness * (
                            new_follower.idm_acceleration(gap_behind,    car.velocity,        nf_v0)
                            - new_follower.idm_acceleration(nf_gap_before, target_leader_vel, nf_v0)
                        )
                    # Old follower in current lane: gap they gain when we depart
                    _, old_follower, _, old_gap_behind = \
                        self.road.find_lane_neighbors(car, car.lane)
                    if old_follower is not None:
                        of_v0 = min(
                            old_follower.desired_velocity,
                            self.road.lane_speed_limits[car.lane],
                        )
                        of_gap_after = old_gap_behind + car.length + current_gap
                        gain += self.politeness * (
                            old_follower.idm_acceleration(of_gap_after,   current_leader_vel, of_v0)
                            - old_follower.idm_acceleration(old_gap_behind, car.velocity,     of_v0)
                        )

                if gain > self.delta_a_threshold:
                    self._do_lane_change(car, target_lane)
                    return
            else:
                # Moving RIGHT — keep-right: skip if inside a merge zone
                if self._in_merge_zone(car, target_lane):
                    continue
                # Exiting cars bypass the keep_right_gap threshold — they must get right
                if car.exiting:
                    self._do_lane_change(car, target_lane)
                    return
                if self.keep_right_gap > 0 and gap_ahead >= self.keep_right_gap:
                    self._do_lane_change(car, target_lane)
                    return

    def _update_exiting_flags(self) -> None:
        """In destination mode, mark cars on their final lap as exiting when near the off-ramp.

        Called at the top of step() before lane changes so exiting cars immediately
        start moving right. _do_lane_change resets exiting=False, so this must re-apply
        every step.
        """
        if not self.cfg.destination.enabled:
            return
        lookahead = self.cfg.destination.exit_lookahead_m
        for ramp in self.road.ramps:
            if ramp.is_onramp:
                continue
            for car in self.cars:
                if car.destination_laps > 0 and car.laps_completed >= car.destination_laps:
                    dist_to_ramp = (ramp.position - car.position) % self.road.length
                    if dist_to_ramp <= lookahead:
                        car.exiting = True

    def _in_merge_zone(self, car: Car, target_lane: int) -> bool:
        """Return True if car is within ramp_length_m upstream of an on-ramp in target_lane."""
        zone = self.cfg.ramp.ramp_length_m
        for ramp in self.road.ramps:
            if not ramp.is_onramp or ramp.lane != target_lane:
                continue
            dist_to_ramp = (ramp.position - car.position) % self.road.length
            if dist_to_ramp <= zone:
                return True
        return False

    # ------------------------------------------------------------------
    # Ramp logic
    # ------------------------------------------------------------------

    def _update_ramp_control(self, dt: float) -> None:
        """Proportional controller: both ramps respond to the same car-count error.

        On-ramp metering (primary): reduce intake when over target — mirrors real ramp signals.
        Off-ramp probability (secondary): raise exit chance when over target.
        Both are bounded: onramp by the configured free-flow ceiling, offramp by [0, 1].
        """
        target = self.cfg.ramp.target_cars
        if target <= 0:
            return
        error = self.car_count - target  # positive = too many cars
        for ramp in self.road.ramps:
            if ramp.is_onramp:
                gain = self.cfg.ramp.onramp_control_gain
                ramp.rate = max(0.0, min(self._onramp_rate_max, ramp.rate - gain * error * dt))
            else:
                # In destination mode, offramp_prob is unused — only onramp_rate is controlled
                if self.cfg.destination.enabled:
                    continue
                gain = self.cfg.ramp.offramp_control_gain
                ramp.rate = max(0.0, min(1.0, ramp.rate + gain * error * dt))

    def _process_offramp(self, car: Car, ramp: Ramp) -> bool:
        """Return True if the car should be removed from the simulation."""
        if car.lane != ramp.lane:
            return False
        if self.cfg.destination.enabled:
            # Destination mode: exit only when lap target is reached (guaranteed)
            if ramp.position in car._passed_ramps:
                return False  # already evaluated this lap
            car._passed_ramps.add(ramp.position)
            return car.destination_laps > 0 and car.laps_completed >= car.destination_laps
        # Classic probabilistic mode
        if random.random() < ramp.rate and ramp.position not in car._passed_ramps:
            car._passed_ramps.add(ramp.position)
            return True
        car._passed_ramps.add(ramp.position)
        return False

    def _process_onramp(self, ramp: Ramp, dt: float) -> None:
        """Add a car to the on-ramp queue at the configured rate."""
        if ramp.rate <= 0:
            return
        ramp._timer += dt
        interval = 1.0 / ramp.rate
        if ramp._timer < interval:
            return
        ramp._timer -= interval

        max_q = self.cfg.ramp.max_queue
        if max_q > 0 and len(ramp.queue) >= max_q:
            return  # ramp is backed up

        # Spawn at back of ramp (position 0) at standstill
        car = self._make_car(self._next_id, ramp.lane, 0.0)
        car.velocity = 0.0
        self._next_id += 1
        ramp.queue.append(car)

    def _step_ramp_queues(self, dt: float) -> None:
        """Advance IDM physics for all on-ramp queue cars; merge lead car when gap allows."""
        ramp_length = self.cfg.ramp.ramp_length_m
        min_gap = self.cfg.ramp.min_gap_m
        merge_window = min(self.cfg.ramp.merge_window_m, ramp_length)
        zipper_speed = self.cfg.ramp.zipper_speed_kmh / 3.6
        zipper_gap = self.cfg.ramp.zipper_gap_m

        for ramp in self.road.ramps:
            if not ramp.is_onramp or not ramp.queue:
                continue

            lane_cars = self.road.sorted_lane(ramp.lane)
            speed_limit = self.road.lane_speed_limits[ramp.lane]

            # IDM update — front to back so each car sees fresh positions ahead
            for i, car in enumerate(ramp.queue):
                effective_v0 = min(car.desired_velocity, speed_limit)
                if i == 0:
                    # Lead car: treat nearest road car ahead of merge point as virtual leader
                    ahead = [c for c in lane_cars if c.position > ramp.position]
                    if ahead:
                        road_gap = ahead[0].position - ramp.position - ahead[0].length
                        lead_v = ahead[0].velocity
                        lead_len = ahead[0].length
                    else:
                        road_gap = LARGE_GAP
                        lead_v = speed_limit
                        lead_len = 5.0
                    # Map road leader into ramp-space: virtual front = ramp_length + road_gap
                    effective_gap = max(
                        0.0, ramp_length + road_gap - lead_len - car.position
                    )
                else:
                    leader = ramp.queue[i - 1]
                    effective_gap = max(
                        0.0, leader.position - car.position - leader.length
                    )
                    lead_v = leader.velocity

                accel = car.idm_acceleration(effective_gap, lead_v, effective_v0)
                car.velocity = max(0.0, min(car.velocity + accel * dt, speed_limit))
                car.position = min(car.position + car.velocity * dt, ramp_length)

            # Merge check: lead car can merge anywhere in the last merge_window metres of the ramp
            lead = ramp.queue[0]
            if lead.position < ramp_length - merge_window:
                continue

            # Map ramp position → road entry position
            # At ramp_length the car is exactly at ramp.position; earlier positions
            # are further upstream on the road.
            entry_pos = ramp.position - (ramp_length - lead.position)

            lane_cars = self.road.sorted_lane(
                ramp.lane
            )  # refresh after position updates

            # Zipper merge: lower gap requirement when rightmost lane is crawling
            nearby = [
                c
                for c in lane_cars
                if 0 < (ramp.position - c.position) % self.road.length <= merge_window
            ]
            avg_speed = (
                sum(c.velocity for c in nearby) / len(nearby)
                if nearby
                else float("inf")
            )
            effective_min_gap = zipper_gap if avg_speed < zipper_speed else min_gap

            ahead = [c for c in lane_cars if c.position > entry_pos]
            behind = [c for c in lane_cars if c.position <= entry_pos]
            gap_ahead = (
                (ahead[0].position - entry_pos - ahead[0].length)
                if ahead
                else LARGE_GAP
            )
            gap_behind = (
                (entry_pos - behind[-1].position - lead.length) if behind else LARGE_GAP
            )

            # In zipper mode (slow traffic) only gap_ahead matters — cars are nearly
            # stopped so the one behind can react within a tick via IDM.
            # In normal mode also require a safe gap behind.
            zipper_active = avg_speed < zipper_speed
            if zipper_active:
                can_merge = gap_ahead >= effective_min_gap
            else:
                can_merge = (
                    gap_ahead >= effective_min_gap
                    and gap_behind >= effective_min_gap * 0.5
                )
            if can_merge:
                lead.position = entry_pos  # enter road at mapped position
                lead.lane = ramp.lane
                ramp.queue.pop(0)
                self.cars.append(lead)
                self.road.add_car(lead)
                # Animate merge: slide from ramp lane (visual index = num_lanes)
                # into the rightmost road lane, reusing the same smoothstep system
                self._lane_transitions[lead.car_id] = (self.road.num_lanes, 0.0)

    # ------------------------------------------------------------------
    # Main step
    # ------------------------------------------------------------------

    def step(self, dt: float) -> None:
        # 0. Mark exiting cars (destination mode only — no-op otherwise)
        self._update_exiting_flags()

        # 1. Lane changes
        for car in list(self.cars):
            self._try_lane_change(car)

        # 2. IDM update — enforce per-lane speed limit
        #    Road cars in the rightmost lane also yield to any ramp lead car waiting to merge
        ramp_length = self.cfg.ramp.ramp_length_m
        destination_enabled = self.cfg.destination.enabled
        for car in self.cars:
            old_pos = car.position  # saved for lap detection below
            gap, lead_v = self.road.find_leader(car)
            # Cooperative merge: yield to ramp lead car if it is near the merge point
            for ramp in self.road.ramps:
                if not ramp.is_onramp or not ramp.queue:
                    continue
                if car.lane != ramp.lane:
                    continue
                lead_q = ramp.queue[0]
                if lead_q.position < ramp_length - 15.0:
                    continue  # ramp car is still far back
                if car.position < ramp.position:
                    ramp_gap = ramp.position - car.position - lead_q.length
                    # Only yield when there is actual space to give; if ramp_gap <= 0
                    # the car is already inside the ramp car's body — let it pass through
                    # rather than holding it frozen in place (which creates a deadlock).
                    if 0 < ramp_gap < gap:
                        gap = ramp_gap
                        lead_v = lead_q.velocity
            limit = self.road.lane_speed_limits[car.lane]
            car.update(dt, gap, lead_v, self.road.length, speed_limit=limit)
            # Lap detection: position wrapped past 0 → increment lap counter
            if destination_enabled and car.destination_laps > 0:
                if old_pos > car.position + self.road.length * 0.5:
                    car.laps_completed += 1
                    car._passed_ramps.clear()  # allow exit re-evaluation this lap

        # 3. Ramp queue physics + merge
        self._step_ramp_queues(dt)

        # 4. Off-ramp processing — check which cars crossed a ramp this tick
        to_remove: list[Car] = []
        for car in self.cars:
            for ramp in self.road.ramps:
                if ramp.is_onramp:
                    continue
                dist = (car.position - ramp.position) % self.road.length
                if dist < car.velocity * dt * 2 + 2.0:
                    if self._process_offramp(car, ramp):
                        to_remove.append(car)
                        break

        for car in to_remove:
            self._lane_transitions.pop(car.car_id, None)
            self.road.remove_car(car)
            self.cars.remove(car)

        # 4b. Dynamic off-ramp control
        self._update_ramp_control(dt)

        # 5. On-ramp queue spawning
        for ramp in self.road.ramps:
            if ramp.is_onramp:
                self._process_onramp(ramp, dt)

        # 6. Advance visual lane-change transitions
        for cid in list(self._lane_transitions):
            from_lane, progress = self._lane_transitions[cid]
            progress += dt / self.lane_change_duration
            if progress >= 1.0:
                del self._lane_transitions[cid]
            else:
                self._lane_transitions[cid] = (from_lane, progress)

        self.time += dt

    # ------------------------------------------------------------------
    # Visual helpers
    # ------------------------------------------------------------------

    def get_visual_lane(self, car: Car) -> float:
        """Return the car's interpolated (float) lane position for rendering."""
        if car.car_id not in self._lane_transitions:
            return float(car.lane)
        from_lane, progress = self._lane_transitions[car.car_id]
        # Smoothstep easing: 3t² − 2t³
        t = progress * progress * (3.0 - 2.0 * progress)
        return from_lane + (car.lane - from_lane) * t

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------

    @property
    def avg_speed_kmh(self) -> float:
        if not self.cars:
            return 0.0
        return sum(c.velocity for c in self.cars) / len(self.cars) * 3.6

    @property
    def car_count(self) -> int:
        return len(self.cars)
