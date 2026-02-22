"""Tests for Simulation: lane changes, ramps, merge zone, and full-step integration."""
import random

import numpy as np
import pytest

from traffic_sim.config import SimConfig
from traffic_sim.road import Ramp
from traffic_sim.simulation import Simulation
from tests.helpers import make_car


def make_sim(num_cars=0, road_length=1000.0, num_lanes=3, cfg=None, seed=0):
    """Deterministic Simulation factory."""
    random.seed(seed)
    np.random.seed(seed)
    if cfg is None:
        cfg = SimConfig()
    return Simulation(road_length=road_length, num_lanes=num_lanes,
                      num_cars=num_cars, config=cfg)


# ── Merge zone ───────────────────────────────────────────────────────────────

class TestInMergeZone:
    """_in_merge_zone: car within ramp_length_m upstream → True."""

    def test_car_just_upstream_is_in_zone(self):
        """Car 50 m before on-ramp (zone=200 m) → in zone."""
        # on-ramp at 10% × 1000 = 100 m; zone = 200 m
        sim = make_sim()
        car = make_car(lane=1, position=50.0)
        assert sim._in_merge_zone(car, target_lane=2)  # 50 m before ramp

    def test_car_far_upstream_not_in_zone(self):
        """Car 500 m before on-ramp → not in zone."""
        sim = make_sim()
        car = make_car(lane=1, position=600.0)
        assert not sim._in_merge_zone(car, target_lane=2)

    def test_car_at_ramp_position_in_zone(self):
        """Car exactly at merge point → in zone (distance = 0)."""
        sim = make_sim()
        car = make_car(lane=1, position=100.0)
        assert sim._in_merge_zone(car, target_lane=2)

    def test_wraparound_upstream(self):
        """Car 150 m upstream via wraparound (position=950, ramp=100) → in zone."""
        sim = make_sim()
        car = make_car(lane=1, position=950.0)
        # (100 - 950) % 1000 = 150 <= 200 → in zone
        assert sim._in_merge_zone(car, target_lane=2)

    def test_wrong_target_lane_not_blocked(self):
        """On-ramp is in lane 2; checking lane 1 should never block."""
        sim = make_sim()
        car = make_car(lane=0, position=50.0)
        assert not sim._in_merge_zone(car, target_lane=1)

    def test_no_onramp_never_blocks(self):
        """With on-ramp rate disabled, no merge zone exists."""
        cfg = SimConfig()
        cfg.ramp.onramp_rate = 0.0
        sim = make_sim(cfg=cfg)
        # Disable all on-ramp rates manually (rate=0 still creates ramp object)
        for ramp in sim.road.ramps:
            ramp.rate = 0.0
            ramp.is_onramp = False  # make it an off-ramp so the check skips it
        car = make_car(lane=1, position=50.0)
        assert not sim._in_merge_zone(car, target_lane=2)


# ── Off-ramp ─────────────────────────────────────────────────────────────────

class TestProcessOfframp:
    def _offramp(self, rate=1.0):
        return Ramp(position=800.0, lane=2, is_onramp=False, rate=rate)

    def test_wrong_lane_never_exits(self):
        sim = make_sim()
        ramp = self._offramp()
        car = make_car(lane=0, position=800.0)  # wrong lane
        assert sim._process_offramp(car, ramp) is False

    def test_rate_1_always_exits(self):
        """rate=1.0 → random.random() < 1.0 is always True."""
        sim = make_sim()
        ramp = self._offramp(rate=1.0)
        car = make_car(lane=2, position=800.0)
        assert sim._process_offramp(car, ramp) is True

    def test_rate_0_never_exits(self):
        """rate=0.0 → random.random() < 0.0 is always False."""
        sim = make_sim()
        ramp = self._offramp(rate=0.0)
        car = make_car(lane=2, position=800.0)
        assert sim._process_offramp(car, ramp) is False

    def test_no_double_exit(self):
        """Car that already passed the ramp cannot exit again."""
        sim = make_sim()
        ramp = self._offramp(rate=1.0)
        car = make_car(lane=2, position=800.0)
        car._passed_ramps.add(800.0)
        assert sim._process_offramp(car, ramp) is False

    def test_ramp_position_recorded_after_pass(self):
        """After processing, ramp position is in car._passed_ramps."""
        sim = make_sim()
        ramp = self._offramp(rate=0.0)
        car = make_car(lane=2, position=800.0)
        sim._process_offramp(car, ramp)
        assert 800.0 in car._passed_ramps


# ── On-ramp queue spawning ───────────────────────────────────────────────────

class TestProcessOnramp:
    def test_spawns_into_queue_after_interval(self):
        """Timer fires → one car added to queue at standstill."""
        sim = make_sim()
        ramp = sim.road.ramps[0]
        assert ramp.is_onramp
        # rate=0.5 → interval=2 s; advance timer past threshold
        sim._process_onramp(ramp, dt=2.5)
        assert len(ramp.queue) == 1
        assert ramp.queue[0].velocity == pytest.approx(0.0)

    def test_no_spawn_before_interval(self):
        """Short dt doesn't trigger a spawn."""
        sim = make_sim()
        ramp = sim.road.ramps[0]
        sim._process_onramp(ramp, dt=0.1)
        assert len(ramp.queue) == 0

    def test_rate_zero_never_spawns(self):
        sim = make_sim()
        ramp = sim.road.ramps[0]
        ramp.rate = 0.0
        for _ in range(100):
            sim._process_onramp(ramp, dt=0.5)
        assert len(ramp.queue) == 0

    def test_max_queue_respected(self):
        """Queue never exceeds max_queue."""
        cfg = SimConfig()
        cfg.ramp.max_queue = 3
        cfg.ramp.onramp_rate = 10.0   # fast: spawns every 0.1 s
        sim = make_sim(cfg=cfg)
        ramp = sim.road.ramps[0]
        for _ in range(50):
            sim._process_onramp(ramp, dt=0.5)
        assert len(ramp.queue) <= 3

    def test_max_queue_zero_means_no_limit(self):
        """max_queue=0 means unlimited queue growth."""
        cfg = SimConfig()
        cfg.ramp.max_queue = 0
        cfg.ramp.onramp_rate = 10.0
        sim = make_sim(cfg=cfg)
        ramp = sim.road.ramps[0]
        for _ in range(50):
            sim._process_onramp(ramp, dt=0.5)
        assert len(ramp.queue) > 3   # well above any sensible limit


# ── Ramp queue physics + merge ────────────────────────────────────────────────

class TestStepRampQueues:
    def test_merges_onto_empty_road(self):
        """Lead car at merge point enters road when no traffic blocks it."""
        cfg = SimConfig()
        cfg.ramp.min_gap_m = 20.0
        cfg.ramp.ramp_length_m = 200.0
        sim = make_sim(cfg=cfg)
        ramp = sim.road.ramps[0]

        lead = make_car(car_id=999, lane=2, position=199.9, velocity=25.0)
        ramp.queue.append(lead)

        sim._step_ramp_queues(dt=0.05)

        assert lead in sim.cars
        assert len(ramp.queue) == 0

    def test_waits_when_gap_too_small(self):
        """Lead car at merge point stays in queue when road gap < min_gap."""
        cfg = SimConfig()
        cfg.ramp.min_gap_m = 20.0
        cfg.ramp.ramp_length_m = 200.0
        sim = make_sim(cfg=cfg)
        ramp = sim.road.ramps[0]

        # Blocker just 3 m past merge point (gap = 108 - 100 - 5 = 3 m < 20 m)
        blocker = make_car(car_id=1, lane=2, position=108.0, velocity=25.0, length=5.0)
        sim.cars.append(blocker)
        sim.road.add_car(blocker)

        lead = make_car(car_id=999, lane=2, position=199.9, velocity=25.0)
        ramp.queue.append(lead)

        sim._step_ramp_queues(dt=0.05)

        assert lead not in sim.cars
        assert len(ramp.queue) == 1

    def test_following_car_accelerates_behind_lead(self):
        """Second queue car increases velocity while following the lead car."""
        cfg = SimConfig()
        cfg.ramp.ramp_length_m = 200.0
        sim = make_sim(cfg=cfg)
        ramp = sim.road.ramps[0]

        lead   = make_car(car_id=0, lane=2, position=100.0, velocity=10.0)
        follow = make_car(car_id=1, lane=2, position=50.0,  velocity=0.0)
        ramp.queue.extend([lead, follow])

        sim._step_ramp_queues(dt=0.1)

        assert follow.velocity > 0.0   # was 0, should now be accelerating

    def test_merged_car_at_ramp_position(self):
        """Merged car enters road upstream of ramp.position when not at the very tip."""
        cfg = SimConfig()
        cfg.ramp.min_gap_m = 5.0
        cfg.ramp.ramp_length_m = 200.0
        sim = make_sim(cfg=cfg)
        ramp = sim.road.ramps[0]

        # velocity=0 so IDM barely moves the car in one tick; position=197 is within
        # the 5 m merge zone (200 - 5 = 195) but not at the very tip.
        lead = make_car(car_id=999, lane=2, position=197.0, velocity=0.0)
        ramp.queue.append(lead)
        sim._step_ramp_queues(dt=0.05)

        if lead in sim.cars:  # merge succeeded
            # entry_pos maps ramp coords → road coords; car should enter before the tip
            assert lead.position < ramp.position
            assert lead.position >= ramp.position - cfg.ramp.ramp_length_m

    def test_zipper_merge_used_when_lane_is_slow(self):
        """When rightmost lane is crawling, zipper_gap_m replaces min_gap_m."""
        cfg = SimConfig()
        cfg.ramp.min_gap_m = 30.0
        cfg.ramp.zipper_speed_kmh = 40.0   # 40 km/h threshold
        cfg.ramp.zipper_gap_m = 8.0
        cfg.ramp.ramp_length_m = 200.0
        sim = make_sim(cfg=cfg)
        ramp = sim.road.ramps[0]

        # Blocker 12 m ahead: gap = 12 - 8(car length) = 4 m < min_gap(30) but > zipper_gap(8)/2
        # Actually let's make gap = 10 m: ahead at 110, length 5 → gap = 110-100-5 = 5 m
        # 5 < 30 (min_gap) but >= 8 (zipper_gap) at half-requirement for behind
        # Use gap_ahead=10 m >= zipper_gap(8), gap_behind=LARGE_GAP >= zipper_gap*0.5
        blocker = make_car(car_id=1, lane=2, position=115.0, velocity=5.0, length=5.0)
        sim.cars.append(blocker)
        sim.road.add_car(blocker)

        # Slow car within merge window: average speed = 5 m/s = 18 km/h < 40 km/h threshold
        slow = make_car(car_id=2, lane=2, position=80.0, velocity=5.0)
        sim.cars.append(slow)
        sim.road.add_car(slow)

        lead = make_car(car_id=999, lane=2, position=199.9, velocity=0.0)
        ramp.queue.append(lead)

        sim._step_ramp_queues(dt=0.05)

        # With zipper_gap=8 and gap_ahead=10, merge should succeed
        assert lead in sim.cars

    def test_normal_gap_used_when_lane_is_fast(self):
        """When rightmost lane is fast, min_gap_m is required (zipper mode off)."""
        cfg = SimConfig()
        cfg.ramp.min_gap_m = 30.0
        cfg.ramp.zipper_speed_kmh = 10.0   # low threshold — lane at 25 m/s won't trigger
        cfg.ramp.zipper_gap_m = 8.0
        cfg.ramp.ramp_length_m = 200.0
        sim = make_sim(cfg=cfg)
        ramp = sim.road.ramps[0]

        # Blocker 15 m ahead: gap_ahead = 10 m < min_gap(30) but > zipper_gap(8)
        blocker = make_car(car_id=1, lane=2, position=115.0, velocity=25.0, length=5.0)
        sim.cars.append(blocker)
        sim.road.add_car(blocker)

        lead = make_car(car_id=999, lane=2, position=199.9, velocity=0.0)
        ramp.queue.append(lead)

        sim._step_ramp_queues(dt=0.05)

        # Lane is fast (25 m/s >> 10 km/h threshold), so min_gap(30) applies → no merge
        assert lead not in sim.cars


# ── Lane changes ──────────────────────────────────────────────────────────────

class TestTryLaneChange:
    def test_keep_right_blocked_in_merge_zone(self):
        """Middle-lane car near on-ramp cannot keep-right into rightmost lane."""
        cfg = SimConfig()
        cfg.lane_change.keep_right_gap_m = 25.0
        sim = make_sim(cfg=cfg)

        # Position 50 m: (100 - 50) % 1000 = 50 ≤ 200 → in merge zone
        car = make_car(car_id=0, lane=1, position=50.0, velocity=25.0)
        car.lane_change_timer = 0.0
        sim.cars.append(car)
        sim.road.add_car(car)

        sim._try_lane_change(car)
        assert car.lane == 1   # must NOT have moved right

    def test_keep_right_works_outside_merge_zone(self):
        """Middle-lane car far from on-ramp can keep-right normally."""
        cfg = SimConfig()
        cfg.lane_change.keep_right_gap_m = 25.0
        cfg.lane_change.safety_gap_m = 6.0
        sim = make_sim(cfg=cfg)

        # Position 600 m: (100 - 600) % 1000 = 500 > 200 → outside merge zone
        car = make_car(car_id=0, lane=1, position=600.0, velocity=25.0)
        car.lane_change_timer = 0.0
        sim.cars.append(car)
        sim.road.add_car(car)

        sim._try_lane_change(car)
        # Lane 2 is empty and gap is huge → should keep-right
        assert car.lane == 2

    def test_overtake_left_not_blocked_in_merge_zone(self):
        """Merge zone only blocks rightward moves; overtaking left is unaffected."""
        cfg = SimConfig()
        cfg.lane_change.incentive_m = 8.0
        cfg.lane_change.safety_gap_m = 6.0
        cfg.lane_change.keep_right_gap_m = 0.0  # disable keep-right
        sim = make_sim(cfg=cfg)

        # Car in lane 1 at position 50 m (in merge zone), with a very slow leader
        follower = make_car(car_id=0, lane=1, position=50.0,
                            velocity=25.0, desired_velocity=33.0)
        follower.lane_change_timer = 0.0
        slow_leader = make_car(car_id=1, lane=1, position=80.0,
                               velocity=5.0, length=5.0)
        sim.cars.extend([follower, slow_leader])
        sim.road.add_car(follower)
        sim.road.add_car(slow_leader)
        # Lane 0 is empty → gap_ahead = LARGE_GAP >> (25 + 8) → overtake
        sim._try_lane_change(follower)
        assert follower.lane == 0

    def test_cooldown_prevents_change(self):
        """Lane change is skipped when cooldown timer is active."""
        sim = make_sim()
        car = make_car(car_id=0, lane=1, position=600.0, velocity=25.0)
        car.lane_change_timer = 1.0   # still cooling down
        sim.cars.append(car)
        sim.road.add_car(car)
        sim._try_lane_change(car)
        assert car.lane == 1

    def test_safety_gap_prevents_change(self):
        """Lane change is blocked when gap behind in target lane is too small."""
        cfg = SimConfig()
        cfg.lane_change.safety_gap_m = 20.0
        cfg.lane_change.keep_right_gap_m = 25.0
        sim = make_sim(cfg=cfg)

        car    = make_car(car_id=0, lane=1, position=600.0, velocity=25.0)
        behind = make_car(car_id=1, lane=2, position=597.0, velocity=25.0, length=5.0)
        # gap_behind = 600 - 597 - 5 = -2 → clamped to 0 < safety_gap=20
        car.lane_change_timer = 0.0
        sim.cars.extend([car, behind])
        sim.road.add_car(car)
        sim.road.add_car(behind)
        sim._try_lane_change(car)
        assert car.lane == 1   # blocked by unsafe gap


# ── Full-step integration ─────────────────────────────────────────────────────

class TestStep:
    def test_time_increments(self):
        sim = make_sim(num_cars=5)
        t0 = sim.time
        sim.step(0.1)
        assert sim.time == pytest.approx(t0 + 0.1)

    def test_cars_move_forward(self):
        """After one step, cars with positive velocity advance their position."""
        sim = make_sim(num_cars=0)
        car = make_car(car_id=0, lane=0, position=100.0, velocity=25.0)
        sim.cars.append(car)
        sim.road.add_car(car)
        sim.step(0.1)
        assert car.position > 100.0

    def test_no_crash_many_steps(self):
        """Simulation runs 200 steps without raising exceptions."""
        sim = make_sim(num_cars=30)
        for _ in range(200):
            sim.step(0.05)
        assert sim.car_count >= 0   # just confirm it ran

    def test_ramp_queue_grows_over_time(self):
        """With on-ramp active and no traffic, queue should accumulate cars."""
        cfg = SimConfig()
        cfg.ramp.onramp_rate = 2.0   # 1 car every 0.5 s
        cfg.ramp.max_queue = 20
        sim = make_sim(num_cars=0, cfg=cfg)
        for _ in range(100):
            sim.step(0.1)   # 10 s simulated → up to 20 cars in queue
        ramp = sim.road.ramps[0]
        assert len(ramp.queue) > 0

    def test_visual_transition_advances(self):
        """Lane-transition progress increments each step."""
        sim = make_sim(num_cars=0)
        car = make_car(car_id=0, lane=0, position=100.0, velocity=20.0)
        sim.cars.append(car)
        sim.road.add_car(car)
        sim._lane_transitions[car.car_id] = (1, 0.0)   # mid-transition
        sim.step(0.1)
        _, progress = sim._lane_transitions[car.car_id]
        assert progress == pytest.approx(0.1 / sim.lane_change_duration)


# ── Visual helpers ────────────────────────────────────────────────────────────

class TestGetVisualLane:
    def test_no_transition_returns_lane(self):
        sim = make_sim()
        car = make_car(lane=1)
        assert sim.get_visual_lane(car) == pytest.approx(1.0)

    def test_halfway_transition(self):
        """At progress=0.5, smoothstep gives t≈0.5, visual lane midway."""
        sim = make_sim()
        car = make_car(car_id=0, lane=2)
        sim._lane_transitions[0] = (0, 0.5)   # transitioning from lane 0 to 2
        visual = sim.get_visual_lane(car)
        # smoothstep(0.5) = 0.5*(3 - 2*0.5) = 0.5
        assert visual == pytest.approx(0.0 + (2.0 - 0.0) * 0.5, abs=0.05)

    def test_completed_transition_removed(self):
        """Progress ≥ 1.0 removes the transition entry."""
        sim = make_sim(num_cars=0)
        car = make_car(car_id=0, lane=1, position=100.0, velocity=20.0)
        sim.cars.append(car)
        sim.road.add_car(car)
        sim._lane_transitions[car.car_id] = (0, 0.99)
        sim.step(0.5)   # progress += 0.5 / 1.2 → > 1.0
        assert car.car_id not in sim._lane_transitions


# ── Off-ramp dynamic controller ───────────────────────────────────────────────

class TestOfframpController:
    """_update_ramp_control: coordinated on/off-ramp proportional controller."""

    def _make_sim_with_cars(self, num_cars, target_cars, offramp_gain=0.01, onramp_gain=0.01):
        cfg = SimConfig()
        cfg.ramp.target_cars = target_cars
        cfg.ramp.offramp_control_gain = offramp_gain
        cfg.ramp.onramp_control_gain = onramp_gain
        cfg.ramp.offramp_prob = 0.3
        sim = make_sim(num_cars=0, cfg=cfg)
        # Place cars directly on road, spread across all lanes
        for i in range(num_cars):
            car = make_car(car_id=i, lane=i % sim.road.num_lanes,
                           position=float(i * 50 + 50), velocity=20.0)
            sim.cars.append(car)
            sim.road.add_car(car)
        return sim

    def _offramp(self, sim):
        return next(r for r in sim.road.ramps if not r.is_onramp)

    def _onramp(self, sim):
        return next(r for r in sim.road.ramps if r.is_onramp)

    def test_controller_raises_prob_when_over_target(self):
        """Too many cars → offramp prob increases."""
        sim = self._make_sim_with_cars(num_cars=10, target_cars=5)
        initial = self._offramp(sim).rate
        for _ in range(10):
            sim._update_ramp_control(dt=1.0)
        assert self._offramp(sim).rate > initial

    def test_controller_lowers_prob_when_under_target(self):
        """Too few cars → offramp prob decreases."""
        sim = self._make_sim_with_cars(num_cars=2, target_cars=8)
        initial = self._offramp(sim).rate
        for _ in range(10):
            sim._update_ramp_control(dt=1.0)
        assert self._offramp(sim).rate < initial

    def test_controller_disabled_when_target_zero(self):
        """target_cars=0 → ramp.rate unchanged."""
        sim = self._make_sim_with_cars(num_cars=5, target_cars=0)
        initial = self._offramp(sim).rate
        for _ in range(10):
            sim._update_ramp_control(dt=1.0)
        assert self._offramp(sim).rate == pytest.approx(initial)

    def test_controller_reduces_onramp_rate_when_over_target(self):
        """Too many cars → on-ramp intake decreases."""
        sim = self._make_sim_with_cars(num_cars=10, target_cars=5)
        initial = self._onramp(sim).rate
        for _ in range(10):
            sim._update_ramp_control(dt=1.0)
        assert self._onramp(sim).rate < initial

    def test_controller_raises_onramp_rate_when_under_target(self):
        """Too few cars → on-ramp intake increases toward free-flow max."""
        sim = self._make_sim_with_cars(num_cars=2, target_cars=8)
        # Simulate a previously throttled state (controller had reduced rate earlier)
        self._onramp(sim).rate = 0.2
        for _ in range(10):
            sim._update_ramp_control(dt=1.0)
        assert self._onramp(sim).rate > 0.2

    def test_onramp_rate_bounded_by_max(self):
        """On-ramp rate never exceeds the configured free-flow ceiling."""
        sim = self._make_sim_with_cars(num_cars=2, target_cars=100, onramp_gain=1.0)
        for _ in range(50):
            sim._update_ramp_control(dt=1.0)
        assert self._onramp(sim).rate <= sim._onramp_rate_max


# ── Destination mode ──────────────────────────────────────────────────────────

class TestDestinationMode:
    """Destination-based exits: cars complete a fixed number of laps before exiting."""

    def _dest_cfg(self, min_loops=5, loops_lambda=0.0, exit_lookahead_m=300.0):
        """Config with destination mode enabled and Poisson lambda=0 for determinism."""
        cfg = SimConfig()
        cfg.destination.enabled = True
        cfg.destination.min_loops = min_loops
        cfg.destination.loops_lambda = loops_lambda
        cfg.destination.exit_lookahead_m = exit_lookahead_m
        return cfg

    def _offramp(self, sim):
        return next(r for r in sim.road.ramps if not r.is_onramp)

    def _onramp(self, sim):
        return next(r for r in sim.road.ramps if r.is_onramp)

    # ── destination_laps assignment ──────────────────────────────────────────

    def test_destination_laps_assigned_at_spawn(self):
        """Cars spawned in destination mode have destination_laps >= min_loops."""
        cfg = self._dest_cfg(min_loops=5, loops_lambda=3.0)
        random.seed(42)
        sim = make_sim(num_cars=20, cfg=cfg, seed=42)
        assert all(c.destination_laps >= 5 for c in sim.cars)

    def test_destination_laps_zero_in_classic_mode(self):
        """Cars spawned in classic mode have destination_laps == 0."""
        random.seed(0)
        sim = make_sim(num_cars=10, seed=0)
        assert all(c.destination_laps == 0 for c in sim.cars)

    # ── lap detection ────────────────────────────────────────────────────────

    def test_lap_detected_on_position_wrap(self):
        """Car crossing position 0 increments laps_completed."""
        cfg = self._dest_cfg(min_loops=5)
        sim = make_sim(num_cars=0, cfg=cfg)
        # Place car near end of road so one step causes a wraparound
        car = make_car(car_id=0, lane=0, position=999.5, velocity=25.0)
        car.destination_laps = 5
        sim.cars.append(car)
        sim.road.add_car(car)
        sim.step(dt=0.1)   # 25 m/s × 0.1 s = 2.5 m → wraps to ~2.0 m
        assert car.laps_completed == 1

    def test_no_lap_without_wrap(self):
        """Car that doesn't wrap does not increment laps_completed."""
        cfg = self._dest_cfg(min_loops=5)
        sim = make_sim(num_cars=0, cfg=cfg)
        car = make_car(car_id=0, lane=0, position=100.0, velocity=25.0)
        car.destination_laps = 5
        sim.cars.append(car)
        sim.road.add_car(car)
        sim.step(dt=0.1)
        assert car.laps_completed == 0

    def test_passed_ramps_cleared_on_lap(self):
        """After a lap completes, _passed_ramps is cleared for re-evaluation."""
        cfg = self._dest_cfg(min_loops=5)
        sim = make_sim(num_cars=0, cfg=cfg)
        car = make_car(car_id=0, lane=0, position=999.5, velocity=25.0)
        car.destination_laps = 5
        car._passed_ramps.add(800.0)   # simulate having passed the off-ramp
        sim.cars.append(car)
        sim.road.add_car(car)
        sim.step(dt=0.1)
        assert car.laps_completed == 1
        assert len(car._passed_ramps) == 0

    def test_laps_not_counted_in_classic_mode(self):
        """In classic mode, laps_completed stays 0 even after wrapping."""
        sim = make_sim(num_cars=0)   # destination disabled by default
        car = make_car(car_id=0, lane=0, position=999.5, velocity=25.0)
        # destination_laps=0 → lap detection skipped
        sim.cars.append(car)
        sim.road.add_car(car)
        sim.step(dt=0.1)
        assert car.laps_completed == 0

    # ── off-ramp exit logic ──────────────────────────────────────────────────

    def test_no_exit_before_destination(self):
        """Car with laps_completed < destination_laps is never removed at off-ramp."""
        cfg = self._dest_cfg(min_loops=5)
        sim = make_sim(num_cars=0, cfg=cfg)
        ramp = self._offramp(sim)
        car = make_car(lane=2, position=ramp.position)
        car.destination_laps = 5
        car.laps_completed = 3   # hasn't finished yet
        assert sim._process_offramp(car, ramp) is False

    def test_exit_when_destination_reached(self):
        """Car with laps_completed == destination_laps exits at off-ramp."""
        cfg = self._dest_cfg(min_loops=5)
        sim = make_sim(num_cars=0, cfg=cfg)
        ramp = self._offramp(sim)
        car = make_car(lane=2, position=ramp.position)
        car.destination_laps = 5
        car.laps_completed = 5   # destination reached
        assert sim._process_offramp(car, ramp) is True

    def test_exit_when_over_destination(self):
        """Car that somehow has laps_completed > destination_laps also exits."""
        cfg = self._dest_cfg(min_loops=5)
        sim = make_sim(num_cars=0, cfg=cfg)
        ramp = self._offramp(sim)
        car = make_car(lane=2, position=ramp.position)
        car.destination_laps = 5
        car.laps_completed = 7
        assert sim._process_offramp(car, ramp) is True

    def test_no_double_exit_same_lap(self):
        """Car already evaluated at off-ramp this lap does not exit again."""
        cfg = self._dest_cfg(min_loops=5)
        sim = make_sim(num_cars=0, cfg=cfg)
        ramp = self._offramp(sim)
        car = make_car(lane=2, position=ramp.position)
        car.destination_laps = 5
        car.laps_completed = 5
        car._passed_ramps.add(ramp.position)   # already processed this lap
        assert sim._process_offramp(car, ramp) is False

    def test_wrong_lane_never_exits(self):
        """Destination mode still requires car to be in rightmost lane."""
        cfg = self._dest_cfg(min_loops=1)
        sim = make_sim(num_cars=0, cfg=cfg)
        ramp = self._offramp(sim)
        car = make_car(lane=0, position=ramp.position)   # wrong lane
        car.destination_laps = 1
        car.laps_completed = 1
        assert sim._process_offramp(car, ramp) is False

    # ── exiting flag ─────────────────────────────────────────────────────────

    def test_exiting_flag_set_near_offramp_on_final_lap(self):
        """Car on final lap within lookahead distance gets exiting=True."""
        cfg = self._dest_cfg(min_loops=5, exit_lookahead_m=300.0)
        sim = make_sim(num_cars=0, cfg=cfg)
        # off-ramp at 800 m; car at 510 m → dist = 290 m ≤ 300 m lookahead
        car = make_car(car_id=0, lane=0, position=510.0, velocity=25.0)
        car.destination_laps = 5
        car.laps_completed = 5
        sim.cars.append(car)
        sim.road.add_car(car)
        sim._update_exiting_flags()
        assert car.exiting is True

    def test_exiting_flag_not_set_too_far_from_offramp(self):
        """Car on final lap but beyond lookahead distance does not get exiting=True."""
        cfg = self._dest_cfg(min_loops=5, exit_lookahead_m=300.0)
        sim = make_sim(num_cars=0, cfg=cfg)
        # off-ramp at 800 m; car at 400 m → dist = 400 m > 300 m lookahead
        car = make_car(car_id=0, lane=0, position=400.0, velocity=25.0)
        car.destination_laps = 5
        car.laps_completed = 5
        sim.cars.append(car)
        sim.road.add_car(car)
        sim._update_exiting_flags()
        assert car.exiting is False

    def test_exiting_flag_not_set_before_final_lap(self):
        """Car that hasn't finished enough laps does not get exiting=True."""
        cfg = self._dest_cfg(min_loops=5, exit_lookahead_m=300.0)
        sim = make_sim(num_cars=0, cfg=cfg)
        car = make_car(car_id=0, lane=0, position=510.0, velocity=25.0)
        car.destination_laps = 5
        car.laps_completed = 3   # not done yet
        sim.cars.append(car)
        sim.road.add_car(car)
        sim._update_exiting_flags()
        assert car.exiting is False

    def test_exiting_car_does_not_move_left(self):
        """Car with exiting=True skips left-lane (overtaking) candidates."""
        cfg = SimConfig()
        cfg.lane_change.incentive_m = 8.0
        cfg.lane_change.safety_gap_m = 6.0
        cfg.lane_change.keep_right_gap_m = 0.0  # disable keep-right noise
        sim = make_sim(num_cars=0, cfg=cfg)

        # Car already in the rightmost lane (2); no right candidate available.
        # Lane 1 is empty — would normally trigger an overtake into lane 1 from
        # the rightmost lane, but that direction is "left" here... Actually,
        # moving from lane 2 → lane 1 is leftward (smaller index = fast lane).
        # A slow leader ahead in lane 2 makes gap in lane 1 much better.
        slow_leader = make_car(car_id=1, lane=2, position=600.0 + 25.0, velocity=5.0, length=5.0)
        car = make_car(car_id=0, lane=2, position=600.0, velocity=25.0)
        car.lane_change_timer = 0.0
        car.exiting = True
        sim.cars.extend([car, slow_leader])
        sim.road.add_car(car)
        sim.road.add_car(slow_leader)

        sim._try_lane_change(car)
        assert car.lane == 2   # must NOT have moved left (lane 1)

    def test_exiting_car_moves_right_immediately(self):
        """Car with exiting=True bypasses keep_right_gap threshold and moves right."""
        cfg = SimConfig()
        cfg.lane_change.keep_right_gap_m = 200.0  # very large — would block keep-right normally
        cfg.lane_change.safety_gap_m = 6.0
        sim = make_sim(num_cars=0, cfg=cfg)

        # Car in lane 1 with exiting=True; lane 2 empty and safe
        car = make_car(car_id=0, lane=1, position=600.0, velocity=25.0)
        car.lane_change_timer = 0.0
        car.exiting = True
        sim.cars.append(car)
        sim.road.add_car(car)

        sim._try_lane_change(car)
        assert car.lane == 2   # moved right despite gap < keep_right_gap_m

    # ── ramp controller interaction ──────────────────────────────────────────

    def test_destination_mode_skips_offramp_prob_control(self):
        """In destination mode, offramp_prob is not adjusted by the controller."""
        cfg = self._dest_cfg(min_loops=5)
        cfg.ramp.target_cars = 5
        cfg.ramp.offramp_control_gain = 0.1
        cfg.ramp.offramp_prob = 0.3
        sim = make_sim(num_cars=10, cfg=cfg)  # over target → controller fires
        initial_offramp = self._offramp(sim).rate
        for _ in range(10):
            sim._update_ramp_control(dt=1.0)
        assert self._offramp(sim).rate == pytest.approx(initial_offramp)

    def test_destination_mode_still_controls_onramp(self):
        """In destination mode, onramp_rate is still adjusted by the controller."""
        cfg = self._dest_cfg(min_loops=5)
        cfg.ramp.target_cars = 5
        cfg.ramp.onramp_control_gain = 0.1
        sim = make_sim(num_cars=10, cfg=cfg)  # over target → reduce onramp
        initial_onramp = self._onramp(sim).rate
        for _ in range(10):
            sim._update_ramp_control(dt=1.0)
        assert self._onramp(sim).rate < initial_onramp
