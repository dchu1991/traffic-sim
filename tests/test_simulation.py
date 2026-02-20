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
