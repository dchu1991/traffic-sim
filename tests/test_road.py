"""Tests for Road gap queries: find_leader and find_gap_in_lane."""
import pytest
from traffic_sim.road import Road, LARGE_GAP
from tests.helpers import make_car


def make_road(length=1000.0, num_lanes=3, limits_ms=None):
    if limits_ms is None:
        limits_ms = [36.1, 30.6, 25.0]
    return Road(length=length, num_lanes=num_lanes, lane_speed_limits=limits_ms)


class TestFindLeader:
    def test_no_other_cars_returns_large_gap(self):
        """Single car in lane → LARGE_GAP and own desired_velocity."""
        road = make_road(num_lanes=1, limits_ms=[30.0])
        car = make_car(lane=0, position=100.0, desired_velocity=25.0)
        road.add_car(car)
        gap, lead_v = road.find_leader(car)
        assert gap == pytest.approx(LARGE_GAP)
        assert lead_v == pytest.approx(car.desired_velocity)

    def test_normal_leader_ahead(self):
        """Gap = leader.position − follower.position − leader.length."""
        road = make_road(num_lanes=1, limits_ms=[30.0])
        follower = make_car(car_id=0, lane=0, position=100.0, velocity=20.0)
        leader   = make_car(car_id=1, lane=0, position=150.0, velocity=25.0, length=5.0)
        road.add_car(follower)
        road.add_car(leader)
        gap, lead_v = road.find_leader(follower)
        assert gap == pytest.approx(45.0)   # 150 - 100 - 5
        assert lead_v == pytest.approx(25.0)

    def test_nearest_leader_chosen(self):
        """When multiple cars are ahead, the closest one is returned."""
        road = make_road(num_lanes=1, limits_ms=[30.0])
        follower = make_car(car_id=0, lane=0, position=100.0, velocity=20.0)
        near     = make_car(car_id=1, lane=0, position=120.0, velocity=15.0, length=4.0)
        far      = make_car(car_id=2, lane=0, position=200.0, velocity=10.0, length=4.0)
        for c in (follower, near, far):
            road.add_car(c)
        gap, lead_v = road.find_leader(follower)
        assert gap == pytest.approx(16.0)   # 120 - 100 - 4
        assert lead_v == pytest.approx(15.0)

    def test_wraparound_leader(self):
        """Leader is behind on the circular road — gap computed across boundary."""
        road = make_road(length=1000.0, num_lanes=1, limits_ms=[30.0])
        follower = make_car(car_id=0, lane=0, position=900.0, velocity=20.0)
        leader   = make_car(car_id=1, lane=0, position=50.0,  velocity=15.0, length=5.0)
        road.add_car(follower)
        road.add_car(leader)
        gap, lead_v = road.find_leader(follower)
        # gap = (1000 - 900) + 50 - 5 = 145
        assert gap == pytest.approx(145.0)
        assert lead_v == pytest.approx(15.0)

    def test_gap_nonnegative_when_overlapping(self):
        """Gap is clamped to 0 if cars are overlapping (shouldn't happen, but safe)."""
        road = make_road(num_lanes=1, limits_ms=[30.0])
        follower = make_car(car_id=0, lane=0, position=100.0, length=5.0)
        leader   = make_car(car_id=1, lane=0, position=102.0, length=5.0)
        road.add_car(follower)
        road.add_car(leader)
        gap, _ = road.find_leader(follower)
        assert gap >= 0.0

    def test_ignores_cars_in_other_lanes(self):
        """Leader search only looks within the car's own lane."""
        road = make_road(num_lanes=2, limits_ms=[30.0, 25.0])
        car        = make_car(car_id=0, lane=0, position=100.0)
        other_lane = make_car(car_id=1, lane=1, position=120.0, velocity=15.0)
        road.add_car(car)
        road.add_car(other_lane)
        gap, _ = road.find_leader(car)
        assert gap == pytest.approx(LARGE_GAP)


class TestFindGapInLane:
    def test_empty_target_lane(self):
        """No cars in target lane → LARGE_GAP both ways."""
        road = make_road(num_lanes=2, limits_ms=[30.0, 25.0])
        car = make_car(car_id=0, lane=0, position=500.0)
        road.add_car(car)
        gap_ahead, gap_behind = road.find_gap_in_lane(car, target_lane=1)
        assert gap_ahead  == pytest.approx(LARGE_GAP)
        assert gap_behind == pytest.approx(LARGE_GAP)

    def test_gaps_with_neighbors(self):
        """Correct gap_ahead and gap_behind with one car each side."""
        road = make_road(num_lanes=2, limits_ms=[30.0, 25.0])
        car    = make_car(car_id=0, lane=0, position=100.0, length=5.0)
        ahead  = make_car(car_id=1, lane=1, position=130.0, length=5.0)
        behind = make_car(car_id=2, lane=1, position=70.0,  length=5.0)
        for c in (car, ahead, behind):
            road.add_car(c)
        gap_ahead, gap_behind = road.find_gap_in_lane(car, target_lane=1)
        # gap_ahead  = 130 - 100 - 5 = 25
        # gap_behind = 100 - 70 - 5  = 25  (car.position − behind.position − car.length)
        assert gap_ahead  == pytest.approx(25.0)
        assert gap_behind == pytest.approx(25.0)

    def test_wraparound_gap_ahead(self):
        """Wraparound: all target-lane cars are behind → wrap-around gap_ahead."""
        road = make_road(length=1000.0, num_lanes=2, limits_ms=[30.0, 25.0])
        car    = make_car(car_id=0, lane=0, position=900.0, length=5.0)
        target = make_car(car_id=1, lane=1, position=50.0,  length=5.0)
        road.add_car(car)
        road.add_car(target)
        gap_ahead, _ = road.find_gap_in_lane(car, target_lane=1)
        # gap_ahead = (1000 - 900) + 50 - 5 = 145
        assert gap_ahead == pytest.approx(145.0)

    def test_gap_nonnegative(self):
        """Gaps are always ≥ 0 even for overlapping positions."""
        road = make_road(num_lanes=2, limits_ms=[30.0, 25.0])
        car    = make_car(car_id=0, lane=0, position=100.0, length=10.0)
        target = make_car(car_id=1, lane=1, position=103.0, length=5.0)
        road.add_car(car)
        road.add_car(target)
        gap_ahead, gap_behind = road.find_gap_in_lane(car, target_lane=1)
        assert gap_ahead  >= 0.0
        assert gap_behind >= 0.0
