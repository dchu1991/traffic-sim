"""Tests for Car IDM acceleration and kinematic update."""
import pytest
from tests.helpers import make_car


class TestIdmAcceleration:
    def test_free_flow_accelerates(self):
        """Car with large gap and low velocity accelerates toward desired speed."""
        car = make_car(velocity=10.0, desired_velocity=30.0)
        accel = car.idm_acceleration(gap=500.0, lead_velocity=30.0)
        assert accel > 0

    def test_close_leader_brakes(self):
        """Car close behind a slow leader decelerates."""
        car = make_car(velocity=30.0, desired_velocity=30.0)
        accel = car.idm_acceleration(gap=3.0, lead_velocity=0.0)
        assert accel < 0

    def test_zero_gap_max_braking(self):
        """Gap of zero triggers the maximum braking clamp (-9 m/s²)."""
        car = make_car(velocity=20.0, desired_velocity=30.0)
        accel = car.idm_acceleration(gap=0.0, lead_velocity=20.0)
        assert accel == pytest.approx(-9.0)

    def test_at_desired_speed_no_net_accel(self):
        """Car already at desired speed with large gap has near-zero acceleration."""
        car = make_car(velocity=30.0, desired_velocity=30.0)
        accel = car.idm_acceleration(gap=500.0, lead_velocity=30.0)
        assert abs(accel) < 0.1

    def test_effective_v0_caps_target(self):
        """Speed-limit override suppresses acceleration when already at limit."""
        car = make_car(velocity=25.0, desired_velocity=40.0)
        # At v=25 with effective_v0=25 the free-flow term is ~0 → near-zero accel
        accel = car.idm_acceleration(gap=500.0, lead_velocity=40.0, effective_v0=25.0)
        assert accel < 0.1

    def test_result_clipped_to_max_accel(self):
        """Positive acceleration never exceeds max_accel."""
        car = make_car(velocity=0.0, desired_velocity=30.0, max_accel=1.5)
        accel = car.idm_acceleration(gap=500.0, lead_velocity=30.0)
        assert accel <= car.max_accel + 1e-9


class TestUpdate:
    def test_position_wraps_on_circular_road(self):
        """Position wraps back to near-zero after crossing road_length."""
        car = make_car(position=990.0, velocity=20.0)
        car.update(dt=1.0, gap=500.0, lead_velocity=30.0, road_length=1000.0)
        assert car.position < 50.0

    def test_velocity_stays_nonnegative(self):
        """Velocity is clamped to zero even under heavy braking."""
        car = make_car(velocity=0.1)
        car.update(dt=1.0, gap=0.0, lead_velocity=0.0, road_length=1000.0)
        assert car.velocity >= 0.0

    def test_speed_limit_enforced(self):
        """Car doesn't exceed the lane speed limit."""
        car = make_car(velocity=20.0, desired_velocity=40.0)
        speed_limit = 25.0
        for _ in range(200):
            car.update(dt=0.1, gap=500.0, lead_velocity=40.0,
                       road_length=1000.0, speed_limit=speed_limit)
        assert car.velocity <= speed_limit + 1e-6

    def test_cooldown_decrements(self):
        """Lane-change cooldown timer decreases by dt each step."""
        car = make_car(velocity=20.0)
        car.lane_change_timer = 3.0
        car.update(dt=0.5, gap=500.0, lead_velocity=20.0, road_length=1000.0)
        assert car.lane_change_timer == pytest.approx(2.5)

    def test_cooldown_does_not_go_negative(self):
        """Cooldown clamps at zero, not below."""
        car = make_car(velocity=20.0)
        car.lane_change_timer = 0.1
        car.update(dt=1.0, gap=500.0, lead_velocity=20.0, road_length=1000.0)
        assert car.lane_change_timer == pytest.approx(0.0)
