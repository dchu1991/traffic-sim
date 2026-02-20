"""Shared test helpers — deterministic car factory."""
from traffic_sim.car import Car


def make_car(
    car_id: int = 0,
    lane: int = 0,
    position: float = 0.0,
    velocity: float = 20.0,
    length: float = 4.5,
    desired_velocity: float = 30.0,
    time_headway: float = 1.5,
    min_gap: float = 2.0,
    max_accel: float = 1.5,
    comfortable_decel: float = 2.0,
) -> Car:
    return Car(
        car_id=car_id,
        lane=lane,
        position=position,
        velocity=velocity,
        color=(100, 150, 200),
        desired_velocity=desired_velocity,
        time_headway=time_headway,
        min_gap=min_gap,
        max_accel=max_accel,
        comfortable_decel=comfortable_decel,
        length=length,
    )
