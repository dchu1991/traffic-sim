"""Unit tests for the Recorder class."""
from __future__ import annotations

import glob
import tempfile

import polars as pl
import pytest

from traffic_sim.recorder import Recorder
from traffic_sim.simulation import Simulation


def _make_sim(num_cars: int = 1) -> Simulation:
    """Return a minimal Simulation with a fixed seed for reproducibility."""
    import random
    import numpy as np
    random.seed(42)
    np.random.seed(42)
    return Simulation(road_length=500.0, num_lanes=1, num_cars=num_cars, truck_fraction=0.0)


class TestRecorder:
    def test_sample_populates_aggregate_columns(self):
        rec = Recorder(sample_interval=0.0)
        sim = _make_sim(num_cars=5)
        rec.sample(sim)
        assert rec.sample_count == 1
        buf = rec._agg
        assert set(buf.keys()) == {
            "time_s", "car_count", "avg_speed_kmh",
            "density_veh_per_km", "flow_veh_per_h",
            "onramp_rate", "offramp_prob",
        }
        assert buf["car_count"][0] == len(sim.cars)
        assert buf["time_s"][0] == pytest.approx(round(sim.time, 2))
        # All 7 keys must have exactly 1 entry
        for key in buf:
            assert len(buf[key]) == 1

    def test_sample_populates_trajectory_columns(self):
        rec = Recorder(sample_interval=0.0, record_cars=True)
        sim = _make_sim(num_cars=1)
        rec.sample(sim)
        buf = rec._traj
        assert set(buf.keys()) == {
            "time_s", "car_id", "lane", "position_m",
            "speed_kmh", "accel_ms2", "laps_completed", "destination_laps",
        }
        # One row per car
        assert len(buf["car_id"]) == len(sim.cars)
        car = sim.cars[0]
        assert buf["lane"][0] == car.lane
        assert buf["laps_completed"][0] == car.laps_completed
        assert buf["destination_laps"][0] == car.destination_laps

    def test_save_writes_aggregate_parquet(self):
        rec = Recorder(sample_interval=0.0)
        sim = _make_sim(num_cars=3)
        rec.sample(sim)
        output_dir = tempfile.mkdtemp()
        written = rec.save(output_dir=output_dir)
        agg_files = glob.glob(f"{output_dir}/traffic_aggregate_*.parquet")
        assert len(agg_files) == 1
        df = pl.read_parquet(agg_files[0])
        assert df.shape[0] == 1
        assert set(df.columns) == {
            "time_s", "car_count", "avg_speed_kmh",
            "density_veh_per_km", "flow_veh_per_h",
            "onramp_rate", "offramp_prob",
        }

    def test_save_writes_trajectory_parquet(self):
        rec = Recorder(sample_interval=0.0, record_cars=True)
        sim = _make_sim(num_cars=2)
        rec.sample(sim)
        output_dir = tempfile.mkdtemp()
        rec.save(output_dir=output_dir)
        traj_files = glob.glob(f"{output_dir}/traffic_cars_*.parquet")
        assert len(traj_files) == 1
        df = pl.read_parquet(traj_files[0])
        assert df.shape[0] == len(sim.cars)
        assert set(df.columns) == {
            "time_s", "car_id", "lane", "position_m",
            "speed_kmh", "accel_ms2", "laps_completed", "destination_laps",
        }
