from __future__ import annotations

import json
import os
from datetime import datetime
from typing import TYPE_CHECKING

import polars as pl

if TYPE_CHECKING:
    from .simulation import Simulation


class Recorder:
    """
    Samples simulation state at regular intervals and writes Parquet files on save().

    Aggregate file columns:
        time_s, car_count, avg_speed_kmh, density_veh_per_km, flow_veh_per_h

    Per-car trajectory file columns (--record-cars):
        time_s, car_id, lane, position_m, speed_kmh, accel_ms2
    """

    def __init__(self, sample_interval: float = 1.0, record_cars: bool = False,
                 metadata: dict | None = None):
        self.sample_interval = sample_interval
        self.record_cars = record_cars
        self._metadata: dict = metadata or {}
        self._next_sample: float = 0.0

        # Column-oriented buffers — faster to build DataFrames from than list[dict]
        self._agg: dict[str, list] = {
            "time_s": [], "car_count": [], "avg_speed_kmh": [],
            "density_veh_per_km": [], "flow_veh_per_h": [], "offramp_prob": [],
        }
        self._traj: dict[str, list] = {
            "time_s": [], "car_id": [], "lane": [],
            "position_m": [], "speed_kmh": [], "accel_ms2": [],
        }

    # ------------------------------------------------------------------

    def sample(self, sim: Simulation) -> None:
        if sim.time < self._next_sample:
            return
        self._next_sample += self.sample_interval

        speeds = [c.velocity * 3.6 for c in sim.cars]
        n = len(speeds)
        avg_speed = sum(speeds) / n if n else 0.0
        density   = n / (sim.road.length / 1000.0)   # veh / km
        flow      = density * avg_speed               # veh / h

        offramp_prob = next((r.rate for r in sim.road.ramps if not r.is_onramp), 0.0)

        self._agg["time_s"].append(round(sim.time, 2))
        self._agg["car_count"].append(n)
        self._agg["avg_speed_kmh"].append(round(avg_speed, 2))
        self._agg["density_veh_per_km"].append(round(density, 2))
        self._agg["flow_veh_per_h"].append(round(flow, 1))
        self._agg["offramp_prob"].append(round(offramp_prob, 4))

        if self.record_cars:
            t = round(sim.time, 2)
            for car in sim.cars:
                self._traj["time_s"].append(t)
                self._traj["car_id"].append(car.car_id)
                self._traj["lane"].append(car.lane)
                self._traj["position_m"].append(round(car.position, 1))
                self._traj["speed_kmh"].append(round(car.velocity * 3.6, 1))
                self._traj["accel_ms2"].append(round(car.acceleration, 3))

    # ------------------------------------------------------------------

    def save(self, output_dir: str = ".") -> list[str]:
        """Write Parquet files. Returns list of written paths."""
        if not self._agg["time_s"]:
            return []

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        written: list[str] = []

        agg_path = os.path.join(output_dir, f"traffic_aggregate_{ts}.parquet")
        pl.DataFrame(self._agg).write_parquet(agg_path)
        written.append(agg_path)

        if self._traj["time_s"]:
            traj_path = os.path.join(output_dir, f"traffic_cars_{ts}.parquet")
            pl.DataFrame(self._traj).write_parquet(traj_path)
            written.append(traj_path)

        if self._metadata:
            meta_path = os.path.join(output_dir, f"traffic_meta_{ts}.json")
            with open(meta_path, "w") as f:
                json.dump(self._metadata, f, indent=2)
            written.append(meta_path)

        return written

    # ------------------------------------------------------------------

    def aggregate_df(self) -> pl.DataFrame:
        """Return the current aggregate data as a live Polars DataFrame."""
        return pl.DataFrame(self._agg)

    def trajectories_df(self) -> pl.DataFrame:
        """Return the current per-car trajectory data as a live Polars DataFrame."""
        return pl.DataFrame(self._traj)

    @property
    def sample_count(self) -> int:
        return len(self._agg["time_s"])
