from __future__ import annotations

import argparse
import dataclasses
import os

from .config import SimConfig
from .recorder import Recorder
from .simulation import Simulation
from .visualizer import Visualizer

_DEFAULT_CONFIG = "config.toml"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Freeway traffic simulator — IDM + lane changes + on/off ramps"
    )
    parser.add_argument("--lanes",           type=int,   default=3,      help="Number of lanes (default: 3)")
    parser.add_argument("--cars",            type=int,   default=50,     help="Initial number of cars (default: 50)")
    parser.add_argument("--length",          type=float, default=1000.0, help="Road length in metres (default: 1000)")
    parser.add_argument("--fps",             type=int,   default=60,     help="Target FPS (default: 60)")
    parser.add_argument("--width",           type=int,   default=1400,   help="Window width in pixels (default: 1400)")
    parser.add_argument("--trucks",          type=float, default=0.15,   help="Fraction of cars that are trucks (default: 0.15)")
    parser.add_argument("--config",          type=str,   default=None,   help=f"Path to behaviour config TOML (default: {_DEFAULT_CONFIG} if present)")
    parser.add_argument("--record",          action="store_true",        help="Record aggregate stats to Parquet on exit")
    parser.add_argument("--record-cars",     action="store_true",        help="Also record per-car trajectories (larger file)")
    parser.add_argument("--record-interval", type=float, default=1.0,   help="Sample interval in sim-seconds (default: 1.0)")
    args = parser.parse_args()

    # Load config: explicit --config > default config.toml > built-in defaults
    config_path = args.config or (_DEFAULT_CONFIG if os.path.exists(_DEFAULT_CONFIG) else None)
    if config_path:
        cfg = SimConfig.from_toml(config_path)
        print(f"Loaded config: {config_path}")
    else:
        cfg = SimConfig()

    sim = Simulation(
        road_length=args.length,
        num_lanes=args.lanes,
        num_cars=args.cars,
        truck_fraction=args.trucks,
        config=cfg,
    )

    recorder = None
    if args.record or args.record_cars:
        metadata = {
            "road_length_m":    args.length,
            "num_lanes":        args.lanes,
            "num_cars":         args.cars,
            "truck_fraction":   args.trucks,
            "record_interval_s": args.record_interval,
            "config":           dataclasses.asdict(cfg),
        }
        recorder = Recorder(
            sample_interval=args.record_interval,
            record_cars=args.record_cars,
            metadata=metadata,
        )

    viz = Visualizer(sim, recorder=recorder, width=args.width, fps=args.fps)
    viz.run()


if __name__ == "__main__":
    main()
