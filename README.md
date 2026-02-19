# Traffic Simulator

A freeway traffic simulator built in Python with real-time pygame visualization. Cars follow the **Intelligent Driver Model (IDM)** for physically realistic car-following, perform **MOBIL-inspired lane changes**, and interact with **on/off ramps** — producing emergent stop-and-go traffic jams at high densities.

![demo](https://via.placeholder.com/900x200/373a47/ffffff?text=uv+run+traffic-sim)

## Features

- **IDM car-following** — each car has randomised `desired_velocity`, `time_headway`, `min_gap`, and `max_accel`; heterogeneity produces phantom traffic jams naturally
- **Trucks** — longer, slower vehicles that act as bottlenecks
- **Lane changes** — safety + incentive check (MOBIL-lite), smooth animated transitions
- **On/off ramps** — on-ramp spawns cars when a sufficient gap exists; off-ramp removes passing cars probabilistically
- **Data recording** — optional Parquet export of aggregate stats and per-car trajectories for analysis with Polars
- **pygame HUD** — live speed, car count, density bar; pause/speed controls

## Quick start

```bash
# requires Python 3.13+ and uv
git clone https://github.com/dchu1991/traffic-sim
cd traffic-sim
uv run traffic-sim
```

## CLI options

```
uv run traffic-sim [OPTIONS]

  --lanes     INT    Number of lanes                     (default: 3)
  --cars      INT    Initial number of cars              (default: 50)
  --length    FLOAT  Road length in metres               (default: 1000)
  --trucks    FLOAT  Fraction of vehicles that are trucks(default: 0.15)
  --fps       INT    Target frame rate                   (default: 60)
  --width     INT    Window width in pixels              (default: 1400)

  --record             Record aggregate stats to Parquet on exit
  --record-cars        Also record per-car trajectories (larger file)
  --record-interval FLOAT  Sample interval in sim-seconds (default: 1.0)
```

### Examples

```bash
# Dense traffic — jams form visibly
uv run traffic-sim --lanes 3 --cars 80

# Long road with more trucks
uv run traffic-sim --length 2000 --trucks 0.3

# Record a 5-minute run with per-car data
uv run traffic-sim --record --record-cars --record-interval 0.5
```

## Controls

| Key | Action |
|-----|--------|
| `SPACE` | Pause / resume |
| `↑` / `↓` | Double / halve simulation speed (0.25× – 16×) |
| `Q` / `Esc` | Quit (saves data if recording) |

Car colour encodes speed: 🔴 stopped → 🟡 medium → 🟢 highway speed.

## Data analysis

When `--record` is passed, two Parquet files are written to `logs/` on exit:

| File | Columns |
|------|---------|
| `logs/traffic_aggregate_<ts>.parquet` | `time_s`, `car_count`, `avg_speed_kmh`, `density_veh_per_km`, `flow_veh_per_h` |
| `logs/traffic_cars_<ts>.parquet` | `time_s`, `car_id`, `lane`, `position_m`, `speed_kmh`, `accel_ms2` |

The `logs/` directory is git-ignored.

```python
import polars as pl

agg  = pl.read_parquet("logs/traffic_aggregate_*.parquet")
cars = pl.read_parquet("logs/traffic_cars_*.parquet")

# Fundamental diagram (speed vs density)
agg.select(["density_veh_per_km", "avg_speed_kmh", "flow_veh_per_h"])

# Space-time diagram for one car
cars.filter(pl.col("car_id") == 5).sort("time_s")

# Average speed per lane over time
cars.group_by(["time_s", "lane"]).agg(pl.col("speed_kmh").mean())
```

## Project structure

```
src/traffic_sim/
├── car.py          # Car dataclass + IDM acceleration model
├── road.py         # Road (lanes + ramps) + gap query helpers
├── simulation.py   # Orchestrates IDM, lane changes, on/off ramps
├── recorder.py     # Polars-backed data sampler + Parquet writer
├── visualizer.py   # pygame top-down renderer
└── main.py         # CLI entry point (argparse)
```

## Development

```bash
# Install dependencies
uv sync

# Run with extra density to see jams form
uv run traffic-sim --cars 80 --lanes 3
```

Dependencies: `pygame`, `numpy`, `polars` — all managed by `uv`.
