# Traffic Simulator

A freeway traffic simulator built in Python with real-time pygame visualization. Cars follow the **Intelligent Driver Model (IDM)** for physically realistic car-following, perform **MOBIL-inspired lane changes**, and interact with an **on-ramp queue** — producing emergent stop-and-go traffic jams at high densities.

## Features

- **IDM car-following** — each car has randomised `desired_velocity`, `time_headway`, `min_gap`, and `max_accel`; heterogeneity produces phantom traffic jams naturally
- **Trucks** — longer, slower vehicles that act as bottlenecks
- **Lane changes** — full MOBIL criterion for overtaking (acceleration gain across self + new/old follower, configurable politeness factor `p`), smooth animated transitions; keep-right rule enforced when the fast lane is clear
- **Per-lane speed limits** — default 130 / 110 / 90 km/h; enforced via IDM effective target speed
- **On-ramp queue** — cars wait in a separate ramp lane and merge when a safe gap opens; the merge window is highlighted with dashed lines and a tinted background
- **Zipper merge** — when the rightmost lane slows below a configurable threshold, the gap requirement is relaxed (~1 car length) so the queue can drain even in stop-and-go traffic
- **Off-ramp** — removes passing cars probabilistically (classic) or after a set number of laps (destination mode)
- **Cooperative yielding** — road cars near the ramp tip shorten their gap to the ramp car, helping it slot in
- **Behaviour config** — all driving parameters live in `config.toml`; no code changes needed
- **Dynamic ramp control** — optional proportional controller holds a target car count by adjusting `onramp_rate` and `offramp_prob` in real time (`target_cars` in `[ramp]`)
- **Destination exits** — each car is assigned a random number of laps (`min_loops + Poisson(λ)`) and only exits at the off-ramp after completing them; the car commits to the rightmost lane early via a configurable lookahead distance (`[destination]` in `config.toml`)
- **Data recording** — optional Parquet export of aggregate stats, per-car trajectories, and a JSON metadata sidecar; interactive analysis notebook included
- **pygame HUD** — live speed, car count, density bar, per-lane speed limits; pause/speed controls

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

  --lanes     INT    Number of lanes                      (default: 3)
  --cars      INT    Initial number of cars               (default: 50)
  --length    FLOAT  Road length in metres                (default: 1000)
  --trucks    FLOAT  Fraction of vehicles that are trucks (default: 0.15)
  --fps       INT    Target frame rate                    (default: 60)
  --width     INT    Window width in pixels               (default: 1400)
  --config    PATH   Behaviour config TOML                (default: config.toml if present)

  --record              Record aggregate stats to Parquet on exit
  --record-cars         Also record per-car trajectories (larger file)
  --record-interval FLOAT  Sample interval in sim-seconds (default: 1.0)
```

### Examples

```bash
# Dense traffic — jams form visibly, ramp queue fills
uv run traffic-sim --lanes 3 --cars 80

# Long road with more trucks
uv run traffic-sim --length 2000 --trucks 0.3

# Custom behaviour (autobahn-style speeds, aggressive merging)
uv run traffic-sim --config autobahn.toml

# Record a 5-minute run with per-car trajectories
uv run traffic-sim --record --record-cars --record-interval 0.5
```

## Controls

| Key | Action |
|-----|--------|
| `SPACE` | Pause / resume |
| `↑` / `↓` | Double / halve simulation speed (0.25× – 16×) |
| `Q` / `Esc` | Quit (saves data if recording) |

Car colour encodes speed: red (stopped) → yellow (medium) → green (highway speed).

## Behaviour configuration

All driving parameters are in `config.toml`. Edit it to tune behaviour without touching any Python.

```toml
[road]
# Speed limits in km/h, left (fast lane) → right (slow lane)
lane_speed_limits_kmh = [130, 110, 90]

[lane_change]
cooldown_s            = 3.0   # minimum seconds between lane changes per car
safety_gap_m          = 6.0   # minimum gap (m) behind in target lane (hard floor)
keep_right_gap_m      = 25.0  # gap (m) needed to merge RIGHT; 0 = disabled
duration_s            = 1.2   # visual lane-change animation duration (cosmetic)
politeness            = 0.0   # MOBIL p: 0 = selfish, 0.3 = polite, 1 = altruistic
delta_a_threshold_ms2 = 0.2   # m/s² gain required to overtake LEFT (replaces incentive_m)

[ramp]
onramp_position  = 0.10  # fraction of road length
offramp_position = 0.80
onramp_rate      = 0.5   # new cars spawned per second (0 = disabled)
offramp_prob     = 0.3   # probability a passing car takes the off-ramp (ignored in destination mode)
min_gap_m        = 30.0  # safety gap (m) required in front and behind to merge
merge_window_m   = 100.0 # how far back from the ramp tip a car may attempt to merge
zipper_speed_kmh = 30.0  # rightmost lane avg speed below which zipper merge activates
zipper_gap_m     = 8.0   # gap required in zipper mode (~1 car length)
max_queue        = 10    # max cars waiting on ramp (0 = no limit)
ramp_length_m    = 200.0 # physical ramp length in metres
# Dynamic ramp control — adjusts onramp_rate + offramp_prob to hold this car count (0 = off)
target_cars           = 0
onramp_control_gain   = 0.001
offramp_control_gain  = 0.001

[cars]
desired_v_mean_ms = 33.0  # ~120 km/h
time_headway_mean =  2.0  # s (tighter = more unstable traffic)

[trucks]
desired_v_mean_ms = 22.0  # ~80 km/h

[destination]
# When enabled, each car drives a set number of laps before exiting (offramp_prob is ignored).
enabled          = false
min_loops        = 5       # every car completes at least this many laps
loops_lambda     = 3.0     # Poisson(λ) extra laps; mean destination = min_loops + λ
exit_lookahead_m = 300.0   # metres before off-ramp where car commits to rightmost lane
```

Key tuning knobs:

| Goal | Parameter |
|------|-----------|
| More / fewer jams | `--cars`, or `time_headway_mean` |
| Higher speed limit | `lane_speed_limits_kmh` |
| Aggressive overtaking | lower `delta_a_threshold_ms2` (even negative) |
| Reduce weaving / polite merges | raise `politeness` to 0.3–0.5 |
| Disable keep-right | `keep_right_gap_m = 0` |
| Busier ramp | raise `onramp_rate` |
| More zipper merging | raise `zipper_speed_kmh` |
| Hold a steady car count | set `target_cars` (enables proportional controller) |
| Destination-based exits | `enabled = true` in `[destination]` |
| Adjust exit lap spread | `min_loops` (floor) + `loops_lambda` (Poisson mean) |

## On-ramp queue & merge logic

Cars spawned at the on-ramp enter a **separate ramp lane** and drive along the acceleration lane at increasing speed. When a car reaches the **merge window** (last `merge_window_m` of the ramp), it starts looking for a gap in the rightmost road lane.

The merge window is rendered with a **tinted background** and **dashed lines** along the top edge and left boundary, so it's immediately visible on screen.

Under normal conditions a car merges when:
- gap ahead ≥ `min_gap_m` (default 30 m)
- gap behind ≥ `min_gap_m × 0.5`

When the rightmost road lane is congested (average speed < `zipper_speed_kmh`), **zipper merge** activates:
- gap ahead ≥ `zipper_gap_m` (default 8 m)
- gap behind — not checked (road car reacts via IDM within one tick)

Road cars within the merge window also **cooperatively yield** by treating the ramp car as a virtual leader, shortening their gap and creating space to merge.

## Data analysis

When `--record` is passed, three files are written to `logs/` on exit:

| File | Contents |
|------|---------|
| `logs/traffic_aggregate_<ts>.parquet` | `time_s`, `car_count`, `avg_speed_kmh`, `density_veh_per_km`, `flow_veh_per_h`, `onramp_rate`, `offramp_prob` |
| `logs/traffic_cars_<ts>.parquet` | `time_s`, `car_id`, `lane`, `position_m`, `speed_kmh`, `accel_ms2`, `laps_completed`, `destination_laps` (`--record-cars` only) |
| `logs/traffic_meta_<ts>.json` | CLI args + full config snapshot for the run |

`onramp_rate` and `offramp_prob` are sampled live — if the dynamic controller is active they reflect the actual values at each moment, not just the config defaults.

The `logs/` directory is git-ignored.

### Analysis notebook

An interactive Jupyter notebook lives in `notebook/analysis.ipynb` (also git-ignored):

```bash
uv run jupyter lab notebook/analysis.ipynb
```

Charts included: speed & car count over time (per-lane bands), ramp control signals, fundamental diagram, speed distribution by lane, space–time diagram (all lanes, custom traffic colorscale), car lifetime table, travel time histogram + violin by destination laps, TTC histogram / near-miss rate / mean TTC by lane, destination lap distribution (destination mode only).

## Project structure

```
config.toml             # behaviour config (edit to tune without code changes)
src/traffic_sim/
├── config.py       # SimConfig dataclass + TOML loader (incl. DestinationConfig)
├── car.py          # Car dataclass + IDM acceleration model
├── road.py         # Road (lanes + ramps + per-lane speed limits) + gap helpers
├── simulation.py   # Orchestrates IDM, lane changes, ramp queue, zipper merge, destination exits
├── recorder.py     # Polars-backed data sampler + Parquet writer
├── visualizer.py   # pygame renderer + merge window visuals + HUD
└── main.py         # CLI entry point (argparse)
tests/
└── test_simulation.py  # unit tests (car, road, config, simulation, destination mode)
docs/
├── idm-model.md            # IDM equations, alternative car-following models, citations
├── mobil-lane-change.md    # MOBIL criterion, politeness factor, implementation notes
├── lane-change-models.md   # survey of lane-change models (Gipps, Ahmed, MOBIL, Lmrs, RSS, Toledo)
└── destination-exits.md    # design notes for destination-based exit behaviour
```

## Development

```bash
uv sync                        # install dependencies
uv run pytest tests/ -q        # run all tests
uv run traffic-sim --cars 80   # dense traffic to see jams and ramp queue
```

Runtime dependencies: `pygame`, `numpy`, `polars` — all managed by `uv` (Python 3.13).
Dev dependencies (notebook): `jupyter`, `matplotlib`, `plotly`, `pandas`, `pyarrow`, `anywidget`.
