# Traffic Simulator — Claude Notes

## Running the project

```bash
uv run traffic-sim                             # default: 3 lanes, 50 cars, 1000m road
uv run traffic-sim --lanes 4 --cars 80         # more lanes and cars
uv run traffic-sim --length 2000 --trucks 0.3  # longer road, more trucks
uv run traffic-sim --record --record-cars      # record to Parquet on exit
```

Full options: `--lanes`, `--cars`, `--length`, `--fps`, `--width`, `--trucks`,
`--record`, `--record-cars`, `--record-interval`

## Project structure

```
src/traffic_sim/
├── car.py          # Car dataclass + IDM acceleration model
├── road.py         # Road (lanes + ramps) + gap query helpers (handles wraparound)
├── simulation.py   # Orchestrates IDM, lane changes, on/off ramps, lane transitions
├── recorder.py     # Polars-backed sampler; writes Parquet on save()
├── visualizer.py   # pygame top-down renderer
└── main.py         # CLI entry point (argparse)
```

## Key concepts

**IDM (Intelligent Driver Model)** — physics-based car-following in `car.py`:
- Each car has randomised `desired_velocity`, `time_headway`, `min_gap`, `max_accel`, `comfortable_decel`
- This heterogeneity is what causes emergent stop-and-go waves at high density
- `car.position` is the **front bumper**; gap formula: `gap = leader.position - car.position - leader.length`

**Lane changes** — MOBIL-inspired in `simulation.py:_try_lane_change()`:
- Incentive: target lane must offer ≥ 8 m more gap ahead (`LANE_CHANGE_INCENTIVE`)
- Safety: gap behind in target lane must be ≥ 6 m (`SAFETY_GAP`)
- Cooldown: 3 s between changes (`LANE_CHANGE_COOLDOWN`)
- Visual transition: smoothstep interpolation over 1.2 s (`LANE_CHANGE_DURATION`), tracked in `simulation._lane_transitions`; rendered via `sim.get_visual_lane(car)` which returns a `float`

**Circular road** — positions are `% road_length`; `road.py:find_leader()` handles wraparound correctly

**Ramps** — configured in `Simulation.__init__()`:
- On-ramp spawns at `rate` cars/s when gap ≥ `ONRAMP_MIN_GAP` (20 m)
- Off-ramp removes passing cars with probability `rate` per crossing event

**Car rendering** — `visualizer.py:_draw_cars()`:
- Front bumper = right edge of rect (`Rect(cx - cw, cy - ch//2, cw, ch)`)
- `CAR_W=16 × CAR_H=10` px for regular cars; `TRUCK_W=26 × TRUCK_H=14` px for trucks (`length > 8 m`)
- `_lane_cy()` accepts a `float` lane to support mid-transition positions

**Data recording** — `recorder.py`:
- Column-oriented `dict[str, list]` buffers, flushed to `pl.DataFrame` on `save()`
- Output directory: `logs/` (created automatically; git-ignored)
- Aggregate file: `logs/traffic_aggregate_<ts>.parquet` — `time_s, car_count, avg_speed_kmh, density_veh_per_km, flow_veh_per_h`
- Trajectory file (`--record-cars`): `logs/traffic_cars_<ts>.parquet` — `time_s, car_id, lane, position_m, speed_kmh, accel_ms2`
- Live access: `recorder.aggregate_df()` / `recorder.trajectories_df()`

## Editing tips

- Tweak traffic density: change `--cars` or adjust ramp `rate` in `Simulation.__init__()`
- Change driving behaviour: adjust IDM params in `simulation.py:_make_car()`
- Lane change aggressiveness: adjust `LANE_CHANGE_INCENTIVE` / `SAFETY_GAP` in `simulation.py`
- Lane change animation speed: adjust `LANE_CHANGE_DURATION` in `simulation.py`
- Car visual size: adjust `CAR_W/H` and `TRUCK_W/H` in `visualizer.py`
- Car colour encodes speed: red (0 km/h) → yellow (60 km/h) → green (120+ km/h)
- Simulation substep count scales with `speed_mult` to keep IDM numerically stable

## Python / tooling

- Python 3.13, managed by `uv`
- Dependencies: `pygame>=2.5.0`, `numpy>=1.24.0`, `polars>=1.38.1`
- Add packages: `uv add <pkg>`
- No test suite yet — run `uv run traffic-sim` to verify changes visually
