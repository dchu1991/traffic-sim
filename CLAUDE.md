# Traffic Simulator — Claude Notes

## Running the project

```bash
uv run traffic-sim                             # default: reads config.toml automatically
uv run traffic-sim --lanes 4 --cars 80         # more lanes and cars
uv run traffic-sim --length 2000 --trucks 0.3  # longer road, more trucks
uv run traffic-sim --config my_config.toml     # custom behaviour config
uv run traffic-sim --record --record-cars      # record to Parquet on exit
```

Full options: `--lanes`, `--cars`, `--length`, `--fps`, `--width`, `--trucks`,
`--config`, `--record`, `--record-cars`, `--record-interval`

## Project structure

```
config.toml             # behaviour config (speed limits, lane-change, ramp, IDM params)
src/traffic_sim/
├── config.py       # SimConfig dataclass + TOML loader (tomllib, stdlib)
├── car.py          # Car dataclass + IDM acceleration model
├── road.py         # Road (lanes + ramps + per-lane speed limits) + gap query helpers
├── simulation.py   # Orchestrates IDM, lane changes, on/off ramps, lane transitions
├── recorder.py     # Polars-backed sampler; writes Parquet on save()
├── visualizer.py   # pygame top-down renderer
└── main.py         # CLI entry point (argparse)
```

## Key concepts

**Config system** — `config.py` / `config.toml`:
- `SimConfig` dataclass loaded via `SimConfig.from_toml(path)`
- `main.py` auto-reads `config.toml` if present; override with `--config`
- Sections: `[road]`, `[lane_change]`, `[ramp]`, `[cars]`, `[trucks]`
- Any section/key can be omitted — built-in defaults apply

**IDM (Intelligent Driver Model)** — physics-based car-following in `car.py`:
- Each car has randomised `desired_velocity`, `time_headway`, `min_gap`, `max_accel`, `comfortable_decel`
- This heterogeneity is what causes emergent stop-and-go waves at high density
- `car.position` is the **front bumper**; gap formula: `gap = leader.position - car.position - leader.length`
- `car.update(dt, gap, lead_v, road_length, speed_limit=inf)` — caps desired velocity to lane limit

**Lane speed limits** — `road.lane_speed_limits: list[float]` (m/s), index 0 = leftmost (fast) lane:
- Default: L1 130 km/h → L2 110 km/h → L3 90 km/h (configured in `config.toml [road]`)
- IDM uses `min(desired_velocity, lane_speed_limit)` as effective target speed
- Change via `config.toml`: `lane_speed_limits_kmh = [160, 130, 100]` for autobahn-style

**Lane changes** — MOBIL-inspired + keep-right rule in `simulation.py:_try_lane_change()`:
- Move LEFT (overtake): gap ahead must improve ≥ `incentive_m` (default 8 m)
- Move RIGHT (keep-right): gap ahead ≥ `keep_right_gap_m` (default 25 m); set to 0 to disable
- Safety: gap behind in target lane ≥ `safety_gap_m` (default 6 m)
- Cooldown: `cooldown_s` (default 3 s) between changes
- Visual transition: smoothstep interpolation over `duration_s` (1.2 s), `_lane_transitions` dict

**Circular road** — positions are `% road_length`; `road.py:find_leader()` handles wraparound correctly

**Ramps** — positions / rates from `config.toml [ramp]`:
- On-ramp at 10% of road (rightmost lane), 0.5 cars/s; needs ≥ 20 m gap to merge
- Off-ramp at 80% of road (rightmost lane), 30% exit probability per crossing

**Car rendering** — `visualizer.py:_draw_cars()`:
- Front bumper = right edge of rect (`Rect(cx - cw, cy - ch//2, cw, ch)`)
- `CAR_W=16 × CAR_H=10` px for regular cars; `TRUCK_W=26 × TRUCK_H=14` px for trucks (`length > 8 m`)
- `_lane_cy()` accepts a `float` lane to support mid-transition positions
- HUD shows per-lane speed limits: `Limits (L→R): L1 130 km/h | L2 110 km/h | L3 90 km/h`

**Data recording** — `recorder.py`:
- Column-oriented `dict[str, list]` buffers, flushed to `pl.DataFrame` on `save()`
- Output directory: `logs/` (created automatically; git-ignored)
- Aggregate file: `logs/traffic_aggregate_<ts>.parquet` — `time_s, car_count, avg_speed_kmh, density_veh_per_km, flow_veh_per_h`
- Trajectory file (`--record-cars`): `logs/traffic_cars_<ts>.parquet` — `time_s, car_id, lane, position_m, speed_kmh, accel_ms2`

## Editing tips

- **All behaviour params**: edit `config.toml` — no code changes needed
- **Fast lane / speed limits**: `lane_speed_limits_kmh` in `[road]`
- **Keep-right aggressiveness**: `keep_right_gap_m` in `[lane_change]` (0 = disabled)
- **Overtaking threshold**: `incentive_m` in `[lane_change]`
- **Traffic density**: `--cars` or `onramp_rate` in `[ramp]`
- **Driver aggression**: `desired_v_mean_ms`, `time_headway_mean` etc. in `[cars]`
- Car colour encodes speed: red (0 km/h) → yellow (60 km/h) → green (120+ km/h)
- Simulation substep count scales with `speed_mult` to keep IDM numerically stable

## Python / tooling

- Python 3.13, managed by `uv`
- Dependencies: `pygame>=2.5.0`, `numpy>=1.24.0`, `polars>=1.38.1`
- `tomllib` is stdlib (Python 3.11+) — no extra dep needed for config loading
- Add packages: `uv add <pkg>`
- No test suite yet — run `uv run traffic-sim` to verify changes visually
