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
- Sections: `[road]`, `[lane_change]`, `[ramp]`, `[cars]`, `[trucks]`, `[destination]`
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

**Lane changes** — full MOBIL + keep-right rule in `simulation.py:_try_lane_change()`:
- Move LEFT (overtake): full MOBIL criterion — `(ã_self - a_self) + p * (follower_deltas) > delta_a_threshold_ms2`; uses `road.find_lane_neighbors()` to get car objects for IDM re-evaluation
- Move RIGHT (keep-right): gap-based — gap ahead ≥ `keep_right_gap_m` (default 25 m); set to 0 to disable
- Safety (both): gap behind in target lane ≥ `safety_gap_m` (default 6 m) — hard floor, always checked
- Cooldown: `cooldown_s` (default 3 s) between changes
- Visual transition: smoothstep interpolation over `duration_s` (1.2 s), `_lane_transitions` dict
- `incentive_m` is deprecated for left moves (still in config for backward compat, ignored)
- See `docs/mobil-lane-change.md` for full criterion, gap formulas, and tuning guide

**Circular road** — positions are `% road_length`; `road.py:find_leader()` handles wraparound correctly

**Ramps** — positions / rates from `config.toml [ramp]`:
- On-ramp at 10% of road (rightmost lane), 0.5 cars/s
- Off-ramp at 80% of road (rightmost lane), 30% exit probability per crossing
- On-ramp cars queue on a visible ramp lane below the road; IDM physics drives them forward
- Merge params: `min_gap_m` (safety gap, default 30 m), `merge_window_m` (how far back merging can start, default 100 m)
- **Zipper merge**: when rightmost lane avg speed < `zipper_speed_kmh` (30 km/h), only `zipper_gap_m` (8 m) gap ahead is required — gap behind is ignored since cars are near-stopped
- Cooperative yield: road cars in rightmost lane treat the ramp lead car as a virtual obstacle when within 15 m of merge point; only applies when `ramp_gap > 0` (avoids deadlock)
- Merge animation: ramp car slides from `_lane_cy(num_lanes)` → rightmost lane using same smoothstep system as lane changes
- Merge window is highlighted in visualizer with a green tint + dashed top edge and start marker
- **Dynamic ramp control**: `_update_ramp_control()` in `simulation.py` — proportional controller adjusts both `ramp.rate` (on-ramp) and `ramp.rate` (off-ramp prob) to drive `car_count → target_cars`; gains set by `onramp_control_gain` / `offramp_control_gain`; disabled when `target_cars = 0`

**Destination exits** — `config.toml [destination]` + `simulation.py`:
- Each car is assigned `destination_laps = min_loops + Poisson(λ)` at spawn; exits after that many laps
- `offramp_prob` is ignored in destination mode (exits are deterministic per-car)
- `car.exiting = True` is set within `exit_lookahead_m` of the off-ramp on the final lap (`_update_exiting_flags`, called every step before lane changes)
- Exiting cars skip leftward moves and bypass `keep_right_gap` to prioritise reaching the rightmost lane
- The safety-gap check still applies — if blocked, the car misses the exit and retries next lap (intentional; see `docs/destination-exits.md`)
- `_passed_ramps` is cleared on each lap wrap so the off-ramp is re-evaluated every lap
- In destination mode, `_update_ramp_control` skips `offramp_prob` adjustment; only `onramp_rate` is controlled
- `laps_completed` and `destination_laps` recorded in trajectory Parquet; HUD shows avg laps in destination mode
- Design notes: `docs/destination-exits.md`

**Car rendering** — `visualizer.py:_draw_cars()`:
- Front bumper = right edge of rect (`Rect(cx - cw, cy - ch//2, cw, ch)`)
- `CAR_W=16 × CAR_H=10` px for regular cars; `TRUCK_W=26 × TRUCK_H=14` px for trucks (`length > 8 m`)
- `_lane_cy()` accepts a `float` lane to support mid-transition positions
- HUD shows per-lane speed limits: `Limits (L→R): L1 130 km/h | L2 110 km/h | L3 90 km/h`

**Data recording** — `recorder.py`:
- Column-oriented `dict[str, list]` buffers, flushed to `pl.DataFrame` on `save()`
- Output directory: `logs/` (created automatically; git-ignored)
- Aggregate file: `logs/traffic_aggregate_<ts>.parquet` — `time_s, car_count, avg_speed_kmh, density_veh_per_km, flow_veh_per_h, onramp_rate, offramp_prob`
- Trajectory file (`--record-cars`): `logs/traffic_cars_<ts>.parquet` — `time_s, car_id, lane, position_m, speed_kmh, accel_ms2, laps_completed, destination_laps`
- Metadata sidecar: `logs/traffic_meta_<ts>.json` — CLI args + `dataclasses.asdict(cfg)` snapshot; written whenever `metadata` dict is passed to `Recorder`
- `onramp_rate` and `offramp_prob` are sampled live from `ramp.rate` each tick (reflect controller adjustments when `target_cars > 0`)

## Editing tips

- **All behaviour params**: edit `config.toml` — no code changes needed
- **Fast lane / speed limits**: `lane_speed_limits_kmh` in `[road]`
- **Keep-right aggressiveness**: `keep_right_gap_m` in `[lane_change]` (0 = disabled)
- **Overtaking threshold**: `delta_a_threshold_ms2` in `[lane_change]` (lower = more aggressive; negative = willing to move at slight cost)
- **Overtaking politeness**: `politeness` in `[lane_change]` (0 = selfish, 0.3–0.5 = realistic, 1 = altruistic)
- **Traffic density**: `--cars` or `onramp_rate` in `[ramp]`
- **Steady car count**: set `target_cars` in `[ramp]`; tune with `onramp_control_gain` / `offramp_control_gain`
- **Merge aggressiveness**: `merge_window_m` (wider = earlier), `min_gap_m` (normal), `zipper_gap_m` (congested), `zipper_speed_kmh` (threshold)
- **Driver aggression**: `desired_v_mean_ms`, `time_headway_mean` etc. in `[cars]`
- **Destination mode on/off**: `enabled` in `[destination]`; when `true`, `offramp_prob` is ignored
- **Exit lap distribution**: `min_loops` (floor) + `loops_lambda` (Poisson mean extra laps) in `[destination]`
- **Missed-exit rate**: `exit_lookahead_m` (more = earlier right-lane commit) and `safety_gap_m` (lower = more permissive)
- Car colour encodes speed: red (0 km/h) → yellow (60 km/h) → green (120+ km/h)
- Simulation substep count scales with `speed_mult` to keep IDM numerically stable

## Analysis notebook

- Location: `notebook/analysis.ipynb` (git-ignored via `notebook/` in `.gitignore`)
- Launch: `uv run jupyter lab notebook/analysis.ipynb`
- Loads most recent `logs/traffic_aggregate_<ts>.parquet` + matching `traffic_cars_<ts>.parquet` and `traffic_meta_<ts>.json` automatically
- Charts: speed + car count (per-lane ±std bands), ramp control signals (`onramp_rate` / `offramp_prob`), fundamental diagram, speed distribution by lane, space–time diagram (all lanes, custom traffic colorscale), car lifetime table, destination lap distribution (destination mode only)
- Entry/exit derived from trajectory data (`first/last appearance of car_id`); accurate to ±1 sample interval
- Destination section: histogram of assigned `destination_laps` + scatter of assigned vs actual — points above the diagonal = cars that missed an exit and retried (expected)

## Python / tooling

- Python 3.13, managed by `uv`
- Runtime deps: `pygame>=2.5.0`, `numpy>=1.24.0`, `polars>=1.38.1`
- Dev deps (notebook): `jupyter`, `matplotlib`, `plotly`, `pandas`, `pyarrow`, `anywidget`
- `tomllib` is stdlib (Python 3.11+) — no extra dep needed for config loading
- Add packages: `uv add <pkg>`; notebook-only: `uv add --dev <pkg>`
- Test suite: `uv run pytest tests/` (tests across car, road, config, simulation + destination mode)
