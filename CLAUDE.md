# Traffic Simulator — Claude Notes

## Running the project

```bash
uv run traffic-sim                          # default: 3 lanes, 50 cars, 1000m road
uv run traffic-sim --lanes 4 --cars 80      # more lanes and cars
uv run traffic-sim --length 2000 --trucks 0.3  # longer road, more trucks
```

Full options: `--lanes`, `--cars`, `--length`, `--fps`, `--width`, `--trucks`

## Project structure

```
src/traffic_sim/
├── car.py          # Car dataclass + IDM acceleration model
├── road.py         # Road (lanes + ramps) + gap query helpers (handles wraparound)
├── simulation.py   # Orchestrates IDM, lane changes, on/off ramps
├── visualizer.py   # pygame top-down renderer
└── main.py         # CLI entry point (argparse)
```

## Key concepts

**IDM (Intelligent Driver Model)** — physics-based car-following in `car.py`:
- Each car has randomised `desired_velocity`, `time_headway`, `min_gap`, `max_accel`, `comfortable_decel`
- This heterogeneity is what causes emergent stop-and-go waves at high density

**Lane changes** — MOBIL-inspired in `simulation.py:_try_lane_change()`:
- Incentive: target lane must offer ≥ 8 m more gap ahead than current lane
- Safety: gap behind in target lane must be ≥ 6 m
- Cooldown: 3 s between changes (constant `LANE_CHANGE_COOLDOWN`)

**Circular road** — positions are `% road_length`; `road.py:find_leader()` handles wraparound correctly

**Ramps** — configured in `Simulation.__init__()`:
- On-ramp spawns at rate cars/s when a sufficient gap exists
- Off-ramp removes passing cars with a fixed probability per event

## Editing tips

- Tweak traffic density by changing `--cars` or adjusting ramp `rate` in `Simulation.__init__()`
- Change driving behaviour by adjusting IDM params in `simulation.py:_make_car()`
- Lane change aggressiveness: adjust `LANE_CHANGE_INCENTIVE` / `SAFETY_GAP` in `simulation.py`
- Car colour encodes speed: red (0 km/h) → yellow (60 km/h) → green (120+ km/h)
- Simulation substep count scales with `speed_mult` to keep IDM numerically stable

## Python / tooling

- Python 3.13, managed by `uv`
- Dependencies: `pygame>=2.5.0`, `numpy>=1.24.0`
- Add packages: `uv add <pkg>`
- No test suite yet — run `uv run traffic-sim` to verify changes visually
