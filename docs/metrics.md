# Traffic Simulation Metrics

A reference for the metrics this simulator records and how to derive safety and
travel-time measures from the log files in the analysis notebook.

---

## Overview — what questions simulation answers

| Question family | Key metrics |
|-----------------|-------------|
| **Capacity / throughput** | Flow (veh/h), density (veh/km), speed; fundamental diagram |
| **Congestion / instability** | Space–time diagram, shock wave speed, stop-and-go amplitude |
| **Ramp performance** | Queue length, merge success rate, mainline speed drop |
| **Travel time / delay** | Travel time per car, delay vs. free-flow |
| **Safety** | TTC, near-miss rate, hard-braking events |
| **Emissions** | VSP from accel × speed (deferred — not currently recorded) |

---

## What the logs record

### Aggregate file — `traffic_aggregate_<ts>.parquet`

Sampled at `--record-interval` (default 1 s).

| Column | Unit | Enables |
|--------|------|---------|
| `time_s` | s | X-axis for all time-series charts |
| `car_count` | veh | Density proxy; ramp controller target |
| `avg_speed_kmh` | km/h | Overall speed; congestion detection |
| `density_veh_per_km` | veh/km | Fundamental diagram X-axis |
| `flow_veh_per_h` | veh/h | Fundamental diagram Y-axis; capacity |
| `onramp_rate` | veh/s | Ramp controller signal |
| `offramp_prob` | — | Ramp controller signal |

### Trajectory file — `traffic_cars_<ts>.parquet` (`--record-cars`)

One row per car per sample.

| Column | Unit | Enables |
|--------|------|---------|
| `time_s` | s | Space–time diagram; time-series |
| `car_id` | — | Per-car grouping |
| `lane` | 0–N | Lane-specific analysis |
| `position_m` | m | Space–time diagram; TTC derivation |
| `speed_kmh` | km/h | Speed distribution; TTC derivation |
| `accel_ms2` | m/s² | Hard-braking detection; emission proxy |
| `laps_completed` | — | Destination mode; travel time context |
| `destination_laps` | — | Destination mode; expected exit lap |

---

## Travel Time

**Definition:** time from first appearance to last appearance of a car in the
trajectory file.

```
travel_time_s = exit_time_s - entry_time_s
```

**Precision:** ±`sample_interval` (default ±1 s), because the exact exit moment
falls between samples. This is sufficient for distribution analysis.

**Derivation in notebook (Polars):**

```python
lifetimes = (
    traj.group_by("car_id")
    .agg([
        pl.col("time_s").first().alias("entry_time_s"),
        pl.col("time_s").last().alias("exit_time_s"),
        pl.col("laps_completed").last(),
        pl.col("destination_laps").last(),
    ])
    .with_columns(
        (pl.col("exit_time_s") - pl.col("entry_time_s")).alias("lifetime_s")
    )
)

# Exclude cars still on road at end of recording
t_max = traj["time_s"].max()
exited = lifetimes.filter(pl.col("exit_time_s") < t_max)
```

**Interpretation:**
- In **destination mode**: travel time ≈ `destination_laps × lap_time`. Cars with
  `laps_completed > destination_laps` missed their exit and retried next lap
  (expected — see `docs/destination-exits.md`).
- In **classic mode**: travel time follows an approximately exponential distribution
  controlled by `offramp_prob` and traffic speed.

**Notebook charts:**
- Histogram of `lifetime_s` (all exited cars; mean marked with a dashed line)
- Violin plot of `lifetime_s` by `destination_laps` (destination mode only) — shows
  how travel time scales with assigned laps and the spread within each lap bucket;
  box overlay adds median/IQR; individual car dots shown via `points="all"`;
  x-axis sorted numerically via `category_orders`

---

## Time-to-Collision (TTC)

**Definition:** the time until the following car rear-ends its leader if both
maintain current speed and trajectory.

```
TTC  =  gap / closing_speed
     =  (leader.position - car.position) / (car.speed - leader.speed)
```

Only defined when `car.speed > leader.speed` (closing scenario). All other
situations give TTC = undefined (no collision risk).

**Thresholds:**

| TTC | Interpretation |
|-----|----------------|
| > 4 s | Safe — normal following distance |
| 1.5–4 s | Caution zone — driver should be alert |
| < 1.5 s | Near-miss / critical conflict |
| < 0.5 s | Imminent collision (IDM should prevent this in practice) |

**Derivation in notebook (Polars):**

```python
ttc_df = (
    traj
    .sort(["time_s", "lane", "position_m"])
    .with_columns([
        pl.col("position_m").shift(-1).over(["time_s", "lane"]).alias("leader_pos_m"),
        pl.col("speed_kmh").shift(-1).over(["time_s", "lane"]).alias("leader_speed_kmh"),
    ])
    .with_columns([
        (pl.col("leader_pos_m") - pl.col("position_m")).alias("gap_m"),
        ((pl.col("speed_kmh") - pl.col("leader_speed_kmh")) / 3.6).alias("closing_ms"),
    ])
    .with_columns(
        pl.when((pl.col("closing_ms") > 0) & (pl.col("gap_m") > 0))
          .then(pl.col("gap_m") / pl.col("closing_ms"))
          .otherwise(None)
          .alias("ttc_s")
    )
    .drop_nulls("ttc_s")
)
```

**Approximation:** car length is not recorded in the trajectory file, so
`gap ≈ leader.position - car.position` (overestimates gap by ~4–5 m, which
underestimates TTC by ~0.1–0.3 s at typical following distances — acceptable
for safety-proxy purposes).

**Circular road note:** the frontmost car in each lane wraps and sees a large
position gap to the "leader" at the back of the pack. `shift(-1).over(...)` in
Polars assigns `null` for the shifted leader of the last car in each group,
producing `gap_m < 0` or `closing_ms ≤ 0`, which the filter already excludes.

**Notebook charts:**
- TTC histogram by lane (log Y; thresholds at 1.5 s and 4 s)
- Near-miss rate (fraction of closing pairs with TTC < 1.5 s) vs. car count over time
- Mean TTC by lane over time (shows which lane degrades first under high density)

---

## Fundamental Diagram

Flow (`q`), density (`k`), and space-mean speed (`v`) are related by:

```
q = k * v
```

The flow–density scatter plot (fundamental diagram) reveals:

- **Free-flow branch** (low density, high speed) — upper-left of the curve
- **Capacity point** — peak flow; typically 1800–2200 veh/h per lane on motorways
- **Congested branch** (high density, low speed) — lower-right; flow drops back after breakdown
- **Capacity drop** — flow in congested state is 5–15% lower than the peak; visible as a gap between the two branches

---

## Hard-Braking Events

The `accel_ms2` column enables counting deceleration events:

```python
hard_braking = traj.filter(pl.col("accel_ms2") < -3.0)
rate = hard_braking.height / traj.height
```

Hard braking is a proxy for near-misses independent of TTC; it also indicates
stop-and-go wave activity (each wave passage forces −3 to −6 m/s² deceleration).

---

## Deferred: Emissions

Emission estimation requires **Vehicle Specific Power (VSP)**:

```
VSP  =  v * (a + g*sin(θ) + 0.132)  +  0.000302 * v³
```

where `v` is speed (m/s), `a` is acceleration (m/s²), and `θ` is road grade (0
for flat). This can be computed from `speed_kmh` and `accel_ms2` in the trajectory
file; VSP bins then map to emission rates via MOVES or COPERT lookup tables.
Not currently implemented.
