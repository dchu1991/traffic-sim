# Destination Exits — Design Notes

## Overview

Each car is assigned `destination_laps = min_loops + Poisson(λ)` at spawn.
The car exits at the off-ramp only after completing that many laps.

## Exit path

1. When `laps_completed >= destination_laps` and the car is within `exit_lookahead_m`
   of the off-ramp, `car.exiting = True` is set every step (`_update_exiting_flags`).
2. Exiting cars skip leftward (overtake) lane-change candidates and immediately take
   any safe rightward move, bypassing the `keep_right_gap` threshold.
3. `_process_offramp` triggers removal when the car passes within
   `velocity × dt × 2 + 2 m` of the ramp **and** is in the rightmost lane
   **and** `laps_completed >= destination_laps`.

## "Missed exit" — intentional behaviour

If the safety-gap check blocks the rightward lane change and the car reaches the
off-ramp while still in lane 0 or 1, it does **not** exit that lap.
On the next lap `_passed_ramps` is cleared (at the position-wrap point) and
`laps_completed` is already ≥ `destination_laps`, so the car will try again.

**Consequence:** a car may complete 1–2 extra laps before exiting.
The notebook scatter plot (`destination_laps` vs `laps_completed`) will show
points slightly above the diagonal for these cars — this is expected and models
realistic lane-change constraints (a driver cannot force an unsafe merge just
because they "want" to exit).

## Key parameters

| Config key | Default | Effect |
|------------|---------|--------|
| `min_loops` | 5 | Minimum laps before any exit |
| `loops_lambda` | 3.0 | Poisson mean extra laps (avg destination = 8 laps) |
| `exit_lookahead_m` | 300.0 | Distance before off-ramp where exiting flag activates |
| `safety_gap_m` | 6.0 | Min gap behind for a lane change — directly affects miss rate |
| `keep_right_gap_m` | 25.0 | Normal keep-right threshold (bypassed for exiting cars) |

To reduce missed exits: increase `exit_lookahead_m` (more time to change lanes) or
decrease `safety_gap_m` (more permissive lane changes). To increase realism of missed
exits: decrease `exit_lookahead_m` or increase traffic density.

## Lap detection

A lap is counted inside `step()` during the IDM update phase:

```python
if old_pos > car.position + road_length * 0.5:   # position wrapped past 0
    car.laps_completed += 1
    car._passed_ramps.clear()   # re-enable exit evaluation for new lap
```

The `road_length * 0.5` threshold prevents false triggers from small position
fluctuations. `_passed_ramps` is cleared so the off-ramp is re-evaluated each lap
(in classic mode it is never cleared — cars get exactly one exit chance per ramp pass).
