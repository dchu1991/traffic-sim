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

---

## Related literature

The destination mode is a single-facility analogue of OD-based micro-simulation.
Each design decision has a corresponding body of literature:

### Mandatory lane change (MLC) and urgency

The `exiting` flag and lookahead distance implement the *urgency* concept from
classical MLC models:

- **Gipps (1986)** — introduced the MLC/DLC split. A car approaching a mandatory
  exit relaxes its safety-gap requirement proportionally to urgency, defined as
  remaining distance to the exit. Our binary `exiting` flag is the discrete
  counterpart of this continuous relaxation.

- **Ahmed (1999)** — formalised urgency as a continuous scalar that overrides the
  DLC utility score as the exit approaches. Calibrated from I-395 (Boston) NGSIM
  trajectories. The `exit_lookahead_m` parameter corresponds directly to the
  distance at which Ahmed's urgency term begins to dominate.

- **Hidas (2002, 2005)** — *Modelling vehicle interactions in microscopic
  simulation of merging and weaving* — studied missed exits explicitly: when a car
  reaches a diverge point in the wrong lane it either forces an unsafe merge or
  misses the exit. The missed-exit-retry behaviour here maps directly to Hidas's
  "forced merge failure" outcome.

- **Toledo, Koutsopoulos & Ben-Akiva (2007)** — integrated MLC urgency into a
  continuous lateral model; a car that misses a deadline carries urgency into
  the next opportunity, equivalent to the retry-next-lap logic here.

### Trip length distribution

Assigning `min_loops + Poisson(λ)` laps at spawn is equivalent to sampling from
a **shifted Poisson trip length distribution**:

- **Cascetta (2001)** *Transportation Systems Analysis* — trip length distributions
  in macro demand models are typically log-normal or gamma; Poisson is appropriate
  when laps are short relative to total facility distance and trips are independent.

- **Erlang (1917)** — telephone traffic with exponential holding times. A Poisson
  lap count gives approximately geometric exit probabilities in steady state,
  which is the simplest memoryless departure model.

### Lookahead distance and diverge capacity

The trade-off controlled by `exit_lookahead_m` (earlier commit → less last-minute
weaving, but more right-lane congestion) is analysed in:

- **Daganzo (2006)** — *In traffic flow, cellular automata = kinematic waves* —
  shows that the look-ahead distance for exit commitment is the key parameter
  governing throughput at a diverge: too short causes last-minute forced merges
  that reduce capacity; too long causes a slow-moving right-lane queue that bleeds
  into the through lanes.

- **Wei, Lee, Liu & Chen (2000)** — *Capacity Analysis of Weave Segments* —
  empirically quantified how the proportion of exiting vehicles and their
  lane-commitment distance affect weave capacity; findings support the default
  300 m lookahead for motorway speeds.

### OD-based micro-simulation

Commercial simulators assign full routes from OD matrices; destination mode is the
single-facility limit of this approach:

- **VISSIM / PTV (1994–present)** — each vehicle follows a pre-assigned route; MLC
  urgency is the Ahmed (1999) model.
- **SUMO — Lopez et al. (2018)** — open-source, OD-based; lane-change urgency uses
  a distance-to-junction model analogous to the exiting flag here.

---

## References

<a id="ref-gipps-mlc"></a>
**Gipps (1986)**

> Gipps, P. G. (1986).
> *A model for the structure of lane-changing decisions.*
> Transportation Research Part B, 20(5), 403–414.
> https://doi.org/10.1016/0191-2615(86)90012-3

<a id="ref-ahmed"></a>
**Ahmed (1999)**

> Ahmed, K. I. (1999).
> *Modeling drivers' acceleration and lane changing behavior.*
> PhD thesis, Massachusetts Institute of Technology.
> http://hdl.handle.net/1721.1/9662

<a id="ref-hidas"></a>
**Hidas (2002, 2005)**

> Hidas, P. (2002).
> *Modelling lane changing and merging in microscopic traffic simulation.*
> Transportation Research Part C, 10(5–6), 351–371.
> https://doi.org/10.1016/S0968-090X(02)00026-8
>
> Hidas, P. (2005).
> *Modelling vehicle interactions in microscopic simulation of merging and weaving.*
> Transportation Research Part C, 13(1), 37–62.
> https://doi.org/10.1016/j.trc.2004.12.003

<a id="ref-toledo"></a>
**Toledo, Koutsopoulos & Ben-Akiva (2007)**

> Toledo, T., Koutsopoulos, H. N., & Ben-Akiva, M. (2007).
> *Integrated driving behavior modeling.*
> Transportation Research Part C, 15(2), 96–112.
> https://doi.org/10.1016/j.trc.2007.02.002

<a id="ref-daganzo"></a>
**Daganzo (2006)**

> Daganzo, C. F. (2006).
> *In traffic flow, cellular automata = kinematic waves.*
> Transportation Research Part B, 40(5), 396–403.
> https://doi.org/10.1016/j.trb.2005.05.004

<a id="ref-sumo"></a>
**Lopez et al. (2018)**

> Lopez, P. A., et al. (2018).
> *Microscopic traffic simulation using SUMO.*
> IEEE ITSC 2018.
> https://doi.org/10.1109/ITSC.2018.8569938
