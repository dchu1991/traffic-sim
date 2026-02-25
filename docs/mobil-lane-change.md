# MOBIL Lane-Change Model <sup>[(ref)](#ref-mobil)</sup>

MOBIL (Minimizing Overall Braking Induced by Lane changes) is a lane-change
decision model designed to sit on top of any car-following model (IDM, Gipps,
etc.).  It answers: *should this car change lanes right now?*

A proposed lane change passes only if **both** criteria below are satisfied.

---

## 1. Safety criterion

The new follower in the target lane must not be forced into dangerously hard
braking:

```
ã_new_follower  >=  -b_safe
```

| Symbol          | Meaning |
|-----------------|---------|
| `ã_new_follower`| acceleration of the car behind in target lane *after* the change |
| `b_safe`        | maximum braking you may impose on someone else (m/s²), typically 4–9 |

If this is violated the move is blocked regardless of the incentive.

---

## 2. Incentive criterion

The lane change happens only if the weighted acceleration gain exceeds a
threshold:

```
(ã_self - a_self)
  + p * ( (ã_new_follower - a_new_follower) + (ã_old_follower - a_old_follower) )
  >  delta_a_th
```

| Symbol             | Meaning |
|--------------------|---------|
| `ã_X` / `a_X`      | acceleration of car X after / before the hypothetical change |
| `p`                | politeness factor: 0 = purely selfish, 1 = fully altruistic |
| `delta_a_th`       | incentive threshold — prevents lane-switching for negligible gain |

Three cars are evaluated:

- **self** — the car considering the move
- **new follower** — whoever would be directly behind in the target lane
- **old follower** — whoever is currently behind in the current lane (they gain
  space when the car departs)

### Politeness factor `p`

| Value | Behaviour |
|-------|-----------|
| 0     | Purely selfish — ignores cost to new follower entirely |
| 0.2–0.5 | Empirically realistic — moves only if own gain outweighs follower's braking |
| 1     | Fully altruistic — net societal gain must be positive |

Higher `p` reduces unnecessary weaving and cuts in, which in turn reduces
phantom jams triggered by abrupt merges.

### Keep-right bias

For European-style keep-right rules, `delta_a_th` is set **negative** for
rightward moves — the car is willing to move right even at a small cost to
itself — and **positive** for leftward (overtaking) moves, requiring a clear
gain before pulling out.

---

## How this simulator implements MOBIL

**Left (overtaking) moves** use the full MOBIL incentive criterion, evaluating
IDM accelerations for all three cars (self, new follower, old follower):

```
gain  =  (ã_self - a_self)
       + p * [(ã_new_follower - a_new_follower) + (ã_old_follower - a_old_follower)]

Move left if:  gain  >  delta_a_threshold_ms2
Safety floor:  gap_behind_target  >=  safety_gap_m   (hard minimum, always checked)
```

**Right (keep-right) moves** remain gap-based (simpler heuristic):

```
Right (keep-right): gap_ahead_target  >=  keep_right_gap_m
Safety (both):      gap_behind_target >=  safety_gap_m
```

Config keys in `config.toml [lane_change]`:

| Key                    | MOBIL equivalent              | Default |
|------------------------|-------------------------------|---------|
| `politeness`           | politeness factor `p`         | 0.0     |
| `delta_a_threshold_ms2`| `delta_a_th` (left moves)     | 0.2 m/s²|
| `keep_right_gap_m`     | negative `delta_a_th` (right) | 25 m    |
| `safety_gap_m`         | `b_safe` (gap proxy)          | 6 m     |
| `incentive_m`          | *(deprecated — left moves now use accel criterion)* | 8 m |

### Gap formulas used in IDM evaluation

When car inserts into target lane between `new_follower` and `target_leader`:

```
nf_gap_before  =  gap_behind + car.length + gap_ahead   # nf → target_leader, pre-change
```

When car departs current lane, releasing `old_follower` toward `current_leader`:

```
of_gap_after   =  old_gap_behind + car.length + current_gap  # of → current_leader, post-change
```

### Tuning guide

| Goal | Config change |
|------|--------------|
| More aggressive overtaking | lower `delta_a_threshold_ms2` (even negative) |
| Less weaving / more courteous merges | raise `politeness` to 0.3–0.5 |
| Fully selfish (original behaviour) | `politeness = 0.0`, `delta_a_threshold_ms2 = 0.2` |
| Disable keep-right entirely | `keep_right_gap_m = 0` |

---

## References

<a id="ref-mobil"></a>
**MOBIL — Kesting, Treiber & Helbing (2007)**

> Kesting, A., Treiber, M., & Helbing, D. (2007).
> *General lane-changing model MOBIL for car-following models.*
> Transportation Research Record, 1999(1), 86–94.
> https://doi.org/10.3141/1999-10
