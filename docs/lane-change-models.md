# Lane-Change Models

A survey of the major lane-change decision frameworks used in traffic simulation,
from classical gap-acceptance rules through utility maximisation to the
acceleration-based MOBIL family implemented here.

Every model must answer two questions:

1. **Safety** — will the move force anyone into dangerously hard braking?
2. **Incentive** — is there a good enough reason to change lanes?

---

## 1. Gipps (1986) <sup>[(ref)](#ref-gipps)</sup>

The canonical rule-based model.  Lane changes are categorised as:

- **Mandatory** (MLC) — must change to reach a destination or avoid a road end
- **Discretionary** (DLC) — want to travel faster or more comfortably

### Safety criterion

The new follower (car B in target lane) must not be pushed below its safe speed:

```
v_B_after  >=  v_safe(B, gap_B_after)
```

where `v_safe` is Gipps's own safe-speed formula:
```
v_safe = b*tau + sqrt( (b*tau)^2 + v_B^2 + 2*b*(gap_B - s0) )
```

| Symbol | Meaning |
|--------|---------|
| `b`    | comfortable deceleration (m/s²) |
| `tau`  | reaction time (s) |
| `s0`   | standstill gap (m) |

### Incentive criterion

DLC: the changing car must be able to travel at least as fast in the target lane,
accounting for the target leader's speed and gap.  Threshold comparisons are used;
no continuous incentive score.

MLC overrides DLC — if the car must change (e.g. to reach a ramp), the safety
check is relaxed proportionally to the urgency (distance remaining to the forced
move point).

### Notes

- Binary decision per timestep (no continuous gain score)
- Widely used as a building block; VISSIM and most early simulators derive from it
- Does not consider the old follower's gain

---

## 2. Ahmed / MLC–DLC Utility Model (1999) <sup>[(ref)](#ref-ahmed)</sup>

Treats lane choice as a **discrete-choice utility maximisation** problem.  Each
available lane gets a utility score; the car picks the highest.

### Utility function

```
U_n  =  beta_0
       + beta_1 * speed_gain_n
       + beta_2 * gap_ahead_n
       + beta_3 * gap_behind_n
       + beta_4 * urgency_n
       + epsilon_n
```

| Symbol | Meaning |
|--------|---------|
| `speed_gain_n` | expected speed gain from moving to lane n |
| `gap_ahead_n`  | gap to leader in lane n |
| `gap_behind_n` | gap to follower in lane n |
| `urgency_n`    | MLC urgency score (distance-to-forced-move) |
| `epsilon_n`    | Gumbel-distributed random error term |
| `beta_*`       | coefficients calibrated from trajectory data |

A **logit** model gives the probability of choosing each lane; the car samples
from this distribution.

### Notes

- Parameters are empirically calibrated (not from physics)
- Captures heterogeneity naturally via the random term
- Cornerstone of VISSIM's lane-change model
- Computationally light; does not evaluate the follower's car-following model

---

## 3. MOBIL (2007) <sup>[(ref)](#ref-mobil)</sup>

See [mobil-lane-change.md](mobil-lane-change.md) for the full description and
implementation notes.  Summary:

```
gain  =  (a~_self - a_self)
       + p * [ (a~_new_follower - a_new_follower)
             + (a~_old_follower - a_old_follower) ]

Move if:  gain > delta_a_th
Safety:   a~_new_follower >= -b_safe
```

Key feature: evaluates the **car-following model** (IDM) directly for three cars
under two hypothetical lane configurations.  Physically consistent and does not
require empirical calibration.

---

## 4. STDE / Lmrs (Schakel et al. 2012) <sup>[(ref)](#ref-lmrs)</sup>

**Lmrs** (Lane change Model with Relaxation and Synchronisation) extends MOBIL
with two mechanisms absent in the original:

### Relaxation

After a lane change the car does not instantly adopt IDM headway; it relaxes
over a timescale `tau_relax` (typically 3–5 s).  This prevents the
over-deceleration spike that standard IDM produces immediately after insertion.

```
T_eff(t)  =  T_IDM - (T_IDM - T_lc) * exp(-t_since_lc / tau_relax)
```

| Symbol | Meaning |
|--------|---------|
| `T_IDM`     | steady-state IDM time headway |
| `T_lc`      | headway immediately after lane change (typically 0) |
| `tau_relax` | relaxation timescale (s) |

### Synchronisation (cooperative yielding)

The yielding car in the target lane actively opens a gap when it detects an
adjacent car wishing to merge:

```
a_yield  =  IDM(gap_to_merging_car, v_merging)   if incentive > 0
```

The merging car's incentive is shared — if the yield improves the total system
gain (MOBIL criterion), the follower cooperates.

### Notes

- Implemented in OpenTrafficSim (open-source Java simulator)
- Reduces the unrealistic "aggressive cut-in" artefact of basic MOBIL
- Our cooperative-yield on the on-ramp uses a similar idea

---

## 5. RSS — Responsibility-Sensitive Safety (2017) <sup>[(ref)](#ref-rss)</sup>

Not a full lane-change model, but a **formal safety envelope** framework.  It
defines mathematically what constitutes a "safe" longitudinal and lateral
state, and attributes fault when a collision occurs.

### Longitudinal safe distance

```
d_safe  =  v_rear * rho  +  v_rear^2 / (2 * a_min_brake)
                          -  v_front^2 / (2 * a_max_brake)
```

| Symbol | Meaning |
|--------|---------|
| `v_rear / v_front` | speeds of rear and front car |
| `rho`              | reaction time |
| `a_min_brake`      | minimum braking capability of rear car |
| `a_max_brake`      | maximum braking capability of front car |

### Lateral safe distance

```
d_lat_safe  =  mu  +  (v_lat_rear^2 / (2 * a_lat_brake))
                    +  (v_lat_front^2 / (2 * a_lat_brake))
```

The car must ensure `d_lat >= d_lat_safe` before initiating a lateral move.

### Notes

- Designed for autonomous vehicles; provides formal proof of non-culpability
- Does not define incentive — only what is safe
- Often layered on top of MOBIL or utility models to provide a hard safety floor

---

## 6. Toledo continuous lateral model (2007) <sup>[(ref)](#ref-toledo)</sup>

Instead of discrete lane indices, each car has a continuous **lateral position**
`y(t)` and a lateral velocity `y'(t)`.  Lane changes emerge from lateral
acceleration commands.

### Lateral acceleration

```
y''(t)  =  K_lat * (y_target - y(t))  -  C_lat * y'(t)  +  epsilon(t)
```

| Symbol | Meaning |
|--------|---------|
| `y_target`  | desired lateral position (lane centre or between-lane during change) |
| `K_lat`     | stiffness — how strongly the car is attracted to `y_target` |
| `C_lat`     | damping — suppresses oscillation |
| `epsilon`   | stochastic term (Wiener process) |

`y_target` itself is driven by a higher-level discrete-choice (utility) model
that selects the desired lane.  The continuous dynamics then move the car
smoothly toward it.

### Notes

- Naturally models aborted and gradual lane changes
- Captures lateral oscillation and drift observed in real trajectories
- High implementation cost; requires continuous collision detection
- Best suited for micro-simulation studies of lateral behaviour

---

## Comparison

| Model | Safety check | Incentive type | Followers evaluated | Calibration needed |
|-------|-------------|----------------|---------------------|--------------------|
| Gipps (1986) | Safe speed formula | Rule-based (binary) | New follower only | Analytical |
| Ahmed (1999) | Gap threshold | Utility (logit) | Implicit via gap | Empirical |
| MOBIL (2007) | IDM accel floor | Accel gain + politeness | Self + new + old | Analytical + `p` |
| Lmrs (2012) | MOBIL + yield | MOBIL + cooperation | Self + new + old + yielder | Analytical |
| RSS (2017) | Formal safe distance | None (safety only) | New follower (worst-case) | Formal proof |
| Toledo (2007) | Continuous collision | Continuous utility | Implicit | Empirical |

---

## References

<a id="ref-gipps"></a>
**Gipps (1986)**

> Gipps, P. G. (1986).
> *A model for the structure of lane-changing decisions.*
> Transportation Research Part B: Methodological, 20(5), 403–414.
> https://doi.org/10.1016/0191-2615(86)90012-3

<a id="ref-ahmed"></a>
**Ahmed (1999)**

> Ahmed, K. I. (1999).
> *Modeling drivers' acceleration and lane changing behavior.*
> PhD thesis, Massachusetts Institute of Technology.
> http://hdl.handle.net/1721.1/9662

<a id="ref-mobil"></a>
**MOBIL — Kesting, Treiber & Helbing (2007)**

> Kesting, A., Treiber, M., & Helbing, D. (2007).
> *General lane-changing model MOBIL for car-following models.*
> Transportation Research Record, 1999(1), 86–94.
> https://doi.org/10.3141/1999-10

<a id="ref-lmrs"></a>
**Lmrs — Schakel, van Arem & Knoop (2012)**

> Schakel, W. J., van Arem, B., & Knoop, V. L. (2012).
> *Integrated lane change model with relaxation and synchronization.*
> Transportation Research Record, 2316(1), 47–57.
> https://doi.org/10.3141/2316-06

<a id="ref-rss"></a>
**RSS — Shalev-Shwartz, Shammah & Shashua (2017)**

> Shalev-Shwartz, S., Shammah, S., & Shashua, A. (2017).
> *On a formal model of safe and scalable self-driving cars.*
> arXiv preprint arXiv:1708.06374.
> https://arxiv.org/abs/1708.06374

<a id="ref-toledo"></a>
**Toledo, Koutsopoulos & Ben-Akiva (2007)**

> Toledo, T., Koutsopoulos, H. N., & Ben-Akiva, M. (2007).
> *Integrated driving behavior modeling.*
> Transportation Research Part C: Emerging Technologies, 15(2), 96–112.
> https://doi.org/10.1016/j.trc.2007.02.002
