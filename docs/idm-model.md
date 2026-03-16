# Intelligent Driver Model (IDM) <sup>[(ref)](#ref-idm)</sup>

## Overview

Each car follows the car immediately ahead using the Intelligent Driver Model.
IDM is a continuous-time, physics-based car-following model that produces
realistic acceleration, smooth braking, and emergent stop-and-go waves without
explicit rule-based logic.

The model has two competing parts:

- A **free-road term** that accelerates the car toward its desired speed.
- An **interaction term** that brakes the car when the gap to the leader
  shrinks below a desired minimum.

---

## Acceleration equation

```
a_out = a_max * [ 1 - (v / v0)^delta - (s_star / s)^2 ]
```

| Symbol    | Meaning                                          |
|-----------|--------------------------------------------------|
| `a_out`   | Resulting acceleration (m/s²), clipped to [-9, a_max] |
| `a_max`   | Maximum acceleration (m/s²)                      |
| `v`       | Current speed of this car (m/s)                  |
| `v0`      | Effective desired speed = min(desired_velocity, lane_speed_limit) |
| `delta`   | Acceleration exponent (fixed at 4)               |
| `s_star`  | Desired minimum gap to the leader (m) — see below |
| `s`       | Actual bumper-to-bumper gap to the leader (m)    |

**Free-road term:** `1 - (v/v0)^delta`

- When `v << v0`, this is close to 1 — car accelerates freely.
- When `v ≈ v0`, this term approaches 0 — car stops accelerating.

**Interaction term:** `(s_star / s)^2`

- When the gap `s` is much larger than desired (`s >> s_star`), this is near 0 — no braking.
- When `s ≈ s_star`, this equals the free-road term — car holds steady following.
- When `s < s_star`, this exceeds 1 — car brakes hard.

---

## Desired gap equation

```
s_star = s0 + max(0,  v * T  +  v * dv / (2 * sqrt(a_max * b)) )
```

| Symbol | Meaning                                           |
|--------|---------------------------------------------------|
| `s0`   | Minimum standstill gap (m) — bumper-to-bumper when stopped |
| `T`    | Desired time headway (s) — gap in seconds at current speed |
| `v`    | Current speed (m/s)                               |
| `dv`   | Approach rate = this car's speed minus leader's speed (m/s) |
| `b`    | Comfortable deceleration (m/s²)                   |

The term `v * dv / (2 * sqrt(a_max * b))` is the **intelligent braking buffer**.

- When closing on the leader (`dv > 0`), desired gap grows — the faster you
  approach, the more space you want.
- When pulling away from the leader (`dv < 0`), the term is clamped to zero —
  the car doesn't reduce its desired gap just because the leader is faster.
- This term is derived from the kinematic stopping distance for symmetric
  deceleration at `b`.

---

## Parameters and config keys

| Parameter           | Config key (`[cars]`)          | Typical value  | Meaning                                         |
|---------------------|--------------------------------|----------------|-------------------------------------------------|
| `desired_velocity`  | `desired_v_mean_ms` ± std      | 33 m/s (120 km/h) | Target free-road speed                       |
| `time_headway`      | `time_headway_mean` ± std      | 2.0 s          | Desired seconds of gap to leader at speed       |
| `min_gap`           | `min_gap_mean_m`               | 2.0 m          | Standstill bumper-to-bumper gap                 |
| `max_accel`         | `max_accel_mean`               | 1.5 m/s²       | Peak acceleration                               |
| `comfortable_decel` | `comfortable_decel_mean`       | 2.0 m/s²       | Comfortable braking; controls braking buffer    |
| `accel_exponent`    | (hardcoded)                    | 4              | Smoothness of approach to desired speed         |

All parameters except `accel_exponent` are randomised per car from normal
distributions defined in `config.toml [cars]`.  Trucks use a separate
`[trucks]` section with lower desired speeds and longer vehicle lengths.

---

## How cars interact

Each car knows only one other car: the **leader** (the nearest car ahead in the
same lane).  There is no look-ahead beyond the immediate leader, and no direct
awareness of cars behind.

The gap is computed as:

```
gap = leader.position - this.position - leader.length
```

`position` is the **front bumper**, so subtracting `leader.length` gives the
true bumper-to-bumper space.

Vehicle length affects the gap experienced by the car **behind** the vehicle,
not the vehicle itself.  A truck (10–16 m) leaves less empty space ahead of the
follower than a car (4.5 m) at the same front-bumper position, so the truck is
effectively a larger moving obstacle.  The following car's own length does not
enter the gap formula.

The road is circular, so positions wrap modulo `road_length`.  `road.py`'s
`find_leader()` handles the wraparound correctly: if no leader exists in front,
the car behind (i.e. the "last" car on the loop) is returned, and the gap
accounts for the full road circumference.

---

## Heterogeneity and emergent stop-and-go waves

Because each car's parameters are drawn independently from distributions, faster
drivers (`high v0`) catch up to slower drivers (`low v0`).  This compresses
gaps, which triggers braking, which propagates backward as a phantom traffic
jam — even with no physical bottleneck.

The time headway distribution is the primary tuning knob: lowering
`time_headway_mean` makes drivers tailgate more aggressively and causes denser,
more frequent stop-and-go waves.  Raising it spreads cars out and smooths flow.

See `config.toml [cars]` for all distribution parameters.

---

## Alternative car-following models

### Newell (1961) — kinematic wave / trajectory shift <sup>[(ref)](#ref-newell)</sup>

The simplest plausible model.  A car's position is just a delayed, shifted copy
of its leader's trajectory:

```
x(t + T) = x_leader(t) - d
```

| Symbol | Meaning |
|--------|---------|
| `T`    | Reaction / travel-time delay (s) |
| `d`    | Jam spacing (m) — minimum space per vehicle when stopped |

Equivalently in velocity form:

```
v(t) = min(v_free,  (gap - d) / T)
```

No acceleration physics — the car teleports to the "correct" speed instantly.
Analytically tractable and reproduces shock-wave propagation speed well, but
cannot model the smooth onset of congestion or vehicle heterogeneity.

---

### Gipps (1981) — safety-distance model <sup>[(ref)](#ref-gipps)</sup>

Computes two candidate speeds each timestep and takes the lower one:

```
v_accel = v + 2.5 * a * dt * (1 - v/v0) * sqrt(0.025 + v/v0)

v_brake = b*dt + sqrt( (b*dt)^2 + b*(2*gap - v*dt - v_leader^2 / b_leader) )

v_next  = min(v_accel, v_brake)
```

| Symbol     | Meaning |
|------------|---------|
| `a`        | Maximum acceleration (m/s²) |
| `b`        | Comfortable deceleration (m/s²) |
| `b_leader` | Assumed maximum braking of the leader (m/s²) |

`v_brake` is the highest speed from which the car can still stop before
hitting a fully-braked leader — a pure safety constraint.  Discrete-time by
design; widely used in commercial tools.  Less smooth than IDM near free flow.

---

### Krauss (1998) — stochastic Gipps (used in SUMO) <sup>[(ref)](#ref-krauss)</sup>

Adds a randomisation term to Gipps to generate spontaneous jams:

```
v_safe    = v_leader + (gap - v_leader*tau) / ( (v + v_leader)/(2*b) + tau )

v_desired = min(v + a*dt,  v0,  v_safe)

v_next    = max(0,  v_desired - eps * rand(0,1))
```

| Symbol | Meaning |
|--------|---------|
| `tau`  | Reaction time (s) |
| `eps`  | Maximum random speed reduction (m/s) — controls jam spontaneity |

The stochastic term is what causes jams to form at subcritical densities.
Very fast to simulate; easy to calibrate; but the randomness is phenomenological
rather than derived from driver behaviour.

---

### Optimal Velocity Model / OVM (Bando et al., 1995) <sup>[(ref)](#ref-ovm)</sup>

Acceleration depends only on the gap, via an "optimal velocity" function `V(s)`:

```
dv/dt = a * ( V(s) - v )
```

A common choice for `V(s)`:

```
V(s) = (v0 / 2) * ( tanh(s - s_c) + tanh(s_c) )
```

| Symbol | Meaning |
|--------|---------|
| `a`    | Sensitivity / responsiveness (1/s) |
| `s_c`  | Inflection gap — transition between jam and free flow (m) |

The car accelerates when its current speed is below the "optimal" speed for
the current gap, and brakes otherwise.  Naturally produces stop-and-go waves
but has no explicit relative-velocity term, which can lead to unrealistically
large accelerations when gaps open suddenly.

---

### Full Velocity Difference Model / FVDM (Jiang et al., 2001) <sup>[(ref)](#ref-fvdm)</sup>

Adds a relative-velocity correction to OVM:

```
dv/dt = a * ( V(s) - v )  +  lambda * (v_leader - v)
```

| Symbol   | Meaning |
|----------|---------|
| `lambda` | Relative-velocity sensitivity (1/s) |

The extra term damps the overshoot problem of OVM: when a gap opens, the
positive `v_leader - v` term prevents the car from accelerating too sharply.
Closer in spirit to IDM than OVM.

---

### Enhanced IDM / EIDM (Treiber & Kesting, 2009) <sup>[(ref)](#ref-eidm)</sup>

Uses the same acceleration equation as IDM but wraps it with a reaction-time
delay and a perception noise model:

```
-- perceive with delay --
s_perceived  = s(t - T_react)   + noise_s
dv_perceived = dv(t - T_react)  + noise_dv

-- plug into IDM acceleration equation --
a_out = IDM( s_perceived, dv_perceived )

-- "coolness factor" moderates braking in IDM --
a_out = c * a_IDM_free  +  (1 - c) * a_IDM_full
```

| Symbol    | Meaning |
|-----------|---------|
| `T_react` | Reaction time delay (s), typically 0.5–1.5 s |
| `c`       | Coolness factor 0–1: 0 = full IDM braking, 1 = free-road only |

Reaction time introduces a lag that is the primary cause of real stop-and-go
instability; perception noise produces scatter in following distances matching
empirical data.  More realistic than IDM but has more parameters to calibrate.

---

### Wiedemann (1974) — psycho-physical / threshold model (used in VISSIM) <sup>[(ref)](#ref-wiedemann)</sup>

Drivers only react when they *perceive* a change in gap or relative speed.
There is no single closed-form EOM; instead the car is always in one of four
regimes determined by whether two thresholds are crossed:

```
Regime         Condition (approximate)
-----------    -------------------------------------------------------
Free driving   gap > SDX  (gap larger than "desired following distance")
Approaching    gap <= SDX  and  dv < -CLDV  (closing faster than threshold)
Following      |dv| < OPDV  (speed difference below perception threshold)
Braking        gap < ABX   (gap below minimum acceptable)
```

| Symbol | Meaning |
|--------|---------|
| `SDX`  | Desired following distance threshold (m) |
| `ABX`  | Minimum acceptable gap (m) |
| `CLDV` | Closing speed perception threshold (m/s) |
| `OPDV` | Opening speed perception threshold (m/s) |

Within each regime a simple rule updates speed.  Transitions happen when
perceived quantities cross thresholds, which are themselves stochastic.
Captures human inattention and the accordion effect in platoons very well,
but the many threshold parameters make calibration difficult without
empirical trajectory data.

---

### LWR — macroscopic continuum model (Lighthill, Whitham, Richards, 1955–1956) <sup>[(ref)](#ref-lwr)</sup>

Not a car-following model — treats traffic as a compressible fluid:

```
d(rho)/dt  +  d(rho * V(rho))/dx  =  0
```

| Symbol    | Meaning |
|-----------|---------|
| `rho`     | Vehicle density (vehicles/km) |
| `V(rho)`  | Speed–density relation from the fundamental diagram |

This is a first-order PDE (conservation of vehicles).  Shock waves
correspond to the propagating boundaries between congested and free-flow
regions.  No individual vehicles; cannot model heterogeneity or lane changes.
Useful for large-scale network planning and analytical insight into wave speed.

---

## Comparison summary

| Model        | Time | Stochastic | Reaction delay | Vehicle length | Parameters |
|--------------|------|------------|----------------|----------------|------------|
| Newell       | cont | no  | via shift T    | via d          | 2          |
| Gipps        | disc | no  | implicit in dt | yes            | 4          |
| Krauss       | disc | yes | tau            | yes            | 5          |
| OVM          | cont | no  | no             | no             | 3          |
| FVDM         | cont | no  | no             | no             | 4          |
| **IDM**      | cont | no  | no             | via gap        | 5          |
| EIDM         | cont | yes | yes            | via gap        | 7+         |
| Wiedemann    | disc | yes | implicit       | via gap        | 8+         |
| LWR          | cont | no  | no             | via density    | 1 (V(rho)) |

---

## References

<a id="ref-newell"></a>
**Newell (1961)**

> Newell, G. F. (1961).
> *Nonlinear effects in the dynamics of car following.*
> Operations Research, 9(2), 209–229.
> https://doi.org/10.1287/opre.9.2.209

<a id="ref-gipps"></a>
**Gipps (1981)**

> Gipps, P. G. (1981).
> *A behavioural car-following model for computer simulation.*
> Transportation Research Part B: Methodological, 15(2), 105–111.
> https://doi.org/10.1016/0191-2615(81)90037-0

<a id="ref-krauss"></a>
**Krauss (1998)**

> Krauss, S. (1998).
> *Microscopic modeling of traffic flow: Investigation of collision free vehicle dynamics.*
> PhD thesis, University of Cologne / DLR.
> https://elib.dlr.de/6108/

<a id="ref-ovm"></a>
**OVM — Bando et al. (1995)**

> Bando, M., Hasebe, K., Nakayama, A., Shibata, A., & Sugiyama, Y. (1995).
> *Dynamical model of traffic congestion and numerical simulation.*
> Physical Review E, 51(2), 1035–1042.
> https://doi.org/10.1103/PhysRevE.51.1035

<a id="ref-fvdm"></a>
**FVDM — Jiang et al. (2001)**

> Jiang, R., Wu, Q., & Zhu, Z. (2001).
> *Full velocity difference model for a car-following theory.*
> Physical Review E, 64(1), 017101.
> https://doi.org/10.1103/PhysRevE.64.017101

<a id="ref-eidm"></a>
**EIDM — Treiber & Kesting (2009)**

> Treiber, M., & Kesting, A. (2009).
> *An open-source microscopic traffic simulator.*
> IEEE Intelligent Transportation Systems Magazine, 2(3), 6–13.
> https://doi.org/10.1109/MITS.2009.1659

<a id="ref-wiedemann"></a>
**Wiedemann (1974)**

> Wiedemann, R. (1974).
> *Simulation des Strassenverkehrsflusses.*
> Schriftenreihe des Instituts für Verkehrswesen der Universität Karlsruhe, Heft 8.
> (Original in German; no public DOI — summarised in the VISSIM documentation and in
> Olstam & Tapani, 2004: https://doi.org/10.3141/1876-06)

<a id="ref-lwr"></a>
**LWR — Lighthill, Whitham & Richards (1955–1956)**

> Lighthill, M. J., & Whitham, G. B. (1955).
> *On kinematic waves II: A theory of traffic flow on long crowded roads.*
> Proceedings of the Royal Society A, 229(1178), 317–345.
> https://doi.org/10.1098/rspa.1955.0089

> Richards, P. I. (1956).
> *Shock waves on the highway.*
> Operations Research, 4(1), 42–51.
> https://doi.org/10.1287/opre.4.1.42

---

<a id="ref-idm"></a>
**IDM original paper**

> Treiber, M., Hennecke, A., & Helbing, D. (2000).
> *Congested traffic states in empirical observations and microscopic simulations.*
> Physical Review E, 62(2), 1805–1824.
> https://doi.org/10.1103/PhysRevE.62.1805

This is where the acceleration and desired-gap equations are derived.
The paper fits the model to German Autobahn loop-detector data and shows it
reproduces synchronized flow and stop-and-go waves.

**Comprehensive textbook**

> Treiber, M., & Kesting, A. (2013).
> *Traffic Flow Dynamics: Data, Models and Simulation.*
> Springer.
> https://doi.org/10.1007/978-3-642-32460-4

Chapter 11 covers IDM in depth including calibration guidance, stability
analysis, and extensions.  Freely available lecture notes from the same
authors cover most of the same ground.

**MOBIL lane-change model** (used in this simulator for overtaking and keep-right)

> Kesting, A., Treiber, M., & Helbing, D. (2007).
> *General lane-changing model MOBIL for car-following models.*
> Transportation Research Record, 1999(1), 86–94.
> https://doi.org/10.3141/1999-10

MOBIL defines when a lane change is beneficial by comparing the acceleration
gain for the changing car against the inconvenience imposed on the new follower.
This simulator uses a simplified version: leftward moves require a gap
improvement of `incentive_m`; rightward (keep-right) moves require a free gap
of `keep_right_gap_m` with no explicit follower check.
