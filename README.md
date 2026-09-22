# 🏎️ Vehicle Dynamics Simulator — Bicycle Model & AE86 vs FC3S Duel

A vehicle-dynamics simulator in **MATLAB**, from the academic linear model up to a
**race duel that emerges from the physics** — with no scripted trajectory and no
numerical trick.

![AE86 vs FC3S duel](img/duel.gif)

> Toyota **AE86** (blue) vs Mazda **FC3S** (red). The FC leads. At `t = 3.5 s` it
> loses grip: it understeers, has to lift off the throttle, and the 86 passes it
> **on the inside line**. None of this is scripted — see [The duel](#-the-duel-a-race-that-emerges-from-the-physics).

---

## Contents

- [Overview](#overview)
- [The three models](#the-three-models)
- [The duel: a race that emerges from the physics](#-the-duel-a-race-that-emerges-from-the-physics)
- [Report](#-report)
- [Repository structure](#repository-structure)
- [Running the simulation](#running-the-simulation)
- [The physics model](#the-physics-model)
- [Parameters](#parameters)
- [Limitations & next steps](#limitations--next-steps)
- [Author](#author)

---

## Overview

Project for the course **Vehicle Performance and Dynamic Behaviour**
(SeaTech — University of Toulon). The goal: build a simulator that reproduces a car's
behaviour as faithfully as possible from the **bicycle model**, to get a concrete grasp
of the physics of yaw, sideslip and grip.

The work is split into three stages, from the simplest to the most realistic.

---

## The three models

| File | Tyre model | What it shows |
|---|---|---|
| **`Voiture.m`** | Linear (`F = −C·α`) | Theoretical baseline: cornering response, stability, natural understeer |
| **`Voiture_Saturation.m`** | Saturation at the grip limit (`F_max = μ·F_z`) | Behaviour at the limits of adhesion (sliding, breakaway) |
| **`Battle.m`** | Pacejka (magic formula) | **Closed-loop race duel** (see below) |

Each script produces the usual physical plots (trajectory, yaw rate, sideslip angle at
the CG, axle slip angles, lateral forces, longitudinal and lateral velocities) and an
animation.

---

## 🏁 The duel: a race that emerges from the physics

`Battle.m` pits a **Toyota AE86** against a **Mazda FC3S** in a hairpin bend (38 m
radius). It is the core of the project, and it was fully reworked to be
**physically honest**: the outcome of the race is not written in advance, it
**comes out of the computation**.

### Three real mechanisms, zero cheating

| Naive approach (scripted) | This project's approach (emergent) |
|---|---|
| Steering written by hand, frame by frame | **Autopilot** (*Stanley* controller): steering is *computed* at each step from the error to the path |
| Artificial yaw-damping term to avoid spinning out | **No yaw term at all**: stability comes solely from the driver following its line and managing its speed |
| Speeds hand-tuned to force the overtake | **Grip-limited speed**; the driver lifts off if it feels the car running wide |

### The only scripted event

At `t = 3.5 s`, the FC's grip drops (`μ ÷ 2`, worn tyres / slippery surface).
**That is the only thing imposed.** Everything else follows from the simulation:

```
grip drops → the FC understeers (can't turn enough) → the driver lifts off
to hold the corner → it loses ~1.5 s and drops to ~42 km/h → the 86, which kept
its grip, stays on the inside line and passes at ~6 s (+11 m at the finish).
```

### Proof that it is not planned

`Battle.m` also simulates **the same FC without the grip loss** (control run).

- **Without** the loss → the FC keeps the lead (+3 m). No overtake.
- **With** the loss → the 86 passes.

So it is indeed the grip loss — and nothing else — that causes the overtake.

![Duel analysis](img/analyse.png)

*Left to right, top to bottom: trajectories on the circuit · race gap
(control vs actual) · yaw rate bounded without any artificial damping · drop in the
FC's lateral forces at t = 3.5 s.*

---

## 📄 Report

The full project report — bicycle-model theory, stability analysis, transition to the
Pacejka model and a detailed analysis of the duel — is available here (in French):

**➡️ [Rapport.pdf](Rapport.pdf)**

---

## Repository structure

```
.
├── Voiture.m               % Linear bicycle model
├── Voiture_Saturation.m    % Model with tyre-force saturation
├── Battle.m                % AE86 vs FC3S duel (closed-loop, emergent)
├── Rapport.pdf             % Full project report (French)
├── img/
│   ├── duel.gif            % Race animation
│   └── analyse.png         % Analysis panel (4 plots)
└── README.md
```

---

## Running the simulation

**Requirements:** MATLAB **R2016b or later** (`Battle.m` uses local functions in a
script). No specific toolbox is needed.

```matlab
% In MATLAB, from the repository folder:
Battle.m                 % the duel: opens the analysis + the animation
Voiture.m                % the linear model on its own
Voiture_Saturation.m     % the saturation model on its own
```

`Battle.m` prints to the console the overtake time, the final gap, and the FC's peak
yaw rate.

---

## The physics model

**Bicycle model**: the two wheels of an axle are lumped at its centre.
States: heading `ψ`, yaw rate `r = ψ̇`, sideslip angle at the CG `β`. Input: steering
angle `δ`. Integrated with the **Euler method**.

**Axle slip angles**

$$\alpha_f = \beta + \frac{L_f\,r}{u} - \delta \qquad \alpha_r = \beta - \frac{L_r\,r}{u}$$

**Lateral force — Pacejka magic formula**

$$F_y = -F_{max}\,\sin\!\Big(C\,\arctan\big(B\alpha - E(B\alpha - \arctan B\alpha)\big)\Big)$$

with $F_{max} = \mu\,F_z$ and the static loads $F_{z,f} = \dfrac{m g L_r}{L}$,
$F_{z,r} = \dfrac{m g L_f}{L}$.

**Equations of motion (Newton's second law)**

$$\dot r = \frac{L_f F_f - L_r F_r}{I_z} \qquad \dot\beta = \frac{F_f + F_r}{m\,u} - r$$

**Closed loop (the driver), specific to `Battle.m`**

- *Lateral (Stanley)* — steering computed from the heading error and the lateral
  error `e` to the followed line:
  $$\delta = (\psi_{path} - \psi) + \arctan\!\frac{k\,e}{u}$$
- *Longitudinal* — target speed bounded by grip, with a margin:
  $$v_{target} = \min\!\Big(v_{cruise},\ \sqrt{a_{lat}\,\mu\,g\,R_{local}}\Big)$$
  plus an extra reduction ("lift off") as soon as the car drifts away from its line.

---

## Parameters

Everything sits at the top of `Battle.m`:

| Parameter | Role | Default |
|---|---|---|
| `grip.fac` | Severity of the FC's grip loss (`0.5` = μ÷2) | `0.5` |
| `grip.t0`, `grip.t1` | Time window of the loss | `3.5 s → 8.5 s` |
| `fc.x0` | FC's head start at the line (m) | `16` |
| `ae.line_offset` | Inside-line offset of the 86 (the apex line) | `2.5 m` |
| `car.kc` | Optional driver counter-steer (0 = disabled) | `0` |

💡 To see the **control run** for yourself: set `grip.fac = 1` (no grip loss) and
re-run — the FC keeps the lead and the 86 does not pass.

---

## Limitations & next steps

- **Bicycle model**: roll is neglected, vertical loads are static (no dynamic load
  transfer under acceleration/braking/cornering).
- **No tyre self-aligning torque (SAT)**: adding it would stabilise the yaw further in a
  purely physical way.
- **Euler integration**: simple but less accurate than a Runge–Kutta scheme; too large a
  time step can make the forces oscillate near saturation.
- **Fixed racing line**: each car follows a predefined line; a real trajectory planner
  would choose the line dynamically.

---

## Author

**Tom Hurard** — engineering student, SeaTech (University of Toulon).
Project *Vehicle Performance and Dynamic Behaviour*, 2025.

*A nod to Initial D 🏔️ for the AE86 vs FC3S duel.*
