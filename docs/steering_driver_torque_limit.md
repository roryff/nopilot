# Disabling the driver-torque steering clamp (right-turn under-actuation)

## Summary

On the Kia Niro EV, closed-loop path following tracked **left turns much better than
right turns** (cross-track RMS ~0.12 m left vs ~0.80 m right on the `rorySuperPath`
run). The root cause is **not** the steering rate limiter. It is the **driver-torque
clamp** inside openpilot's steer-torque limiting: at high steering angle the EPS
torsion-bar sensor reads the road's **self-aligning torque**, the limiter interprets
that as a driver fighting the wheel, and it **clamps the applied torque below the
commanded value**. Because the sharp, sustained corners on the route are right-handed,
the clamp robs right turns of authority far more than left turns.

This document records the evidence and the fix. The fix disables the **driver-torque
component** of the limit on **both** sides of the safety boundary — the opendbc car
controller and the panda safety mode — while keeping the global max, rate, and
real-time-delta limits intact.

## Symptom

With the lateral command saturated (`actuators_torque = -1`, a hard right), the actually
applied torque (`car_output_torque = carOutput.actuatorsOutput.torque`) did **not** sit
at -1. It hovered noisily and bounced *back toward zero* before eventually reaching -1.
Example window from `rorySuperPath` (command pinned at -1.00 throughout):

```
 t[s]   actuators  car_output  wheel_angle
 39.0     -1.00      -0.94       -65 deg
 39.4     -1.00      -0.92       -94 deg     <- moving the WRONG way while cmd held at -1
 40.5     -1.00      -0.82      -116 deg
 41.2     -1.00      -0.88      -132 deg
 43.0     -1.00      -1.00      -170 deg     <- finally locks to -1 deep in the corner
```

## Why it is not the rate limiter

The rate limiter (`STEER_DELTA_UP=4`, `STEER_DELTA_DOWN=7`, 100 Hz) can only move the
applied torque *toward* the target. With the command held at -1 it can ramp to -1 and
hold — it can **never** move the applied torque back toward zero. But the data does
exactly that. Counting samples where the applied torque steps *away* from a saturated
command:

| command | backing-off steps |
|---|---|
| left (`+1`) | 4.4 % of 366 samples |
| right (`-1`) | **17.1 % of 1126 samples** |

Only a limit whose ceiling *moves* can do this. In the whole opendbc + panda chain the
only such term is the driver-torque clamp, whose bound is a function of the measured
torsion-bar torque.

## Mechanism

`apply_driver_steer_torque_limits` (opendbc `car/lateral.py`) and `driver_limit_check`
(panda `safety/lateral.h`) both compute, for a right (negative) command:

```
min_allowed = -STEER_MAX + (-ALLOWANCE + tau_driver) * MULTIPLIER
```

With `STEER_MAX=600`, `ALLOWANCE=50`, `MULTIPLIER=2` the clamp starts biting once the
measured torque `tau_driver` exceeds the **50-unit allowance**, and the achievable
magnitude falls as `(700 - 2*tau_driver)/600`. The torsion-bar signal is noisy, so the
ceiling jitters — hence the noisy plateau. (The 50-unit threshold is independent of
`STEER_MAX`.)

This is a tire effect, not a fixed timer. Self-aligning torque rises with steering
slip to a peak (mid-corner) and then **falls past peak slip** at the deepest steering.
Shortfall `(1 - |car_output|)` for the right saturated samples, binned by wheel angle,
shows exactly that — the clamp bites in the mid-angle band and releases when the wheel
is deep into the turn:

| wheel angle | mean shortfall | reaches >0.97 |
|---|---|---|
| 85–120° | 0.111 | 3 % |
| 120–165° | 0.085 | 12 % |
| **165–210°** | **0.008** | **90 %** |
| **210–300°** | **0.000** | **100 %** |

Left turns barely enter the clamp-biting regime (left reaches max torque 76–100 % of the
time above 85°), because in this route the left corners never load the torsion bar past
the allowance. The asymmetry is therefore **demand-driven**: the route's sharp, sustained
corners are right-handed, so right turns sit in the high-self-aligning-torque band far
longer and get clamped far more.

Why the clamp exists: it lets a human always overpower LKAS. With hands off the wheel,
self-aligning torque masquerades as a driver override, so the safety logic needlessly
throttles the actuator precisely when the controller needs full authority (hard
cornering).

## Fix

Disable the **driver-torque term only**, on both sides of the safety boundary. Both
sides must agree: if opendbc commands more torque than panda's driver clamp allows,
panda flags a safety violation and faults the steering. The global max-torque, rate
(`max_rate_up`/`max_rate_down` / `STEER_DELTA_UP`/`DOWN`) and real-time-delta limits are
**kept** — only the measured-driver-torque dependency is removed.

### Controller side — `opendbc/car/hyundai/carcontroller.py`

Pass `driver_torque=0` so the driver bound is never reached:

```python
apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, 0, self.params)
```

### Panda side — `opendbc/safety/`

A scoped flag `disable_driver_torque_limit` was added so this is opt-in per safety mode
(it does not change any other car):

- `safety/safety_declarations.h` — new `const bool disable_driver_torque_limit;` field in
  `TorqueSteeringLimits` (defaults to false).
- `safety/lateral.h` — in `steer_torque_cmd_checks`, when the flag is set the
  `driver_limit_check` is given a **zero driver sample**, which makes the driver bound
  non-binding while the rate and RT limits still apply.
- `safety/modes/hyundai.h` — `HYUNDAI_LIMITS` sets `.disable_driver_torque_limit = true`.

Passing a zero driver sample is the exact analogue of the controller's `driver_torque=0`:
`driver_max_limit = STEER_MAX + ALLOWANCE*MULT` (> STEER_MAX) and
`driver_min_limit = -(STEER_MAX + ALLOWANCE*MULT)`, so the driver clamp never binds and
`steer_torque_cmd_checks` reduces to the global-max + rate + RT-delta checks.

## Safety implications

- The driver can no longer overpower LKAS by torque alone within the normal envelope; the
  steering will resist a hand input up to the rate/global limits. This is acceptable for a
  closed-course autonomous research vehicle and **must be reviewed before any public-road
  use**. The angle-based EPS fault-avoidance (cutting the request bit above 85°) and the
  global torque ceiling are unchanged.
- Both sides are disabled together intentionally. Disabling only the controller side would
  leave panda faulting whenever self-aligning torque pushed the command past the panda
  driver bound; disabling only panda would leave the controller still throttling itself.

## How to verify

`steeringTorque` (the measured torsion-bar torque) is **not** currently in the ROS
`VehicleState` message, so the clamp can only be confirmed by elimination from
`actuators_torque` vs `car_output_torque`. To confirm directly, add `steeringTorque` /
`steeringTorqueEps` to `VehicleState.msg` and `comma_node` (the comma already sends both
in `tools/adb_tcp_client.py`). After the fix you should see:

- `car_output_torque` reaches and holds ±1 whenever `actuators_torque` is saturated,
  in both directions, with no backing-off steps;
- the right-turn cross-track error drops toward the left-turn level;
- (with `steeringTorque` logged) the pre-fix shortfall tracks `2*(steeringTorque - 50)`.

## References

- Investigation bag: `rorySuperPath/rorySuperPath_0.mcap`; analysis in
  `rorySuperPath/ANALYSIS_rorySuperPath.md`.
- Related: `docs/raise_steer_limit.md` (global torque ceiling), `docs/SAFETY.md`.
