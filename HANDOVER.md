# nopilot — NAP-lab handover (comma side)

This repo is the openpilot fork that runs on the comma device in the lab's Kia Niro EV.
It covers the **comma side only**. The ROS 2 / Jetson side lives in a separate repo,
[`NAPLabNTNU/thor`](https://github.com/NAPLabNTNU/thor) (`car_control` package).

## What is where

| | |
|---|---|
| Device | comma 3X (`tizi`), hostname `comma-8ff32d45`, AGNOS 12.8 |
| Dongle ID | `12fdf6595e41b23a` |
| Vehicle | Kia Niro EV |
| Checkout on device | `/data/openpilot`, branch `comma` |
| Base version | openpilot 0.10.1 |
| Entry point | `/data/continue.sh` → `/data/openpilot/launch_openpilot.sh` |

`comma` is the branch that runs on the car. `master`, `test`, `test_test`, `working`
and `copilot/…` are earlier or experimental lines, kept for history.

## How it talks to the ROS side

The comma **initiates** the connection outward:

```
comma device                                Jetson (thor)
tools/adb_tcp_client.py   ──TCP──>   thor.lan:3000   car_control/comma_node
```

`system/manager/process_config.py` starts it as the `adb_joystick` process. It must be
`tools.adb_tcp_client` (client, connects out), **not** `tools.adb_bridge_server` — both
files exist and the wrong one leaves the link dead with no obvious error. On the ROS
side this corresponds to `comma_node.yaml` with `use_tcp_tunnel: true`,
`tcp_listen_port: 3000`.

## ⚠ First thing after a fresh clone: patch opendbc

`opendbc_repo` is a submodule of upstream `commaai/opendbc`, which we cannot push to.
The car's steering behaviour depends on edits that live **only** in
`docs/opendbc_local_changes.patch`:

```bash
git submodule update --init opendbc_repo
cd opendbc_repo && git apply ../docs/opendbc_local_changes.patch
```

Expect 6 files / 31 insertions. Without this the car steers differently or not at all.

Do **not** rely on `docs/patch_opendbc_driver_torque.py` alone — it applies only 4 of
the 7 edits and reports success anyway. Details and the full list:
[`docs/opendbc_local_changes.md`](docs/opendbc_local_changes.md).

The two most dangerous values to get wrong are a matched pair:

- `opendbc/car/hyundai/values.py` → `STEER_DELTA_UP = 4`
- `opendbc/safety/modes/hyundai.h` → `HYUNDAI_LIMITS(384, 4, 7)`

If these disagree, panda silently drops the LKAS message and steering goes dead.

## Local modifications worth knowing about

- **Driver-torque clamp disabled.** EPS self-aligning torque at high steering angle was
  being read as a driver override and throttling the actuator (right turns tracked far
  worse than left). Removed on both sides of the safety boundary; global max-torque,
  rate and RT-delta limits still apply. Write-up:
  [`docs/steering_driver_torque_limit.md`](docs/steering_driver_torque_limit.md).
- **Steer rate raised** `STEER_DELTA_UP` 3 → 4, `STEER_THRESHOLD` 150 → 300.
  Method (not current values) in [`docs/raise_steer_limit.md`](docs/raise_steer_limit.md).
- **Per-wheel speeds** exposed via `ret.wheelSpeeds.{fl,fr,rl,rr}` in
  `opendbc/car/hyundai/carstate.py`, for both the CAN and CAN-FD paths.
- **50 Hz logging** and joystick/torque-limit changes — see `git log` on `comma`.

## Caveats

- Route recordings (`/data/media`, ~38 GB, 204 segments) are **not** in this repo and
  are not backed up anywhere.
- `docs/raise_steer_limit.md` quotes `STEER_MAX = 600` / `HYUNDAI_LIMITS(600, 10, 10)`
  as "current"; the car actually runs `HYUNDAI_LIMITS(384, 4, 7)`.
- This is a research vehicle with safety limits deliberately relaxed. Read
  `docs/steering_driver_torque_limit.md` before changing anything in `opendbc/safety/`.
