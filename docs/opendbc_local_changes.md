# Local opendbc changes (NAP-lab)

`opendbc_repo` is a git submodule pointing at upstream `commaai/opendbc`. We have no
fork with push access, so edits made there are **not** tracked by this repo and are
lost on `git submodule update` or a fresh checkout.

`opendbc_local_changes.patch` in this folder is the complete, verbatim snapshot of
those edits as they run on the car. It is the authoritative record.

## Applying

From a clean checkout of the pinned submodule commit
(`c2ba07083c1cfbb0c5b86469bd7b87090291a0d3`):

```bash
git submodule update --init opendbc_repo
cd opendbc_repo
git apply ../docs/opendbc_local_changes.patch
```

Verify with `git -C opendbc_repo diff --stat` — expect 6 files, 31 insertions.

## What is in it

| File | Change |
|---|---|
| `car/hyundai/carcontroller.py` | driver-torque clamp disabled (passes `driver_torque=0`) |
| `safety/lateral.h` | honours `disable_driver_torque_limit` |
| `safety/modes/hyundai.h` | sets `.disable_driver_torque_limit = true` |
| `safety_declarations.h` | declares the flag |
| `car/hyundai/values.py` | `STEER_DELTA_UP` 3→4, `STEER_THRESHOLD` 150→300 |
| `safety/modes/hyundai.h` | `HYUNDAI_LIMITS(384, 3, 7)` → `(384, 4, 7)` |
| `car/hyundai/carstate.py` | per-wheel speeds into `ret.wheelSpeeds.{fl,fr,rl,rr}` |

Background on the first four: [steering_driver_torque_limit.md](steering_driver_torque_limit.md).
Background on the rate limit: [raise_steer_limit.md](raise_steer_limit.md).

## Warning: `patch_opendbc_driver_torque.py` is only a subset

That script applies **4 of the 7 edits above** — the driver-torque clamp group only.
It will report `all 4 patches already applied` on a tree that is still missing:

- `values.py` `STEER_DELTA_UP` / `STEER_THRESHOLD`
- `safety/modes/hyundai.h` `HYUNDAI_LIMITS(384, 4, 7)`
- `carstate.py` per-wheel speeds

The first two are a **matched pair**: `STEER_DELTA_UP` in openpilot and the rate
argument to `HYUNDAI_LIMITS` in panda must be equal, or panda silently drops the LKAS
message and steering goes dead with no obvious error. Use the patch file, not the
script, unless you only want the driver-torque group.

## Note on `raise_steer_limit.md`

That document describes `STEER_MAX = 600` / `HYUNDAI_LIMITS(600, 10, 10)` as "current".
The car actually runs `HYUNDAI_LIMITS(384, 4, 7)`. Treat the document as method, not as
a record of current values.
