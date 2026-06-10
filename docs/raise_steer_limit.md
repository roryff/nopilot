# Raising Hyundai Steer Torque Limit (Niro EV)

## What controls the limit

Two layers must match — if only one is changed, panda will drop the LKAS message silently.

| Layer | File | Current |
|---|---|---|
| openpilot | `opendbc_repo/opendbc/car/hyundai/values.py` | `STEER_MAX = 600` |
| panda firmware | `opendbc_repo/opendbc/safety/modes/hyundai.h` | `HYUNDAI_LIMITS(600, 10, 10)` |

The DBC signal `CR_Lkas_StrToqReq` has a physical range of **±1024**, so that is the hardware ceiling.

---

## Step 1 — Change openpilot

**`opendbc_repo/opendbc/car/hyundai/values.py` line ~56:**
```python
# Before
self.STEER_MAX = 600

# After (example: 700)
self.STEER_MAX = 700
```

---

## Step 2 — Change panda firmware

**`opendbc_repo/opendbc/safety/modes/hyundai.h` line ~182:**
```c
// Before
const TorqueSteeringLimits HYUNDAI_STEERING_LIMITS = HYUNDAI_LIMITS(600, 10, 10);

// After — match the value you set in Step 1
const TorqueSteeringLimits HYUNDAI_STEERING_LIMITS = HYUNDAI_LIMITS(700, 10, 10);
```

`HYUNDAI_LIMITS(max_torque, rate_up, rate_down)` — the rate values control how fast torque can ramp per frame at 100 Hz.

---

## Step 3 — Build and flash panda

```bash
cd /path/to/nopilot/panda

# Install build deps (first time only)
pip install -e .

# Build and flash (panda must be connected via USB)
python board/flash.py
```

If the panda is not responding, use recovery mode:
```bash
python board/recover.py
```

---

## Notes

- **Safe ceiling:** The EPS hardware accepts up to ±1024 but likely saturates well below that — the stock LKAS never exceeds ~400. Start conservatively (e.g., 700) and increase gradually.
- **If panda limit < openpilot value:** Panda drops the entire CAN frame → EPS times out → disengages.
- **If panda limit > openpilot value:** Safe, openpilot just never reaches the panda ceiling.
- The Niro EV 2nd gen (CANFD, 2023+) has a separate limit: `HYUNDAI_STEERING_LIMITS` in `hyundai_canfd.h`.
