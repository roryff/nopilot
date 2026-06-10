#!/usr/bin/env python3
"""
Read Kia Niro EV steering torque from CAN (MDPS12, addr 0x251)
and print decoded values in real time.

Signals decoded from hyundai_kia_generic.dbc:
  CR_Mdps_StrColTq : 0|11@1+  (1.0, -1024.0)  no unit  — raw LKAS-scale torque
  CR_Mdps_StrTq    : 40|12@1+ (0.01, -20.48)   Nm       — driver column torque
  CR_Mdps_OutTq    : 52|12@1+ (0.1, -204.8)    no unit  — EPS output torque

Usage:
  python read_steer_torque.py

Requires:
  pip install pandacan   (or: the panda repo in PYTHONPATH)
"""

import time
import struct
from panda import Panda

MDPS12_ADDR = 0x251  # 593 decimal
LKAS11_ADDR = 0x340  # 832 decimal — the torque *request* we send

# ── bit extraction helpers ──────────────────────────────────────────────────

def extract_bits(data: bytes, start_bit: int, length: int, signed: bool = False) -> int:
    """Extract an Intel (little-endian) CAN signal from a bytes object."""
    value = 0
    for i in range(length):
        bit_pos = start_bit + i
        byte_idx = bit_pos // 8
        bit_idx  = bit_pos % 8
        if byte_idx < len(data) and (data[byte_idx] >> bit_idx) & 1:
            value |= (1 << i)
    if signed and (value & (1 << (length - 1))):
        value -= (1 << length)
    return value


def decode_mdps12(data: bytes) -> dict:
    raw_col  = extract_bits(data, 0,  11)   # CR_Mdps_StrColTq
    raw_tq   = extract_bits(data, 40, 12)   # CR_Mdps_StrTq
    raw_out  = extract_bits(data, 52, 12)   # CR_Mdps_OutTq

    col_tq_raw = raw_col * 1.0 - 1024.0    # same scale as LKAS request (±1024)
    str_tq_nm  = raw_tq  * 0.01 - 20.48    # Nm  (-20.48 .. +20.47)
    out_tq     = raw_out * 0.1  - 204.8    # unitless output

    return {
        "col_tq_raw":  col_tq_raw,
        "str_tq_nm":   str_tq_nm,
        "out_tq":      out_tq,
    }


def decode_lkas11_request(data: bytes) -> float:
    """Decode the torque request openpilot is sending (CR_Lkas_StrToqReq)."""
    raw = extract_bits(data, 16, 11)
    return raw * 1.0 - 1024.0


# ── main ────────────────────────────────────────────────────────────────────

def main():
    print("Connecting to panda...")
    p = Panda()
    p.set_safety_mode(Panda.SAFETY_SILENT)  # read-only, do not actuate
    p.can_clear(0xFFFF)                     # flush stale messages

    print(f"{'Time':>8}  {'ColTqRaw':>10}  {'SteerTq(Nm)':>12}  {'OutTq':>8}  {'LKASReq':>9}")
    print("-" * 60)

    last_request = 0.0

    try:
        while True:
            msgs = p.can_recv()
            for addr, _, data, bus in msgs:
                if bus != 0:
                    continue

                if addr == MDPS12_ADDR:
                    d = decode_mdps12(bytes(data))
                    ts = time.strftime("%H:%M:%S")
                    print(
                        f"{ts:>8}  "
                        f"{d['col_tq_raw']:>10.1f}  "
                        f"{d['str_tq_nm']:>12.3f} Nm  "
                        f"{d['out_tq']:>8.1f}  "
                        f"{last_request:>9.1f}"
                    )

                elif addr == LKAS11_ADDR:
                    last_request = decode_lkas11_request(bytes(data))

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        p.close()


if __name__ == "__main__":
    main()
