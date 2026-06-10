#!/usr/bin/env python3
"""
patch_opendbc_driver_torque.py

Disable the driver-torque steering clamp in the vendored opendbc submodule.

Why
---
On the Kia Niro EV the EPS torsion-bar self-aligning torque at high steering angle is
read as a driver override and clamps the applied steering torque below the commanded
value (worst on the sharper right turns -> right turns track ~7x worse). This patch
removes the *driver-torque component* of the steer limit on both sides of the safety
boundary (the opendbc car controller and the panda safety mode) while keeping the
global max-torque, rate (STEER_DELTA_UP/DOWN), and real-time-delta limits intact.
Full write-up: docs/steering_driver_torque_limit.md

Why a script
------------
opendbc_repo is a git submodule, so file edits there are NOT tracked by the nopilot
repo and are lost on `git submodule update`/re-checkout. This script lives in nopilot
and re-applies the edits to the submodule. It is idempotent: safe to run repeatedly,
and it detects whether each edit is already present.

Usage
-----
    python3 docs/patch_opendbc_driver_torque.py            # apply (default)
    python3 docs/patch_opendbc_driver_torque.py --check     # report only, exit 1 if unapplied
    python3 docs/patch_opendbc_driver_torque.py --opendbc /path/to/opendbc_repo

Run it from anywhere; by default it finds opendbc_repo next to the nopilot/docs folder.
Each edit either reports "applied", "already applied (skipped)", or fails loudly if the
anchor text is missing (i.e. upstream opendbc changed and the patch needs review).
"""

from __future__ import annotations

import argparse
import os
import re
import sys

DOC = "docs/steering_driver_torque_limit.md"


class PatchError(Exception):
    pass


def _read(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def _write(path: str, text: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


# --- individual patches -----------------------------------------------------
# Each returns (new_text, status) where status is one of:
#   "applied", "already"  -- and raises PatchError if the anchor can't be found.


def patch_carcontroller(text: str) -> tuple[str, str]:
    """car/hyundai/carcontroller.py: pass driver_torque=0 into the steer limiter."""
    call_re = re.compile(
        r"(apply_driver_steer_torque_limits\(\s*new_torque,\s*self\.apply_torque_last,\s*)"
        r"([^,]+?)"
        r"(\s*,\s*self\.params\))"
    )
    m = call_re.search(text)
    if not m:
        raise PatchError("could not find apply_driver_steer_torque_limits(new_torque, ...) call")

    already = m.group(2).strip() == "0" and "NAP-lab: driver-torque clamp disabled" in text
    if already:
        return text, "already"

    # force the 3rd argument (driver torque) to 0
    text = call_re.sub(r"\g<1>0\g<3>", text, count=1)

    # add a traceability comment immediately above the call if not present
    if "NAP-lab: driver-torque clamp disabled" not in text:
        line_re = re.compile(r"^([ \t]*)apply_torque = apply_driver_steer_torque_limits\(", re.M)
        lm = line_re.search(text)
        indent = lm.group(1) if lm else "    "
        comment = (
            f"{indent}# NAP-lab: driver-torque clamp disabled by passing driver_torque=0 so self-aligning\n"
            f"{indent}# torque at high steering angle is not read as a driver override (rate limiting kept).\n"
            f"{indent}# Panda side disabled via .disable_driver_torque_limit in safety/modes/hyundai.h.\n"
            f"{indent}# See {DOC}\n"
        )
        text = text[: lm.start()] + comment + text[lm.start():]
    return text, "applied"


def patch_safety_declarations(text: str) -> tuple[str, str]:
    """safety/safety_declarations.h: add disable_driver_torque_limit field."""
    if "disable_driver_torque_limit" in text:
        return text, "already"
    anchor = "  const int driver_torque_multiplier;\n"
    if anchor not in text:
        raise PatchError("could not find 'const int driver_torque_multiplier;' in TorqueSteeringLimits")
    addition = (
        anchor
        + "  // NAP-lab: when true, measured driver torque is ignored in the driver limit check\n"
        + "  // (self-aligning torque was read as a driver override and throttled the actuator).\n"
        + "  // Global max, rate and RT-delta limits still apply. Defaults to false.\n"
        + "  const bool disable_driver_torque_limit;\n"
    )
    return text.replace(anchor, addition, 1), "applied"


def patch_lateral(text: str) -> tuple[str, str]:
    """safety/lateral.h: honor the flag with a zero driver sample."""
    if "disable_driver_torque_limit" in text:
        return text, "already"
    block_re = re.compile(
        r"if \(limits\.type == TorqueDriverLimited\) \{\s*\n"
        r"(?P<ind>[ \t]*)violation \|= driver_limit_check\(desired_torque, desired_torque_last, &torque_driver,"
        r"(?P<rest>.*?)\);",
        re.DOTALL,
    )
    m = block_re.search(text)
    if not m:
        raise PatchError("could not find driver_limit_check(... &torque_driver ...) block in steer_torque_cmd_checks")
    ind = m.group("ind")
    rest = m.group("rest")
    replacement = (
        "if (limits.type == TorqueDriverLimited) {\n"
        f"{ind}// NAP-lab: when disable_driver_torque_limit is set, feed a zero driver sample so the\n"
        f"{ind}// driver bound never binds (self-aligning torque can't clamp the actuator below the\n"
        f"{ind}// command). Global max, rate and RT-delta limits still apply. See {DOC}\n"
        f"{ind}static const struct sample_t zero_driver = {{0}};\n"
        f"{ind}const struct sample_t *driver_sample = limits.disable_driver_torque_limit ? &zero_driver : &torque_driver;\n"
        f"{ind}violation |= driver_limit_check(desired_torque, desired_torque_last, driver_sample,"
        f"{rest});"
    )
    return text[: m.start()] + replacement + text[m.end():], "applied"


def patch_hyundai_mode(text: str) -> tuple[str, str]:
    """safety/modes/hyundai.h: set .disable_driver_torque_limit = true in HYUNDAI_LIMITS."""
    if "disable_driver_torque_limit" in text:
        return text, "already"
    anchor = "  .driver_torque_multiplier = 2, \\\n"
    if anchor not in text:
        raise PatchError("could not find '.driver_torque_multiplier = 2, \\' in HYUNDAI_LIMITS")
    addition = (
        anchor
        + "  .disable_driver_torque_limit = true,  /* NAP-lab: self-aligning torque was throttling the "
        + "actuator; see " + DOC + " */ \\\n"
    )
    return text.replace(anchor, addition, 1), "applied"


PATCHES = [
    ("opendbc/car/hyundai/carcontroller.py", patch_carcontroller),
    ("opendbc/safety/safety_declarations.h", patch_safety_declarations),
    ("opendbc/safety/lateral.h", patch_lateral),
    ("opendbc/safety/modes/hyundai.h", patch_hyundai_mode),
]


def default_opendbc_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))      # nopilot/docs
    return os.path.normpath(os.path.join(here, "..", "opendbc_repo"))


def main() -> int:
    ap = argparse.ArgumentParser(description="Disable the opendbc driver-torque steering clamp (controller + panda).")
    ap.add_argument("--opendbc", default=default_opendbc_path(),
                    help="path to the opendbc submodule (default: ../opendbc_repo next to this script)")
    ap.add_argument("--check", action="store_true",
                    help="report only, do not write; exit 1 if any patch is not yet applied")
    args = ap.parse_args()

    root = args.opendbc
    if not os.path.isdir(root):
        print(f"error: opendbc repo not found at {root}", file=sys.stderr)
        return 2

    print(f"opendbc: {root}\n")
    n_applied = n_already = 0
    failures = []

    for rel, fn in PATCHES:
        path = os.path.join(root, rel)
        if not os.path.isfile(path):
            print(f"  MISSING  {rel}")
            failures.append(rel)
            continue
        try:
            new_text, status = fn(_read(path))
        except PatchError as e:
            print(f"  FAILED   {rel}: {e}")
            failures.append(rel)
            continue

        if status == "already":
            print(f"  ok       {rel} (already applied)")
            n_already += 1
        else:  # applied
            if args.check:
                print(f"  WOULD    {rel} (needs patch)")
            else:
                _write(path, new_text)
                print(f"  patched  {rel}")
            n_applied += 1

    print()
    if failures:
        print(f"{len(failures)} file(s) could not be patched - upstream opendbc may have changed; review manually.")
        return 2
    if args.check:
        if n_applied:
            print(f"{n_applied} patch(es) not yet applied, {n_already} already applied.")
            return 1
        print(f"all {n_already} patches already applied.")
        return 0
    print(f"done: {n_applied} applied, {n_already} already applied.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
