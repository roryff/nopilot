#!/usr/bin/env bash

# Test launch script for joystick mode without real car

export FINGERPRINT="TOYOTA_COROLLA_TSS2"
export SKIP_FW_QUERY="1"
export STARTED="1"  # Reduce some checks

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
$DIR/../launch_chffrplus.sh
