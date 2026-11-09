#!/usr/bin/env bash
# Compile one sketch on the PC with the Arduino shim and run the tests.
#
# Usage:  bash run_tests.sh path/to/Sketch.ino [number_of_targets] [--avr-float] [--serial]
#
#   number_of_targets  how many random targets the IK test tries (default 500)
#   --avr-float        compile the sketch with 4-byte doubles, like an AVR Arduino does
#   --serial           run the serial command tests instead (Robot_Arms_Final only)
#
# Environment:  WARN=1  turns compiler warnings on (off by default, the old sketches have many)
#
# Only sketches that have invKine() can be tested (the Jacobian versions and later).
#
# Some of the older sketches call moveServo() before it is defined. The Arduino IDE
# adds missing prototypes for you, plain C++ does not, so this script adds a
# moveServo() prototype to a temporary copy. Your sketch file is never modified.

set -euo pipefail

SKETCH="${1:?usage: bash run_tests.sh path/to/Sketch.ino [N] [--avr-float] [--serial]}"
N=500
FLAGS=""
TEST="test_ik.cpp"
shift
for a in "$@"; do
    case "$a" in
        --avr-float) FLAGS="$FLAGS -DEMULATE_AVR_FLOAT" ;;
        --serial)    TEST="test_serial.cpp" ;;
        *[!0-9]*)    echo "unknown option: $a" >&2; exit 2 ;;
        *)           N="$a" ;;
    esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

SRC="$(cd "$(dirname "$SKETCH")" && pwd)/$(basename "$SKETCH")"
cp "$SRC" "$TMP/sketch.ino"

# Use whatever parameter type this sketch's moveServo() really takes (int or double).
TYPE="$(grep -o '^void moveServo(RobotArm &arm, ServoIndex servo, [a-z]* degree)' "$SRC" | awk '{print $(NF-1)}' || true)"
if [ -n "$TYPE" ]; then
    sed -i "/^RobotArm Arm2(servos2, motion2);/a void moveServo(RobotArm \&arm, ServoIndex servo, $TYPE degree);" "$TMP/sketch.ino"
fi

# The extra tests need the closed-form seed and the Jacobian refinement functions.
grep -q "seedFromClosedForm" "$SRC" && FLAGS="$FLAGS -DHAVE_SEED_AND_REFINE"

W="-w"; [ -n "${WARN:-}" ] && W="-Wall -Wextra"

g++ -std=c++17 -O2 $W $FLAGS -I"$HERE/include" -I"$HERE" \
    -DSKETCH="\"$TMP/sketch.ino\"" "$HERE/$TEST" -o "$TMP/test_bin"

echo "== $(basename "$SKETCH") =="
if [ "$TEST" = "test_ik.cpp" ]; then "$TMP/test_bin" "$N"; else "$TMP/test_bin"; fi
