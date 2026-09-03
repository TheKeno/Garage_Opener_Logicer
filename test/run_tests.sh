#!/usr/bin/env bash
#
# Host tests for the Garage_Opener_Logicer_R6 serial command protocol -
# OPEN / CLOSE / STATE / SET / SAVE. Compiles the real sketch against the
# stubs in stub/ and runs 33 checks. No board required.
#
#   ./run_tests.sh
#
# Set BUTTON_DIR or ACCELDIAL_DIR if a library is somewhere this cannot find it.
set -euo pipefail
cd "$(dirname "$0")"

SKETCH=../Garage_Opener_Logicer_R6

for candidate in "${BUTTON_DIR:-}" \
                 "$HOME/Arduino/libraries/Button" \
                 "/mnt/old/backup from main drive/documents/Arduino/libraries/Button"; do
	if [ -n "$candidate" ] && [ -f "$candidate/Button.h" ]; then
		BUTTON="$candidate"
		break
	fi
done

if [ -z "${BUTTON:-}" ]; then
	echo "Cannot find the Button library." >&2
	echo "Set BUTTON_DIR=/path/to/libraries/Button and try again." >&2
	exit 1
fi

for candidate in "${ACCELDIAL_DIR:-}" \
                 "$HOME/Arduino/libraries/AccelDial" \
                 "../../libraries/AccelDial"; do
	if [ -n "$candidate" ] && [ -f "$candidate/AccelDial.h" ]; then
		DIAL="$candidate"
		break
	fi
done

if [ -z "${DIAL:-}" ]; then
	echo "Cannot find the AccelDial library." >&2
	echo "Set ACCELDIAL_DIR=/path/to/libraries/AccelDial and try again." >&2
	exit 1
fi

OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

g++ -std=c++17 -O1 -Wall -Wno-reorder \
	-Istub -I"$SKETCH" -I"$BUTTON" -I"$DIAL" \
	test.cpp "$SKETCH/DistanceSensor.cpp" "$SKETCH/LightPulseSensor.cpp" \
	"$BUTTON/Button.cpp" "$DIAL/AccelDial.cpp" \
	-o "$OUT/protocol_tests"

"$OUT/protocol_tests"
