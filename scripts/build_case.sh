#!/bin/sh
# Export the printable case and plate parts to build/case/*.stl.
# OPENSCAD=/path/to/openscad overrides the binary; extra args go to OpenSCAD (e.g. -D typing_angle=4).
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
OPENSCAD=${OPENSCAD:-openscad}
OUT="$ROOT/build/case"
mkdir -p "$OUT"
python3 "$ROOT/scripts/gen_case_data.py"
for p in case_left case_right plate_left plate_right; do
    echo "exporting $p.stl"
    "$OPENSCAD" -q -D "part=\"print_$p\"" "$@" -o "$OUT/$p.stl" "$ROOT/case/case.scad" &
done
wait
ls -l "$OUT"/*.stl
