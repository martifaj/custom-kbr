#!/usr/bin/env bash
# DRC + manufacturing outputs (gerbers, drill, pick-and-place, BOM, renders).
set -euo pipefail
cd "$(dirname "$0")/.."
PCB=build/custom-kbr.kicad_pcb
OUT=build/fab
mkdir -p "$OUT/gerbers"

kicad-cli pcb drc --severity-all --format json --exit-code-violations -o build/drc.json "$PCB" \
  || echo "DRC reported violations, see build/drc.json"

kicad-cli pcb export gerbers -o "$OUT/gerbers/" \
  -l F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts "$PCB"
kicad-cli pcb export drill -o "$OUT/gerbers/" --format excellon --excellon-separate-th "$PCB"
(cd "$OUT/gerbers" && rm -f ../gerbers.zip && zip -q ../gerbers.zip ./*)

kicad-cli pcb export pos --side back --format csv --units mm --exclude-dnp -o "$OUT/pos-bottom.csv" "$PCB"
python3 scripts/bom.py "$PCB" "$OUT/bom.csv"

for side in top bottom; do
  kicad-cli pcb render --side "$side" --width 3000 --height 1100 --zoom 2.4 --quality basic \
    -o "build/render-$side.png" "$PCB"
done
echo "outputs in $OUT"
