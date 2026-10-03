# custom-kbr — scripted 65% keyboard PCB

A 68-key 65% ANSI keyboard PCB generated entirely with the KiCad 9 `pcbnew` Python API,
autorouted with Freerouting, plus a matching QMK keyboard definition.

## Design

| | |
|---|---|
| Layout | 65% ANSI, 68 keys (2u Backspace, 2.25u Enter / LShift, 1.75u RShift, 6.25u Space, arrows + Del/PgUp/PgDn/`) |
| Board | 304.8 × 103.75 mm (key area + 7.5 mm strip on top for USB + 1 mm at the front so the spacebar stabilizer holes stay inside the edge), 2 layers, 1.6 mm, GND pour both sides |
| Switches | Cherry MX compatible, **Kailh hotswap sockets** (CPG151101S11) on the bottom, 3- or 5-pin switches, PCB-mount stabilizers on 2u+ keys. `--switch solder` builds a soldered variant instead |
| Matrix | 5 × 15, COL2ROW, one 1N4148W (SOD-123) per key on the bottom side |
| MCU | ATmega32U4-MU (QFN-44), 16 MHz crystal, DFU bootloader (reset button enters it via HWB pull-down) |
| USB | USB-C (HRO TYPE-C-31-M-12) centred on the top strip, 5.1k CC pull-downs (USB 2.0 device), USBLC6-2SC6 ESD, 500 mA PTC, 22 Ω series resistors |
| Mounting | 6 × M2 plated holes on the row gaps (positions chosen automatically to clear parts) |

All SMD parts, including the sockets, are on the bottom side. The top is switches only, so a plate fits.
Hotswap sockets cover the upper half of every key on the bottom, which is why the board has an
extra strip on top for the USB-C connector, fuse, ESD and CC resistors.
Parts that are fixed by design (MCU, USB, crystal) are placed in `scripts/generate_pcb.py`;
matrix pins are assigned automatically so the fan-out around the MCU doesn't cross,
and the result is written to `build/pinmap.json` and fed into the QMK config.

## Files

- `scripts/layout.py` — key layout, matrix and MCU pinout (single source of truth)
- `scripts/gen_footprints.py` — writes the Kailh MX hotswap footprints (1u–6.25u, with stabilizer holes) into `lib/kbr.pretty`
- `scripts/generate_pcb.py` — builds the board: placement, nets, design rules, autoroute, GND pours
- `scripts/fab.sh` — DRC, gerbers + drill (zip), bottom pick-and-place, BOM, renders
- `scripts/gen_case_data.py`, `scripts/build_case.sh`, `case/case.scad` — 3D-printed case and plate (see below)
- `scripts/gen_qmk.py` — writes `firmware/qmk/keyboards/custom_kbr/` (keyboard.json + default keymap)
- `lib/kbr.pretty/` — hotswap footprints (generated) + the other footprints, vendored from the official KiCad library (CC-BY-SA 4.0 w/ exception)
- `tools/freerouting-1.9.0.jar` — autorouter. 2.x hangs on this board (internal error on the +5V net); 1.9.0 routes it in about 3 minutes

## Build

Requires KiCad 9 (`sudo apt install kicad`), Java 17+ and, on a machine without a desktop, `xvfb`
(Freerouting 1.9 opens a progress window even in batch mode; the script uses `xvfb-run` when
`$DISPLAY` is unset, or set `FREEROUTING_DISPLAY=:N` to pick a display). Freerouting runs with `-da`
so it doesn't send anonymous usage analytics.

```sh
python3 scripts/gen_footprints.py      # -> lib/kbr.pretty/SW_MX_Hotswap_*.kicad_mod
python3 scripts/generate_pcb.py          # -> build/custom-kbr.kicad_pcb (+ .kicad_pro)
python3 scripts/gen_qmk.py               # -> firmware/qmk/keyboards/custom_kbr
scripts/fab.sh                           # -> build/drc.json, build/fab/*
```

`--no-route` stops after placement (fast; useful while tweaking the layout).
`--passes N` sets the Freerouting pass limit. `--resume build/custom-kbr.kicad_pcb` continues routing
an existing board (for example after hand edits): it cleans dangling stubs and routes only what is missing.

Routing pipeline:
1. **Pre-routing** of the parts the autorouter handles badly: every QFN ground pin is tied inward to the
   exposed pad, which gets 4 vias to the pours; the USB D+/D− pair runs as parallel 0.25 mm tracks to the
   22 Ω resistors; UCAP gets a stub and a via. These are locked, so Freerouting keeps them.
2. **Freerouting** routes everything else (about 3 minutes on a Raspberry Pi).
3. **Clean-up:** dangling stubs are deleted. Any connection still open is closed by a small local search
   (a straight or L-shaped track, or a hop over the other layer through two vias). Each candidate is kept
   only if `kicad-cli` DRC reports no new errors. As a last resort, Freerouting re-runs on the routed board.
4. **GND pours** on both layers. Netclasses and design rules are written into `build/custom-kbr.kicad_pro`,
so KiCad's DRC checks the same rules the router used (0.2 mm tracks / 0.15 mm clearance, 0.25 mm power).

## Case (3D printed, OpenSCAD)

`case/case.scad` is a tray-mount case plus an optional switch plate, built around the real board:
`scripts/gen_case_data.py` writes `case/kbr_data.scad` (outline, M2 holes, USB-C and reset-button
positions from `build/custom-kbr.kicad_pcb`, key positions from `layout.py`), so re-run it after changing the board.

```sh
scripts/build_case.sh                          # -> build/case/{case,plate}_{left,right}.stl
scripts/build_case.sh -D typing_angle=4        # any parameter can be overridden
openscad case/case.scad                        # interactive preview (part = "assembly")
```

- **Tray mount:** the PCB screws onto 6 standoffs (M2 × 6 mm screws into M2 heat-set inserts, or
  `screw_mode="selftap"` for screws cut straight into the plastic). The standoffs are 5 mm wide where they touch
  the PCB (2.54 mm of free board around each hole) and flare out below the height of the sockets.
- **Height:** 4.5 mm under the PCB (sockets 1.8 mm, USB-C receptacle 3.26 mm). The rim is flush with the plate top.
- **Typing angle** 6° built into the bottom (`typing_angle`, 0 = flat). 6 recesses for 10 mm rubber bumpers.
- **USB-C:** 13 × 7.5 mm opening in the back wall, sized for most cable overmolds. **Reset:** paperclip hole in the floor under the button.
- **Split:** the case is 312 mm wide, so it prints as two halves (172 and 149 × 114 mm, bottom face on the bed, no supports),
  joined by 4 vertical dovetails in the floor. Glue them; once the PCB is screwed to both halves the whole case is stiff.
  `split=false` gives one piece for beds of 320 mm or more.
- **Plate** (optional, 1.5 mm, cutouts for PCB-mount stabilizer housings): it floats on the switches and stops hotswap
  switches from coming out with the keycaps. It is split along key boundaries (190 + 152 mm), so each half is held by its own switches.
  Print in PETG if you can; PLA works.

Assembly: screw the PCB in first, then press the switches through the plate into the sockets.
Unverified against real parts: the stabilizer cutout (`stab_cut_*`) and the USB opening; print a test piece first if your stabilizers or cable are unusual.

## Firmware

```sh
cp -r firmware/qmk/keyboards/custom_kbr ~/qmk_firmware/keyboards/
qmk compile -kb custom_kbr -km default
qmk flash   -kb custom_kbr -km default   # press the reset button on the back
```

Fn layer: F1–F12 on the number row, Fn+` = bootloader, media on arrows/space.

## Before ordering

- Open the board in KiCad and look at it. Autorouted boards work but are not pretty;
  check the USB D+/D− pair in particular (full-speed USB is tolerant of this, but shorter is better).
- No schematic is generated — the netlist lives in `generate_pcb.py`. DRC parity checks are therefore off.
- Outline and hole positions are not matched to any commercial 65% case (the board is 8.5 mm taller than a standard 65% PCB).
- The hotswap footprint is generated from the commonly used Kailh CPG151101S11 dimensions. Print the board 1:1 and check it against a real socket before ordering.
- Crystal load caps (22 pF) assume a ~18–20 pF CL crystal; adjust for your part.
- The VID/PID in `keyboard.json` are placeholders.
