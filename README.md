# custom-kbr — scripted 65% keyboard PCB

A 68-key 65% ANSI hotswap keyboard PCB generated entirely with the KiCad 9 `pcbnew` Python API,
autorouted with Freerouting, plus a matching QMK keyboard definition. The outline, screw holes and
USB-C position follow the Tada68 form factor, so it should fit cases made for Tada68 / DZ65 / KBD67 rev2
style boards (see "Before ordering").

## Design

| | |
|---|---|
| Layout | 65% ANSI, 68 keys (2u Backspace, 2.25u Enter / LShift, 1.75u RShift, 6.25u Space, arrows + Del/PgUp/PgDn/`) |
| Board | Tada68 form factor: 301.2 × 94.65 mm with the ta-65 outline (USB tab and four case notches), 2 layers, 1.6 mm, GND pour both sides |
| Switches | Cherry MX compatible, **Kailh hotswap sockets** (CPG151101S11) on the bottom, 3- or 5-pin switches, PCB-mount stabilizers on 2u+ keys; the spacebar stabilizer mounts upside down (wire towards the back), as on other Tada68 boards. `--switch solder` builds a soldered variant (not checked against the new outline) |
| Matrix | 5 × 15, COL2ROW, one 1N4148W (SOD-123) per key on the bottom side |
| MCU | ATmega32U4-MU (QFN-44), 16 MHz crystal, DFU bootloader (reset button enters it via HWB pull-down) |
| USB | USB-C (HRO TYPE-C-31-M-12) at the Tada68 position between Esc and 1, 5.1k CC pull-downs (USB 2.0 device), USBLC6-2SC6 ESD, 500 mA PTC, 22 Ω series resistors |
| Mounting | 6 × M2 non-plated holes at the Tada68 positions |

All SMD parts, including the sockets, are on the bottom side. The top is switches only, so a plate fits.
Hotswap sockets cover the upper half of every key on the bottom, so a few switches are turned 180°
(their socket moves to the lower half; keycaps don't care): Esc and 1, to make room for the USB-C
receptacle, fuse, ESD and CC resistors, and any key whose socket would sit on a case standoff (H and Down).
The script finds those automatically and stops if a hole can't be cleared that way.
Parts that are fixed by design (MCU, USB, crystal) are placed in `scripts/generate_pcb.py`;
matrix pins are assigned automatically so the fan-out around the MCU doesn't cross,
and the result is written to `build/pinmap.json` and fed into the QMK config.

## Files

- `scripts/layout.py` — key layout, matrix and MCU pinout (single source of truth)
- `scripts/gen_footprints.py` — writes the Kailh MX hotswap footprints (1u–6.25u, with stabilizer holes, plus a 6.25u variant with the stabilizer upside down) into `lib/kbr.pretty`
- `scripts/generate_pcb.py` — builds the board: placement, nets, design rules, autoroute, GND pours.
  Outline, holes and USB-C position from [ta-65 rev4](https://github.com/evyd13/ta-65) by evyd13 (CC BY-SA 4.0)
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
   exposed pad, which gets 4 vias to the pours; D+/D− run as parallel 0.25 mm tracks to the 22 Ω resistors;
   UCAP gets a stub and a via. The two long runs between the USB-C corner and the MCU under the spacebar
   are found by a grid search on the top layer, which only has drill holes on it: the D+/D− pair (two tracks
   0.4 mm apart, vias at both ends) and the +5V feed from the fuse to the MCU's bulk capacitor, which Freerouting
   could not fit through the matrix. All of these are locked, so Freerouting keeps them.
2. **Freerouting** routes everything else (about 3 minutes on a Raspberry Pi).
3. **Clean-up:** dangling stubs are deleted. Any connection still open is closed by a small local search
   (a straight or L-shaped track, or a hop over the other layer through two vias). Each candidate is kept
   only if `kicad-cli` DRC reports no new errors. As a last resort, Freerouting re-runs on the routed board.
4. **GND pours** on both layers. Netclasses and design rules are written into `build/custom-kbr.kicad_pro`,
so KiCad's DRC checks the same rules the router used (0.2 mm tracks / 0.15 mm clearance, 0.25 mm power).

## Case (3D printed, OpenSCAD)

`case/case.scad` is a tray-mount case plus an optional switch plate, built around the real board:
`scripts/gen_case_data.py` writes `case/kbr_data.scad` (outline, M2 holes, USB-C, reset button and switch
positions, all read from `build/custom-kbr.kicad_pcb`), so re-run it after changing the board.
Since the board now follows the Tada68 form factor, a store-bought case and plate for that family are an
alternative to printing.

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
- **Split:** the case is 309 mm wide, so it prints as two halves (172 and 145 × 105 mm, bottom face on the bed, no supports),
  joined by 4 vertical dovetails in the floor. Glue them; once the PCB is screwed to both halves the whole case is stiff.
  `split=false` gives one piece for beds of 320 mm or more.
- **Plate** (optional, 1.5 mm, cutouts for PCB-mount stabilizer housings): it floats on the switches and stops hotswap
  switches from coming out with the keycaps. It is split along key boundaries (188 + 150 mm), so each half is held by its own switches.
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

- Open the board in KiCad and look at it. Autorouted boards work but are not pretty.
  D+/D− run about 12 cm as a pair (the MCU is under the spacebar, the connector top-left); fine for full-speed USB.
- No schematic is generated — the netlist lives in `generate_pcb.py`. DRC parity checks are therefore off.
- Case fit is unverified: outline, holes and USB-C position are copied from ta-65 rev4, which says it fits Tada68 cases.
  Compare them with your case (or its stock PCB) before ordering. The receptacle sits 0.35 mm further in than on
  ta-65 (to keep its shield pins 0.3 mm from the edge), so its mouth sticks out 1.3 mm instead of 1.65 mm.
- If you use a commercial plate: ta-65 also mounts the spacebar stabilizer upside down, so plates for this family
  should match, but check its stabilizer cutouts. Plates come in layout variants; this board needs 2u Backspace,
  1.75u RShift and a 6.25u space with three 1u keys to its right.
- The hotswap footprint is generated from the commonly used Kailh CPG151101S11 dimensions. Print the board 1:1 and check it against a real socket before ordering.
- Crystal load caps (22 pF) assume a ~18–20 pF CL crystal; adjust for your part.
- The VID/PID in `keyboard.json` are placeholders.
