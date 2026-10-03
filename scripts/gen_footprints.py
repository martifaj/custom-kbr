#!/usr/bin/env python3
"""Generate Kailh MX hotswap footprints (CPG151101S11 socket) into lib/kbr.pretty.

The footprints are bottom-side parts: they are written as front-side footprints in
mirrored coordinates, and generate_pcb.py flips them onto B.Cu. That way the socket
pads land on the bottom, the switch holes end up in the right place, and pick-and-place
output lists the sockets as bottom-side parts. Origin = switch centre.

Geometry below is in *real* coordinates (seen from the top, Y down); mirroring
happens on write.
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB = os.path.join(ROOT, "lib", "kbr.pretty")

WIDTHS = [1, 1.25, 1.5, 1.75, 2, 2.25, 6.25]

CENTER_HOLE = 4.0
PEG_HOLE = 1.75                  # 5-pin switch plastic pegs at (+-5.08, 0)
SOCKET_HOLES = [(-3.81, -2.54), (2.54, -5.08)]   # switch pins, 3.0 mm for the socket
SOCKET_HOLE = 3.0
SOCKET_PADS = {"1": (5.842, -5.08), "2": (-7.085, -2.54)}  # pad 1 = pin at (2.54, -5.08)
SOCKET_PAD_SIZE = (2.55, 2.5)
SOCKET_BODY = (-8.6, -7.3, 7.4, -0.3)            # courtyard: x0, y0, x1, y1
# PCB-mount (Cherry) stabilizer: x offset per key width; holes at y=-7.0 (3.05) and y=+8.24 (4.0)
STAB_X = {2: 11.9, 2.25: 11.9, 2.75: 11.9, 6.25: 50.0, 7: 57.15}


def _m(x):  # mirror x (front-side definition of a bottom-side part)
    return -x


def npth(x, y, d):
    return (f'\t(pad "" np_thru_hole circle (at {_m(x):.4f} {y:.4f}) (size {d} {d}) (drill {d})'
            f' (layers "*.Cu" "*.Mask"))\n')


def rect(layer, x0, y0, x1, y1, width):
    return (f'\t(fp_rect (start {_m(x0):.4f} {y0:.4f}) (end {_m(x1):.4f} {y1:.4f})'
            f' (stroke (width {width}) (type solid)) (fill none) (layer "{layer}"))\n')


def footprint(w):
    name = f"SW_MX_Hotswap_{w:.2f}u"
    half = w * 19.05 / 2
    s = [f'(footprint "{name}"\n\t(version 20240108)\n\t(generator "custom-kbr")\n\t(layer "F.Cu")\n',
         f'\t(descr "Cherry MX compatible switch, Kailh CPG151101S11 hotswap socket, {w}u keycap")\n',
         '\t(tags "MX hotswap Kailh keyboard")\n\t(attr smd)\n',
         '\t(property "Reference" "SW**" (at 0 3.2 0) (layer "F.SilkS")'
         ' (effects (font (size 1 1) (thickness 0.15))))\n',
         f'\t(property "Value" "{name}" (at 0 8 0) (layer "F.Fab")'
         ' (effects (font (size 1 1) (thickness 0.15))))\n']
    # holes
    s.append(npth(0, 0, CENTER_HOLE))
    s.append(npth(-5.08, 0, PEG_HOLE))
    s.append(npth(5.08, 0, PEG_HOLE))
    for x, y in SOCKET_HOLES:
        s.append(npth(x, y, SOCKET_HOLE))
    if w in STAB_X:
        for sx in (-STAB_X[w], STAB_X[w]):
            s.append(npth(sx, -7.0, 3.05))
            s.append(npth(sx, 8.24, 4.0))
    # socket pads
    for num, (x, y) in SOCKET_PADS.items():
        s.append(f'\t(pad "{num}" smd rect (at {_m(x):.4f} {y:.4f}) (size {SOCKET_PAD_SIZE[0]} {SOCKET_PAD_SIZE[1]})'
                 ' (layers "F.Cu" "F.Paste" "F.Mask"))\n')
    # socket body: courtyard + fab + silk
    s.append(rect("F.CrtYd", *SOCKET_BODY, 0.05))
    s.append(rect("F.Fab", -5.95, -6.95, 4.75, -0.6, 0.1))
    s.append(rect("F.SilkS", -5.95, -6.95, 4.75, -0.6, 0.12))
    # keycap outline and switch housing for reference
    s.append(rect("Dwgs.User", -half, -9.525, half, 9.525, 0.1))
    s.append(rect("Dwgs.User", -7, -7, 7, 7, 0.1))
    s.append(")\n")
    return name, "".join(s)


def main():
    os.makedirs(LIB, exist_ok=True)
    for w in WIDTHS:
        name, text = footprint(w)
        with open(os.path.join(LIB, name + ".kicad_mod"), "w") as f:
            f.write(text)
        print("wrote", name)


if __name__ == "__main__":
    main()
