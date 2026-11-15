#!/usr/bin/env python3
"""Generate the custom-kbr 60% keyboard PCB with the KiCad (pcbnew) Python API.

Stages:
  1. place switches, diodes, MCU + USB circuitry, mounting holes; assign nets
  2. autoroute with Freerouting (Specctra DSN -> SES round trip)
  3. add GND pours on both layers and fill them

Usage:
  python3 scripts/generate_pcb.py [--switch hotswap|solder] [--no-route] [--passes N]
"""
import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
from layout import (KEYS, N_ROWS, N_COLS, MCU_GPIO, MCU_FIXED)  # noqa: E402
from gen_footprints import SOCKET_OUTLINE, SOCKET_PADS, SOCKET_PAD_SIZE, FLIP_STAB_WIDTHS  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB = os.path.join(ROOT, "lib", "kbr.pretty")
BUILD = os.path.join(ROOT, "build")
PCB_PATH = os.path.join(BUILD, "custom-kbr.kicad_pcb")
# 1.9.0 rather than 2.x: 2.1 hangs on this board (NullPointerException in its maze search
# on the +5V net), 1.9 routes it in ~3 minutes. 1.9 needs a display; see freerouting_cmd().
FREEROUTING_JAR = os.path.join(ROOT, "tools", "freerouting-1.9.0.jar")

# Netclasses / design rules. pcbnew.SaveBoard() does not write netclasses of an
# in-memory board to the .kicad_pro, so they are also patched into the project file.
NETCLASSES = {  # name: (track, clearance, via diameter, via drill) in mm
    "Default": (0.2, 0.15, 0.6, 0.3),
    # Must still leave a 0.25 mm QFN pad at 0.5 mm pitch; 0.25 mm on 1 oz copper is
    # good for ~1 A and a keyboard draws <100 mA.
    "Power": (0.25, 0.15, 0.7, 0.35),
    "USB": (0.25, 0.15, 0.6, 0.3),
}
NETCLASS_PATTERNS = {"GND": "Power", "GND_*": "Power", "+5V": "Power", "VBUS": "Power",
                     "USB_DP": "USB", "USB_DN": "USB", "U_DP": "USB", "U_DN": "USB"}
MIN_RULES = {  # JLCPCB / PCBWay standard 2-layer capabilities, with margin
    "min_clearance": 0.15, "min_track_width": 0.15, "min_via_diameter": 0.5,
    "min_through_hole_diameter": 0.3, "min_hole_clearance": 0.25, "min_hole_to_hole": 0.25,
    "min_copper_edge_clearance": 0.3,
}

# --- geometry helpers -------------------------------------------------------

def mm(v):
    return pcbnew.FromMM(v)


def pt(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


def to_mm(p):
    return pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)


def arc_points(start, mid, end, n=8):
    """n+1 points along the circular arc through start, mid and end."""
    (x1, y1), (x2, y2), (x3, y3) = start, mid, end
    d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    ux = ((x1**2 + y1**2) * (y2 - y3) + (x2**2 + y2**2) * (y3 - y1) + (x3**2 + y3**2) * (y1 - y2)) / d
    uy = ((x1**2 + y1**2) * (x3 - x2) + (x2**2 + y2**2) * (x1 - x3) + (x3**2 + y3**2) * (x2 - x1)) / d
    r = math.hypot(x1 - ux, y1 - uy)
    a1, a2, a3 = (math.atan2(y - uy, x - ux) for x, y in (start, mid, end))
    # sweep from a1 to a3 in the direction that passes through a2
    sweep = (a3 - a1) % (2 * math.pi)
    if (a2 - a1) % (2 * math.pi) > sweep:
        sweep -= 2 * math.pi
    return [(ux + r * math.cos(a1 + sweep * k / n), uy + r * math.sin(a1 + sweep * k / n)) for k in range(n + 1)]


def outline_chords(segments):
    """The outline as straight (a, b) pieces, arcs split into chords."""
    out = []
    for kind, *p in segments:
        pts = [p[0], p[1]] if kind == "line" else arc_points(*p)
        out += list(zip(pts, pts[1:]))
    return out


def outline_bbox(segments):
    pts = [q for a, b in outline_chords(segments) for q in (a, b)]
    return (min(x for x, _ in pts), min(y for _, y in pts), max(x for x, _ in pts), max(y for _, y in pts))


# --- board construction -----------------------------------------------------

class BoardBuilder:
    def __init__(self):
        self.board = pcbnew.BOARD()
        self.nets = {}

    def net(self, name):
        if name not in self.nets:
            n = pcbnew.NETINFO_ITEM(self.board, name)
            self.board.Add(n)
            self.nets[name] = n
        return self.nets[name]

    def footprint(self, lib_name, ref, value, x, y, rot=0.0, back=False, pads=None):
        """Place a library footprint with its origin at (x, y). pads: {pad_name: net}."""
        fp = pcbnew.FootprintLoad(LIB, lib_name)
        if fp is None:
            raise RuntimeError(f"footprint {lib_name} not found in {LIB}")
        fp.SetReference(ref)
        fp.SetValue(value)
        fp.SetPosition(pt(x, y))
        fp.SetOrientationDegrees(rot)
        self.board.Add(fp)
        if back:
            fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
        for pad in fp.Pads():
            if pads and pad.GetNumber() in pads and pads[pad.GetNumber()]:
                pad.SetNet(self.net(pads[pad.GetNumber()]))
        return fp

    def outline(self, segments):
        """Board outline on Edge.Cuts from ("line", start, end) / ("arc", start, mid, end)."""
        for kind, *p in segments:
            shape = pcbnew.PCB_SHAPE(self.board, pcbnew.SHAPE_T_SEGMENT if kind == "line" else pcbnew.SHAPE_T_ARC)
            if kind == "line":
                shape.SetStart(pt(*p[0]))
                shape.SetEnd(pt(*p[1]))
            else:
                shape.SetArcGeometry(pt(*p[0]), pt(*p[1]), pt(*p[2]))
            shape.SetLayer(pcbnew.Edge_Cuts)
            shape.SetWidth(mm(0.1))
            self.board.Add(shape)

    def text(self, s, x, y, layer, size=1.5, mirror=False):
        t = pcbnew.PCB_TEXT(self.board)
        t.SetText(s)
        t.SetPosition(pt(x, y))
        t.SetLayer(layer)
        t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
        t.SetTextThickness(mm(size * 0.15))
        t.SetMirrored(mirror)
        self.board.Add(t)

    def track(self, x0, y0, x1, y1, layer, width, net, locked=False):
        t = pcbnew.PCB_TRACK(self.board)
        t.SetStart(pt(x0, y0))
        t.SetEnd(pt(x1, y1))
        t.SetWidth(mm(width))
        t.SetLayer(layer)
        t.SetNet(self.net(net))
        t.SetLocked(locked)
        self.board.Add(t)
        return t

    def via(self, x, y, net, size=0.6, drill=0.3, locked=False):
        v = pcbnew.PCB_VIA(self.board)
        v.SetPosition(pt(x, y))
        v.SetFrontWidth(mm(size))  # SetWidth() needs a layer argument in KiCad 9
        v.SetDrill(mm(drill))
        v.SetNet(self.net(net))
        v.SetLocked(locked)
        self.board.Add(v)
        return v

    def edge_keepout(self, segments, width=0.35):
        """Track/via keep-out strips along the board edge. Freerouting ignores KiCad's
        copper-to-edge clearance, but it does respect keep-outs. One thin rectangle
        straddling each edge piece (arcs split into chords)."""
        for a, b in outline_chords(segments):
            dx, dy = b[0] - a[0], b[1] - a[1]
            n = math.hypot(dx, dy)
            if n < 1e-6:
                continue
            nx, ny = -dy / n * width, dx / n * width
            z = pcbnew.ZONE(self.board)
            z.SetIsRuleArea(True)
            z.SetLayerSet(pcbnew.LSET.AllCuMask())
            z.SetDoNotAllowTracks(True)
            z.SetDoNotAllowVias(True)
            z.SetDoNotAllowPads(False)
            z.SetDoNotAllowFootprints(False)
            z.SetDoNotAllowCopperPour(False)
            poly = z.Outline()
            poly.NewOutline()
            for x, y in [(a[0] + nx, a[1] + ny), (b[0] + nx, b[1] + ny), (b[0] - nx, b[1] - ny), (a[0] - nx, a[1] - ny)]:
                poly.Append(mm(x), mm(y))
            self.board.Add(z)

    def circle_keepout(self, cx, cy, r, layer, n=24):
        z = pcbnew.ZONE(self.board)
        z.SetIsRuleArea(True)
        z.SetLayer(layer)
        z.SetDoNotAllowTracks(True)
        z.SetDoNotAllowVias(True)
        z.SetDoNotAllowPads(False)
        z.SetDoNotAllowFootprints(False)
        z.SetDoNotAllowCopperPour(True)
        poly = z.Outline()
        poly.NewOutline()
        for k in range(n):
            a = 2 * math.pi * k / n
            poly.Append(mm(cx + r * math.cos(a)), mm(cy + r * math.sin(a)))
        self.board.Add(z)

    def zone(self, net, layer, bbox):
        """Pour over the outline's bounding box; the filler clips it to the board edge."""
        z = pcbnew.ZONE(self.board)
        z.SetLayer(layer)
        z.SetNet(self.net(net))
        poly = z.Outline()
        poly.NewOutline()
        x0, y0, x1, y1 = bbox
        for x, y in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]:
            poly.Append(mm(x), mm(y))
        z.SetLocalClearance(mm(0.3))
        z.SetMinThickness(mm(0.25))
        z.SetThermalReliefGap(mm(0.3))
        z.SetThermalReliefSpokeWidth(mm(0.4))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THT_THERMAL)  # solid on SMD pads
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        self.board.Add(z)
        return z


def setup_rules(board):
    ds = board.GetDesignSettings()
    ds.m_TrackMinWidth = mm(MIN_RULES["min_track_width"])
    ds.m_MinClearance = mm(MIN_RULES["min_clearance"])
    ds.m_ViasMinSize = mm(MIN_RULES["min_via_diameter"])
    ds.m_MinThroughDrill = mm(MIN_RULES["min_through_hole_diameter"])
    ds.m_HoleClearance = mm(MIN_RULES["min_hole_clearance"])
    ds.m_HoleToHoleMin = mm(MIN_RULES["min_hole_to_hole"])
    ds.m_CopperEdgeClearance = mm(MIN_RULES["min_copper_edge_clearance"])
    ds.SetBoardThickness(mm(1.6))

    ns = ds.m_NetSettings
    for name, (track, clearance, via, drill) in NETCLASSES.items():
        nc = ns.GetDefaultNetclass() if name == "Default" else pcbnew.NETCLASS(name)
        nc.SetTrackWidth(mm(track))
        nc.SetClearance(mm(clearance))
        nc.SetViaDiameter(mm(via))
        nc.SetViaDrill(mm(drill))
        if name != "Default":
            ns.SetNetclass(name, nc)
    for net, cls in NETCLASS_PATTERNS.items():
        ns.SetNetclassPatternAssignment(net, cls)
    board.SynchronizeNetsAndNetClasses(True)


def save_board(board, path):
    """SaveBoard + write netclasses and design rules into the matching .kicad_pro."""
    pcbnew.SaveBoard(path, board)
    write_project_rules(os.path.splitext(path)[0] + ".kicad_pro")


def write_project_rules(pro):
    project = {}
    if os.path.exists(pro):
        with open(pro) as f:
            project = json.load(f)
    rules = project.setdefault("board", {}).setdefault("design_settings", {}).setdefault("rules", {})
    rules.update(MIN_RULES)
    net = project.setdefault("net_settings", {})
    template = next((c for c in net.get("classes", []) if c["name"] == "Default"), {})
    classes = []
    for prio, (name, (track, clearance, via, drill)) in enumerate(NETCLASSES.items()):
        c = dict(template, name=name, track_width=track, clearance=clearance,
                 via_diameter=via, via_drill=drill)
        c["priority"] = 2147483647 if name == "Default" else prio
        classes.append(c)
    net["classes"] = classes
    net["netclass_patterns"] = [{"netclass": c, "pattern": n} for n, c in NETCLASS_PATTERNS.items()]
    with open(pro, "w") as f:
        json.dump(project, f, indent=2)


# --- keyboard-specific placement -------------------------------------------

# GH60-compatible 60% board: outline, M2 mounting holes and USB-C position follow ai03's
# open-source hotswap JP60 (github.com/ai03-2725/JP60, MIT), cross-checked against the
# original GH60 (github.com/komar007/gh60). Coordinates are relative to the key grid of
# layout.py (top-left corner of Esc at 0, 0); the outline is inset 0.375 / 0.325 mm from the
# 15u x 5u key area, with a small tab for the USB-C receptacle and the GH60 side slots.
OUTLINE = [
    ("line", (13.0750, -0.1750), (24.0750, -0.1750)),        # USB-C tab
    ("arc", (24.0750, -0.1750), (24.2214, 0.1786), (24.5750, 0.3250)),
    ("line", (24.5750, 0.3250), (283.3750, 0.3250)),
    ("arc", (283.3750, 0.3250), (284.7892, 0.9108), (285.3750, 2.3250)),
    ("line", (285.3750, 2.3250), (285.3750, 55.4250)),
    ("line", (285.3750, 55.4250), (281.5750, 55.4250)),       # right side slot
    ("arc", (281.5750, 55.4250), (280.5851, 55.8351), (280.1750, 56.8250)),
    ("arc", (280.1750, 56.8250), (280.5851, 57.8149), (281.5750, 58.2250)),
    ("line", (281.5750, 58.2250), (285.3750, 58.2250)),
    ("line", (285.3750, 58.2250), (285.3750, 92.9250)),
    ("arc", (285.3750, 92.9250), (284.7892, 94.3392), (283.3750, 94.9250)),
    ("line", (283.3750, 94.9250), (2.3750, 94.9250)),
    ("arc", (2.3750, 94.9250), (0.9608, 94.3392), (0.3750, 92.9250)),
    ("line", (0.3750, 92.9250), (0.3750, 58.2250)),
    ("line", (0.3750, 58.2250), (4.1750, 58.2250)),           # left side slot
    ("arc", (4.1750, 58.2250), (5.1649, 57.8149), (5.5750, 56.8250)),
    ("arc", (5.5750, 56.8250), (5.1649, 55.8351), (4.1750, 55.4250)),
    ("line", (4.1750, 55.4250), (0.3750, 55.4250)),
    ("line", (0.3750, 55.4250), (0.3750, 2.3250)),
    ("arc", (0.3750, 2.3250), (0.9608, 0.9108), (2.3750, 0.3250)),
    ("line", (2.3750, 0.3250), (12.5750, 0.3250)),
    ("arc", (12.5750, 0.3250), (12.9286, 0.1786), (13.0750, -0.1750)),
]
# GH60 standard holes (JP60 leaves the one next to the spacebar undrilled; it's drilled here)
MOUNT_HOLES = [(25.575, 28.225), (260.425, 28.225), (128.575, 47.325), (190.875, 85.525)]
SIDE_SLOTS = [(4.175, 56.825), (281.575, 56.825)]
STANDOFF_R = 2.5        # case standoff radius: bottom-side parts must stay this far from a hole
# Four semicircular PCB support ledges on the top and bottom inner case walls (bottom side only)
CASE_TABS = [(64.5, 2.8), (220.0, 2.8), (43.5, 93.8), (242.5, 93.8)]
TAB_R = 3.2
# Same position as JP60 (same receptacle): the mouth sticks out 1.1 mm past the USB tab
# and the front shield pins stay 0.7 mm from the edge.
USB_POS = (18.574, 2.373)  # HRO TYPE-C-31-M-12, between Esc and 1
# The receptacle sits where Esc's and 1's hotswap sockets would be, so those two switches
# are turned 180 degrees (sockets in the lower half of the key). A keycap doesn't care.
USB_KEYS = ("Esc", "1")
DIODE_OFFSET = (0.0, 5.5)                  # diode position relative to switch centre (mm)
MCU_POS = (104.0, 81.9)                    # under the spacebar, bottom side
SWITCH = "hotswap"                         # or "solder"; set from the command line


def key_center(k):
    return k.center_mm


def socket_rects(cx, cy, rotated):
    """Hotswap socket body and pads (x0, y0, x1, y1) on the bottom, in board coordinates."""
    rects = [SOCKET_OUTLINE] + [(x - SOCKET_PAD_SIZE[0] / 2, y - SOCKET_PAD_SIZE[1] / 2,
                                 x + SOCKET_PAD_SIZE[0] / 2, y + SOCKET_PAD_SIZE[1] / 2)
                                for x, y in SOCKET_PADS.values()]
    if rotated:
        rects = [(-x1, -y1, -x0, -y0) for x0, y0, x1, y1 in rects]
    return [(cx + x0, cy + y0, cx + x1, cy + y1) for x0, y0, x1, y1 in rects]


def rect_distance(x, y, r):
    x0, y0, x1, y1 = r
    return math.hypot(max(x0 - x, 0, x - x1), max(y0 - y, 0, y - y1))


def rotated_keys():
    """Keys whose switch is turned 180 degrees: the two under the USB-C receptacle, plus any
    key whose socket would sit on a case standoff or wall support tab."""
    rot = {k.label for k in KEYS if k.label in USB_KEYS}
    supports = [(hx, hy, STANDOFF_R) for hx, hy in MOUNT_HOLES + SIDE_SLOTS] + [
        (tx, ty, TAB_R) for tx, ty in CASE_TABS
    ]
    for hx, hy, rad in supports:
        for k in KEYS:
            cx, cy = key_center(k)
            if min(rect_distance(hx, hy, r) for r in socket_rects(cx, cy, k.label in rot)) >= rad:
                continue
            if k.label in rot or k.w != 1 or \
               min(rect_distance(hx, hy, r) for r in socket_rects(cx, cy, True)) < rad:
                sys.exit(f"mounting hole ({hx}, {hy}) collides with the socket of {k.label!r}")
            rot.add(k.label)
    return rot


def switch_center_offset(fp):
    """Offset from footprint origin to the switch centre (the 4 mm hole nearest pin 1)."""
    p1 = next(p for p in fp.Pads() if p.GetNumber() == "1").GetPosition()
    holes = [p for p in fp.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH
             and abs(pcbnew.ToMM(p.GetDrillSize().x) - 4.0) < 0.05]
    c = min(holes, key=lambda h: (h.GetPosition() - p1).EuclideanNorm()).GetPosition()
    return pcbnew.ToMM(c.x - p1.x), pcbnew.ToMM(c.y - p1.y)


def place_matrix(bb):
    """Switches on top, one SOD-123 diode per switch on the bottom (COL2ROW)."""
    rot = rotated_keys() if SWITCH == "hotswap" else set()
    print("switches turned 180 degrees:", sorted(rot))
    for i, k in enumerate(KEYS, start=1):
        cx, cy = key_center(k)
        col_net, row_net, mid_net = f"COL{k.col}", f"ROW{k.row}", f"SD{i}"
        pads = {"1": col_net, "2": mid_net}
        # Values end up unquoted in the Specctra DSN, where labels like ' ; ( \ break the
        # Freerouting parser -- so use a sanitised keycode name (e.g. KEY_SLSH, KEY_MO_1).
        value = "KEY_" + re.sub(r"[^A-Za-z0-9]+", "_", k.keycode.removeprefix("KC_")).strip("_")
        if SWITCH == "hotswap":
            # generated by gen_footprints.py; origin at switch centre, flipped to the bottom
            name = f"SW_MX_Hotswap_{k.w:.2f}u" + ("_FlippedStab" if k.w in FLIP_STAB_WIDTHS else "")
            fp = bb.footprint(name, f"SW{i}", value, cx, cy, rot=180 if k.label in rot else 0,
                              back=True, pads=pads)
            fp.Reference().SetVisible(False)  # would collide with the diode label
        else:
            fp = bb.footprint(f"SW_Cherry_MX_{k.w:.2f}u_PCB", f"SW{i}", value, 0, 0, pads=pads)
            dx, dy = switch_center_offset(fp)
            fp.SetPosition(pt(cx - dx, cy - dy))
        # Diode in the half of the key the socket leaves free. On row 0 (under the USB-C
        # receptacle or top-wall support tabs) that upper half is taken too, so those go on
        # the row gap below.
        dx, dy = DIODE_OFFSET
        if k.row == 0 and k.label in rot:
            dx, dy = 0.0, 9.525
        elif k.label in rot:
            dx, dy = -dx, -dy
        # COL2ROW: column -> switch -> diode anode (pad 2) -> cathode (pad 1) -> row
        dfp = bb.footprint("D_SOD-123", f"D{i}", "1N4148W", cx + dx, cy + dy,
                           back=True, pads={"1": row_net, "2": mid_net})
        if k.row == 0 and k.label in rot:
            dfp.Reference().SetPosition(pt(cx + dx + 3.9, cy + dy))


def place_usb(bb):
    """USB-C receptacle on the top edge (bottom side); fuse, ESD and CC resistors beside it,
    in the upper half of Esc and 1, which their turned-around sockets leave free."""
    x, y = USB_POS
    j1 = bb.footprint("USB_C_Receptacle_HRO_TYPE-C-31-M-12", "J1", "TYPE-C-31-M-12", x, y, rot=180, back=True,
                 pads={"A1": "GND", "A12": "GND", "B1": "GND", "B12": "GND", "S1": "GND",
                       "A4": "VBUS", "A9": "VBUS", "B4": "VBUS", "B9": "VBUS",
                       "A5": "CC1", "B5": "CC2",
                       "A6": "USB_DP", "B6": "USB_DP", "A7": "USB_DN", "B7": "USB_DN"})
    for pad in j1.Pads():  # shell pins: solid to the GND pour (thermal spokes get starved)
        if pad.GetNumber() == "S1":
            pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
    bb.footprint("Fuse_1206_3216Metric", "F1", "500mA PTC", x - 9.0, 4.0, back=True,
                 pads={"1": "VBUS", "2": "+5V"})
    # USBLC6-2SC6: 1/6 = I/O1, 3/4 = I/O2, 2 = GND, 5 = VBUS. The two channels are identical;
    # D+ on I/O1 lets the pre-routed pair come in from the right without crossing.
    bb.footprint("SOT-23-6", "U2", "USBLC6-2SC6", x + 9.0, 4.0, back=True,
                 pads={"1": "USB_DP", "6": "USB_DP", "3": "USB_DN", "4": "USB_DN", "2": "GND", "5": "VBUS"})
    bb.footprint("R_0603_1608Metric", "R5", "5.1k", x - 13.5, 4.0, rot=90, back=True, pads={"1": "CC1", "2": "GND"})
    # 16.5 mm out: the pre-routed D+/D- pair comes into U2 from the right
    bb.footprint("R_0603_1608Metric", "R6", "5.1k", x + 16.5, 4.0, rot=90, back=True, pads={"1": "CC2", "2": "GND"})


def place_mcu(bb):
    """ATmega32U4-MU and support parts on the bottom side under the spacebar.

    The footprint is mirrored (bottom side), so viewed from the top pins 1-11 are on
    the right, 12-22 at the bottom, 23-33 on the left and 34-44 on top.
    """
    mx, my = MCU_POS
    u1 = bb.footprint("QFN-44-1EP_7x7mm_P0.5mm_EP5.2x5.2mm", "U1", "ATmega32U4-MU", mx, my, back=True,
                      pads={str(p): n for p, n in MCU_FIXED.items()})

    def part(fp, ref, value, dx, dy, rot, pads, ref_offset=None):
        f = bb.footprint(fp, ref, value, mx + dx, my + dy, rot=rot, back=True, pads=pads)
        if ref_offset is not None:
            f.Reference().SetPosition(pt(mx + dx + ref_offset[0], my + dy + ref_offset[1]))
        return f

    R, C = "R_0603_1608Metric", "C_0603_1608Metric"
    # bottom: crystal, VCC decoupling, reset pull-up. Y1 pin 3 (top-left) = XTAL1 (left),
    # pin 1 (bottom-right) = XTAL2 (right), so XTAL1/XTAL2 fan out without crossing.
    part("Crystal_SMD_3225-4Pin_3.2x2.5mm", "Y1", "16MHz", -1.6, 7.8, 0,
         {"3": "XTAL1", "1": "XTAL2", "2": "GND", "4": "GND"})
    part(C, "C1", "18pF", -5.4, 7.8, -90, {"1": "XTAL1", "2": "GND"}, ref_offset=(-1.7, 0.0))
    part(C, "C2", "18pF", -1.6, 11.4, 0, {"1": "XTAL2", "2": "GND"}, ref_offset=(-2.4, 0.0))
    part(C, "C3", "100nF", 2.2, 6.8, 90, {"1": "+5V", "2": "GND"}, ref_offset=(0.0, 2.1))
    part("C_0805_2012Metric", "C7", "10uF", 4.6, 7.6, 90, {"1": "+5V", "2": "GND"}, ref_offset=(0.0, 2.9))
    part(R, "R1", "10k", 7.0, 6.8, 90, {"1": "+5V", "2": "RESET"}, ref_offset=(0.0, 2.1))
    part("SW_SPST_SKQG_WithoutStem", "SW0", "RESET", 13.5, 7.8, 0, {"1": "RESET", "2": "GND"})
    # right (pins 1-11): UVCC decoupling, UCAP, USB series resistors
    part(C, "C9", "100nF", 6.8, -4.0, 0, {"1": "+5V", "2": "GND"}, ref_offset=(0.8, -1.4))
    # below-right corner, so pins 5-11 keep a clear escape path to the right
    part(C, "C8", "1uF", 5.4, 4.2, 90, {"1": "UCAP", "2": "GND"})
    # rotated 180 so pad 1 (MCU side) faces U1 once mirrored onto the bottom
    part(R, "R3", "22R", 10.3, -1.9, 180, {"1": "U_DN", "2": "USB_DN"}, ref_offset=(0.0, -1.4))
    part(R, "R4", "22R", 10.3, -0.1, 180, {"1": "U_DP", "2": "USB_DP"}, ref_offset=(0.0, 1.4))
    # top (pins 34-44): VCC/AVCC decoupling
    # out at the corners: pins 36-41 (matrix) need the whole top side to fan out
    part(C, "C4", "100nF", -5.0, -6.2, 0, {"1": "+5V", "2": "GND"})
    part(C, "C6", "100nF", 5.0, -6.2, 0, {"1": "+5V", "2": "GND"})
    # left (pins 23-33): AVCC decoupling, HWB pull-down (reset -> DFU bootloader)
    part(C, "C5", "100nF", -6.6, 3.2, 0, {"1": "+5V", "2": "GND"})
    part(R, "R2", "10k", -6.6, -3.6, 0, {"1": "HWB", "2": "GND"})
    ground_mcu(bb, u1)
    # Pre-route XTAL1 (pin 17 -> Y1.3 -> C1.1) and XTAL2 (pin 16 -> Y1.1 -> C2.1) cleanly on B.Cu
    for net, pts in (("XTAL1", [(104.0, 85.2375), (104.0, 86.15), (101.3, 88.85), (98.675, 88.85), (98.6, 88.925)]),
                     ("XTAL2", [(104.5, 85.2375), (104.5, 89.55), (103.5, 90.55),
                                (103.5, 92.975), (103.175, 93.3)])):
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            bb.track(x0, y0, x1, y1, pcbnew.B_Cu, 0.2, net, locked=True)
    # UCAP (pin 6, middle of the right side) to C8 would cut across the fan-out of
    # pins 7-11 on the bottom layer: hop over F.Cu between U1.6 and C8.1.
    px, py = to_mm(next(p for p in u1.Pads() if p.GetNumber() == "6").GetPosition())
    vx = px + 1.55
    bb.track(px, py, vx, py, pcbnew.B_Cu, 0.2, "UCAP", locked=True)
    bb.via(vx, py, "UCAP", locked=True)
    bb.track(vx, py, 108.5, py + (vx - 108.5), pcbnew.F_Cu, 0.2, "UCAP", locked=True)
    bb.track(108.5, py + (vx - 108.5), 108.5, 86.875, pcbnew.F_Cu, 0.2, "UCAP", locked=True)
    bb.via(108.5, 86.875, "UCAP", locked=True)
    bb.track(108.5, 86.875, 109.4, 86.875, pcbnew.B_Cu, 0.2, "UCAP", locked=True)
    # D-/D+ (pins 3/4) to their 22R series resistors as a parallel pair, pre-routed:
    # the autorouter otherwise tangles them with UVCC/UCAP in this corner.
    for pin, ref in (("3", "R3"), ("4", "R4")):
        px, py = to_mm(next(p for p in u1.Pads() if p.GetNumber() == pin).GetPosition())
        r = bb.board.FindFootprintByReference(ref)
        rx, ry = to_mm(r.FindPadByNumber("1").GetPosition())
        net = u1.FindPadByNumber(pin).GetNetname()
        bb.track(px, py, rx - 0.6, py, pcbnew.B_Cu, 0.25, net, locked=True)
        bb.track(rx - 0.6, py, rx, ry, pcbnew.B_Cu, 0.25, net, locked=True)
    return u1


def ground_mcu(bb, u1):
    """Tie every QFN GND pin inward to the exposed pad (connected at its origin so
    Specctra/Freerouting sees the net as internally tied) and drop vias in the pad to
    the pours. Ground then needs no fan-out, which leaves room for the signal pins."""
    cx, cy = to_mm(u1.GetPosition())
    for pad in u1.Pads():
        if pad.GetNetname() != "GND" or pad.GetNumber() == "45":
            continue
        px, py = to_mm(pad.GetPosition())
        # straight in towards the centre, then to pad 45 origin inside the 5.2 mm square
        if abs(px - cx) > abs(py - cy):
            ex, ey = cx + math.copysign(2.0, px - cx), py
        else:
            ex, ey = px, cy + math.copysign(2.0, py - cy)
        bb.track(px, py, ex, ey, pcbnew.B_Cu, 0.25, "GND", locked=True)
        bb.track(ex, ey, cx, cy, pcbnew.B_Cu, 0.25, "GND", locked=True)
    for dx in (-1.3, 1.3):
        for dy in (-1.3, 1.3):
            bb.via(cx + dx, cy + dy, "GND", locked=True)
            bb.track(cx + dx, cy + dy, cx, cy, pcbnew.B_Cu, 0.25, "GND", locked=True)


def ground_board(bb):
    """Tie every non-MCU GND pad to a local locked GND via (or J1 TH shield pad), each in
    its own temporary GND_k net during autorouting so Freerouting does not daisy-chain
    GND across the board before the copper pours are added."""
    board = bb.board

    def set_pads(net, specs):
        for ref, num, xy in specs:
            fp = board.FindFootprintByReference(ref)
            for p in fp.Pads():
                if p.GetNumber() == num and (xy is None or math.dist(to_mm(p.GetPosition()), xy) < 0.2):
                    p.SetNet(bb.net(net))

    groups = [
        # J1 left shield + A1/B12 (top TH shield pad in its own group; both S1 pads hit F.Cu & B.Cu pours)
        ([("J1", "A1", None), ("J1", "B12", None), ("J1", "S1", (14.254, 5.503))],
         [((15.324, 6.418), (14.254, 5.503))], None),
        ([("J1", "S1", (14.254, 1.323))], [], None),
        # J1 right shield + A12/B1
        ([("J1", "A12", None), ("J1", "B1", None), ("J1", "S1", (22.894, 5.503))],
         [((21.824, 6.418), (22.894, 5.503))], None),
        ([("J1", "S1", (22.894, 1.323))], [], None),
        ([("R5", "2", None)], [((5.074, 3.175), (5.074, 2.0))], (5.074, 2.0)),
        ([("R6", "2", None)], [((35.074, 3.175), (35.074, 2.0))], (35.074, 2.0)),
        ([("U2", "2", None)], [((28.7115, 4.0), (29.9, 4.0))], (29.9, 4.0)),
        # Y1 + C1 + C2 ground cluster
        ([("Y1", "2", None), ("Y1", "4", None), ("C1", "2", None), ("C2", "2", None)],
         [((103.5, 88.85), (101.3, 90.55)), ((101.3, 90.55), (98.675, 90.55)), ((98.675, 90.55), (98.6, 90.475)),
          ((101.3, 90.55), (101.625, 90.875)), ((101.625, 90.875), (101.625, 93.3)),
          ((101.3, 90.55), (100.0, 91.85))], (100.0, 91.85)),
        # C3 + C7 ground cluster
        ([("C3", "2", None), ("C7", "2", None)],
         [((106.2, 87.925), (107.4, 87.925)), ((107.4, 87.925), (108.025, 88.55)), ((108.025, 88.55), (108.6, 88.55))],
         (107.4, 87.925)),
        # SW0 both pad 2s
        ([("SW0", "2", None)],
         [((114.4, 91.55), (117.5, 91.55)), ((117.5, 91.55), (120.6, 91.55))], (117.5, 91.55)),
        ([("C8", "2", None)], [((109.4, 85.325), (110.6, 85.325))], (110.6, 85.325)),
        ([("C9", "2", None)], [((110.025, 77.9), (110.025, 79.1))], (110.025, 79.1)),
        ([("C6", "2", None)], [((108.225, 75.7), (108.225, 74.5))], (108.225, 74.5)),
        ([("C4", "2", None)], [((98.225, 75.7), (98.225, 74.5))], (98.225, 74.5)),
        ([("R2", "2", None)], [((96.575, 78.3), (95.3, 78.3))], (95.3, 78.3)),
        ([("C5", "2", None)], [((96.625, 85.1), (95.3, 85.1))], (95.3, 85.1)),
    ]
    for idx, (pads, segs, via_xy) in enumerate(groups, start=1):
        net = f"GND_{idx}"
        set_pads(net, pads)
        for (x0, y0), (x1, y1) in segs:
            bb.track(x0, y0, x1, y1, pcbnew.B_Cu, 0.25, net, locked=True)
        if via_xy is not None:
            bb.via(*via_xy, net, locked=True)


def merge_gnd_nets(board):
    """Merge temporary GND_* nets back into GND before pouring the ground planes."""
    gnd = board.FindNet("GND")
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname().startswith("GND_"):
                p.SetNet(gnd)
    for t in board.GetTracks():
        if t.GetNetname().startswith("GND_"):
            t.SetNet(gnd)


def assign_matrix_pins(bb, u1):
    """Map ROW*/COL* nets onto free GPIOs so the angular order around the MCU matches
    the direction each net leaves in -- this keeps the fan-out free of crossings."""
    mx, my = to_mm(u1.GetPosition())
    pads = {p.GetNumber(): p for p in u1.Pads()}

    def angle(x, y):
        return math.atan2(y - my, x - mx) % (2 * math.pi)

    pins = sorted(MCU_GPIO, key=lambda n: angle(*to_mm(pads[str(n)].GetPosition())))
    pin_ang = [angle(*to_mm(pads[str(n)].GetPosition())) for n in pins]

    net_pts = {}
    for fp in bb.board.GetFootprints():
        for p in fp.Pads():
            name = p.GetNetname()
            if name.startswith(("ROW", "COL")):
                net_pts.setdefault(name, []).append(to_mm(p.GetPosition()))
    # Columns are ordered by direction. Rows all run the same way, so they are kept as one
    # consecutive block at their mean direction: a row pin between two column pins gets
    # fenced in by the column fan-out and can't be routed.
    cols = sorted((n for n in net_pts if n.startswith("COL")), key=lambda n: angle(*_centroid(net_pts[n])))
    rows = [f"ROW{r}" for r in range(N_ROWS)]
    row_pts = [p for r in rows for p in net_pts[r]]
    row_ang = angle(*_centroid(row_pts))
    i = sum(1 for c in cols if angle(*_centroid(net_pts[c])) < row_ang)
    nets = cols[:i] + rows + cols[i:]
    net_ang = [row_ang if n.startswith("ROW") else angle(*_centroid(net_pts[n])) for n in nets]

    def diff(a, b):
        d = abs(a - b) % (2 * math.pi)
        return min(d, 2 * math.pi - d)

    # order-preserving matching with skips, tried for every cyclic rotation of the pins
    best = None
    P, N = len(pins), len(nets)
    for rot in range(P):
        rp = pins[rot:] + pins[:rot]
        ra = pin_ang[rot:] + pin_ang[:rot]
        INF = float("inf")
        dp = [[INF] * (P + 1) for _ in range(N + 1)]
        dp[0] = [0.0] * (P + 1)
        for i in range(1, N + 1):
            for j in range(i, P + 1):
                dp[i][j] = min(dp[i][j - 1], dp[i - 1][j - 1] + diff(net_ang[i - 1], ra[j - 1]))
        if best is None or dp[N][P] < best[0]:
            # backtrack
            match, i, j = {}, N, P
            while i > 0:
                if dp[i][j] == dp[i][j - 1]:
                    j -= 1
                else:
                    match[nets[i - 1]] = rp[j - 1]
                    i, j = i - 1, j - 1
            best = (dp[N][P], match)

    mapping = best[1]
    for net, pin in mapping.items():
        pads[str(pin)].SetNet(bb.net(net))
    return {net: MCU_GPIO[pin] for net, pin in mapping.items()}


def _centroid(points):
    return sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points)


def place_mounting_holes(bb):
    """M2 holes at the GH60 positions (non-plated: hotswap sockets come close to some),
    plus copper keep-outs at the two GH60 side mounting slots and four wall support tabs."""
    for i, (x, y) in enumerate(MOUNT_HOLES, start=1):
        bb.footprint("MountingHole_2.2mm_M2", f"H{i}", "M2", x, y)
    for x, y in SIDE_SLOTS:
        bb.circle_keepout(x, y, 2.2, pcbnew.F_Cu)
        bb.circle_keepout(x, y, STANDOFF_R, pcbnew.B_Cu)
    for x, y in CASE_TABS:
        bb.circle_keepout(x, y, TAB_R, pcbnew.B_Cu)
    return MOUNT_HOLES


USB_PAIR_PITCH = 0.4   # D+/D- centre to centre: 0.25 mm tracks with a 0.15 mm gap
PAIR_GRID = 0.25       # search grid for the pair's centre line (mm)


class _CopperMap:
    """Copper and holes of the board as simple shapes, for clearance tests and for
    rasterising a search grid. Shapes: ("circle", x, y, r), ("rect", x0, y0, x1, y1),
    ("seg", x0, y0, x1, y1, half_width); each with the layers it blocks and its clearance."""

    def __init__(self, board, edge_margin):
        self.items = []  # (shape, on_front, on_back, clearance, net)
        for x, y in MOUNT_HOLES + SIDE_SLOTS:
            self.items.append((("circle", x, y, STANDOFF_R), True, True, 0.2, ""))
        for x, y in CASE_TABS:
            self.items.append((("circle", x, y, TAB_R), False, True, 0.2, ""))
        for fp in board.GetFootprints():
            for p in fp.Pads():
                x, y = to_mm(p.GetPosition())
                if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                    drill = pcbnew.ToMM(p.GetDrillSize().x)
                    self.items.append((("circle", x, y, drill / 2), True, True, 0.3, ""))
                    if abs(drill - 3.05) < 0.05:  # stabilizer M2 screw head keep-out on B.Cu
                        self.items.append((("circle", x, y, STANDOFF_R), False, True, 0.2, ""))
                    continue
                front, back = p.IsOnLayer(pcbnew.F_Cu), p.IsOnLayer(pcbnew.B_Cu)
                if p.GetShape() == pcbnew.PAD_SHAPE_CIRCLE:
                    shape = ("circle", x, y, pcbnew.ToMM(p.GetSize().x) / 2)
                else:
                    bb = p.GetBoundingBox()
                    (x0, y0), w, h = to_mm(bb.GetOrigin()), pcbnew.ToMM(bb.GetWidth()), pcbnew.ToMM(bb.GetHeight())
                    shape = ("rect", x0, y0, x0 + w, y0 + h)
                self.items.append((shape, front, back, 0.2, p.GetNetname()))
        for t in board.GetTracks():
            if t.GetClass() == "PCB_VIA":
                x, y = to_mm(t.GetPosition())
                self.items.append((("circle", x, y, pcbnew.ToMM(t.GetWidth(pcbnew.F_Cu)) / 2), True, True, 0.2,
                                   t.GetNetname()))
            else:
                (x0, y0), (x1, y1) = to_mm(t.GetStart()), to_mm(t.GetEnd())
                on_f = t.GetLayer() == pcbnew.F_Cu
                self.items.append((("seg", x0, y0, x1, y1, pcbnew.ToMM(t.GetWidth()) / 2), on_f, not on_f, 0.2,
                                   t.GetNetname()))
        self.edges = outline_chords(OUTLINE)
        self.edge_margin = edge_margin

    @staticmethod
    def distance(shape, x, y):
        kind = shape[0]
        if kind == "circle":
            return math.hypot(x - shape[1], y - shape[2]) - shape[3]
        if kind == "rect":
            return rect_distance(x, y, shape[1:])
        _, x0, y0, x1, y1, hw = shape
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        t = 0 if L2 == 0 else max(0, min(1, ((x - x0) * dx + (y - y0) * dy) / L2))
        return math.hypot(x - x0 - t * dx, y - y0 - t * dy) - hw

    def clear(self, x, y, r, front=True, back=True, net=None):
        """True if a disc of radius r at (x, y) keeps clearance on the given layers
        (copper of `net` itself doesn't count)."""
        if any(self.distance(("seg", *a, *b, 0), x, y) < r + self.edge_margin for a, b in self.edges):
            return False
        return all(self.distance(s, x, y) >= r + c for s, f, b, c, n in self.items
                   if ((front and f) or (back and b)) and not (net and n == net))

    def segment_clear(self, a, b, r, front=True, back=True, net=None):
        n = max(1, int(math.dist(a, b) / 0.1))
        return all(self.clear(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, r, front, back, net)
                   for k in range(n + 1))


class _SearchGrid:
    """The top layer rasterised for a grid search: a cell is blocked if a track centred on
    it, `half` wide to each side, would come too close to copper, a hole or the edge."""

    def __init__(self, cmap, half, origin, step=PAIR_GRID):
        bx0, by0, bx1, by1 = outline_bbox(OUTLINE)
        self.step = step
        self.x0 = origin[0] - math.floor((origin[0] - bx0) / step) * step
        self.y0 = origin[1] - math.floor((origin[1] - by0) / step) * step
        self.nx, self.ny = int((bx1 - self.x0) / step) + 1, int((by1 - self.y0) / step) + 1
        nx, ny = self.nx, self.ny
        self.blocked = blocked = bytearray(nx * ny)
        shapes = [(s, c) for s, f, _, c, _ in cmap.items if f] + \
                 [(("seg", *a, *b, 0), cmap.edge_margin) for a, b in cmap.edges]
        for shape, c in shapes:
            reach = half + c
            if shape[0] == "circle":
                sx0, sy0, sx1, sy1 = shape[1] - shape[3], shape[2] - shape[3], shape[1] + shape[3], shape[2] + shape[3]
            elif shape[0] == "rect":
                sx0, sy0, sx1, sy1 = shape[1:]
            else:
                sx0, sy0 = min(shape[1], shape[3]) - shape[5], min(shape[2], shape[4]) - shape[5]
                sx1, sy1 = max(shape[1], shape[3]) + shape[5], max(shape[2], shape[4]) + shape[5]
            for j in range(max(0, int((sy0 - reach - self.y0) / step)), min(ny, int((sy1 + reach - self.y0) / step) + 2)):
                for i in range(max(0, int((sx0 - reach - self.x0) / step)), min(nx, int((sx1 + reach - self.x0) / step) + 2)):
                    if not blocked[j * nx + i] and _CopperMap.distance(shape, *self.pt(i, j)) < reach:
                        blocked[j * nx + i] = 1
        # outside the outline (the notches and the USB tab) -- scanline fill
        for j in range(ny):
            y = self.y0 + j * step
            xs = sorted(a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
                        for a, b in cmap.edges if (a[1] <= y < b[1]) or (b[1] <= y < a[1]))
            for i in range(nx):
                if sum(1 for e in xs if e < self.x0 + i * step) % 2 == 0:
                    blocked[j * nx + i] = 1

    def pt(self, i, j):
        return (self.x0 + i * self.step, self.y0 + j * self.step)

    def cell(self, x, y):
        return round((x - self.x0) / self.step), round((y - self.y0) / self.step)

    def free(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny and not self.blocked[j * self.nx + i]

    def corners(self, cells):
        """Grid path -> corner points in mm."""
        pts = [self.pt(*cells[0])]
        for a, b, c in zip(cells, cells[1:], cells[2:]):
            if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
                pts.append(self.pt(*b))
        pts.append(self.pt(*cells[-1]))
        return pts


def _astar(blocked, nx, ny, start, goal, start_dir=None):
    """8-connected grid search with a turn penalty (keeps the pair to few, long segments)."""
    import heapq
    dirs = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
    step = [1.0, math.sqrt(2)] * 4
    gx, gy = goal

    def h(x, y):
        dx, dy = abs(x - gx), abs(y - gy)
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    starts = range(8) if start_dir is None else [start_dir]
    best = {(start, d): 0.0 for d in starts}
    prev = {}
    heap = [(h(*start), 0.0, start, d) for d in starts]
    while heap:
        _, g, (x, y), d = heapq.heappop(heap)
        if (x, y) == goal:
            path = [(x, y)]
            key = ((x, y), d)
            while key in prev:
                key = prev[key]
                path.append(key[0])
            return path[::-1]
        if g > best.get(((x, y), d), float("inf")):
            continue
        for nd, (dx, dy) in enumerate(dirs):
            turn = min((nd - d) % 8, (d - nd) % 8)
            if turn > 2:
                continue  # no turns sharper than 90 degrees
            x2, y2 = x + dx, y + dy
            if not (0 <= x2 < nx and 0 <= y2 < ny) or blocked[y2 * nx + x2]:
                continue
            g2 = g + step[nd] + 2.0 * turn
            k2 = ((x2, y2), nd)
            if g2 < best.get(k2, float("inf")):
                best[k2] = g2
                prev[k2] = ((x, y), d)
                heapq.heappush(heap, (g2 + h(x2, y2), g2, (x2, y2), nd))
    return None


def _offset(points, o):
    """Polyline offset by o to the left of travel (mitred joints)."""
    norms = []
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        L = math.hypot(x1 - x0, y1 - y0)
        norms.append(((y1 - y0) / L, -(x1 - x0) / L))
    out = [(points[0][0] + o * norms[0][0], points[0][1] + o * norms[0][1])]
    for i in range(1, len(points) - 1):
        (ax, ay), (bx, by) = norms[i - 1], norms[i]
        k = o / (1 + ax * bx + ay * by)
        out.append((points[i][0] + k * (ax + bx), points[i][1] + k * (ay + by)))
    out.append((points[-1][0] + o * norms[-1][0], points[-1][1] + o * norms[-1][1]))
    return out


def route_usb_pair(bb):
    """Pre-route the long D+/D- run between the 22R resistors (next to U1, under the
    spacebar) and the USB-C corner as a locked parallel pair on the top layer, which only
    has drill holes on it. Stubs and two vias at both ends (the resistors, and the ESD chip
    U2 next to the receptacle), a grid search for the centre line in between, and the two
    tracks offset from it. Freerouting then only has the few mm from U2 to J1 left."""
    board = bb.board
    width, clearance, via_d, via_drill = NETCLASSES["USB"]
    half = USB_PAIR_PITCH / 2 + width / 2 + 0.05     # centre line to outer track edge, + margin
    edge_margin = MIN_RULES["min_copper_edge_clearance"] + 0.35 + 0.05  # + edge keep-out strip
    cmap = _CopperMap(board, edge_margin)

    # resistor end: pad 2 (connector side) of R3 (D-) and R4 (D+), stubs to two vias above
    (dnx, dny), (dpx, dpy) = (to_mm(board.FindFootprintByReference(r).FindPadByNumber("2").GetPosition())
                              for r in ("R3", "R4"))
    if not dny < dpy:
        sys.exit("route_usb_pair: expected R3 above R4")
    vx, vy = round((max(dnx, dpx) + 1.2) / PAIR_GRID) * PAIR_GRID, round((dny - 2.0) / PAIR_GRID) * PAIR_GRID
    start_vias = {"USB_DN": (vx, vy), "USB_DP": (vx + 1.0, vy)}
    bb.track(dnx, dny, vx, dny, pcbnew.B_Cu, width, "USB_DN", locked=True)
    bb.track(vx, dny, vx, vy, pcbnew.B_Cu, width, "USB_DN", locked=True)
    bb.track(dpx, dpy, vx + 1.0, dpy, pcbnew.B_Cu, width, "USB_DP", locked=True)
    bb.track(vx + 1.0, dpy, vx + 1.0, vy, pcbnew.B_Cu, width, "USB_DP", locked=True)
    start = (vx + 0.5, vy - 1.0)

    grid = _SearchGrid(cmap, half, start)   # aligned so the start lies on a grid point
    nx, ny, blocked, gpt = grid.nx, grid.ny, grid.blocked, grid.pt

    # connector end: straight in from the right to two vias beside U2's I/O1 (D+, pin 1)
    # and I/O2 (D-, pin 3), then short bottom-layer stubs into the pins
    u2 = board.FindFootprintByReference("U2")
    p1, p3 = (to_mm(u2.FindPadByNumber(n).GetPosition()) for n in ("1", "3"))
    if abs(p1[0] - p3[0]) > 0.01 or not p1[1] < p3[1] or p1[0] < to_mm(u2.GetPosition())[0]:
        sys.exit("route_usb_pair: expected U2 pins 1 (top) and 3 (bottom) on its right side")
    end_vias = {"USB_DP": (p1[0] + 1.7, p1[1]), "USB_DN": (p3[0] + 1.7, p3[1])}
    for net, (px, py) in (("USB_DP", p1), ("USB_DN", p3)):
        bb.track(px, py, *end_vias[net], pcbnew.B_Cu, width, net, locked=True)
    ei, ej = grid.cell(p1[0] + 2.7, (p1[1] + p3[1]) / 2)
    lead_in = [(ei + 5 - k, ej) for k in range(6)]   # arriving heading -x
    if any(not grid.free(i, j) for i, j in lead_in):
        sys.exit("route_usb_pair: no room for the pair to come into U2 from the right")
    si, sj = grid.cell(*start)
    # 1 mm straight up first, so the fan-in from the two vias stays clear of the pair
    lead = [(si, sj - k) for k in range(5)]
    if any(not grid.free(i, j) for i, j in lead):
        sys.exit("route_usb_pair: no room above the resistor vias")
    cells = _astar(blocked, nx, ny, lead[-1], lead_in[0], 6)
    if cells is None:
        sys.exit("route_usb_pair: no path for the USB pair")
    pts = grid.corners(lead[:-1] + cells + lead_in[1:])
    length = 0.0
    for net, o in (("USB_DN", USB_PAIR_PITCH / 2), ("USB_DP", -USB_PAIR_PITCH / 2)):
        line = [start_vias[net]] + _offset(pts, o) + [end_vias[net]]
        for (ax, ay), (bx, by) in zip(line, line[1:]):
            bb.track(ax, ay, bx, by, pcbnew.F_Cu, width, net, locked=True)
            length += math.hypot(bx - ax, by - ay) / 2
        for v in (start_vias[net], end_vias[net]):
            bb.via(*v, net, size=via_d, drill=via_drill, locked=True)
    # J1 -> U2 flow-through on B.Cu (entering U2 on pins 6/4 and exiting on pins 1/3)
    b_paths = [
        ("USB_DP", [(18.324, 6.418), (18.324, 5.35), (19.324, 5.35), (19.324, 6.418),
                    (19.324, 7.65), (19.824, 8.15), (22.10, 8.15), (24.75, 5.50),
                    (24.75, 3.05), (26.4365, 3.05), (28.7115, 3.05)]),
        ("USB_DN", [(17.824, 6.418), (17.824, 7.50), (18.824, 7.50), (18.824, 6.418)]),
        ("USB_DN", [(18.824, 7.50), (18.824, 7.85), (19.574, 8.60), (22.55, 8.60),
                    (26.20, 4.95), (26.4365, 4.95), (28.7115, 4.95)]),
        # VBUS: F1.1 <-> J1.A4/B9 <-> J1.A9/B4 on B.Cu, and hop on F.Cu to U2.5
        ("VBUS", [(10.974, 4.0), (15.924, 4.0), (16.524, 4.6), (16.524, 5.4), (16.124, 5.8), (16.124, 6.418)]),
        ("VBUS", [(16.524, 4.6), (20.1, 4.6), (20.6, 5.1), (20.6, 5.4), (21.024, 5.824), (21.024, 6.418)]),
        ("VBUS", [(20.1, 4.6), (20.1, 3.5)]),
        ("VBUS", [(25.5, 4.0), (26.4365, 4.0)]),
    ]
    for net, path in b_paths:
        for (ax, ay), (bx, by) in zip(path, path[1:]):
            bb.track(ax, ay, bx, by, pcbnew.B_Cu, width, net, locked=True)
    for (ax, ay), (bx, by) in [((20.1, 3.5), (25.0, 3.5)), ((25.0, 3.5), (25.5, 4.0))]:
        bb.track(ax, ay, bx, by, pcbnew.F_Cu, width, "VBUS", locked=True)
    for vx_v, vy_v in ((20.1, 3.5), (25.5, 4.0)):
        bb.via(vx_v, vy_v, "VBUS", size=0.6, drill=0.3, locked=True)
    # CC2: J1.B5 (20.324, 6.418) -> R6.1 (35.074, 4.825) via F.Cu hop above SW2 center hole
    bb.track(20.324, 6.418, 20.324, 7.55, pcbnew.B_Cu, 0.2, "CC2", locked=True)
    bb.via(20.324, 7.55, "CC2", size=0.6, drill=0.3, locked=True)
    for (ax, ay), (bx, by) in [((20.324, 7.55), (20.924, 6.95)),
                               ((20.924, 6.95), (31.699, 6.95)),
                               ((31.699, 6.95), (33.824, 4.825))]:
        bb.track(ax, ay, bx, by, pcbnew.F_Cu, 0.2, "CC2", locked=True)
    bb.via(33.824, 4.825, "CC2", size=0.6, drill=0.3, locked=True)
    bb.track(33.824, 4.825, 35.074, 4.825, pcbnew.B_Cu, 0.2, "CC2", locked=True)
    print(f"USB pair pre-routed on F.Cu: {len(pts) - 1} segments, {length:.0f} mm")


FEED_WIDTH = 0.3


def _via_near(cmap, grid, x, y, net, via_d, stub_w, max_r=5.0):
    """Nearest grid point to (x, y) where a via of `net` fits, reachable from (x, y) by a
    straight bottom-layer stub."""
    ci, cj = grid.cell(x, y)
    n = int(max_r / grid.step)
    for i, j in sorted(((i, j) for i in range(ci - n, ci + n + 1) for j in range(cj - n, cj + n + 1)),
                       key=lambda ij: math.dist(grid.pt(*ij), (x, y))):
        v = grid.pt(i, j)
        if math.dist(v, (x, y)) < 0.9 or not grid.free(i, j):
            continue  # not in the pad itself
        if cmap.clear(*v, via_d / 2, net=net) and \
           cmap.segment_clear((x, y), v, stub_w / 2, front=False, net=net):
            return (i, j), v
    return None, None


def route_power_feed(bb):
    """Pre-route the +5V feed from the fuse (next to the USB-C receptacle) to U1's
    decoupling as one locked top-layer track: Freerouting doesn't find a way for a run this
    long through the switch matrix. Bottom-layer stubs and a via at each end."""
    board = bb.board
    _, clearance, via_d, via_drill = NETCLASSES["Power"]
    cmap = _CopperMap(board, MIN_RULES["min_copper_edge_clearance"] + 0.35 + 0.05)
    src = to_mm(next(p for p in board.FindFootprintByReference("F1").Pads() if p.GetNetname() == "+5V").GetPosition())
    # into U1's +5V decoupling at the capacitor nearest the fuse, staying off the top layer
    # over U1 (its fan-out needs vias there) and out of the J1-U2 corner (D+/D- go there)
    caps = [p for fp in board.GetFootprints() if re.fullmatch(r"C\d+", fp.GetReference())
            for p in fp.Pads() if p.GetNetname() == "+5V"]
    dst = min((to_mm(p.GetPosition()) for p in caps), key=lambda q: math.dist(q, src))
    ux, uy = to_mm(board.FindFootprintByReference("U1").GetPosition())
    jx, _ = to_mm(board.FindFootprintByReference("J1").GetPosition())
    u2x, _ = to_mm(board.FindFootprintByReference("U2").GetPosition())
    keepouts = [(ux - 5.5, uy - 5.5, ux + 5.5, uy + 5.5), (jx - 5.5, -1.0, u2x + 6.0, 9.0)]
    grid = _SearchGrid(cmap, FEED_WIDTH / 2 + 0.05, src)
    for kx0, ky0, kx1, ky1 in keepouts:
        for j in range(grid.ny):
            for i in range(grid.nx):
                x, y = grid.pt(i, j)
                if kx0 <= x <= kx1 and ky0 <= y <= ky1:
                    grid.blocked[j * grid.nx + i] = 1
    (si, sj), sv = _via_near(cmap, grid, *src, "+5V", via_d, FEED_WIDTH)
    (gi, gj), gv = _via_near(cmap, grid, *dst, "+5V", via_d, FEED_WIDTH)
    if sv is None or gv is None:
        sys.exit("route_power_feed: no room for a via next to F1 or the target capacitor")
    cells = _astar(grid.blocked, grid.nx, grid.ny, (si, sj), (gi, gj))
    if cells is None:
        sys.exit("route_power_feed: no path for the +5V feed")
    line = [src] + grid.corners(cells) + [dst]
    for (ax, ay), (bx, by) in zip(line, line[1:]):
        layer = pcbnew.B_Cu if (ax, ay) in (src, dst) or (bx, by) in (src, dst) else pcbnew.F_Cu
        bb.track(ax, ay, bx, by, layer, FEED_WIDTH, "+5V", locked=True)
    for v in (sv, gv):
        bb.via(*v, "+5V", size=via_d, drill=via_drill, locked=True)
    length = sum(math.dist(a, b) for a, b in zip(line, line[1:]))
    print(f"+5V feed pre-routed on F.Cu: {len(line) - 1} segments, {length:.0f} mm")


# --- autorouting -------------------------------------------------------------

def autoroute(board, passes, timeout):
    if not os.path.exists(FREEROUTING_JAR):
        sys.exit(f"Freerouting jar missing: {FREEROUTING_JAR} (see README)")
    dsn = os.path.join(BUILD, "custom-kbr.dsn")
    ses = os.path.join(BUILD, "custom-kbr.ses")
    if not pcbnew.ExportSpecctraDSN(board, dsn):
        sys.exit("DSN export failed")
    if os.path.exists(ses):
        os.remove(ses)
    cmd = freerouting_cmd() + ["-de", dsn, "-do", ses, "-mp", str(passes), "-da"]
    print("+", " ".join(cmd), flush=True)
    try:
        subprocess.run(cmd, check=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f"warning: Freerouting did not finish within {timeout}s (see /tmp/freerouting/freerouting.log)")
        return False
    if not os.path.exists(ses) or not pcbnew.ImportSpecctraSES(board, ses):
        sys.exit("SES import failed")
    return True


def freerouting_cmd():
    """Freerouting 1.9 opens a progress window even in batch mode, so it needs an X
    display: use FREEROUTING_DISPLAY if set, else $DISPLAY, else run it under xvfb-run."""
    cmd = ["java", "-jar", FREEROUTING_JAR]
    display = os.environ.get("FREEROUTING_DISPLAY")
    if display:
        os.environ["DISPLAY"] = display
    elif not os.environ.get("DISPLAY"):
        if not shutil.which("xvfb-run"):
            sys.exit("Freerouting needs a display: set DISPLAY or install xvfb (sudo apt install xvfb)")
        cmd = ["xvfb-run", "-a"] + cmd
    return cmd


def remove_dangling(board):
    """Delete sub-micron (<0.01 mm) track segments and segments with an end that touches
    nothing (Freerouting leaves a few short stubs behind). Repeats until stable."""
    removed = 0
    for t in [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]:
        if (t.GetEnd() - t.GetStart()).EuclideanNorm() < mm(0.01):
            board.Remove(t)
            removed += 1
    while True:
        tracks = [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]
        items = list(board.GetTracks()) + [p for fp in board.GetFootprints() for p in fp.Pads()]
        dead = []
        for t in tracks:
            for end in (t.GetStart(), t.GetEnd()):
                hit = False
                for o in items:
                    if o is t or o.GetNetCode() != t.GetNetCode() or not o.IsOnLayer(t.GetLayer()):
                        continue
                    if o.HitTest(end, 0):
                        hit = True
                        break
                if not hit:
                    dead.append(t)
                    break
        if not dead:
            return removed
        for t in dead:
            board.Remove(t)
        removed += len(dead)


def drc_report(board):
    """kicad-cli DRC (errors only) of the board as it is in memory. Used instead of the
    connectivity API, which is not usable from Python on boards loaded from a file."""
    path = os.path.join(BUILD, "_check.kicad_pcb")
    report = os.path.join(BUILD, "_check.json")
    # Write a copy with the file-format writer: SaveBoard() to another path would re-point
    # the board at a new project, which breaks the Python wrappers once it is deleted.
    pcbnew.PCB_IO_MGR.Save(pcbnew.PCB_IO_MGR.KICAD_SEXP, path, board)
    write_project_rules(os.path.join(BUILD, "_check.kicad_pro"))
    subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-error",
                    "-o", report, path], check=True, capture_output=True)
    with open(report) as f:
        result = json.load(f)
    for ext in (".kicad_pcb", ".kicad_pro", ".kicad_prl", ".json"):
        if os.path.exists(os.path.join(BUILD, "_check" + ext)):
            os.remove(os.path.join(BUILD, "_check" + ext))
    return result


def unconnected_count(board):
    return len(drc_report(board)["unconnected_items"])


def _anchors(item):
    """Points a new track can start from: pad/via centres, track end points."""
    if item.GetClass() == "PCB_TRACK":
        return [to_mm(item.GetStart()), to_mm(item.GetEnd())]
    return [to_mm(item.GetPosition())]


def close_gaps(board, max_drc_trials=12):
    """Close the last few open connections Freerouting leaves behind. For each unconnected
    pair DRC reports, try a straight track or an L-shape between the nearest anchor points,
    then a hop over the other layer (stub, via, run, via, stub) between via sites found by
    a local search around both sides. Candidates are pre-screened geometrically, and one is
    kept only if KiCad's DRC then has fewer unconnected items and no new errors."""
    report = drc_report(board)
    base = len(report["violations"])
    by_uuid = {t.m_Uuid.AsString(): t for t in board.GetTracks()}
    by_uuid.update({p.m_Uuid.AsString(): p for fp in board.GetFootprints() for p in fp.Pads()})
    for u in report["unconnected_items"]:
        items = [by_uuid.get(i["uuid"]) for i in u["items"]]
        if len(items) != 2 or not all(items):
            continue
        net = items[0].GetNet()
        name = net.GetNetname()
        width, clearance, via_d, via_drill = NETCLASSES[NETCLASS_PATTERNS.get(name, "Default")]
        open_now = len(report["unconnected_items"])
        geo = _Obstacles(board, net.GetNetCode(), items, clearance + 0.02)
        trials = 0
        done = False
        for cand in _gap_candidates(items, geo, width, via_d):
            if trials >= max_drc_trials:
                break
            trials += 1
            added = []
            for kind, *g in cand:
                if kind == "via":
                    v = pcbnew.PCB_VIA(board)
                    v.SetPosition(pt(*g[0]))
                    v.SetFrontWidth(mm(via_d))
                    v.SetDrill(mm(via_drill))
                    v.SetNet(net)
                    board.Add(v)
                    added.append(v)
                elif math.dist(g[0], g[1]) > 1e-6:
                    t = pcbnew.PCB_TRACK(board)
                    t.SetStart(pt(*g[0]))
                    t.SetEnd(pt(*g[1]))
                    t.SetWidth(mm(width))
                    t.SetLayer(g[2])
                    t.SetNet(net)
                    board.Add(t)
                    added.append(t)
            trial = drc_report(board)
            if len(trial["unconnected_items"]) < open_now and len(trial["violations"]) <= base:
                print(f"closed gap on {name} with {len(added)} item(s) ({trials} DRC trial(s))")
                report, done = trial, True
                break
            for item in added:
                board.Remove(item)
        if not done:
            print(f"could not close gap on {name} ({trials} DRC trial(s))")


class _Obstacles:
    """Other-net copper near a gap, for quick clearance pre-checks (DRC has the last word)."""

    def __init__(self, board, netcode, items, clearance):
        pts = [p for i in items for p in _anchors(i)]
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        reach = max(math.dist((cx, cy), p) for p in pts) + 6
        near = lambda x, y: math.dist((cx, cy), (x, y)) < reach + 10
        self.clearance = clearance
        self.items = {pcbnew.F_Cu: [], pcbnew.B_Cu: []}
        cands = list(board.GetTracks()) + [p for fp in board.GetFootprints() for p in fp.Pads()]
        for it in cands:
            if it.GetNetCode() == netcode and netcode > 0:
                continue
            if not near(*to_mm(it.GetPosition() if it.GetClass() != "PCB_TRACK" else it.GetStart())) and \
               not (it.GetClass() == "PCB_TRACK" and near(*to_mm(it.GetEnd()))):
                continue
            for layer in self.items:
                # NPTH holes have no copper layer but must be avoided on both
                if it.IsOnLayer(layer) or (it.GetClass() == "PAD" and it.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH):
                    self.items[layer].append(it)
        self.bounds = board.GetBoardEdgesBoundingBox()

    def point_clear(self, p, radius, layer):
        x, y = p
        bx0, by0 = to_mm(self.bounds.GetOrigin())
        if not (bx0 + 0.6 < x < bx0 + pcbnew.ToMM(self.bounds.GetWidth()) - 0.6 and
                by0 + 0.6 < y < by0 + pcbnew.ToMM(self.bounds.GetHeight()) - 0.6):
            return False
        if any(math.hypot(x - hx, y - hy) < STANDOFF_R + radius + self.clearance
               for hx, hy in MOUNT_HOLES + SIDE_SLOTS):
            return False
        if layer == pcbnew.B_Cu and any(math.hypot(x - tx, y - ty) < TAB_R + radius + self.clearance
                                        for tx, ty in CASE_TABS):
            return False
        v = pt(x, y)
        acc = mm(radius + self.clearance)
        return not any(it.HitTest(v, acc) for it in self.items[layer])

    def segment_clear(self, a, b, half_width, layer):
        n = max(1, int(math.dist(a, b) / 0.1))
        return all(self.point_clear((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n),
                                    half_width, layer) for k in range(n + 1))


def _gap_candidates(items, geo, width, via_d):
    """Candidate bridges, cheapest first, already screened against nearby copper."""
    a, b = min(((a, b) for a in _anchors(items[0]) for b in _anchors(items[1])),
               key=lambda ab: math.dist(*ab))
    layers = [l for l in (pcbnew.B_Cu, pcbnew.F_Cu) if all(i.IsOnLayer(l) for i in items)]
    hw = width / 2
    for layer in layers:
        for path in ([a, b], [a, (b[0], a[1]), b], [a, (a[0], b[1]), b]):
            if all(geo.segment_clear(p, q, hw, layer) for p, q in zip(path, path[1:])):
                yield [("track", p, q, layer) for p, q in zip(path, path[1:])]

    # hop over the other layer between via sites found around each side
    def sites(item, layer):
        base = list(_anchors(item))
        if item.GetClass() == "PCB_TRACK":
            (x0, y0), (x1, y1) = _anchors(item)
            n = max(1, int(math.dist((x0, y0), (x1, y1)) / 0.5))
            base += [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(1, n)]
        offsets = [(0.0, 0.0)] + [(r * math.cos(t * math.pi / 4), r * math.sin(t * math.pi / 4))
                                  for r in (0.5, 1.0) for t in range(8)]
        out = []
        for bx, by in base:
            for ox, oy in offsets:
                s = (bx + ox, by + oy)
                on_smd = item.GetClass() == "PAD" and item.GetAttribute() == pcbnew.PAD_ATTRIB_SMD
                if on_smd and math.dist(s, (bx, by)) < 0.9:
                    continue  # no via in an SMD pad
                if not (geo.point_clear(s, via_d / 2, pcbnew.F_Cu) and geo.point_clear(s, via_d / 2, pcbnew.B_Cu)):
                    continue
                if (ox, oy) != (0.0, 0.0) and not geo.segment_clear((bx, by), s, hw, layer):
                    continue
                out.append(((bx, by), s))
        return out

    for layer in layers:
        other = pcbnew.F_Cu if layer == pcbnew.B_Cu else pcbnew.B_Cu
        sa, sb = sites(items[0], layer), sites(items[1], layer)
        pairs = sorted(((x, y) for x in sa for y in sb if math.dist(x[1], y[1]) > via_d + 0.3),
                       key=lambda xy: math.dist(xy[0][1], xy[1][1]))
        for (pa, va), (pb, vb) in pairs:
            if geo.segment_clear(va, vb, hw, other):
                yield [("track", pa, va, layer), ("via", va), ("track", va, vb, other),
                       ("via", vb), ("track", vb, pb, layer)]


def route_until_done(board, passes, timeout, max_rounds=2):
    """Full autoroute, then: delete dangling stubs, close leftover gaps with DRC-checked
    tracks, and if anything is still open re-run Freerouting on the routed board (it keeps
    existing wiring and only routes what is missing)."""
    if not autoroute(board, passes, timeout):
        sys.exit("initial autoroute timed out")
    for i in range(max_rounds):
        stubs = remove_dangling(board)
        left = unconnected_count(board)
        print(f"after round {i + 1}: removed {stubs} dangling stubs, {left} unconnected", flush=True)
        if left:
            close_gaps(board)
            left = unconnected_count(board)
        if left == 0:
            print("all connections routed")
            return
        if i + 1 < max_rounds and not autoroute(board, 10, min(timeout, 300)):
            break
    print(f"warning: {unconnected_count(board)} connection(s) still unrouted -- finish by hand in KiCad")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--switch", choices=("hotswap", "solder"), default="hotswap")
    ap.add_argument("--no-route", action="store_true", help="placement only")
    ap.add_argument("--passes", type=int, default=40, help="max Freerouting passes")
    ap.add_argument("--route-timeout", type=int, default=3600, help="seconds before giving up on Freerouting")
    ap.add_argument("--resume", metavar="PCB", help="continue routing an existing (partly routed) board")
    args = ap.parse_args()
    os.makedirs(BUILD, exist_ok=True)
    global SWITCH
    SWITCH = args.switch

    if args.resume:
        board = pcbnew.LoadBoard(args.resume)
        # drop the GND pours (re-added at the end); Delete(), since Remove() on a zone
        # of a loaded board leaves the Python wrappers broken
        for z in [z for z in board.Zones() if not z.GetIsRuleArea()]:
            board.Delete(z)
        route_until_done(board, args.passes, args.route_timeout)
        merge_gnd_nets(board)
        save_board(board, PCB_PATH)
        add_gnd_pours(PCB_PATH)
        return

    bb = BoardBuilder()
    bb.net("GND")  # make sure the common nets exist before netclass assignment
    bb.outline(OUTLINE)
    bb.edge_keepout(OUTLINE)
    place_matrix(bb)
    place_usb(bb)
    u1 = place_mcu(bb)
    ground_board(bb)
    pinmap = assign_matrix_pins(bb, u1)
    holes = place_mounting_holes(bb)
    setup_rules(bb.board)

    route_usb_pair(bb)
    route_power_feed(bb)
    bb.text(f"custom-kbr 60% ({SWITCH})", 158, 91.5, pcbnew.B_SilkS, mirror=True)

    with open(os.path.join(BUILD, "pinmap.json"), "w") as f:
        json.dump({"rows": [pinmap[f"ROW{r}"] for r in range(N_ROWS)],
                   "cols": [pinmap[f"COL{c}"] for c in range(N_COLS)]}, f, indent=2)
    print("matrix pins:", {k: pinmap[k] for k in sorted(pinmap, key=lambda s: (s[:3], int(s[3:])))})
    print("mounting holes:", [(round(x, 2), round(y, 2)) for x, y in holes])

    if not args.no_route:
        snapshot = os.path.join(BUILD, "custom-kbr-unrouted.kicad_pcb")
        pcbnew.PCB_IO_MGR.Save(pcbnew.PCB_IO_MGR.KICAD_SEXP, snapshot, bb.board)
        write_project_rules(snapshot.replace(".kicad_pcb", ".kicad_pro"))
        route_until_done(bb.board, args.passes, args.route_timeout)

    merge_gnd_nets(bb.board)
    save_board(bb.board, PCB_PATH)
    add_gnd_pours(PCB_PATH)


def add_gnd_pours(path):
    # The zone filler needs a board with connectivity built; an in-memory board
    # segfaults, so reload the saved file before adding and filling the pours.
    board = pcbnew.LoadBoard(path)
    bb = BoardBuilder()
    bb.board = board
    bb.nets = {"GND": board.FindNet("GND")}
    z_f = bb.zone("GND", pcbnew.F_Cu, outline_bbox(OUTLINE))
    z_b = bb.zone("GND", pcbnew.B_Cu, outline_bbox(OUTLINE))
    board.BuildConnectivity()
    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())

    # Stitch any isolated GND pour islands to a connected GND pour island on the opposite layer
    gnd_code = bb.nets["GND"].GetNetCode()
    vias = [t.GetPosition() for t in board.GetTracks()
            if t.GetClass() == "PCB_VIA" and t.GetNetCode() == gnd_code]
    f_poly = z_f.GetFilledPolysList(pcbnew.F_Cu)
    b_poly = z_b.GetFilledPolysList(pcbnew.B_Cu)
    f_outs = [f_poly.Outline(i) for i in range(f_poly.OutlineCount())]
    b_outs = [b_poly.Outline(i) for i in range(b_poly.OutlineCount())]
    if f_outs and b_outs:
        f_main = max(range(len(f_outs)), key=lambda i: f_outs[i].BBox().GetWidth())
        b_main = max(range(len(b_outs)), key=lambda i: b_outs[i].BBox().GetWidth())
        f_conn, b_conn = {f_main}, {b_main}
        changed = True
        while changed:
            changed = False
            for v in vias:
                fi = next((i for i, o in enumerate(f_outs) if o.PointInside(v)), None)
                bi = next((i for i, o in enumerate(b_outs) if o.PointInside(v)), None)
                if fi is not None and bi is not None and ((fi in f_conn) != (bi in b_conn)):
                    f_conn.add(fi)
                    b_conn.add(bi)
                    changed = True
        cands = [it for it in list(board.GetTracks()) + [p for fp in board.GetFootprints() for p in fp.Pads()]
                 if it.GetNetCode() != gnd_code]
        stitched = False
        for fi, fo in enumerate(f_outs):
            if fi in f_conn:
                continue
            box = fo.BBox()
            found = False
            for xi in range(int(box.GetLeft() / 1e6) * 2, int(box.GetRight() / 1e6) * 2 + 1):
                for yi in range(int(box.GetTop() / 1e6) * 2, int(box.GetBottom() / 1e6) * 2 + 1):
                    x, y = xi * 0.5, yi * 0.5
                    p = pt(x, y)
                    if (fo.PointInside(p)
                            and any(b_outs[bi].PointInside(p) for bi in b_conn)
                            and not any(it.HitTest(p, mm(0.6)) for it in cands)):
                        bb.via(x, y, "GND", locked=True)
                        stitched = found = True
                        break
                if found:
                    break
        if stitched:
            board.BuildConnectivity()
            filler.Fill(board.Zones())

    save_board(board, path)
    print("wrote", path)


if __name__ == "__main__":
    main()
