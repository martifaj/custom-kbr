#!/usr/bin/env python3
"""Generate the custom-kbr 65% keyboard PCB with the KiCad (pcbnew) Python API.

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
from layout import (KEYS, N_ROWS, N_COLS, BOARD_W, BOARD_H, MCU_GPIO, MCU_FIXED)  # noqa: E402

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
NETCLASS_PATTERNS = {"GND": "Power", "+5V": "Power", "VBUS": "Power",
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

    def outline(self, w, h, r=1.0):
        """Rounded rectangle on Edge.Cuts."""
        def seg(x1, y1, x2, y2):
            s = pcbnew.PCB_SHAPE(self.board, pcbnew.SHAPE_T_SEGMENT)
            s.SetStart(pt(x1, y1))
            s.SetEnd(pt(x2, y2))
            s.SetLayer(pcbnew.Edge_Cuts)
            s.SetWidth(mm(0.1))
            self.board.Add(s)

        def arc(cx, cy, sx, sy, deg):
            a = pcbnew.PCB_SHAPE(self.board, pcbnew.SHAPE_T_ARC)
            a.SetCenter(pt(cx, cy))
            a.SetStart(pt(sx, sy))
            a.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(deg, pcbnew.DEGREES_T), True)
            a.SetLayer(pcbnew.Edge_Cuts)
            a.SetWidth(mm(0.1))
            self.board.Add(a)

        seg(r, 0, w - r, 0)
        seg(w, r, w, h - r)
        seg(w - r, h, r, h)
        seg(0, h - r, 0, r)
        arc(w - r, r, w - r, 0, 90)
        arc(w - r, h - r, w, h - r, 90)
        arc(r, h - r, r, h, 90)
        arc(r, r, 0, r, 90)

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

    def edge_keepout(self, w, h, width=0.35):
        """Track/via keep-out strips along the board edge. Freerouting ignores KiCad's
        copper-to-edge clearance, but it does respect keep-outs."""
        for x0, y0, x1, y1 in [(0, 0, w, width), (0, h - width, w, h),
                               (0, 0, width, h), (w - width, 0, w, h)]:
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
            for x, y in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]:
                poly.Append(mm(x), mm(y))
            self.board.Add(z)

    def zone(self, net, layer, w, h, inset=0.3):
        z = pcbnew.ZONE(self.board)
        z.SetLayer(layer)
        z.SetNet(self.net(net))
        poly = z.Outline()
        poly.NewOutline()
        for x, y in [(inset, inset), (w - inset, inset), (w - inset, h - inset), (inset, h - inset)]:
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

# Strip above the top row for the USB-C connector and its parts. Hotswap sockets cover
# the upper half of every key on the bottom side, so the connector can't sit between keys.
TOP_MARGIN = 7.5
# Strip below the bottom row: the spacebar stabilizer's 4 mm holes sit 8.24 mm below the
# switch centre and would otherwise break through the front edge by 0.7 mm.
BOTTOM_MARGIN = 1.0
PCB_H = BOARD_H + TOP_MARGIN + BOTTOM_MARGIN
DIODE_OFFSET = (0.0, 5.5)                  # diode position relative to switch centre (mm)
MCU_POS = (104.0, TOP_MARGIN + 82.5)       # under the spacebar, bottom side
USB_X = 7 * 19.05                          # between keys "6" and "7"
SWITCH = "hotswap"                         # or "solder"; set from the command line


def key_center(k):
    cx, cy = k.center_mm
    return cx, cy + TOP_MARGIN


def switch_center_offset(fp):
    """Offset from footprint origin to the switch centre (the 4 mm hole nearest pin 1)."""
    p1 = next(p for p in fp.Pads() if p.GetNumber() == "1").GetPosition()
    holes = [p for p in fp.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH
             and abs(pcbnew.ToMM(p.GetDrillSize().x) - 4.0) < 0.05]
    c = min(holes, key=lambda h: (h.GetPosition() - p1).EuclideanNorm()).GetPosition()
    return pcbnew.ToMM(c.x - p1.x), pcbnew.ToMM(c.y - p1.y)


def place_matrix(bb):
    """Switches on top, one SOD-123 diode per switch on the bottom (COL2ROW)."""
    for i, k in enumerate(KEYS, start=1):
        cx, cy = key_center(k)
        col_net, row_net, mid_net = f"COL{k.col}", f"ROW{k.row}", f"SD{i}"
        pads = {"1": col_net, "2": mid_net}
        # Values end up unquoted in the Specctra DSN, where labels like ' ; ( \ break the
        # Freerouting parser -- so use a sanitised keycode name (e.g. KEY_SLSH, KEY_MO_1).
        value = "KEY_" + re.sub(r"[^A-Za-z0-9]+", "_", k.keycode.removeprefix("KC_")).strip("_")
        if SWITCH == "hotswap":
            # generated by gen_footprints.py; origin at switch centre, flipped to the bottom
            fp = bb.footprint(f"SW_MX_Hotswap_{k.w:.2f}u", f"SW{i}", value, cx, cy, back=True, pads=pads)
            fp.Reference().SetVisible(False)  # would collide with the diode label
        else:
            fp = bb.footprint(f"SW_Cherry_MX_{k.w:.2f}u_PCB", f"SW{i}", value, 0, 0, pads=pads)
            dx, dy = switch_center_offset(fp)
            fp.SetPosition(pt(cx - dx, cy - dy))
        # COL2ROW: column -> switch -> diode anode (pad 2) -> cathode (pad 1) -> row
        bb.footprint("D_SOD-123", f"D{i}", "1N4148W", cx + DIODE_OFFSET[0], cy + DIODE_OFFSET[1],
                     back=True, pads={"1": row_net, "2": mid_net})


def place_usb(bb):
    """USB-C receptacle on the top edge (bottom side); fuse, ESD and CC resistors beside it
    in the strip above the top row."""
    x = USB_X
    j1 = bb.footprint("USB_C_Receptacle_HRO_TYPE-C-31-M-12", "J1", "TYPE-C-31-M-12", x, 3.65, rot=180, back=True,
                 pads={"A1": "GND", "A12": "GND", "B1": "GND", "B12": "GND", "S1": "GND",
                       "A4": "VBUS", "A9": "VBUS", "B4": "VBUS", "B9": "VBUS",
                       "A5": "CC1", "B5": "CC2",
                       "A6": "USB_DP", "B6": "USB_DP", "A7": "USB_DN", "B7": "USB_DN"})
    for pad in j1.Pads():  # shell pins: solid to the GND pour (thermal spokes get starved)
        if pad.GetNumber() == "S1":
            pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
    bb.footprint("Fuse_1206_3216Metric", "F1", "500mA PTC", x - 9.0, 4.0, back=True,
                 pads={"1": "VBUS", "2": "+5V"})
    # USBLC6-2SC6: 1/6 = I/O1, 3/4 = I/O2, 2 = GND, 5 = VBUS
    bb.footprint("SOT-23-6", "U2", "USBLC6-2SC6", x + 9.0, 4.0, back=True,
                 pads={"1": "USB_DN", "6": "USB_DN", "3": "USB_DP", "4": "USB_DP", "2": "GND", "5": "+5V"})
    bb.footprint("R_0603_1608Metric", "R5", "5.1k", x - 13.5, 4.0, rot=90, back=True, pads={"1": "CC1", "2": "GND"})
    bb.footprint("R_0603_1608Metric", "R6", "5.1k", x + 13.5, 4.0, rot=90, back=True, pads={"1": "CC2", "2": "GND"})


def place_mcu(bb):
    """ATmega32U4-MU and support parts on the bottom side under the spacebar.

    The footprint is mirrored (bottom side), so viewed from the top pins 1-11 are on
    the right, 12-22 at the bottom, 23-33 on the left and 34-44 on top.
    """
    mx, my = MCU_POS
    u1 = bb.footprint("QFN-44-1EP_7x7mm_P0.5mm_EP5.2x5.2mm", "U1", "ATmega32U4-MU", mx, my, back=True,
                      pads={str(p): n for p, n in MCU_FIXED.items()})

    def part(fp, ref, value, dx, dy, rot, pads):
        bb.footprint(fp, ref, value, mx + dx, my + dy, rot=rot, back=True, pads=pads)

    R, C = "R_0603_1608Metric", "C_0603_1608Metric"
    # bottom: crystal, VCC decoupling, reset pull-up
    part("Crystal_SMD_3225-4Pin_3.2x2.5mm", "Y1", "16MHz", -1.6, 7.8, 0,
         {"1": "XTAL1", "3": "XTAL2", "2": "GND", "4": "GND"})
    part(C, "C1", "22pF", -5.4, 7.8, 90, {"1": "XTAL1", "2": "GND"})
    part(C, "C2", "22pF", -1.6, 11.4, 0, {"1": "XTAL2", "2": "GND"})
    part(C, "C3", "100nF", 2.2, 6.8, 90, {"1": "+5V", "2": "GND"})
    part("C_0805_2012Metric", "C7", "10uF", 4.6, 7.6, 90, {"1": "+5V", "2": "GND"})
    part(R, "R1", "10k", 7.0, 6.8, 90, {"1": "+5V", "2": "RESET"})
    part("SW_SPST_SKQG_WithoutStem", "SW0", "RESET", 13.5, 7.8, 0, {"1": "RESET", "2": "GND"})
    # right (pins 1-11): UVCC decoupling, UCAP, USB series resistors
    part(C, "C9", "100nF", 6.8, -4.0, 0, {"1": "+5V", "2": "GND"})
    # below-right corner, so pins 5-11 keep a clear escape path to the right
    part(C, "C8", "1uF", 5.4, 4.2, 90, {"1": "UCAP", "2": "GND"})
    # rotated 180 so pad 1 (MCU side) faces U1 once mirrored onto the bottom
    part(R, "R3", "22R", 10.3, -1.9, 180, {"1": "U_DN", "2": "USB_DN"})
    part(R, "R4", "22R", 10.3, -0.1, 180, {"1": "U_DP", "2": "USB_DP"})
    # top (pins 34-44): VCC/AVCC decoupling
    # out at the corners: pins 36-41 (matrix) need the whole top side to fan out
    part(C, "C4", "100nF", -5.0, -6.2, 0, {"1": "+5V", "2": "GND"})
    part(C, "C6", "100nF", 5.0, -6.2, 0, {"1": "+5V", "2": "GND"})
    # left (pins 23-33): AVCC decoupling, HWB pull-down (reset -> DFU bootloader)
    part(C, "C5", "100nF", -6.6, 3.2, 0, {"1": "+5V", "2": "GND"})
    part(R, "R2", "10k", -6.6, -3.6, 0, {"1": "HWB", "2": "GND"})
    ground_mcu(bb, u1)
    # UCAP (pin 6, middle of the right side) to C8 would cut across the fan-out of
    # pins 7-11 on the bottom layer: give it a short stub and a via to the top layer.
    px, py = to_mm(next(p for p in u1.Pads() if p.GetNumber() == "6").GetPosition())
    vx = px + 1.55
    bb.track(px, py, vx, py, pcbnew.B_Cu, 0.2, "UCAP", locked=True)
    bb.via(vx, py, "UCAP", locked=True)
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
    """Tie every QFN GND pin inward to the exposed pad and drop vias in the pad to the
    pours. Ground then needs no fan-out, which leaves room for the signal pins next to
    it (otherwise e.g. pin 36 gets fenced in). Locked so Freerouting keeps them."""
    cx, cy = to_mm(u1.GetPosition())
    for pad in u1.Pads():
        if pad.GetNetname() != "GND" or pad.GetNumber() == "45":
            continue
        px, py = to_mm(pad.GetPosition())
        # straight in towards the centre, ending well inside the exposed pad (5.2 mm square)
        if abs(px - cx) > abs(py - cy):
            ex, ey = cx + math.copysign(2.0, px - cx), py
        else:
            ex, ey = px, cy + math.copysign(2.0, py - cy)
        bb.track(px, py, ex, ey, pcbnew.B_Cu, 0.25, "GND", locked=True)
    for dx in (-1.3, 1.3):
        for dy in (-1.3, 1.3):
            bb.via(cx + dx, cy + dy, "GND", locked=True)


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


def place_mounting_holes(bb, n_per_gap=(3, 0, 0, 3)):
    """M2 holes on the horizontal gaps between key rows, where switch housings leave a
    5 mm band free on top. Position along the gap is picked automatically to clear all
    pads, holes and bottom-side parts (sockets, diodes, MCU)."""
    obstacles = []
    for fp in bb.board.GetFootprints():
        for p in fp.Pads():
            x, y = to_mm(p.GetPosition())
            r = max(pcbnew.ToMM(p.GetSize().x), pcbnew.ToMM(p.GetSize().y),
                    pcbnew.ToMM(p.GetDrillSize().x)) / 2
            obstacles.append((x, y, r))
        if fp.IsFlipped():  # bottom-side part bodies (incl. hotswap sockets)
            bbox = fp.GetCourtyard(pcbnew.B_CrtYd).BBox()
            if bbox.GetWidth() > 0:
                x0, y0 = to_mm(bbox.GetOrigin())
                obstacles.append(("box", x0, y0, x0 + pcbnew.ToMM(bbox.GetWidth()),
                                  y0 + pcbnew.ToMM(bbox.GetHeight())))
    for t in bb.board.GetTracks():  # pre-routed stubs and vias
        x, y = to_mm(t.GetPosition() if t.GetClass() == "PCB_VIA" else t.GetStart())
        obstacles.append((x, y, 0.5))

    hole_r, margin, box_margin = 2.2, 0.6, 0.25
    # Hotswap sockets start 2.2 mm below each row gap, so nudge holes up a little;
    # the switch housings above still leave >2 mm for an M2 screw head.
    y_shift = -0.45 if SWITCH == "hotswap" else 0.0

    def clear(x, y):
        for o in obstacles:
            if o[0] == "box":
                _, x0, y0, x1, y1 = o
                dx = max(x0 - x, 0, x - x1)
                dy = max(y0 - y, 0, y - y1)
                if math.hypot(dx, dy) < hole_r + box_margin:
                    return False
            elif math.hypot(o[0] - x, o[1] - y) < o[2] + hole_r + margin:
                return False
        return True

    placed = []
    for gap, count in enumerate(n_per_gap, start=1):
        if not count:
            continue
        y = TOP_MARGIN + gap * 19.05 + y_shift
        for t in [(i + 0.5) / count for i in range(count)]:
            target = t * BOARD_W
            cands = [x / 4 for x in range(int(8 * 4), int((BOARD_W - 8) * 4))]
            cands.sort(key=lambda x: abs(x - target))
            x = next((c for c in cands if clear(c, y) and all(math.hypot(c - px, y - py) > 30 for px, py in placed)), None)
            if x is None:
                print(f"warning: no room for a mounting hole near ({target:.0f}, {y:.1f})")
                continue
            placed.append((x, y))
            obstacles.append((x, y, hole_r))
    for i, (x, y) in enumerate(placed, start=1):
        bb.footprint("MountingHole_2.2mm_M2_Pad", f"H{i}", "M2", x, y)
    return placed


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
    """Delete track segments with an end that touches nothing (Freerouting leaves a few
    short stubs behind). Repeats until stable; returns the number removed."""
    removed = 0
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
        save_board(board, PCB_PATH)
        add_gnd_pours(PCB_PATH)
        return

    bb = BoardBuilder()
    bb.net("GND")  # make sure the common nets exist before netclass assignment
    bb.outline(BOARD_W, PCB_H)
    bb.edge_keepout(BOARD_W, PCB_H)
    place_matrix(bb)
    place_usb(bb)
    u1 = place_mcu(bb)
    pinmap = assign_matrix_pins(bb, u1)
    holes = place_mounting_holes(bb)
    setup_rules(bb.board)

    bb.text(f"custom-kbr 65% ({SWITCH})", 30, 3.5, pcbnew.B_SilkS, mirror=True)

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

    save_board(bb.board, PCB_PATH)
    add_gnd_pours(PCB_PATH)


def add_gnd_pours(path):
    # The zone filler needs a board with connectivity built; an in-memory board
    # segfaults, so reload the saved file before adding and filling the pours.
    board = pcbnew.LoadBoard(path)
    bb = BoardBuilder()
    bb.board = board
    bb.nets = {"GND": board.FindNet("GND")}
    bb.zone("GND", pcbnew.F_Cu, BOARD_W, PCB_H)
    bb.zone("GND", pcbnew.B_Cu, BOARD_W, PCB_H)
    board.BuildConnectivity()
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    save_board(board, path)
    print("wrote", path)


if __name__ == "__main__":
    main()
