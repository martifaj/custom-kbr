#!/usr/bin/env python3
"""Grouped BOM from the board's footprints (there is no schematic)."""
import collections
import csv
import re
import sys

import pcbnew

MPN = {
    "ATmega32U4-MU": "Microchip ATMEGA32U4-MU",
    "USBLC6-2SC6": "ST USBLC6-2SC6",
    "TYPE-C-31-M-12": "Korean Hroparts TYPE-C-31-M-12",
    "1N4148W": "MDD 1N4148W (SOD-123)",
    "500mA PTC": "Bourns MF-NSMF050-2 (or any 1206 500 mA hold PTC)",
    "RESET": "Alps SKQGABE010",
    "16MHz": "Yangxing X322516MLB4SI (or TXC 7M-16.000MAAE-T, 16 MHz 3225 4-pin crystal, CL=12pF)",
    "Kailh MX hotswap socket": "Kailh CPG151101S11 (switches themselves are not soldered)",
    "18pF": "Yageo CC0603JRNPO9BN180 (18pF 50V C0G 0603)",
    "100nF": "Yageo CC0603KRX7R9BB104 (100nF 50V X7R 0603)",
    "10uF": "Samsung CL21A106KAYNNNE (10uF 25V X5R 0805)",
    "1uF": "Samsung CL10A105KB8NNNC (1uF 50V X5R 0603)",
    "10k": "Yageo RC0603FR-0710KL (10k 1% 0603)",
    "22R": "Yageo RC0603FR-0722RL (22R 1% 0603)",
    "5.1k": "Yageo RC0603FR-075K1L (5.1k 1% 0603)",
}


def main(pcb, out):
    board = pcbnew.LoadBoard(pcb)
    groups = collections.OrderedDict()
    natural = lambda ref: [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", ref)]
    for fp in sorted(board.GetFootprints(), key=lambda f: natural(f.GetReference())):
        ref = fp.GetReference()
        if ref.startswith("H"):
            continue  # mounting holes
        key = (fp.GetValue(), fp.GetFPID().GetLibItemName().wx_str())
        if ref.startswith("SW") and ref != "SW0":
            if "Hotswap" in key[1]:
                key = ("Kailh MX hotswap socket", "SW_MX_Hotswap_*")
            else:
                key = ("MX-compatible switch", "SW_Cherry_MX_*_PCB")
        groups.setdefault(key, []).append(ref)
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Qty", "Value", "Footprint", "Part", "References"])
        for (value, fpname), refs in groups.items():
            w.writerow([len(refs), value, fpname, MPN.get(value, ""), " ".join(refs)])
    print("wrote", out)


if __name__ == "__main__":
    main(*sys.argv[1:3])
