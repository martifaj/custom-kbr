#!/usr/bin/env python3
"""Generate a QMK keyboard definition matching the PCB (reads build/pinmap.json,
which generate_pcb.py writes after assigning matrix pins).

Output: firmware/qmk/keyboards/custom_kbr/  -> copy into qmk_firmware/keyboards/
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from layout import KEYS  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "firmware", "qmk", "keyboards", "custom_kbr")


def main():
    with open(os.path.join(ROOT, "build", "pinmap.json")) as f:
        pins = json.load(f)

    layout = []
    for k in KEYS:
        entry = {"label": k.label, "matrix": [k.row, k.col], "x": k.x, "y": k.y}
        if k.w != 1:
            entry["w"] = k.w
        layout.append(entry)

    keyboard = {
        "keyboard_name": "custom-kbr",
        "manufacturer": "custom-kbr",
        "maintainer": "custom-kbr",
        "url": "",
        "processor": "atmega32u4",
        "bootloader": "atmel-dfu",
        "usb": {"vid": "0xFEED", "pid": "0x6565", "device_version": "1.0.0"},
        "diode_direction": "COL2ROW",
        "matrix_pins": {"rows": pins["rows"], "cols": pins["cols"]},
        "features": {"bootmagic": True, "extrakey": True, "mousekey": False, "nkro": True},
        "layouts": {"LAYOUT_65_ansi": {"layout": layout}},
    }
    keymap = {
        "keyboard": "custom_kbr",
        "keymap": "default",
        "layout": "LAYOUT_65_ansi",
        "layers": [[k.keycode for k in KEYS], [k.fn_keycode for k in KEYS]],
    }

    os.makedirs(os.path.join(OUT, "keymaps", "default"), exist_ok=True)
    with open(os.path.join(OUT, "keyboard.json"), "w") as f:
        json.dump(keyboard, f, indent=4)
    with open(os.path.join(OUT, "keymaps", "default", "keymap.json"), "w") as f:
        json.dump(keymap, f, indent=4)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
