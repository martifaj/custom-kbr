"""Single source of truth for the 60% layout, the switch matrix and the MCU pinout.

Everything else (PCB generator, QMK config) imports from here, so changing a key
here changes both the board and the firmware.
"""

U = 19.05  # 1u key pitch in mm
WIDTH_U = 15  # every row is 15u wide (standard 60%)
QMK_LAYOUT = "LAYOUT_60_ansi"

# (label, width_u, qmk_keycode) per row, left to right
_ROWS = [
    # row 0
    [("Esc", 1, "QK_GESC"), ("1", 1, "KC_1"), ("2", 1, "KC_2"), ("3", 1, "KC_3"),
     ("4", 1, "KC_4"), ("5", 1, "KC_5"), ("6", 1, "KC_6"), ("7", 1, "KC_7"),
     ("8", 1, "KC_8"), ("9", 1, "KC_9"), ("0", 1, "KC_0"), ("-", 1, "KC_MINS"),
     ("=", 1, "KC_EQL"), ("Backspace", 2, "KC_BSPC")],
    # row 1
    [("Tab", 1.5, "KC_TAB"), ("Q", 1, "KC_Q"), ("W", 1, "KC_W"), ("E", 1, "KC_E"),
     ("R", 1, "KC_R"), ("T", 1, "KC_T"), ("Y", 1, "KC_Y"), ("U", 1, "KC_U"),
     ("I", 1, "KC_I"), ("O", 1, "KC_O"), ("P", 1, "KC_P"), ("[", 1, "KC_LBRC"),
     ("]", 1, "KC_RBRC"), ("\\", 1.5, "KC_BSLS")],
    # row 2
    [("Caps", 1.75, "KC_CAPS"), ("A", 1, "KC_A"), ("S", 1, "KC_S"), ("D", 1, "KC_D"),
     ("F", 1, "KC_F"), ("G", 1, "KC_G"), ("H", 1, "KC_H"), ("J", 1, "KC_J"),
     ("K", 1, "KC_K"), ("L", 1, "KC_L"), (";", 1, "KC_SCLN"), ("'", 1, "KC_QUOT"),
     ("Enter", 2.25, "KC_ENT")],
    # row 3
    [("LShift", 2.25, "KC_LSFT"), ("Z", 1, "KC_Z"), ("X", 1, "KC_X"), ("C", 1, "KC_C"),
     ("V", 1, "KC_V"), ("B", 1, "KC_B"), ("N", 1, "KC_N"), ("M", 1, "KC_M"),
     (",", 1, "KC_COMM"), (".", 1, "KC_DOT"), ("/", 1, "KC_SLSH"), ("RShift", 2.75, "KC_RSFT")],
    # row 4
    [("LCtrl", 1.25, "KC_LCTL"), ("LGui", 1.25, "KC_LGUI"), ("LAlt", 1.25, "KC_LALT"),
     ("Space", 6.25, "KC_SPC"), ("RAlt", 1.25, "KC_RALT"), ("RGui", 1.25, "KC_RGUI"),
     ("Fn", 1.25, "MO(1)"), ("RCtrl", 1.25, "KC_RCTL")],
]

# Matrix column for each key, row by row (5 rows x 14 cols, chosen so that each
# column stays roughly vertical across rows).
_COLS = [
    list(range(14)),
    list(range(14)),
    list(range(12)) + [13],
    list(range(11)) + [13],
    [0, 1, 2, 6, 10, 11, 12, 13],
]

# Fn layer for the default QMK keymap: F-keys on the number row, arrows on IJKL with
# Home/PgUp/PgDn/End around them, media on the bottom right, bootloader on Fn+\\.
_FN_LAYER = {
    "Esc": "KC_GRV", "1": "KC_F1", "2": "KC_F2", "3": "KC_F3", "4": "KC_F4",
    "5": "KC_F5", "6": "KC_F6", "7": "KC_F7", "8": "KC_F8", "9": "KC_F9",
    "0": "KC_F10", "-": "KC_F11", "=": "KC_F12", "Backspace": "KC_DEL",
    "Y": "KC_HOME", "U": "KC_PGUP", "I": "KC_UP", "O": "KC_PGDN", "P": "KC_END",
    "J": "KC_LEFT", "K": "KC_DOWN", "L": "KC_RGHT", "[": "KC_PSCR", "]": "KC_INS",
    "\\": "QK_BOOT", "M": "KC_MUTE", ",": "KC_VOLD", ".": "KC_VOLU", "Space": "KC_MPLY",
}


class Key:
    def __init__(self, label, x, y, w, row, col, keycode):
        self.label, self.x, self.y, self.w = label, x, y, w
        self.row, self.col, self.keycode = row, col, keycode
        self.fn_keycode = _FN_LAYER.get(label, "KC_TRNS")

    @property
    def center_mm(self):
        return ((self.x + self.w / 2) * U, (self.y + 0.5) * U)


def _build():
    keys = []
    for r, (row, cols) in enumerate(zip(_ROWS, _COLS)):
        assert len(row) == len(cols), f"row {r}: {len(row)} keys vs {len(cols)} cols"
        x = 0.0
        for (label, w, kc), c in zip(row, cols):
            keys.append(Key(label, x, r, w, r, c, kc))
            x += w
        assert abs(x - WIDTH_U) < 1e-9, f"row {r} is {x}u wide, expected {WIDTH_U}u"
    seen = set()
    for k in keys:
        assert (k.row, k.col) not in seen, f"duplicate matrix position {(k.row, k.col)}"
        seen.add((k.row, k.col))
    return keys


KEYS = _build()
N_ROWS = 5
N_COLS = 14
BOARD_W = WIDTH_U * U   # 285.75 mm key area
BOARD_H = 5 * U         # 95.25 mm

# ATmega32U4 (QFN-44 / TQFP-44 share this pinout): pin number -> port name.
# These are the GPIOs free for the matrix; PE2 (HWB), XTAL, USB and power are fixed.
# PB0 (pin 8) and PD0 (pin 18) are deliberately left unused: they sit next to VBUS
# (pin 7) and XTAL1 (pin 17) and the QFN fan-out needs the room to route those.
MCU_GPIO = {
    1: "E6", 9: "B1", 10: "B2", 11: "B3", 12: "B7",
    19: "D1", 20: "D2", 21: "D3", 22: "D5", 25: "D4", 26: "D6", 27: "D7",
    28: "B4", 29: "B5", 30: "B6", 31: "C6", 32: "C7",
    36: "F7", 37: "F6", 38: "F5", 39: "F4", 40: "F1", 41: "F0",
}

# Fixed MCU pins: pin number -> net
MCU_FIXED = {
    2: "+5V",      # UVCC
    3: "U_DN",     # D-
    4: "U_DP",     # D+
    5: "GND",      # UGND
    6: "UCAP",
    7: "+5V",      # VBUS sense
    13: "RESET",
    14: "+5V",     # VCC
    15: "GND",
    16: "XTAL2",
    17: "XTAL1",
    23: "GND",
    24: "+5V",     # AVCC
    33: "HWB",     # PE2
    34: "+5V",     # VCC
    35: "GND",
    43: "GND",
    44: "+5V",     # AVCC
    45: "GND",     # exposed pad
    # 42 AREF left unconnected
}
