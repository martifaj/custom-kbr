#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HEX="$ROOT/build/custom_kbr_default.hex"

if ! command -v dfu-programmer >/dev/null 2>&1; then
    echo "Error: dfu-programmer not found. Install it with: sudo apt install dfu-programmer" >&2
    exit 1
fi
DFU="dfu-programmer"

if ! "$DFU" atmega32u4 get bootloader-version >/dev/null 2>&1; then
    echo "Waiting for ATmega32U4 in DFU mode... (Press Fn + \\ on your keyboard now)"
    until "$DFU" atmega32u4 get bootloader-version >/dev/null 2>&1; do
        sleep 0.5
    done
fi

echo "Erasing ATmega32U4 flash..."
"$DFU" atmega32u4 erase

echo "Flashing $HEX..."
"$DFU" atmega32u4 flash "$HEX"

echo "Resetting into keyboard firmware..."
"$DFU" atmega32u4 reset

echo "Done! Your custom-kbr keyboard should now be enumerated on USB."
