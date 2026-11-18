#!/usr/bin/env bash
set -euo pipefail

win_tmp='/mnt/c/temp'
mkdir -p "$win_tmp"
cp "$1" "$win_tmp/firmware.hex"

exec /mnt/c/tools/teensy_loader_cli.exe \
  --mcu=TEENSY41 -w -s -v 'C:\Temp\firmware.hex'