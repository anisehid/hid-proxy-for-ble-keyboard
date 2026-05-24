#!/usr/bin/env sh
set -e
PORT="${PORT:-/dev/ttyUSB0}"
idf.py -p "$PORT" -b 115200 flash monitor
