#!/usr/bin/env sh
set -e
PORT=/dev/ttyUSB0
idf.py -p $PORT -b 115200 flash monitor
