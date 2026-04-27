#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
gcc -O2 -Wall -shared -fPIC -o "$SCRIPT_DIR/gps_driver.so" "$SCRIPT_DIR/gps_driver.c"
echo "Built: $SCRIPT_DIR/gps_driver.so"
