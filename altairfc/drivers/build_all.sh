#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for script in "$SCRIPT_DIR"/build_*.sh; do
    [ "$script" = "${BASH_SOURCE[0]}" ] && continue
    echo "Running $script..."
    bash "$script"
done
