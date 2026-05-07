#!/usr/bin/env bash
# Run once on each physical robot after cloning / pulling.
# Marks calibration config files as skip-worktree so that
# git pull never overwrites robot-specific calibration values.

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CONFIG_FILES=(
    qupa_hardware/config/ir_scanner.yaml
    qupa_hardware/config/camera.yaml
    qupa_hardware/config/camera_copy.yaml
    qupa_hardware/config/floor_sensor.yaml
    qupa_hardware/config/leds.yaml
    qupa_hardware/config/motor.yaml
)

cd "$REPO_ROOT"

for f in "${CONFIG_FILES[@]}"; do
    if [ -f "$f" ]; then
        git update-index --skip-worktree "$f"
        echo "protected: $f"
    else
        echo "missing (skipped): $f"
    fi
done

echo ""
echo "Done. Local calibration files are now protected from git pull."
echo "To undo protection on a file: git update-index --no-skip-worktree <file>"
