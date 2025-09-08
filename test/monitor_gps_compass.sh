#!/usr/bin/env bash
set -euo pipefail

# Wrapper to run the realtime GPS/Compass monitor node
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ -f "$REPO_DIR/kaertei_drone/scripts/setup_kaertei.sh" ]]; then
  # shellcheck disable=SC1091
  source "$REPO_DIR/kaertei_drone/scripts/setup_kaertei.sh" >/dev/null 2>&1 || true
fi
if [[ -f "/opt/ros/foxy/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/foxy/setup.bash
fi
if [[ -f "$REPO_DIR/kaertei_drone/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$REPO_DIR/kaertei_drone/install/setup.bash"
fi

exec python3 "$SCRIPT_DIR/monitor_gps_compass.py"

