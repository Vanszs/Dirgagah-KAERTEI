#!/usr/bin/env bash
# Thin wrapper to the maintained script under test/
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Prepare ROS 2 env silently if available
set +u +e
[[ -f "/opt/ros/foxy/setup.bash" ]] && source /opt/ros/foxy/setup.bash || true
[[ -f "$REPO_DIR/kaertei_drone/install/setup.bash" ]] && source "$REPO_DIR/kaertei_drone/install/setup.bash" || true
set -euo pipefail

exec python3 "$REPO_DIR/test/gps_compass_monitor.py" "$@"

