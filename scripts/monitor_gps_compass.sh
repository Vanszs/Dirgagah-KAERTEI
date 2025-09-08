#!/usr/bin/env bash
# Thin wrapper to the maintained script under test/
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
exec bash "$REPO_DIR/test/monitor_gps_compass.sh" "$@"

