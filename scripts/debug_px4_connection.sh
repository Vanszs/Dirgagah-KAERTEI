#!/usr/bin/env bash
# Thin wrapper to the maintained script under test/
# Ensure running under bash even if invoked via sh
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
exec bash "$REPO_DIR/test/debug_px4_connection.sh" "$@"
