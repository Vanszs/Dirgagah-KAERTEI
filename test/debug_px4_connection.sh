#!/usr/bin/env bash
set -euo pipefail

TOPIC="${1:-/mavros/state}"
QOS=(--qos-reliability best_effort --qos-durability volatile)

# pastikan daemon fresh
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start

# cek topic ada
if ! ros2 topic list | grep -qxE "$(printf '%s' "$TOPIC" | sed 's,/,\\/,g')"; then
  echo "[ERR] Topic $TOPIC tidak ditemukan. Pastikan MAVROS berjalan & namespace benar." >&2
  exit 2
fi

# ambil satu pesan dan baca field connected
CONNECTED=$(
  timeout 5s ros2 topic echo "${QOS[@]}" "$TOPIC" 2>/dev/null \
    | awk '/^connected:/ {print tolower($2); exit}'
)

if [[ "$CONNECTED" == "true" ]]; then
  MODE=$(timeout 5s ros2 topic echo "${QOS[@]}" "$TOPIC" 2>/dev/null \
          | awk '/^mode:/ {print $2; exit}')
  echo "[OK] FCU connected. Mode=${MODE:-unknown}"
  exit 0
else
  echo "[FAIL] FCU NOT connected (connected=${CONNECTED:-unknown})." >&2
  exit 1
fi
