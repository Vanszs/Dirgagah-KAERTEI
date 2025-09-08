#!/usr/bin/env bash
set -euo pipefail

# Debug PX4 connection via MAVROS (ROS 2 Foxy)
# - Auto-sources KAERTEI env if available
# - Reads FCU port/baud from kaertei_drone/config/hardware_config.conf
# - Starts mavros_node if not running and prints connection state

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "[PX4] Preparing ROS 2 environment..."
# Some ROS setup scripts rely on unset vars; relax -u/-e while sourcing
set +u +e
if [[ -f "$REPO_DIR/kaertei_drone/scripts/setup_kaertei.sh" ]]; then
  # shellcheck disable=SC1091
  source "$REPO_DIR/kaertei_drone/scripts/setup_kaertei.sh" >/dev/null 2>&1 || true
fi
if [[ -f "/opt/ros/foxy/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/foxy/setup.bash || true
fi
if [[ -f "$REPO_DIR/kaertei_drone/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$REPO_DIR/kaertei_drone/install/setup.bash" || true
fi
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERR] ROS 2 environment not available. Ensure Foxy is installed and sourced." >&2
  exit 1
fi

# Defaults (can override via env FCU_PORT/FCU_BAUD or args)
FCU_PORT_DEFAULT="/dev/ttyACM0"
FCU_BAUD_DEFAULT="115200"

# Read from config if present
CFG_FILE="$REPO_DIR/kaertei_drone/config/hardware_config.conf"
if [[ -f "$CFG_FILE" ]]; then
  FCU_PORT_VAL=$(python3 - "$CFG_FILE" <<'PY' 2>/dev/null || true
import configparser, sys
cfg = configparser.ConfigParser()
cfg.read(sys.argv[1])
print(cfg.get('flight_controller','connection_port',fallback=''))
PY
)
  FCU_BAUD_VAL=$(python3 - "$CFG_FILE" <<'PY' 2>/dev/null || true
import configparser, sys
cfg = configparser.ConfigParser()
cfg.read(sys.argv[1])
print(cfg.get('flight_controller','baud_rate',fallback=''))
PY
)
else
  FCU_PORT_VAL=""
  FCU_BAUD_VAL=""
fi

FCU_PORT="${FCU_PORT:-${FCU_PORT_VAL:-$FCU_PORT_DEFAULT}}"
FCU_BAUD="${FCU_BAUD:-${FCU_BAUD_VAL:-$FCU_BAUD_DEFAULT}}"
FCU_URL="${1:-${FCU_PORT}:${FCU_BAUD}}"

echo "[PX4] Using FCU: $FCU_URL"
if [[ ! -e "$FCU_PORT" ]]; then
  echo "[WARN] Device $FCU_PORT not found. Continue anyway (MAVLink over UDP possible)." >&2
fi

echo "[PX4] Checking mavros node..."
if ros2 node list 2>/dev/null | grep -q "/mavros"; then
  echo "[PX4] mavros already running. Skipping launch."
  LAUNCHED=0
else
  echo "[PX4] Launching mavros_node..."
  set +e
  ros2 run mavros mavros_node --ros-args \
    -p fcu_url:="$FCU_URL" \
    -p target_system_id:=1 \
    -p target_component_id:=1 \
    -p fcu_protocol:="v2.0" \
    -r __node:=mavros \
    >"$REPO_DIR/.mavros_stdout.log" 2>&1 &
  MAVROS_PID=$!
  set -e
  LAUNCHED=1
  trap '[[ ${LAUNCHED:-0} -eq 1 ]] && kill -TERM ${MAVROS_PID:-0} 2>/dev/null || true' EXIT INT TERM
fi

echo "[PX4] Waiting for /mavros/state..."
if ! timeout 15 bash -c 'until ros2 topic list | grep -qx "/mavros/state"; do sleep 0.5; done'; then
  echo "[ERR] /mavros/state not available. Check cabling and permissions (dialout group)." >&2
  exit 1
fi

echo "[PX4] Reading connection state:"
ros2 topic echo -n 1 /mavros/state || true

CONNECTED=$(ros2 topic echo -n 1 /mavros/state 2>/dev/null | awk '/connected:/ {print $2; exit}')
MODE=$(ros2 topic echo -n 1 /mavros/state 2>/dev/null | awk '/mode:/ {print $2; exit}')
if [[ "$CONNECTED" == "true" ]]; then
  echo "[PX4] ✅ Connected. Mode=${MODE:-unknown}"
else
  echo "[PX4] ❌ Not connected to FCU. Check fcu_url and cabling."
fi

echo "[PX4] Heartbeat sample:"
ros2 topic echo -n 1 /mavros/heartbeat || true

echo "[PX4] Done. (Logs: $REPO_DIR/.mavros_stdout.log)"
