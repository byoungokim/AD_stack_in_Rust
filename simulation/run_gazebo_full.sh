#!/bin/bash
# Launch full Gazebo simulation with AD stack and a navigation scenario.
#
# macOS requires separate server/GUI processes for Gazebo.
# This script launches everything and sends a scenario after startup.
#
# Usage: ./simulation/run_gazebo_full.sh [preset_name_or_scenario_file]
#   preset_name: straight_line, square_patrol, slalom, parking, figure_eight
#   scenario_file: path to a scenario YAML (e.g. config/scenarios/town_patrol.yaml)
#   default: square_patrol
#   WORLD env var overrides the world SDF (default: simulation/worlds/test_track.sdf)
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

PRESET="${1:-square_patrol}"
WORLD="${WORLD:-$PROJECT_DIR/simulation/worlds/test_track.sdf}"

if [[ ! -f "$WORLD" ]]; then
    echo "ERROR: world file not found: $WORLD" >&2
    echo "       (wrong branch? available worlds:)" >&2
    ls "$PROJECT_DIR/simulation/worlds/"*.sdf >&2
    exit 1
fi

RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m'

log() { echo -e "${GREEN}[sim]${NC} $1"; }

PIDS=()

cleanup() {
    echo ""
    log "Shutting down everything..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    pkill -f "gz sim" 2>/dev/null
    pkill -f gz_zmq_bridge 2>/dev/null
    pkill -f ped_controller 2>/dev/null
    pkill -f traffic_controller 2>/dev/null
    pkill -f accident_monitor 2>/dev/null
    pkill -f limo_ 2>/dev/null
    wait 2>/dev/null
    log "Done."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Pre-launch cleanup: leftovers from a previous run that didn't fully die
# leave the GUI attached to a dead server and hold the ZMQ ports.
if pgrep -f "gz sim" >/dev/null 2>&1 || pgrep -f "limo_sensperc|limo_planning|limo_control|limo_scenario|limo_sim_bridge" >/dev/null 2>&1; then
    log "Cleaning up leftovers from a previous run..."
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f gz_zmq_bridge 2>/dev/null || true
    pkill -f ped_controller 2>/dev/null || true
    pkill -f traffic_controller 2>/dev/null || true
    pkill -f accident_monitor 2>/dev/null || true
    pkill -f "limo_sensperc|limo_planning|limo_control|limo_scenario|limo_sim_bridge" 2>/dev/null || true
    sleep 2
    pkill -9 -f "gz sim" 2>/dev/null || true
fi

# Ensure venv
if [[ ! -d ".venv" ]]; then
    log "Creating Python venv..."
    python3 -m venv .venv
    .venv/bin/pip install -q pyzmq protobuf
fi
PYTHON="$PROJECT_DIR/.venv/bin/python3"
# Ensure proto
if [[ ! -d "proto/gen_py" ]]; then
    make proto
fi

[ -f "$HOME/.cargo/env" ] && source "$HOME/.cargo/env" || true
export GZ_SIM_RESOURCE_PATH="$PROJECT_DIR/simulation/models"
# macOS blocks multicast discovery; force gz-transport onto loopback.
export GZ_IP="${GZ_IP:-127.0.0.1}"

# The bridge runs on system Python for gz.transport13 and needs zmq+protobuf there.
if ! python3 -c "import zmq, google.protobuf, gz.transport13" 2>/dev/null; then
    echo "ERROR: system python3 is missing bridge deps (gz.transport13, pyzmq, protobuf)." >&2
    echo "       Install Gazebo Harmonic (brew install osrf/simulation/gz-harmonic) and:" >&2
    echo "       python3 -m pip install --break-system-packages pyzmq protobuf" >&2
    exit 1
fi

# === 1. Gazebo Server ===
log "Starting Gazebo server..."
gz sim -s -r "$WORLD" &
PIDS+=($!)
sleep 3

# === 2. Gazebo GUI ===
log "Starting Gazebo GUI..."
gz sim -g &
PIDS+=($!)
sleep 2

# === 3. Gazebo↔ZMQ Bridge (uses system Python for gz.transport13) ===
log "Starting Gazebo↔ZMQ bridge (native gz.transport13)..."
python3 "$PROJECT_DIR/simulation/bridge/gz_zmq_bridge.py" &
PIDS+=($!)
sleep 1

# === 3b. Reactive pedestrian controller (city worlds) ===
# gen_city_world.py emits <world>_peds.json; when present, ped_* models in
# the world are driven (and made to yield to the robot) by the controller.
PEDS_FILE="${WORLD%.sdf}_peds.json"
if [[ -f "$PEDS_FILE" ]]; then
    log "Starting reactive pedestrian controller ($PEDS_FILE)..."
    python3 "$PROJECT_DIR/simulation/bridge/ped_controller.py" "$PEDS_FILE" &
    PIDS+=($!)
fi

# === 3b2. Erratic traffic controller (GTA v2 worlds) ===
# gen_city_world.py emits <world>_traffic.json; when present, vehicle_*
# car models are driven with irregular behavior by the controller.
TRAFFIC_FILE="${WORLD%.sdf}_traffic.json"
if [[ -f "$TRAFFIC_FILE" ]]; then
    log "Starting erratic traffic controller ($TRAFFIC_FILE)..."
    python3 "$PROJECT_DIR/simulation/bridge/traffic_controller.py" "$TRAFFIC_FILE" &
    PIDS+=($!)
fi

# === 3c. Accident monitor ===
# Counts collisions and logs the reason for each (chassis contact sensor +
# perceptual vehicle-overlap detection) to accidents.log.
log "Starting accident monitor (accidents.log)..."
python3 "$PROJECT_DIR/simulation/bridge/accident_monitor.py" &
PIDS+=($!)

# === 4. AD Stack ===
log "Starting ${CYAN}SensPerc${NC} (--sim)..."
./target/release/limo_sensperc config/sensperc.yaml --sim &
PIDS+=($!)

log "Starting ${CYAN}Planning${NC}..."
./target/release/limo_planning config/planning.yaml &
PIDS+=($!)

log "Starting ${CYAN}Control${NC} (--sim)..."
./target/release/limo_control config/control.yaml --sim &
PIDS+=($!)

sleep 2

# === 5. Path Visualizer ===
log "Starting path visualizer (Gazebo markers)..."
$PYTHON simulation/bridge/gz_path_visualizer.py &
PIDS+=($!)

# === 6. Send Scenario ===
log "Sending scenario: ${YELLOW}${PRESET}${NC}"
if [[ -f "$PRESET" ]]; then
    ./target/release/limo_scenario --file "$PRESET" &
else
    ./target/release/limo_scenario --preset "$PRESET" &
fi
PIDS+=($!)

echo ""
log "=========================================="
log "  ${CYAN}Limo Drive Gazebo Simulation Running!${NC}"
log ""
log "  Scenario: ${YELLOW}${PRESET}${NC}"
log "  Gazebo GUI should be visible on screen."
log ""
log "  Processes running:"
log "    Gazebo Server + GUI"
log "    Gazebo↔ZMQ Bridge"
log "    SensPerc (--sim)"
log "    Planning (Hybrid A* + DWA)"
log "    Control (--sim)"
log "    Scenario Manager (${PRESET})"
log ""
log "  Press ${RED}Ctrl+C${NC} to stop everything."
log "=========================================="
echo ""

# Wait indefinitely
while true; do
    sleep 1
done
