#!/bin/bash
# Launch full Gazebo simulation with AD stack and a navigation scenario.
#
# macOS requires separate server/GUI processes for Gazebo.
# This script launches everything and sends a scenario after startup.
#
# Usage: ./simulation/run_gazebo_full.sh [preset_name]
#   preset_name: straight_line, square_patrol, slalom, parking, figure_eight
#   default: square_patrol
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

PRESET="${1:-square_patrol}"

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
    pkill -f limo_ 2>/dev/null
    wait 2>/dev/null
    log "Done."
    exit 0
}

trap cleanup SIGINT SIGTERM

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

source "$HOME/.cargo/env" 2>/dev/null || true
export GZ_SIM_RESOURCE_PATH="$PROJECT_DIR/simulation/models"

# === 1. Gazebo Server ===
log "Starting Gazebo server..."
gz sim -s -r "$PROJECT_DIR/simulation/worlds/test_track.sdf" &
PIDS+=($!)
sleep 3

# === 2. Gazebo GUI ===
log "Starting Gazebo GUI..."
gz sim -g &
PIDS+=($!)
sleep 2

# === 3. Gazebo↔ZMQ Bridge ===
log "Starting Gazebo↔ZMQ bridge..."
$PYTHON "$PROJECT_DIR/simulation/bridge/gz_zmq_bridge.py" &
PIDS+=($!)
sleep 1

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
./target/release/limo_scenario --preset "$PRESET" &
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
