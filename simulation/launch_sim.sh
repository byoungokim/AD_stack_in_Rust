#!/bin/bash
# Launch the full Limo Drive simulation stack.
#
# Usage:
#   ./simulation/launch_sim.sh          # Gazebo + bridge + all processes
#   ./simulation/launch_sim.sh --dummy  # Dummy sim (no Gazebo) + all processes
#
# Prerequisites:
#   - Gazebo Harmonic installed (brew install gz-harmonic)
#   - Rust binaries built (cargo build --release)
#   - Python proto generated (make proto)
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

DUMMY_MODE=false
if [[ "$1" == "--dummy" ]]; then
    DUMMY_MODE=true
fi

# Colors for log output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

log() { echo -e "${GREEN}[launch]${NC} $1"; }
warn() { echo -e "${YELLOW}[launch]${NC} $1"; }
err() { echo -e "${RED}[launch]${NC} $1"; }

# Track PIDs for cleanup
PIDS=()

cleanup() {
    log "Shutting down all processes..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null
            wait "$pid" 2>/dev/null || true
        fi
    done
    log "All processes stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

# --- 1. Build ---
log "Building Rust binaries..."
source "$HOME/.cargo/env" 2>/dev/null || true
cargo build --release 2>&1 | tail -3

# --- 2. Python venv + proto ---
if [[ ! -d ".venv" ]]; then
    log "Creating Python venv..."
    python3 -m venv .venv
    .venv/bin/pip install -q pyzmq protobuf
fi
PYTHON=".venv/bin/python3"

if [[ ! -d "proto/gen_py" ]]; then
    log "Generating Python protobuf..."
    make proto
fi

# --- 3. Start simulator ---
if [[ "$DUMMY_MODE" == "true" ]]; then
    log "Starting DUMMY simulator..."
    ./target/release/limo_sim_bridge config/sim_bridge.yaml --dummy &
    PIDS+=($!)
else
    # Check Gazebo is installed
    if ! command -v gz &>/dev/null; then
        err "Gazebo not found! Install with: brew install gz-harmonic"
        err "Or run with --dummy flag for testing without Gazebo."
        exit 1
    fi

    # Set model path so Gazebo can find our models
    export GZ_SIM_RESOURCE_PATH="$PROJECT_DIR/simulation/models:${GZ_SIM_RESOURCE_PATH:-}"

    log "Starting Gazebo..."
    gz sim -r "$PROJECT_DIR/simulation/worlds/test_track.sdf" &
    PIDS+=($!)
    sleep 3  # Wait for Gazebo to initialize

    log "Starting Gazebo↔ZMQ bridge..."
    $PYTHON "$PROJECT_DIR/simulation/bridge/gz_zmq_bridge.py" &
    PIDS+=($!)
    sleep 1
fi

# --- 4. Start Limo Drive processes in sim mode ---
log "Starting ${CYAN}SensPerc${NC} (--sim)..."
./target/release/limo_sensperc config/sensperc.yaml --sim &
PIDS+=($!)

log "Starting ${CYAN}Planning${NC}..."
./target/release/limo_planning config/planning.yaml &
PIDS+=($!)

log "Starting ${CYAN}Control${NC} (--sim)..."
./target/release/limo_control config/control.yaml --sim &
PIDS+=($!)

sleep 1

# --- 5. Running ---
echo ""
log "=========================================="
log "  Limo Drive Simulation Running!"
if [[ "$DUMMY_MODE" == "true" ]]; then
    log "  Mode: DUMMY (no Gazebo)"
else
    log "  Mode: GAZEBO"
fi
log ""
log "  Processes:"
log "    SensPerc (--sim) : PID ${PIDS[-3]}"
log "    Planning         : PID ${PIDS[-2]}"
log "    Control  (--sim) : PID ${PIDS[-1]}"
if [[ "$DUMMY_MODE" == "true" ]]; then
    log "    DummySim         : PID ${PIDS[0]}"
else
    log "    Gazebo           : PID ${PIDS[0]}"
    log "    GZ-ZMQ Bridge    : PID ${PIDS[1]}"
fi
log ""
log "  Press Ctrl+C to stop all processes."
log "=========================================="
echo ""

# Wait for any process to exit
wait -n "${PIDS[@]}" 2>/dev/null || true
warn "A process exited. Cleaning up..."
cleanup
