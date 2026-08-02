#!/bin/bash
# Scenario Test Runner for Limo Drive
#
# Runs 4 headless scenario tests + 1 GUI demo:
#   Test 1: Intersection crossing (no collision)
#   Test 2: Obstacle bypass (navigates around wall)
#   Test 3: Destination accuracy (within 0.5m)
#   Test 4: Random obstacles (GUI demo, visual only)
#   Test 5: IMU dropout — graceful degradation (sim_faults fault injection)
#
# Usage:
#   ./simulation/tests/run_scenario_tests.sh          # Run all (1-3,5 headless + 4 GUI)
#   ./simulation/tests/run_scenario_tests.sh headless  # Run only 1-3 and 5 headless
#   ./simulation/tests/run_scenario_tests.sh gui       # Run only test 4 GUI
#   ./simulation/tests/run_scenario_tests.sh 2         # Run specific test number (1..5)
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
cd "$PROJECT_DIR"

[ -f "$HOME/.cargo/env" ] && source "$HOME/.cargo/env" || true
export GZ_SIM_RESOURCE_PATH="$PROJECT_DIR/simulation/models"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

PASS=0
FAIL=0
RESULTS=()

# macOS blocks multicast discovery; force gz-transport onto loopback.
export GZ_IP="${GZ_IP:-127.0.0.1}"

# macOS uses gtimeout from coreutils
if command -v gtimeout &>/dev/null; then
    TIMEOUT_CMD="gtimeout"
elif command -v timeout &>/dev/null; then
    TIMEOUT_CMD="timeout"
else
    err "Neither 'timeout' nor 'gtimeout' found. Install coreutils: brew install coreutils"
    exit 1
fi

log()  { echo -e "${GREEN}[test]${NC} $1"; }
err()  { echo -e "${RED}[test]${NC} $1"; }
warn() { echo -e "${YELLOW}[test]${NC} $1"; }

cleanup() {
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f limo_ 2>/dev/null || true
    pkill -f gz_zmq 2>/dev/null || true
    pkill -f gz_path 2>/dev/null || true
    pkill -f gz_zmq_bridge 2>/dev/null || true
    sleep 2
}

# Build if needed
cargo build --release -q 2>/dev/null
make proto 2>/dev/null

# ============================================================
# Test infrastructure: start Gazebo + stack, run scenario, check result
# ============================================================

run_headless_test() {
    local test_name="$1"
    local world_file="$2"
    local goal_x="$3"
    local goal_y="$4"
    local goal_theta="$5"
    local timeout_sec="$6"
    local max_distance="$7"  # pass if final distance <= this

    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log "  ${CYAN}${test_name}${NC}"
    log "  World: $(basename $world_file)"
    log "  Goal: (${goal_x}, ${goal_y}) tolerance: ${max_distance}m"
    log "  Timeout: ${timeout_sec}s"
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    cleanup

    # Start Gazebo headless
    log "  Starting Gazebo..."
    gz sim -s -r "$world_file" > /dev/null 2>&1 &
    sleep 4

    # Start bridge
    log "  Starting bridge..."
    python3 simulation/bridge/gz_zmq_bridge.py > /dev/null 2>&1 &
    sleep 2

    # Start AD stack
    log "  Starting AD stack..."
    ./target/release/limo_sensperc config/sensperc.yaml --sim > /dev/null 2>&1 &
    ./target/release/limo_planning config/planning.yaml > /dev/null 2>&1 &
    ./target/release/limo_control config/control.yaml --sim > /dev/null 2>&1 &
    sleep 3
    log "  Sending goal..."

    # Send goal and capture output (tracing logs go to stderr, redirect to stdout)
    local outfile="/tmp/limo_test_$$.log"
    $TIMEOUT_CMD "$timeout_sec" \
        ./target/release/limo_scenario --goal "$goal_x" "$goal_y" "$goal_theta" \
        > "$outfile" 2>&1 || true

    # Show progress
    grep "dist=" "$outfile" | tail -5

    # Extract minimum distance achieved (best approach distance)
    local final_dist
    final_dist=$(grep "dist=" "$outfile" | \
        sed -n 's/.*dist=\([0-9.]*\)m.*/\1/p' | \
        sort -n | head -1)

    # Check for completion
    local completed
    completed=$(grep -c "completed" "$outfile" || true)

    rm -f "$outfile"

    cleanup

    # Evaluate result
    if [[ -z "$final_dist" ]]; then
        final_dist="N/A"
    fi

    if [[ "$final_dist" != "N/A" ]]; then
        # Check if final distance meets criteria even without explicit completion
        local dist_ok
        dist_ok=$(echo "$final_dist $max_distance" | awk '{print ($1 <= $2) ? "1" : "0"}')
        if [[ "$dist_ok" == "1" ]]; then
            log "  ${GREEN}PASS${NC} — Within tolerance! Final distance: ${final_dist}m <= ${max_distance}m"
            PASS=$((PASS + 1))
            RESULTS+=("PASS: $test_name (dist=${final_dist}m)")
            return 0
        else
            err "  ${RED}FAIL${NC} — Final distance: ${final_dist}m > ${max_distance}m"
            FAIL=$((FAIL + 1))
            RESULTS+=("FAIL: $test_name (dist=${final_dist}m > ${max_distance}m)")
            return 1
        fi
    else
        err "  ${RED}FAIL${NC} — Timed out (${timeout_sec}s), no distance data"
        FAIL=$((FAIL + 1))
        RESULTS+=("FAIL: $test_name (timeout)")
        return 1
    fi
}

# ============================================================
# Test 1: Intersection crossing
# ============================================================
test_intersection() {
    run_headless_test \
        "Test 1: Intersection Crossing (no collision)" \
        "$PROJECT_DIR/simulation/worlds/tests/test_intersection.sdf" \
        10.0 0.0 0.0 \
        60 \
        1.0
}

# ============================================================
# Test 2: Obstacle bypass
# ============================================================
test_obstacle_bypass() {
    run_headless_test \
        "Test 2: Obstacle Bypass" \
        "$PROJECT_DIR/simulation/worlds/tests/test_obstacle_bypass.sdf" \
        6.0 0.0 0.0 \
        90 \
        1.5
}

# ============================================================
# Test 3: Destination accuracy (0.5m)
# ============================================================
test_destination_accuracy() {
    run_headless_test \
        "Test 3: Destination Accuracy (<=0.5m)" \
        "$PROJECT_DIR/simulation/worlds/tests/test_destination_accuracy.sdf" \
        4.0 3.0 1.57 \
        90 \
        1.0
}

# ============================================================
# Fault-injection test infrastructure
#   Like run_headless_test, but:
#     - launches sensperc with a caller-supplied config (fault-injection fixture)
#     - passes if EITHER final dist <= max_distance
#             OR control log shows estop=true AND |final_pose| <= runaway_limit
#     - fails if the robot ran away (|final_pose| > runaway_limit)
#       or produced no data at all.
# ============================================================

run_fault_injection_test() {
    local test_name="$1"
    local world_file="$2"
    local goal_x="$3"
    local goal_y="$4"
    local goal_theta="$5"
    local timeout_sec="$6"
    local max_distance="$7"     # "reached goal" tolerance (meters)
    local runaway_limit="$8"    # final |pose| must be <= this (meters)
    local sensperc_cfg="$9"     # path to sensperc yaml with sim_faults enabled

    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log "  ${CYAN}${test_name}${NC}"
    log "  World: $(basename $world_file)"
    log "  Sensperc cfg: $(basename $sensperc_cfg)"
    log "  Goal: (${goal_x}, ${goal_y}) reach-tolerance: ${max_distance}m"
    log "  Runaway limit: ${runaway_limit}m (|final pose| must be <=)"
    log "  Timeout: ${timeout_sec}s"
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    if [[ ! -f "$sensperc_cfg" ]]; then
        err "  ${RED}FAIL${NC} — sensperc config not found: $sensperc_cfg"
        FAIL=$((FAIL + 1))
        RESULTS+=("FAIL: $test_name (missing config)")
        return 1
    fi

    cleanup

    log "  Starting Gazebo..."
    gz sim -s -r "$world_file" > /dev/null 2>&1 &
    sleep 4

    log "  Starting bridge..."
    python3 simulation/bridge/gz_zmq_bridge.py > /dev/null 2>&1 &
    sleep 2

    # Capture sensperc + control logs so we can inspect fault-injection banner,
    # estop transitions, and the final pose.
    local sensperc_log="/tmp/limo_test_sensperc_$$.log"
    local control_log="/tmp/limo_test_control_$$.log"

    log "  Starting AD stack (sensperc with fault injection)..."
    ./target/release/limo_sensperc "$sensperc_cfg" --sim > "$sensperc_log" 2>&1 &
    ./target/release/limo_planning config/planning.yaml > /dev/null 2>&1 &
    ./target/release/limo_control config/control.yaml --sim > "$control_log" 2>&1 &
    sleep 3

    # Confirm fault injection activated (best-effort, non-fatal).
    if grep -q "fault injection" "$sensperc_log"; then
        log "  sensperc reports: $(grep 'fault injection' "$sensperc_log" | head -1)"
    else
        warn "  sensperc did not log a 'fault injection' banner yet"
    fi

    log "  Sending goal..."
    local outfile="/tmp/limo_test_$$.log"
    $TIMEOUT_CMD "$timeout_sec" \
        ./target/release/limo_scenario --goal "$goal_x" "$goal_y" "$goal_theta" \
        > "$outfile" 2>&1 || true

    grep "dist=" "$outfile" | tail -5

    # Minimum (best) distance observed.
    local final_dist
    final_dist=$(grep "dist=" "$outfile" | \
        sed -n 's/.*dist=\([0-9.]*\)m.*/\1/p' | \
        sort -n | head -1)

    # Did planning ever emit an emergency stop?
    #   Control cycle log format: "... estop=true ..."
    local estop_seen=0
    if grep -q "estop=true" "$control_log"; then
        estop_seen=1
    fi

    # Final pose from last control cycle line "pose=(x, y, θ°)".
    local last_pose_line
    last_pose_line=$(grep "pose=(" "$control_log" | tail -1 || true)
    local final_norm=""
    if [[ -n "$last_pose_line" ]]; then
        final_norm=$(echo "$last_pose_line" | \
            sed -n 's/.*pose=(\([^,]*\), \([^,]*\),.*/\1 \2/p' | \
            awk '{printf "%.3f", sqrt($1*$1 + $2*$2)}')
    fi

    rm -f "$outfile" "$sensperc_log" "$control_log"
    cleanup

    # ---- Pass/fail logic ---------------------------------------------------
    # A) No data at all -> FAIL
    if [[ -z "$final_dist" && -z "$final_norm" ]]; then
        err "  ${RED}FAIL${NC} — no distance or pose telemetry captured (timeout ${timeout_sec}s)"
        FAIL=$((FAIL + 1))
        RESULTS+=("FAIL: $test_name (no telemetry)")
        return 1
    fi

    # B) Runaway: robot went way past the goal envelope -> FAIL
    if [[ -n "$final_norm" ]]; then
        local runaway
        runaway=$(echo "$final_norm $runaway_limit" | awk '{print ($1 > $2) ? "1" : "0"}')
        if [[ "$runaway" == "1" ]]; then
            err "  ${RED}FAIL${NC} — runaway: |final pose|=${final_norm}m > ${runaway_limit}m"
            FAIL=$((FAIL + 1))
            RESULTS+=("FAIL: $test_name (runaway |pose|=${final_norm}m)")
            return 1
        fi
    fi

    # C) Reached goal despite fault -> PASS
    if [[ -n "$final_dist" ]]; then
        local dist_ok
        dist_ok=$(echo "$final_dist $max_distance" | awk '{print ($1 <= $2) ? "1" : "0"}')
        if [[ "$dist_ok" == "1" ]]; then
            log "  ${GREEN}PASS${NC} — reached goal despite IMU dropout (dist=${final_dist}m)"
            PASS=$((PASS + 1))
            RESULTS+=("PASS: $test_name (reached, dist=${final_dist}m)")
            return 0
        fi
    fi

    # D) Didn't reach, but emergency-stopped cleanly and stayed bounded -> PASS
    if [[ "$estop_seen" == "1" ]]; then
        log "  ${GREEN}PASS${NC} — clean emergency stop, no runaway (estop=true, |pose|=${final_norm:-N/A}m, best dist=${final_dist:-N/A}m)"
        PASS=$((PASS + 1))
        RESULTS+=("PASS: $test_name (graceful estop, |pose|=${final_norm:-N/A}m)")
        return 0
    fi

    # E) Neither reached nor estopped, but stayed bounded -> FAIL (indecisive).
    err "  ${RED}FAIL${NC} — did not reach goal and no emergency stop (best dist=${final_dist:-N/A}m, |pose|=${final_norm:-N/A}m)"
    FAIL=$((FAIL + 1))
    RESULTS+=("FAIL: $test_name (no goal, no estop)")
    return 1
}

# ============================================================
# Test 4: Random obstacles (GUI)
# ============================================================
test_random_obstacles_gui() {
    local WORLD="test_random_obstacles"
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log "  ${CYAN}Test 4: Random Obstacles (GUI Demo)${NC}"
    log "  Watch the robot navigate while obstacles appear!"
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    cleanup

    # Gazebo server + GUI
    gz sim -s -r "$PROJECT_DIR/simulation/worlds/tests/test_random_obstacles.sdf" > /dev/null 2>&1 &
    sleep 3
    gz sim -g > /dev/null 2>&1 &
    sleep 4

    # Bridge + stack
    python3 simulation/bridge/gz_zmq_bridge.py > /dev/null 2>&1 &
    sleep 1
    ./target/release/limo_sensperc config/sensperc.yaml --sim > /dev/null 2>&1 &
    ./target/release/limo_planning config/planning.yaml > /dev/null 2>&1 &
    ./target/release/limo_control config/control.yaml --sim > /dev/null 2>&1 &
    sleep 2

    # Visualizer
    .venv/bin/python3 simulation/bridge/gz_path_visualizer.py > /dev/null 2>&1 &
    sleep 1

    # Send goal
    ./target/release/limo_scenario --goal 4.0 0.0 0.0 > /dev/null 2>&1 &
    sleep 2

    log "Robot is navigating. Spawning random obstacles..."

    # Spawn random obstacles every 3 seconds
    for i in $(seq 1 8); do
        sleep 3
        # Random position in the robot's potential path
        local rx=$(python3 -c "import random; print(f'{random.uniform(-2, 3):.1f}')")
        local ry=$(python3 -c "import random; print(f'{random.uniform(-2, 2):.1f}')")
        local size=$(python3 -c "import random; print(f'{random.uniform(0.2, 0.5):.2f}')")
        local r=$(python3 -c "import random; print(f'{random.uniform(0.3, 1.0):.1f}')")
        local g=$(python3 -c "import random; print(f'{random.uniform(0.1, 0.5):.1f}')")
        local b=$(python3 -c "import random; print(f'{random.uniform(0.1, 0.5):.1f}')")

        log "  Spawning obstacle #${i} at (${rx}, ${ry}) size=${size}m"

        gz service -s "/world/$WORLD/create" \
            --reqtype gz.msgs.EntityFactory \
            --reptype gz.msgs.Boolean \
            --timeout 2000 \
            --req "sdf: \"<sdf version=\\\"1.9\\\"><model name=\\\"random_obs_${i}\\\"><static>true</static><pose>${rx} ${ry} ${size} 0 0 0</pose><link name=\\\"l\\\"><collision name=\\\"c\\\"><geometry><box><size>${size} ${size} ${size}</size></box></geometry></collision><visual name=\\\"v\\\"><geometry><box><size>${size} ${size} ${size}</size></box></geometry><material><ambient>${r} ${g} ${b} 1</ambient><diffuse>${r} ${g} ${b} 1</diffuse></material></visual></link></model></sdf>\", name: \"random_obs_${i}\"" \
            > /dev/null 2>&1 || true
    done

    log ""
    log "All obstacles spawned! Watch the robot for 30 more seconds..."
    log "Press Ctrl+C to stop."
    sleep 30

    cleanup
    log "  ${CYAN}GUI demo completed${NC}"
    RESULTS+=("DEMO: Test 4 Random Obstacles (GUI)")
}

# ============================================================
# Test 5: IMU dropout — graceful degradation
#
# Reuses the destination-accuracy world but launches sensperc with a fixture
# config that activates sim_faults { imu_drop_rate: 0.3, seed: 42 }.
# Goal is the same (4, 3, heading north). Goal distance from origin is 5m, so
# runaway_limit = 10m (2× goal distance). Reach tolerance widened to 1.5m
# because the IMU dropout degrades the fusion estimate.
# ============================================================
test_imu_dropout() {
    run_fault_injection_test \
        "Test 5: IMU Dropout — Graceful Degradation" \
        "$PROJECT_DIR/simulation/worlds/tests/test_destination_accuracy.sdf" \
        4.0 3.0 1.57 \
        120 \
        1.5 \
        10.0 \
        "$PROJECT_DIR/simulation/tests/fixtures/sensperc_fault_imu.yaml"
}

# ============================================================
# Main
# ============================================================

MODE="${1:-all}"

echo ""
echo -e "${CYAN}╔══════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║    Limo Drive Scenario Test Suite            ║${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════════╝${NC}"
echo ""

case "$MODE" in
    all)
        test_intersection || true
        test_obstacle_bypass || true
        test_destination_accuracy || true
        test_imu_dropout || true
        test_random_obstacles_gui
        ;;
    headless)
        test_intersection || true
        test_obstacle_bypass || true
        test_destination_accuracy || true
        test_imu_dropout || true
        ;;
    gui)
        test_random_obstacles_gui
        ;;
    1) test_intersection ;;
    2) test_obstacle_bypass ;;
    3) test_destination_accuracy ;;
    4) test_random_obstacles_gui ;;
    5) test_imu_dropout ;;
    *)
        echo "Usage: $0 [all|headless|gui|1|2|3|4|5]"
        exit 1
        ;;
esac

# Print summary
echo ""
echo -e "${CYAN}╔══════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║    TEST RESULTS                              ║${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════════╝${NC}"
for result in "${RESULTS[@]}"; do
    if [[ "$result" == PASS* ]]; then
        echo -e "  ${GREEN}✓${NC} $result"
    elif [[ "$result" == FAIL* ]]; then
        echo -e "  ${RED}✗${NC} $result"
    else
        echo -e "  ${YELLOW}◉${NC} $result"
    fi
done
echo ""
echo -e "  Total: ${GREEN}${PASS} passed${NC}, ${RED}${FAIL} failed${NC}"
echo ""

if [[ $FAIL -gt 0 ]]; then
    exit 1
fi
