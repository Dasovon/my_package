#!/usr/bin/env bash
#
# start_session.sh — Pull latest code, build, and launch on both machines.
# Run this from the dev machine.
#
# Usage:
#   ./scripts/start_session.sh              # Pull, build, and launch everything
#   ./scripts/start_session.sh --pull-only  # Pull and build only (no launch)
#
set -euo pipefail

ROBOT_HOST="hoverbot"
ROBOT_USER="ryan"
ROBOT_WS="\$HOME/robot_ws"
ROBOT_PKG_DIR="\$HOME/robot_ws/src/my_package"

DEV_WS="$HOME/dev_ws"
DEV_PKG_DIR="$HOME/dev_ws/src/my_robot_bringup"

GIT_BRANCH="main"

PULL_ONLY=false
if [[ "${1:-}" == "--pull-only" ]]; then
    PULL_ONLY=true
fi

# --- Colors ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
fail()  { echo -e "${RED}[FAIL]${NC}  $*"; }

# -------------------------------------------------------------------
# 1. Verify robot is reachable
# -------------------------------------------------------------------
info "Checking connectivity to ${ROBOT_HOST}..."
if ! ping -c 1 -W 3 "$ROBOT_HOST" &>/dev/null; then
    fail "Cannot reach ${ROBOT_HOST}. Is the Pi powered on and on the network?"
    exit 1
fi
ok "Robot is reachable."

# -------------------------------------------------------------------
# 2. Pull and build on the robot
# -------------------------------------------------------------------
info "Pulling latest code on ${ROBOT_HOST}..."
ssh "${ROBOT_USER}@${ROBOT_HOST}" bash -l <<REMOTE_SCRIPT
set -euo pipefail
echo "--- Robot: pulling latest code ---"
cd ${ROBOT_PKG_DIR}
git fetch origin ${GIT_BRANCH}
git checkout ${GIT_BRANCH}
git pull origin ${GIT_BRANCH}

echo "--- Robot: building workspace ---"
cd ${ROBOT_WS}
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

echo "--- Robot: running unit tests ---"
colcon test --packages-select my_robot_bringup
colcon test-result --verbose || true

echo "--- Robot: verifying hardware ---"
python3 -c "import RPi.GPIO; print('GPIO: OK')" || echo "GPIO: FAILED"
ls /dev/ttyUSB0 &>/dev/null && echo "RPLIDAR: OK" || echo "RPLIDAR: not detected"

echo "--- Robot: ready ---"
REMOTE_SCRIPT
ok "Robot code updated and built."

# -------------------------------------------------------------------
# 3. Pull and build on dev machine
# -------------------------------------------------------------------
info "Pulling latest code on dev machine..."
cd "$DEV_PKG_DIR"
git fetch origin "$GIT_BRANCH"
git checkout "$GIT_BRANCH"
git pull origin "$GIT_BRANCH"
ok "Dev code updated."

info "Building dev workspace..."
cd "$DEV_WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ok "Dev workspace built."

info "Running unit tests on dev..."
colcon test --packages-select my_robot_bringup
colcon test-result --verbose || true

if $PULL_ONLY; then
    ok "Pull and build complete (--pull-only mode). Skipping launch."
    exit 0
fi

# -------------------------------------------------------------------
# 4. Launch robot nodes (in background SSH session)
# -------------------------------------------------------------------
info "Launching full robot stack on ${ROBOT_HOST}..."
echo "  This will run in a background SSH session."
echo "  To stop: ssh ${ROBOT_USER}@${ROBOT_HOST} 'pkill -f full_bringup'"
ssh -t "${ROBOT_USER}@${ROBOT_HOST}" bash -l -c "
    source /opt/ros/humble/setup.bash
    source ${ROBOT_WS}/install/setup.bash
    ros2 launch my_robot_bringup full_bringup.launch.py
" &
ROBOT_PID=$!
sleep 5

# -------------------------------------------------------------------
# 5. Launch RViz and teleop on dev machine
# -------------------------------------------------------------------
info "Launching RViz on dev machine..."
source /opt/ros/humble/setup.bash
source "$DEV_WS/install/setup.bash"

rviz2 &
sleep 3

info "Launching teleop keyboard..."
echo ""
echo "============================================"
echo "  Controls: W/S = forward/back, A/D = turn"
echo "  Space = stop, Q = quit"
echo "============================================"
echo ""
ros2 run my_robot_bringup teleop_keyboard

# -------------------------------------------------------------------
# 6. Cleanup on exit
# -------------------------------------------------------------------
info "Teleop exited. Stopping robot nodes..."
ssh "${ROBOT_USER}@${ROBOT_HOST}" "pkill -f full_bringup" 2>/dev/null || true
kill $ROBOT_PID 2>/dev/null || true
ok "Session ended."
