#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

echo "=========================================================="
echo "    myCobot 280 M5 - ROS 2 + PS5 DualSense Controller"
echo "=========================================================="

# 1. Start Docker Container if not running
if ! docker compose ps --services --filter "status=running" | grep -q "mycobot-ros2"; then
    echo "[+] Starting ROS 2 Docker container..."
    docker compose up -d
else
    echo "[+] ROS 2 Docker container is running."
fi

VENV_PY="$DIR/../.venv/bin/python"
if [ ! -f "$VENV_PY" ]; then
    VENV_PY="python3"
fi

# 2. Start Serial Bridge in background
echo "[+] Starting Serial Bridge (Mac USB -> Docker /dev/ttyUSB0)..."
$VENV_PY bridge_serial.py &
BRIDGE_PID=$!

trap "echo -e '\nStopping services...'; kill $BRIDGE_PID 2>/dev/null; exit 0" INT TERM EXIT

sleep 1
echo ""
echo "----------------------------------------------------------"
echo "  Web Desktop: http://localhost:6080"
echo "  Run in Web Terminal:"
echo "    ros2 launch mycobot_280 teleop_dualsense.launch.py port:=/dev/ttyUSB0 baud:=115200"
echo "----------------------------------------------------------"
echo ""

# 3. Start DualSense Streamer (Foreground)
echo "[+] Starting DualSense Gamepad Streamer..."
$VENV_PY dualsense_to_ros2.py
