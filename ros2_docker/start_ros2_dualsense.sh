#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

echo "=========================================================="
echo "    myCobot 280 M5 - ROS 2 + PS5 DualSense Controller"
echo "=========================================================="

# 1. Ensure Docker Desktop is running
if ! docker info >/dev/null 2>&1; then
    echo "[!] Docker Desktop is not running."
    if [ -d "/Applications/Docker.app" ]; then
        echo "[+] Launching Docker Desktop..."
        open -a Docker
        echo "[*] Waiting for Docker daemon to initialize..."
        while ! docker info >/dev/null 2>&1; do
            sleep 2
        done
        echo "[+] Docker Desktop is ready."
    else
        echo "[-] Docker is not installed or running. Please start Docker and retry."
        exit 1
    fi
fi

# 2. Start Docker Container if not running
if ! docker compose ps --services --filter "status=running" | grep -q "mycobot-ros2"; then
    echo "[+] Starting ROS 2 Docker container..."
    docker compose up -d
else
    echo "[+] ROS 2 Docker container is running."
fi

# Wait for noVNC Web Desktop (port 6080) to initialize
echo "[*] Waiting for Web Desktop to become ready..."
for i in {1..30}; do
    if curl -s -o /dev/null http://localhost:6080/ 2>/dev/null; then
        echo "[+] Web Desktop is ready at http://localhost:6080"
        break
    fi
    sleep 1
done

# Ensure custom nodes and onnxruntime are installed in container
bash "$DIR/sync_and_build.sh"

VENV_PY="$DIR/../.venv/bin/python"
if [ ! -f "$VENV_PY" ]; then
    VENV_PY="python3"
fi

# Clean up any lingering Python bridge processes and ports from previous runs
pkill -f "python.*(stream_camera|bridge_serial|dualsense_to_ros2)" 2>/dev/null || true
lsof -ti :5005 -ti :5006 -ti :5007 -ti :9998 | xargs kill -9 2>/dev/null || true
sleep 0.5

# 2. Start Serial Bridge in background
echo "[+] Starting Serial Bridge (Mac USB -> Docker /dev/ttyUSB0)..."
$VENV_PY bridge_serial.py &
BRIDGE_PID=$!

# 3. Start Mac Camera Streamer in background
echo "[+] Starting Camera Streamer (Mac Webcam -> Docker /camera/image_raw)..."
$VENV_PY stream_camera.py &
CAMERA_PID=$!

trap "echo -e '\nStopping services...'; kill $BRIDGE_PID $CAMERA_PID 2>/dev/null; exit 0" INT TERM EXIT

sleep 1
echo ""
echo "----------------------------------------------------------"
echo "  Web Desktop: http://localhost:6080/vnc.html?autoconnect=true&password=ubuntu"
echo "  (Alternative direct URL: http://localhost:6080 | Password: ubuntu)"
echo "  Run in Web Terminal:"
echo "    ros2 launch mycobot_280 teleop_dualsense.launch.py port:=/dev/ttyUSB0 baud:=115200"
echo "----------------------------------------------------------"
echo ""

# 4. Start DualSense Streamer (Foreground)
echo "[+] Starting DualSense Gamepad Streamer..."
$VENV_PY dualsense_to_ros2.py
