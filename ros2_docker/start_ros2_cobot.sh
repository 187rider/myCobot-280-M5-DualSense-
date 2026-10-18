#!/bin/bash
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

echo "=========================================================="
echo "      myCobot 280 M5 - ROS2 Humble Docker Launcher"
echo "=========================================================="

# 1. Start Docker Container if not running
if ! docker compose ps --services --filter "status=running" | grep -q "mycobot-ros2"; then
    echo "[+] Starting ROS2 Docker container..."
    docker compose up -d
else
    echo "[+] ROS2 Docker container is already running."
fi

echo ""
echo "----------------------------------------------------------"
echo "  Web Desktop: http://localhost:6080"
echo "  Username:    ubuntu"
echo "  Password:    ubuntu"
echo "----------------------------------------------------------"
echo ""

# 2. Start the Mac Host -> Docker Serial Bridge
echo "[+] Starting Serial Bridge for myCobot..."
python3 bridge_serial.py
