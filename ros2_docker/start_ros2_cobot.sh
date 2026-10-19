#!/bin/bash
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

echo "=========================================================="
echo "      myCobot 280 M5 - ROS2 Humble Docker Launcher"
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
