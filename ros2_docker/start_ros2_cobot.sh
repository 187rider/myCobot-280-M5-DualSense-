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

# Sync custom ROS 2 nodes into container
echo "[*] Syncing custom ROS 2 nodes into container..."
docker cp custom_nodes/setup.py mycobot_ros2:/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/setup.py
docker cp custom_nodes/teleop_dualsense.py mycobot_ros2:/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/mycobot_280/teleop_dualsense.py
docker cp custom_nodes/listen_real_service.py mycobot_ros2:/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/mycobot_280/listen_real_service.py
docker cp custom_nodes/camera_publisher.py mycobot_ros2:/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/mycobot_280/camera_publisher.py
docker cp custom_nodes/detect_objects.py mycobot_ros2:/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/mycobot_280/detect_objects.py
docker cp custom_nodes/color_pick_and_place.py mycobot_ros2:/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/mycobot_280/color_pick_and_place.py
docker cp custom_nodes/color_pick_and_place.launch.py mycobot_ros2:/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/launch/color_pick_and_place.launch.py
docker cp custom_nodes/teleop_dualsense.launch.py mycobot_ros2:/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/launch/teleop_dualsense.launch.py
docker cp custom_nodes/mycobot.rviz mycobot_ros2:/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/config/mycobot.rviz
docker exec mycobot_ros2 pip install --force-reinstall coloredlogs >/dev/null 2>&1 || true
docker exec mycobot_ros2 bash -c "cd /opt/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select mycobot_280 --symlink-install" >/dev/null 2>&1
echo "[+] Custom nodes ready."

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
