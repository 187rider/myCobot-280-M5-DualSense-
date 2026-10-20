#!/bin/bash
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

echo "[*] Syncing custom ROS 2 nodes into Docker container..."
COPYFILE_DISABLE=1 tar --no-xattrs -C custom_nodes -c . | docker exec -i mycobot_ros2 bash -c "
    mkdir -p /tmp/nodes_sync && tar -C /tmp/nodes_sync -x 2>/dev/null
    cp /tmp/nodes_sync/setup.py /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/setup.py
    cp /tmp/nodes_sync/*.launch.py /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/launch/
    cp /tmp/nodes_sync/*.py /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/mycobot_280/ 2>/dev/null || true
    cp /tmp/nodes_sync/mycobot.rviz /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/config/mycobot.rviz 2>/dev/null || true
    cp /tmp/nodes_sync/custom_positions.json /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/config/custom_positions.json 2>/dev/null || true
    chmod +x /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/mycobot_280/*.py
    rm -rf /tmp/nodes_sync
"

echo "[*] Compiling with symlink-install (Python edits become instant)..."
docker exec -i mycobot_ros2 bash -c "
    cd /opt/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --packages-select mycobot_280
    chown -R ubuntu:ubuntu /opt/ros2_ws /home/ubuntu/.bashrc
    chmod -R 777 /opt/ros2_ws
"

echo "[✓] Build complete! Ready to launch in Web Desktop."
