FROM tiryoh/ros2-desktop-vnc:humble

USER root

# Install dependencies for serial bridge, MoveIt2, GUI controls, and robot tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    socat \
    python3-pip \
    git \
    autocutsel \
    xclip \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-moveit \
    && rm -rf /var/lib/apt/lists/*

# Install official python API library
RUN pip3 install --no-cache-dir pymycobot --upgrade

# Clone the mycobot_ros2 repository
WORKDIR /opt/ros2_ws
RUN mkdir -p /opt/ros2_ws/src && \
    cd /opt/ros2_ws/src && \
    git clone -b humble --depth 1 https://github.com/elephantrobotics/mycobot_ros2.git

# Copy custom nodes, launch files, and configuration
COPY custom_nodes/setup.py /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/setup.py
COPY custom_nodes/teleop_dualsense.py /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/mycobot_280/teleop_dualsense.py
COPY custom_nodes/listen_real_service.py /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/mycobot_280/listen_real_service.py
COPY custom_nodes/camera_publisher.py /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/mycobot_280/camera_publisher.py
COPY custom_nodes/detect_objects.py /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/mycobot_280/detect_objects.py
COPY custom_nodes/color_pick_and_place.py /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/mycobot_280/color_pick_and_place.py
COPY custom_nodes/color_pick_and_place.launch.py /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/launch/color_pick_and_place.launch.py
COPY custom_nodes/teleop_dualsense.launch.py /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/launch/teleop_dualsense.launch.py
COPY custom_nodes/mycobot.rviz /opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/config/mycobot.rviz

# Compile workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-ignore ultraarm_p1_moveit2 ultraarm_p1_moveit2_control" && \
    chmod -R 777 /opt/ros2_ws

# Automatically source ROS2 and the myCobot workspace globally for all users
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /opt/ros2_ws/install/setup.bash" >> /etc/bash.bashrc

# Setup automatic background serial bridge in supervisor
COPY cobot.conf /etc/supervisor/conf.d/cobot.conf

WORKDIR /
