import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():
    res = []

    # 1. Hardware Communication Arguments
    port_launch_arg = DeclareLaunchArgument(
        name="port",
        default_value="/dev/ttyUSB0",
        description="Serial port connected to myCobot"
    )
    res.append(port_launch_arg)

    baud_launch_arg = DeclareLaunchArgument(
        name="baud",
        default_value="115200",
        description="Baud rate for myCobot serial communication"
    )
    res.append(baud_launch_arg)

    gripper_launch_arg = DeclareLaunchArgument(
        name="gripper",
        default_value="true",
        description="Whether the robot arm has a claw / adaptive gripper attached"
    )
    res.append(gripper_launch_arg)

    auto_launch_arg = DeclareLaunchArgument(
        name="auto",
        default_value="false",
        description="Whether to immediately capture red cube on start"
    )
    res.append(auto_launch_arg)

    grip_z_launch_arg = DeclareLaunchArgument(
        name="grip_z",
        default_value="95.0",
        description="Grasp height (mm) above base for picking up cubes"
    )
    res.append(grip_z_launch_arg)

    invert_y_launch_arg = DeclareLaunchArgument(
        name="invert_y",
        default_value="true",
        description="Whether to invert the lateral Y tracking direction"
    )
    res.append(invert_y_launch_arg)

    hand_radius_launch_arg = DeclareLaunchArgument(
        name="hand_radius",
        default_value="60.0",
        description="Hand/gripper radius extension (mm) for safe tracking area"
    )
    res.append(hand_radius_launch_arg)

    # 2. URDF Model
    default_arm_urdf = os.path.join(
        get_package_share_directory("mycobot_description"),
        "urdf/mycobot_280_m5/mycobot_280_m5.urdf"
    )
    gripper_arm_urdf = os.path.join(
        get_package_share_directory("mycobot_description"),
        "urdf/mycobot_280_m5/mycobot_280_m5_adaptive_gripper.urdf"
    )

    model_path = PythonExpression([
        f"'{gripper_arm_urdf}' if '",
        LaunchConfiguration("gripper"),
        "' == 'true' else '",
        f"{default_arm_urdf}'"
    ])

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=model_path,
        description="Path to robot URDF model"
    )
    res.append(model_arg)

    robot_desc = ParameterValue(
        Command(['cat ', LaunchConfiguration('model')]),
        value_type=str
    )

    # 3. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}]
    )
    res.append(robot_state_publisher_node)

    # 4. RViz2
    rviz_config = os.path.join(
        get_package_share_directory("mycobot_280"),
        "config/mycobot.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )
    res.append(rviz_node)

    # 5. myCobot Driver Services Node
    service_node = Node(
        package="mycobot_280",
        executable="listen_real_service",
        name="listen_real_service",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "baud": LaunchConfiguration("baud")
        }]
    )
    res.append(service_node)

    # 6. Mac Camera Bridge Node
    camera_node = Node(
        package="mycobot_280",
        executable="camera_publisher",
        name="mac_camera_publisher",
        output="screen"
    )
    res.append(camera_node)

    # 7. Real-time Color Block Detection Node (Block Mode)
    detect_node = Node(
        package="mycobot_280",
        executable="detect_objects",
        name="detect_objects",
        output="screen",
        parameters=[{
            "mode": "block"
        }]
    )
    res.append(detect_node)

    # 8. Red Cube Capture Node
    capture_node = Node(
        package="mycobot_280",
        executable="capture_red_cube",
        name="capture_red_cube",
        output="screen",
        parameters=[{
            "auto": LaunchConfiguration("auto"),
            "grip_z": LaunchConfiguration("grip_z"),
            "invert_y": LaunchConfiguration("invert_y"),
            "hand_radius": LaunchConfiguration("hand_radius")
        }]
    )
    res.append(capture_node)

    # 9. DualSense Gamepad Teleoperation Node
    teleop_dualsense_node = Node(
        package="mycobot_280",
        executable="teleop_dualsense",
        name="teleop_dualsense",
        output="screen"
    )
    res.append(teleop_dualsense_node)

    return LaunchDescription(res)
