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
    port_launch_arg = DeclareLaunchArgument(
        name="port",
        default_value="/dev/ttyUSB0",
        description="Serial port to use"
    )
    res.append(port_launch_arg)

    baud_launch_arg = DeclareLaunchArgument(
        name="baud",
        default_value="115200",
        description="Baud rate to use"
    )
    res.append(baud_launch_arg)

    gripper_launch_arg = DeclareLaunchArgument(
        name="gripper",
        default_value="true",
        description="Whether the robot arm has a claw / adaptive gripper attached"
    )
    res.append(gripper_launch_arg)

    default_arm_urdf = os.path.join(
        get_package_share_directory("mycobot_description"),
        "urdf/mycobot_280_m5/mycobot_280_m5.urdf"
    )
    gripper_arm_urdf = os.path.join(
        get_package_share_directory("mycobot_description"),
        "urdf/mycobot_280_m5/mycobot_280_m5_adaptive_gripper.urdf"
    )

    model_path = PythonExpression([
        f"'{gripper_arm_urdf}' if '", LaunchConfiguration("gripper"), "'.lower() in ['true', '1'] else '{default_arm_urdf}'"
    ])

    model_launch_arg = DeclareLaunchArgument(
        name="model",
        default_value=model_path,
        description="URDF model file to use"
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mycobot_280"),
            "config/mycobot.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        arguments=[LaunchConfiguration("model")]
    )
    res.append(robot_state_publisher_node)

    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )
    res.append(rviz_node)

    listen_real_service_node = Node(
        package="mycobot_280",
        executable="listen_real_service",
        name="listen_real_service",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "baud": LaunchConfiguration("baud")
        }]
    )
    res.append(listen_real_service_node)

    teleop_dualsense_node = Node(
        package="mycobot_280",
        executable="teleop_dualsense",
        name="teleop_dualsense",
        output="screen"
    )
    res.append(teleop_dualsense_node)

    camera_launch_arg = DeclareLaunchArgument(
        name="camera",
        default_value="true",
        description="Whether to run the Mac camera publisher"
    )
    res.append(camera_launch_arg)

    camera_node = Node(
        package="mycobot_280",
        executable="camera_publisher",
        name="mac_camera_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("camera"))
    )
    res.append(camera_node)

    detect_launch_arg = DeclareLaunchArgument(
        name="detect",
        default_value="false",
        description="Whether to run vision object detection"
    )
    res.append(detect_launch_arg)

    mode_launch_arg = DeclareLaunchArgument(
        name="mode",
        default_value="ai",
        description="Vision detection mode: 'ai' (original YOLOv8 80 classes), 'block' (color sorting cubes), 'all' (hybrid)"
    )
    res.append(mode_launch_arg)

    detect_node = Node(
        package="mycobot_280",
        executable="detect_objects",
        name="detect_objects",
        output="screen",
        parameters=[{
            "mode": LaunchConfiguration("mode")
        }],
        condition=IfCondition(LaunchConfiguration("detect"))
    )
    res.append(detect_node)

    return LaunchDescription(res)
