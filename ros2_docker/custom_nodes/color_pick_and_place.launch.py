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

    # 2. Pick & Place Automation Arguments
    color_launch_arg = DeclareLaunchArgument(
        name="color",
        default_value="all",
        description="Target color to sort: 'all', 'red', 'green', 'blue', 'yellow'"
    )
    res.append(color_launch_arg)

    auto_launch_arg = DeclareLaunchArgument(
        name="auto",
        default_value="false",
        description="Whether to immediately start continuous autonomous sorting"
    )
    res.append(auto_launch_arg)

    grip_z_launch_arg = DeclareLaunchArgument(
        name="grip_z",
        default_value="95.0",
        description="Grasp height (mm) above base for picking up cubes"
    )
    res.append(grip_z_launch_arg)

    # 3. URDF Model
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

    # 4. Robot State Publisher
    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        arguments=[LaunchConfiguration("model")]
    )
    res.append(robot_state_publisher_node)

    # 5. RViz2 Visualization
    rviz_launch_arg = DeclareLaunchArgument(
        name="rviz",
        default_value="true",
        description="Whether to run RViz2"
    )
    res.append(rviz_launch_arg)

    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )
    res.append(rviz_node)

    # 6. myCobot Driver / Service Provider Node
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

    # 7. Mac Camera Bridge Node
    camera_node = Node(
        package="mycobot_280",
        executable="camera_publisher",
        name="mac_camera_publisher",
        output="screen"
    )
    res.append(camera_node)

    # 8. Real-time Color Block Detection Node (Block Mode)
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

    # 9. Color Pick and Place Node
    pick_place_node = Node(
        package="mycobot_280",
        executable="color_pick_and_place",
        name="color_pick_and_place",
        output="screen",
        parameters=[{
            "target_color": LaunchConfiguration("color"),
            "auto_mode": LaunchConfiguration("auto"),
            "grip_z": LaunchConfiguration("grip_z")
        }]
    )
    res.append(pick_place_node)

    # 10. PS5 DualSense Teleoperation Node (Allows jogging with sticks during calibration & testing)
    dualsense_launch_arg = DeclareLaunchArgument(
        name="dualsense",
        default_value="true",
        description="Whether to run the PS5 DualSense gamepad teleoperation node"
    )
    res.append(dualsense_launch_arg)

    teleop_dualsense_node = Node(
        package="mycobot_280",
        executable="teleop_dualsense",
        name="teleop_dualsense",
        output="screen",
        condition=IfCondition(LaunchConfiguration("dualsense"))
    )
    res.append(teleop_dualsense_node)

    return LaunchDescription(res)
