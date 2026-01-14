from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile


def generate_launch_description():

    pkg = FindPackageShare("panda_mujoco")

    # =========================================================
    # UR5 URDF (ros2_control + TF only)
    # =========================================================
    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution([pkg, "ur5", "ur5.urdf"]),
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    # =========================================================
    # Controllers config (UR5)
    # =========================================================
    controllers_yaml = ParameterFile(
        PathJoinSubstitution([pkg, "config", "ur5_controller.yaml"]),
        allow_substs=True,
    )

    # =========================================================
    # Robot State Publisher
    # =========================================================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    # =========================================================
    # MuJoCo + ros2_control backend
    # =========================================================
    ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            controllers_yaml,
            {"use_sim_time": True},
        ],
    )

    # =========================================================
    # Controller spawners
    # =========================================================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    # =========================================================
    # Static TFs (table + camera)
    # =========================================================
    static_table_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="table_static_tf",
        arguments=[
            "0.6", "0.0", "0.38",
            "0", "0", "0", "1",
            "world",
            "table_frame",
        ],
    )

    static_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="top_rgb_mount_tf",
        arguments=[
            "-0.02", "0.0", "0.52",
            "0", "0", "0", "1",
            "table_frame",
            "top_rgb_frame",
        ],
    )

    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="top_rgb_optical_tf",
        arguments=[
            "0", "0", "0",
            "1", "0", "0", "0",
            "top_rgb_frame",
            "top_rgb_optical_frame",
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            # static_table_tf,
            # static_camera_tf,
            # camera_optical_tf,
        ]
    )
