from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile


def generate_launch_description():

    pkg = FindPackageShare("panda_mujoco")

    # --- URDF / xacro (ros2_control + TF only) ---
    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution([pkg, "franka_emika_panda", "panda.urdf"]),
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # --- Controllers config ---
    controllers_yaml = ParameterFile(
        PathJoinSubstitution([pkg, "config", "controller.yaml"]),
        allow_substs=True,
    )

    # --- Robot State Publisher (TF only) ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    # --- MuJoCo + ros2_control backend ---
    ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            controllers_yaml,
            {
                "use_sim_time": True,
               
            },
        ],
    )

    # --- Controller spawners ---
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

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )
