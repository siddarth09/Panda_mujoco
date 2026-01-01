#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory("panda_mujoco")

    teleop_config = os.path.join(
        pkg_share,
        "config",
        "so101_teleop.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([

        # ------------------------------------------------------------
        # Launch arguments
        # ------------------------------------------------------------
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock"
        ),

        # ------------------------------------------------------------
        # SO-101 → Panda Cartesian Teleop Node
        # ------------------------------------------------------------
        Node(
            package="panda_mujoco",
            executable="so101.py",
            name="so101_panda_teleop",
            output="screen",
            parameters=[
                teleop_config,
                {"use_sim_time": use_sim_time}
            ]
        ),

        # # ------------------------------------------------------------
        # # Panda IK Node (already implemented by you)
        # # ------------------------------------------------------------
        # Node(
        #     package="panda_mujoco",
        #     executable="panda_ik_node",
        #     name="panda_ik_node",
        #     output="screen",
        #     parameters=[
        #         {"use_sim_time": use_sim_time}
        #     ]
        # ),
    ])
