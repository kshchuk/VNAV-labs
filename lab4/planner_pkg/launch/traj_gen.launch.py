import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _default_simulator_data_directory() -> str:
    if os.environ.get("VNAV_LAB4_DATA"):
        return os.environ["VNAV_LAB4_DATA"]
    return os.path.join(get_package_share_directory("planner_pkg"), "data")


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="planner_pkg",
            executable="traj_vertices_publisher",
            name="traj_vertices_publisher",
            output="screen",
            parameters=[{
                "simulator_data_directory": _default_simulator_data_directory(),
            }],
        ),
    ])
