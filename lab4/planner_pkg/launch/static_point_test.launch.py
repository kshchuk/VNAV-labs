import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from os.path import join


def _default_simulator_data_directory() -> str:
    if os.environ.get("VNAV_LAB4_DATA"):
        return os.environ["VNAV_LAB4_DATA"]
    return os.path.join(get_package_share_directory("planner_pkg"), "data")


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="controller_pkg",
            executable="controller_node",
            name="controller_node",
            output="screen",
            parameters=[join(get_package_share_directory("controller_pkg"), "config", "params.yaml")],
        ),
        Node(
            package="planner_pkg",
            executable="traj_vertices_publisher",
            name="traj_vertices_publisher",
            output="screen",
            parameters=[{
                "simulator_data_directory": _default_simulator_data_directory(),
            }],
        ),
        Node(
            package="planner_pkg",
            executable="simple_traj_planner",
            name="simple_traj_planner",
            output="screen",
        ),
    ])
