import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from os.path import join


def _default_simulator_data_directory() -> str:
    if os.environ.get("VNAV_LAB4_DATA"):
        return os.environ["VNAV_LAB4_DATA"]
    return os.path.join(get_package_share_directory("planner_pkg"), "data")


def generate_launch_description():
    controller_share = get_package_share_directory("controller_pkg")
    gazebo_share = get_package_share_directory("gazebo_quadrotor_pkg")

    gui = LaunchConfiguration("gui")
    planner = LaunchConfiguration("planner")
    use_optimized = PythonExpression(["'", planner, "' == 'optimized'"])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument(
            "planner",
            default_value="simple",
            description="simple (Part 0) or optimized (polynomial trajectory)",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, "launch", "gazebo_quadrotor.launch.py")
            ),
            launch_arguments={
                "gui": gui,
                "use_sim_time": "true",
                "show_traj_vertices": "true",
                "show_reference_circle": "false",
            }.items(),
        ),

        Node(
            package="controller_pkg",
            executable="controller_node",
            name="controller_node",
            output="screen",
            parameters=[
                join(controller_share, "config", "params.yaml"),
                {"use_sim_time": True},
            ],
            remappings=[
                ("motor_speed", "rotor_speed_cmds"),
            ],
        ),

        Node(
            package="planner_pkg",
            executable="traj_vertices_publisher",
            name="traj_vertices_publisher",
            output="screen",
            parameters=[{
                "simulator_data_directory": _default_simulator_data_directory(),
                "use_sim_time": True,
            }],
        ),

        Node(
            package="planner_pkg",
            executable="simple_traj_planner",
            name="simple_traj_planner",
            output="screen",
            parameters=[{"use_sim_time": True}],
            condition=UnlessCondition(use_optimized),
        ),

        Node(
            package="trajectory_generation",
            executable="trajectory_generation_node",
            name="trajectory_generation_node",
            output="screen",
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(use_optimized),
        ),

        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                "port": 8765,
                "send_buffer_limit": 10000000,
            }],
        ),
    ])
