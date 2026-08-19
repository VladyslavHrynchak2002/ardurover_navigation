import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORKSPACE = os.environ.get("ARDUROVER_NAV_ROOT", "/home/developer/ardurover_navigation")


def generate_launch_description() -> LaunchDescription:
    path_file = DeclareLaunchArgument(
        "path_file",
        default_value=os.path.join(WORKSPACE, "paths", "example.path"),
    )
    output_file = DeclareLaunchArgument(
        "output_file",
        default_value=os.path.join(WORKSPACE, "paths", "score.txt"),
    )

    controller = Node(
        package="ardurover_nav",
        executable="trajectory_controller_node",
        name="trajectory_controller_node",
        output="screen",
        parameters=[{"path_file": LaunchConfiguration("path_file")}],
    )
    scorer = Node(
        package="ardurover_nav",
        executable="path_scorer_node",
        name="path_scorer_node",
        output="screen",
        parameters=[
            {
                "path_file": LaunchConfiguration("path_file"),
                "output_file": LaunchConfiguration("output_file"),
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(get_package_share_directory("ardurover_nav"), "rviz", "ugv.rviz"),
        ],
        output="screen",
    )

    return LaunchDescription([path_file, output_file, controller, scorer, rviz])
