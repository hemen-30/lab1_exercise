import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control = os.path.join(
        get_package_share_directory("diffdrive_bot"), "launch", "control.launch.py"
    )
    side_length = LaunchConfiguration("side_length")

    return LaunchDescription([
        DeclareLaunchArgument(
            "side_length", default_value="0.5", description="Side of the square, m"
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(control)),
        Node(package="my_robot_nodes", executable="rotate_server", output="screen"),
        Node(
            package="my_robot_nodes",
            executable="square_driver",
            output="screen",
            parameters=[
                {"side_length": ParameterValue(side_length, value_type=float)}
            ],
        ),
    ])
