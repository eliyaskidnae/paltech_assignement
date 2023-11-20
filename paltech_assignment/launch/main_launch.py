import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    pkg_dir = get_package_share_directory("paltech_assignment")

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_dir, "params", "paltech_assignment.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    get_waypoints_service_cmd = Node(
        package="paltech_assignment",
        name="waypoint_manager",
        executable="waypoint_manager",
        parameters=[params_file],
        emulate_tty=True,
        output="screen",
    )

    ld.add_action(declare_params_file_cmd)
    ld.add_action(get_waypoints_service_cmd)
    return ld
