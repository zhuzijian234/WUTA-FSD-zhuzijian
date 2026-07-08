from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("lidar_sim")
    default_config = PathJoinSubstitution(
        [package_share, "config", "lidar_simulator.yaml"]
    )
    default_track = PathJoinSubstitution(
        [package_share, "tracks", "trackdrive.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to lidar_simulator YAML parameter file.",
            ),
            DeclareLaunchArgument(
                "track_file",
                default_value=default_track,
                description="Path to track YAML file.",
            ),
            Node(
                package="lidar_sim",
                executable="lidar_simulator_node",
                name="lidar_simulator",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"track_file": LaunchConfiguration("track_file")},
                ],
            ),
        ]
    )
