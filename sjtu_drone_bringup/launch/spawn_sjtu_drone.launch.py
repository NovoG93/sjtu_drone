import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the urdf file
    urdf_file_name = "sjtu_drone.sdf"
    urdf = os.path.join(
        get_package_share_directory("sjtu_drone"),
        "models", "sjtu_drone", urdf_file_name
    )    

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', urdf,
            "-entity", "sjtu_drone",
            "-timeout", "90"
        ],
        output='screen',
    )

    ld = LaunchDescription()
    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld