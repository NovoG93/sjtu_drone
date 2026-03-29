#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def _world_name_from_sdf(world_path):
    """Parse the <world name="..."> attribute from an SDF file."""
    try:
        tree = ET.parse(world_path)
        world_elem = tree.find('.//world')
        if world_elem is not None:
            return world_elem.get('name', 'default')
    except ET.ParseError:
        pass
    return 'default'


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    xacro_file_name = 'sjtu_drone.urdf.xacro'
    xacro_file = os.path.join(
        get_package_share_directory('sjtu_drone_description'),
        'urdf', xacro_file_name
    )
    yaml_file_path = os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config', 'drone.yaml'
    )

    robot_description_config = xacro.process_file(
        xacro_file, mappings={'params_path': yaml_file_path}
    )
    robot_desc = robot_description_config.toxml()

    # get ns from yaml
    model_ns = 'drone'
    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.safe_load(f)
        model_ns = yaml_dict['namespace']
    print('namespace: ', model_ns)

    world_file_default = os.path.join(
        get_package_share_directory('sjtu_drone_description'),
        'worlds', 'playground.world'
    )

    world = DeclareLaunchArgument(
        name='world',
        default_value=world_file_default,
        description='Full path to world file to load'
    )

    # World name used in gz-transport topic paths – must match <world name="...">
    # in the SDF world file. Parsed automatically from the world file.
    world_name = _world_name_from_sdf(world_file_default)
    model_name = 'sjtu_drone'
    # Collapsed link name after URDF→SDF fixed-joint merging
    link_name = 'base_footprint'

    # --- Environment variables for gz-sim plugin / model discovery ---
    gz_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.pathsep.join(filter(None, [
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            os.path.join(
                get_package_prefix('sjtu_drone_description'),
                'lib', 'sjtu_drone_description',
            ),
        ])),
    )

    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.pathsep.join(filter(None, [
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            os.path.join(
                get_package_share_directory('sjtu_drone_description'), 'models',
            ),
        ])),
    )

    # --- gz-sim (Gazebo Harmonic) via ros_gz_sim ---
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world', default=world_file_default)],
            'on_exit_shutdown': 'true',
            'extra_gzserver_args': '--verbose'
        }.items(),
    )

    # --- Spawn drone via ros_gz_sim create ---
    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-name', model_name,
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0.3',
        ],
    )

    # --- ros_gz_bridge: bridge gz-transport sensor topics to ROS 2 ---
    def _gz_sensor_topic(sensor, suffix):
        return (
            f'/world/{world_name}/model/{model_name}'
            f'/link/{link_name}/sensor/{sensor}/{suffix}'
        )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (required for use_sim_time)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # IMU
            _gz_sensor_topic('sensor_imu', 'imu')
            + '@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Front camera image
            _gz_sensor_topic('front_camera', 'image')
            + '@sensor_msgs/msg/Image[gz.msgs.Image',
            # Front camera info
            _gz_sensor_topic('front_camera', 'camera_info')
            + '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Bottom camera image
            _gz_sensor_topic('down_camera', 'image')
            + '@sensor_msgs/msg/Image[gz.msgs.Image',
            # Bottom camera info
            _gz_sensor_topic('down_camera', 'camera_info')
            + '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # NavSat
            _gz_sensor_topic('navsat_sensor', 'navsat')
            + '@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            # GPU Lidar (sonar)
            _gz_sensor_topic('sonar', 'scan')
            + '@sensor_msgs/msg/Range[gz.msgs.LaserScan',
        ],
        remappings=[
            (_gz_sensor_topic('sensor_imu', 'imu'), f'{model_ns}/imu'),
            (_gz_sensor_topic('front_camera', 'image'), f'{model_ns}/front/image_raw'),
            (_gz_sensor_topic('front_camera', 'camera_info'), f'{model_ns}/front/camera_info'),
            (_gz_sensor_topic('down_camera', 'image'), f'{model_ns}/bottom/image_raw'),
            (_gz_sensor_topic('down_camera', 'camera_info'), f'{model_ns}/bottom/camera_info'),
            (_gz_sensor_topic('navsat_sensor', 'navsat'), f'{model_ns}/navsat'),
            (_gz_sensor_topic('sonar', 'scan'), f'{model_ns}/sonar'),
        ],
        output='screen',
    )

    return LaunchDescription([
        world,

        gz_plugin_path,
        gz_resource_path,

        gz_sim_launch,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=model_ns,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix': model_ns + '/',
            }],
            arguments=[robot_desc],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=model_ns,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        spawn_drone,

        bridge,

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', f'{model_ns}/odom'],
            output='screen',
        ),
    ])
