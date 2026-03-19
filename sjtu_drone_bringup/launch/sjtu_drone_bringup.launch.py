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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def get_teleop_controller(context, *_, **kwargs) -> Node:
    controller = context.launch_configurations['controller']
    namespace = kwargs['model_ns']

    if controller == 'joystick':
        node = Node(
            package='sjtu_drone_control',
            executable='teleop_joystick',
            namespace=namespace,
            output='screen',
        )

    else:
        node = Node(
            package='sjtu_drone_control',
            executable='teleop',
            namespace=namespace,
            output='screen',
            prefix='xterm -e',
        )

    return [node]


def rviz_node_generator(context, rviz_path):
    """Return a Node action for RViz, omitting --fixed-frame if empty."""
    fixed_frame_value = LaunchConfiguration('fixed_frame').perform(context)

    rviz_arguments = ['-d', rviz_path]

    if fixed_frame_value:
        rviz_arguments.extend(['--fixed-frame', fixed_frame_value])

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_arguments,
            output='screen',
        )
    ]


def generate_launch_description():
    sjtu_drone_bringup_path = get_package_share_directory('sjtu_drone_bringup')

    rviz_path = os.path.join(
        sjtu_drone_bringup_path, 'rviz', 'rviz.rviz'
    )

    yaml_file_path = os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config', 'drone.yaml'
    )

    model_ns = 'drone'

    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.safe_load(f)
        model_ns = yaml_dict['namespace']

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller',
            default_value='keyboard',
            description='Type of controller: keyboard (default) or joystick',
        ),

        DeclareLaunchArgument(
            'fixed_frame',
            default_value='',
            description='If provided, sets the fixed frame in RViz.'
        ),

        OpaqueFunction(
            function=rviz_node_generator,
            kwargs={'rviz_path': rviz_path},
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sjtu_drone_bringup_path, 'launch', 'sjtu_drone_gazebo.launch.py')
            )
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            namespace=model_ns,
            output='screen',
        ),

        OpaqueFunction(
            function=get_teleop_controller,
            kwargs={'model_ns': model_ns},
        ),
    ])
