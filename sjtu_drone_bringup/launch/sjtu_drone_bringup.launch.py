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

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def get_teleop_controller(context, *_, **kwargs) -> Node:
    controller = context.launch_configurations["controller"]
    namespace = kwargs["model_ns"]
    
    if controller == "joystick":
        node = Node(
            package="sjtu_drone_control",
            executable="teleop_joystick",
            namespace=namespace,
            output="screen",
        )

    else:
        node = Node(
            package="sjtu_drone_control",
            executable="teleop",
            namespace=namespace,
            output="screen",
            prefix="xterm -e",
        )

    return [node]

def generate_launch_description():
    sjtu_drone_bringup_path = get_package_share_directory('sjtu_drone_bringup')

    rviz_path = os.path.join(
        sjtu_drone_bringup_path, "rviz", "rviz.rviz"
    )
    
    yaml_file_path = os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config', 'drone.yaml'
    )

    model_ns = "drone"

    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict["namespace"]

    return LaunchDescription([
        DeclareLaunchArgument(
            "controller",
            default_value="keyboard",
            description="Type of controller: keyboard (default) or joystick",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d", rviz_path
            ],
            output="screen",
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
