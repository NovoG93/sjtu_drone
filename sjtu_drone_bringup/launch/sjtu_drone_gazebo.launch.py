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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_gui = DeclareLaunchArgument("use_gui", default_value="true", choices=["true", "false"],
                                    description="Whether to execute gzclient")
    xacro_file_name = "sjtu_drone.urdf.xacro"
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    xacro_file = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "urdf", xacro_file_name
    )
    yaml_file_path = os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config', 'drone.yaml'
    )   
    
    robot_description_config = xacro.process_file(xacro_file, mappings={"params_path": yaml_file_path})
    robot_desc = robot_description_config.toxml()
    # get ns from yaml
    model_ns = "drone"
    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict["namespace"] #+ "/"
    print("namespace: ", model_ns)


    world_file = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "worlds", "playground.world"
    )

    def launch_gzclient(context, *args, **kwargs):
        if context.launch_configurations.get('use_gui') == 'true':
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                ),
                launch_arguments={'verbose': 'true'}.items()
            )]
        return []

    return LaunchDescription([
        use_gui,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=model_ns,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc, "frame_prefix": model_ns + "/"}],
            arguments=[robot_desc]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=model_ns,
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file,
                              'verbose': "true",
                              'extra_gazebo_args': 'verbose'}.items()
        ),

        OpaqueFunction(function=launch_gzclient),

        Node(
            package="sjtu_drone_bringup",
            executable="spawn_drone",
            arguments=[robot_desc, model_ns],
            output="screen"
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", f"{model_ns}/odom"],
            output="screen"
        ),
    ])
