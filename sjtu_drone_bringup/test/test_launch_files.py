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
import pytest


@pytest.fixture
def bringup_share():
    return get_package_share_directory('sjtu_drone_bringup')


def test_gazebo_launch_file_exists(bringup_share):
    launch_file = os.path.join(bringup_share, 'launch', 'sjtu_drone_gazebo.launch.py')
    assert os.path.exists(launch_file), f'Missing launch file: {launch_file}'


def test_bringup_launch_file_exists(bringup_share):
    launch_file = os.path.join(bringup_share, 'launch', 'sjtu_drone_bringup.launch.py')
    assert os.path.exists(launch_file), f'Missing launch file: {launch_file}'


def test_gazebo_launch_generates_description(bringup_share):
    """Verify the gazebo launch file can be loaded and produces a LaunchDescription."""
    import importlib.util
    import ament_index_python.packages as aip

    _this_dir = os.path.dirname(os.path.abspath(__file__))
    _bringup_src = os.path.dirname(_this_dir)
    _description_src = os.path.join(os.path.dirname(_bringup_src), 'sjtu_drone_description')

    _orig_share = aip.get_package_share_directory
    _orig_prefix = aip.get_package_prefix

    def _patched_share(pkg):
        if pkg == 'sjtu_drone_description':
            return _description_src
        return _orig_share(pkg)

    def _patched_prefix(pkg):
        if pkg == 'sjtu_drone_description':
            return os.path.dirname(_description_src)
        return _orig_prefix(pkg)

    aip.get_package_share_directory = _patched_share
    aip.get_package_prefix = _patched_prefix

    try:
        launch_file = os.path.join(bringup_share, 'launch', 'sjtu_drone_gazebo.launch.py')
        spec = importlib.util.spec_from_file_location('sjtu_drone_gazebo_launch', launch_file)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)

        from launch import LaunchDescription
        ld = mod.generate_launch_description()
        assert isinstance(ld, LaunchDescription)
        # Should contain at least: gz_sim_launch, robot_state_publisher, joint_state_publisher,
        # spawn_drone, bridge, static_transform_publisher  (+ env vars + world arg)
        entities = ld.entities
        assert len(entities) >= 6, f'Expected at least 6 launch entities, got {len(entities)}'
    finally:
        aip.get_package_share_directory = _orig_share
        aip.get_package_prefix = _orig_prefix
