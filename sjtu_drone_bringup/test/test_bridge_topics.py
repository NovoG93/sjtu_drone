# Copyright 2024 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html

"""Verify bridge topic mappings are consistent with sensor names in the URDF xacro."""

import contextlib
import importlib.util
import os
import re
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml


# Locate source directories relative to this test file, so the test works
# regardless of whether colcon install overlays are sourced.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_BRINGUP_SRC = os.path.dirname(_THIS_DIR)
_DESCRIPTION_SRC = os.path.join(os.path.dirname(_BRINGUP_SRC), 'sjtu_drone_description')


@contextlib.contextmanager
def _patch_ament_index():
    """Temporarily monkey-patch ament_index so it resolves sjtu_drone_description."""
    import ament_index_python.packages as aip
    _orig_share = aip.get_package_share_directory
    _orig_prefix = aip.get_package_prefix

    def _patched_share(pkg):
        if pkg == 'sjtu_drone_description':
            return _DESCRIPTION_SRC
        if pkg == 'sjtu_drone_bringup':
            return _BRINGUP_SRC
        return _orig_share(pkg)

    def _patched_prefix(pkg):
        if pkg == 'sjtu_drone_description':
            return os.path.dirname(_DESCRIPTION_SRC)
        return _orig_prefix(pkg)

    aip.get_package_share_directory = _patched_share
    aip.get_package_prefix = _patched_prefix
    try:
        yield
    finally:
        aip.get_package_share_directory = _orig_share
        aip.get_package_prefix = _orig_prefix


@pytest.fixture
def drone_yaml():
    yaml_path = os.path.join(_BRINGUP_SRC, 'config', 'drone.yaml')
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)


@pytest.fixture
def xacro_sensor_names():
    """Parse sensor names from the URDF xacro."""
    xacro_file = os.path.join(_DESCRIPTION_SRC, 'urdf', 'sjtu_drone.urdf.xacro')
    yaml_path = os.path.join(_BRINGUP_SRC, 'config', 'drone.yaml')
    with _patch_ament_index():
        urdf_xml = xacro.process_file(xacro_file, mappings={'params_path': yaml_path}).toxml()
    root = ET.fromstring(urdf_xml)
    sensors = set()
    for sensor in root.iter('sensor'):
        name = sensor.get('name')
        if name:
            sensors.add(name)
    return sensors


@pytest.fixture
def launch_module():
    """Load the gazebo launch module directly from source."""
    launch_file = os.path.join(_BRINGUP_SRC, 'launch', 'sjtu_drone_gazebo.launch.py')
    spec = importlib.util.spec_from_file_location('sjtu_drone_gazebo_launch', launch_file)
    mod = importlib.util.module_from_spec(spec)
    with _patch_ament_index():
        spec.loader.exec_module(mod)
        ld = mod.generate_launch_description()
    return ld


def _resolve_substitution(sub):
    """Resolve a launch substitution or tuple of substitutions to a string."""
    from launch import LaunchContext
    ctx = LaunchContext()
    if isinstance(sub, tuple):
        return ''.join(s.perform(ctx) if hasattr(s, 'perform') else str(s) for s in sub)
    if hasattr(sub, 'perform'):
        return sub.perform(ctx)
    return str(sub)


def _extract_bridge_args(ld):
    """Extract bridge Node arguments from a LaunchDescription."""
    from launch_ros.actions import Node as LaunchNode
    for entity in ld.entities:
        if isinstance(entity, LaunchNode):
            pkg_resolved = str(getattr(entity, '_Node__package', ''))
            if 'ros_gz_bridge' in pkg_resolved:
                return getattr(entity, '_Node__arguments', [])
    return []


def test_bridge_sensor_names_match_xacro(xacro_sensor_names, launch_module):
    """Each bridged sensor topic should reference a sensor name present in the URDF."""
    bridge_args = _extract_bridge_args(launch_module)
    bridge_sensor_names = set()
    sensor_pattern = re.compile(r'/sensor/([^/]+)/')
    for arg in bridge_args:
        match = sensor_pattern.search(str(arg))
        if match:
            bridge_sensor_names.add(match.group(1))

    assert len(bridge_sensor_names) > 0, 'No sensor names found in bridge arguments'

    for sensor in bridge_sensor_names:
        assert sensor in xacro_sensor_names, (
            f'Bridge references sensor {sensor!r} which is not in the URDF xacro. '
            f'Available sensors: {xacro_sensor_names}'
        )


def test_expected_ros_topics_in_remappings(launch_module, drone_yaml):
    """Verify expected ROS topic names are present in bridge remappings."""
    ns = drone_yaml.get('namespace', 'drone')

    expected_topics = [
        f'{ns}/imu',
        f'{ns}/front/image_raw',
        f'{ns}/front/camera_info',
        f'{ns}/bottom/image_raw',
        f'{ns}/bottom/camera_info',
        f'{ns}/navsat',
        f'{ns}/sonar',
    ]

    from launch_ros.actions import Node as LaunchNode
    remap_targets = set()
    for entity in launch_module.entities:
        if isinstance(entity, LaunchNode):
            remappings = getattr(entity, '_Node__remappings', None)
            if remappings:
                for _src, dst in remappings:
                    remap_targets.add(_resolve_substitution(dst))

    for topic in expected_topics:
        assert topic in remap_targets, (
            f'Expected ROS topic {topic!r} not found in bridge remappings. '
            f'Remapped targets: {remap_targets}'
        )
