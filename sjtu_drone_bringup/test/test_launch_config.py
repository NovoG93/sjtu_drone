"""Tests for Phase 1 fixes: clock bridge, namespace slash, JSP use_sim_time."""
import importlib.util
import os

import ament_index_python.packages as aip
from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node as LaunchNode
import pytest
import yaml


_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_BRINGUP_SRC = os.path.dirname(_THIS_DIR)
_DESCRIPTION_SRC = os.path.join(os.path.dirname(_BRINGUP_SRC), 'sjtu_drone_description')

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


@pytest.fixture
def launch_description():
    """Load and return the LaunchDescription from sjtu_drone_gazebo.launch.py."""
    aip.get_package_share_directory = _patched_share
    aip.get_package_prefix = _patched_prefix
    try:
        launch_file = os.path.join(
            _BRINGUP_SRC, 'launch', 'sjtu_drone_gazebo.launch.py'
        )
        spec = importlib.util.spec_from_file_location(
            'sjtu_drone_gazebo_launch', launch_file
        )
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        ld = mod.generate_launch_description()
        assert isinstance(ld, LaunchDescription)
        return ld
    finally:
        aip.get_package_share_directory = _orig_share
        aip.get_package_prefix = _orig_prefix


def _find_nodes(ld):
    """Return all LaunchNode entities from a LaunchDescription."""
    return [e for e in ld.entities if isinstance(e, LaunchNode)]


def _resolve_cmd_text(cmd_list):
    """Extract plain text from a Node's cmd list of substitution tuples."""
    texts = []
    for entry in cmd_list:
        if isinstance(entry, list):
            for sub in entry:
                if isinstance(sub, TextSubstitution):
                    texts.append(sub.text)
                else:
                    texts.append(str(sub))
        else:
            texts.append(str(entry))
    return texts


# ---- Test: bridge node includes clock bridge argument ----

def test_bridge_has_clock_argument(launch_description):
    """The parameter_bridge node must include a /clock bridge argument."""
    nodes = _find_nodes(launch_description)
    bridge_nodes = [
        n for n in nodes
        if any('parameter_bridge' in str(c) for c in n.cmd)
    ]
    assert bridge_nodes, 'No parameter_bridge node found in launch description'
    bridge = bridge_nodes[0]

    # Resolve substitution objects to plain text
    args = _resolve_cmd_text(bridge.cmd)
    clock_args = [a for a in args if '/clock' in a and 'rosgraph_msgs' in a]
    assert clock_args, (
        'Bridge node arguments must contain a /clock bridge entry with rosgraph_msgs. '
        f'Got args: {args}'
    )


# ---- Test: JSP node has use_sim_time parameter ----

def test_jsp_has_use_sim_time(launch_description):
    """The joint_state_publisher node must have use_sim_time parameter."""
    nodes = _find_nodes(launch_description)
    jsp_nodes = [
        n for n in nodes
        if any('joint_state_publisher' in str(c) for c in n.cmd)
    ]
    assert jsp_nodes, 'No joint_state_publisher node found in launch description'
    jsp = jsp_nodes[0]

    # Check that the node was given parameters containing use_sim_time
    # LaunchNode stores parameters in _Node__parameters
    params = jsp._Node__parameters
    assert params is not None and len(params) > 0, (
        'joint_state_publisher node has no parameters set'
    )

    # Parameters may contain substitution objects as keys.
    # Flatten and check for 'use_sim_time' in resolved key text.
    found = False
    for p in params:
        if isinstance(p, dict):
            for key in p:
                # key may be a string or a tuple of TextSubstitution
                if isinstance(key, str) and key == 'use_sim_time':
                    found = True
                    break
                elif isinstance(key, tuple):
                    key_text = ''.join(
                        s.text if isinstance(s, TextSubstitution) else str(s)
                        for s in key
                    )
                    if key_text == 'use_sim_time':
                        found = True
                        break
        if found:
            break
    assert found, (
        f'joint_state_publisher parameters must include use_sim_time. Got: {params}'
    )


# ---- Test: drone.yaml namespace has no leading slash ----

def test_drone_yaml_namespace_no_leading_slash():
    """The drone.yaml namespace must NOT start with a slash."""
    yaml_path = os.path.join(_BRINGUP_SRC, 'config', 'drone.yaml')
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    ns = config['namespace']
    assert not ns.startswith('/'), (
        f"namespace must not have a leading slash, got: '{ns}'"
    )
