"""Every LiDAR preprocessing profile must be loadable and complete."""

import ast

from contracts import package_root
import yaml

REQUIRED = {'input_topic', 'target_frame', 'crop.minimum', 'crop.maximum',
            'ground.enabled', 'clustering.enabled', 'clustering.tolerance'}


def _declared_parameters():
    source = (package_root('perception_core') / 'src' / 'nodes'
              / 'pointcloud_preprocess_node.cpp').read_text()
    names = set()
    for marker in ('declare_parameter<', 'vectorParameter(*this, '):
        for chunk in source.split(marker)[1:]:
            start = chunk.index('"')
            names.add(chunk[start + 1:chunk.index('"', start + 1)])
    return names


def _profiles():
    directory = package_root('fusion_bringup') / 'config' / 'pointcloud_preprocess'
    profiles = sorted(directory.glob('*.yaml'))
    assert profiles, f'no profiles in {directory}'
    return profiles


def test_profiles_only_use_parameters_the_node_declares():
    """
    A typo in a profile would be silently ignored by ROS.

    Every key must be one the node declares, plus use_sim_time.
    """
    declared = _declared_parameters() | {'use_sim_time'}
    for path in _profiles():
        document = yaml.safe_load(path.read_text())
        parameters = document['pointcloud_preprocess']['ros__parameters']
        unknown = set(parameters) - declared
        assert not unknown, f'{path.name}: unknown parameters {sorted(unknown)}'
        missing = REQUIRED - set(parameters)
        assert not missing, f'{path.name}: missing {sorted(missing)}'
        assert len(parameters['crop.minimum']) == 3
        bounds = zip(parameters['crop.minimum'], parameters['crop.maximum'])
        assert all(lo < hi for lo, hi in bounds)
        if parameters['ground.enabled']:
            assert parameters['ground.min_height'] <= parameters['ground.max_height']


def test_launch_resolves_every_shipped_profile():
    """The launch file builds the profile path from the sensor_profile name."""
    launch = (package_root('fusion_bringup') / 'launch'
              / 'pointcloud_preprocess.launch.py').read_text()
    ast.parse(launch)
    assert "'config' / 'pointcloud_preprocess' / f'{profile}.yaml'" in launch
    names = {path.stem for path in _profiles()}
    assert {'kitti_hdl64', 'sim_planar', 'custom_lidar'} <= names
