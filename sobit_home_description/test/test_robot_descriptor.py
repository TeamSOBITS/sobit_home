"""Checks config/sobit_home.robot.yaml against the URDF, controllers.yaml and gz sensors."""
from pathlib import Path

import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG / 'config' / 'sobit_home.robot.yaml'
CHECKED_IN_URDF = PKG / 'robots' / 'sobit_home_robot.urdf'
XACRO = PKG / 'robots' / 'sobit_home_robot.urdf.xacro'
CTRL_TYPES = {
    ('trajectory', 'position'): 'joint_trajectory_controller/JointTrajectoryController',
    ('group', 'position'): 'position_controllers/JointGroupPositionController',
    ('group', 'velocity'): 'velocity_controllers/JointGroupVelocityController',
}

try:
    import sobits_robot_descriptor as srd
except ImportError:
    sys.path.insert(0, str(PKG.parents[1] / 'sobits_robot_descriptor'))
    try:
        import sobits_robot_descriptor as srd
    except ImportError:
        pytest.skip('sobits_robot_descriptor not available', allow_module_level=True)


def _render(head_cam_type):
    """Return URDF xml text for the variant, or skip if it cannot be produced."""
    try:
        import xacro
        return xacro.process_file(
            str(XACRO), mappings={'head_cam_type': head_cam_type}).toxml()
    except Exception as exc:  # no xacro / no ament package index
        text = CHECKED_IN_URDF.read_text()
        if ('head_camera_IMU_frame' in text) == (head_cam_type == 'orbbec'):
            return text
        msg = (f'cannot render {head_cam_type} URDF ({exc!r}); '
               'checked-in URDF is the other variant')
        pytest.skip(msg)


def _controllers_yaml():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = (Path(get_package_share_directory('sobit_home_control')) /
                'config' / 'controllers.yaml')
    except Exception:
        path = PKG.parents[0] / 'sobit_home_control' / 'config' / 'controllers.yaml'
    if not path.is_file():
        pytest.skip('sobit_home_control not installed and not found in the source tree')
    return yaml.safe_load(path.read_text())


@pytest.fixture(params=['realsense', 'orbbec'])
def variant(request):
    return request.param


@pytest.fixture
def desc(variant):
    return srd.load_file(str(CONFIG), args={'head_cam_type': variant})


@pytest.fixture
def urdf(variant):
    return ET.fromstring(_render(variant).encode())


@pytest.fixture
def joints(urdf):
    return {j.get('name'): j for j in urdf.findall('joint')}


@pytest.fixture
def links(urdf):
    return {link.get('name') for link in urdf.findall('link')}


def test_validate_clean():
    data = yaml.safe_load(CONFIG.read_text())
    assert srd.validate(data) == []


def test_group_joints_movable(desc, joints):
    for g in desc.groups:
        for name in g.joints:
            assert name in joints, f'{g.name}: {name} not in URDF'
            assert joints[name].get('type') != 'fixed', f'{g.name}: {name} is fixed'


def test_uncommanded_joints_mimic(desc, joints):
    for g in desc.groups:
        for name in g.uncommanded_joints:
            assert name in joints, f'{g.name}: {name} not in URDF'
            assert joints[name].find('mimic') is not None, f'{name} has no <mimic>'


def test_base_controller_joints_exist(desc, joints):
    for c in desc.mobile_base.controllers:
        assert set(c.joints) <= set(joints), c.name


def test_frames_are_links(desc, links):
    frames = {desc.base_frame}
    frames |= {e.ee_link for e in desc.ee} | {e.reference_frame for e in desc.ee}
    for cam in desc.cameras:
        frames.add(cam.frame)
        frames |= {s.frame for s in (cam.color, cam.depth) if s}
    frames |= {lidar.frame for lidar in desc.lidars} | {i.frame for i in desc.imus}
    assert not frames - links, f'frames missing from URDF: {sorted(frames - links)}'


def test_imu_only_with_orbbec(desc, links, variant):
    assert ('head_camera_IMU_frame' in links) == (variant == 'orbbec')
    expected = ['head_camera_IMU_frame'] if variant == 'orbbec' else []
    assert [i.frame for i in desc.imus] == expected


def test_controllers_match_controllers_yaml(desc):
    ctrl = _controllers_yaml()
    manager = ctrl['/**/controller_manager']['ros__parameters']
    entries = list(desc.groups) + list(desc.mobile_base.controllers)
    for e in entries:
        key = f'/**/{e.controller}'
        assert key in ctrl, f'{e.name}: {e.controller} missing in controllers.yaml'
        assert set(ctrl[key]['ros__parameters']['joints']) == set(e.joints), e.name
        assert manager[e.controller]['type'] == CTRL_TYPES[(e.interface, e.kind)], e.name


def test_gz_sensors_match_descriptor(desc, urdf, links):
    ns = desc.namespace
    color = {f'{ns}/{c.color.raw_topic.rsplit("/", 1)[0]}': c.color.frame
             for c in desc.cameras if c.color}
    depth = {f'{ns}/{c.depth.raw_topic.rsplit("/", 1)[0]}'
             for c in desc.cameras if c.depth}
    scans = {f'{ns}/{lidar.scan_topic}': lidar.frame for lidar in desc.lidars}
    seen = set()
    for s in urdf.iter('sensor'):
        topic, frame = s.findtext('topic'), s.findtext('gz_frame_id')
        if s.get('type') == 'camera':
            assert color.get(topic) == frame, f'{s.get("name")}: {topic} / {frame}'
        elif s.get('type') == 'depth_camera':
            assert topic in depth and frame in links, f'{s.get("name")}: {topic} / {frame}'
        elif s.get('type') == 'gpu_lidar':
            assert scans.get(topic) == frame, f'{s.get("name")}: {topic} / {frame}'
        else:
            continue
        seen.add(topic)
    assert set(color) <= seen, f'colour streams without gz sensor: {sorted(set(color) - seen)}'
