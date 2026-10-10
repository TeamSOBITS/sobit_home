import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name',                 default_value='sobit_home'),
        DeclareLaunchArgument('robot_id',                   default_value='0'),
        DeclareLaunchArgument('world_model',                default_value='rcjo2026_arena',
                              description='World under <asset_root>/mjcf (e.g. rcjo2026_arena, rcw2026_arena, '
                                          'precomp2025_arena, empty) or an absolute .xml path'),
        DeclareLaunchArgument('world_closed',               default_value='false',
                              description='Load the <world_model>_closed variant (walls + ceiling + lights)'),
        DeclareLaunchArgument('robot_coords_x',             default_value='-6.0'),
        DeclareLaunchArgument('robot_coords_y',             default_value='1.5'),
        DeclareLaunchArgument('robot_coords_z',             default_value='0.0'),
        DeclareLaunchArgument('robot_coords_Y',             default_value='0.0'),
        DeclareLaunchArgument('asset_root',                 default_value='',
                              description='Directory holding mjcf/. Empty = $SOBITS_SIM_ASSET_ROOT, '
                                          'else the sobits_gazebo_worlds source export/ directory'),
        DeclareLaunchArgument('headless',                   default_value='false',
                              description='Run MuJoCo without the Simulate window'),
        DeclareLaunchArgument('enable_viz',                 default_value='',
                              description='Viewer to start: rerun, rviz, foxglove, or empty for none'),
        DeclareLaunchArgument('enable_teleop',              default_value='false'),
        DeclareLaunchArgument('enable_mobile_base',         default_value='true'),
        DeclareLaunchArgument('enable_body',                default_value='true'),
        DeclareLaunchArgument('enable_arm_left',            default_value='true'),
        DeclareLaunchArgument('enable_arm_right',           default_value='true'),
        DeclareLaunchArgument('enable_hand_left',           default_value='true'),
        DeclareLaunchArgument('enable_hand_right',          default_value='true'),
        DeclareLaunchArgument('enable_head',                default_value='true'),
        DeclareLaunchArgument('enable_head_cam_color',      default_value='true'),
        DeclareLaunchArgument('enable_head_cam_depth',      default_value='true'),
        DeclareLaunchArgument('enable_hand_left_cam_color', default_value='true'),
        DeclareLaunchArgument('enable_hand_right_cam_color',default_value='true'),
        DeclareLaunchArgument('head_cam_type',              default_value='orbbec',
                              description='Head RGB-D camera model in the URDF: orbbec (Gemini 336L) | realsense (D415)'),
        DeclareLaunchArgument('enable_lidar',               default_value='true'),
        DeclareLaunchArgument('enable_display',             default_value='false'),
        DeclareLaunchArgument('enable_moveit',              default_value='true'),
        DeclareLaunchArgument('enable_moveit_rviz',         default_value='false',
                              description="MoveIt's own planning-scene RViz"),
        DeclareLaunchArgument('enable_tf_prefix',           default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])


def _bool(lc, context):
    """Normalize CLI true/True/1 → 'True', false/False/0 → 'False' for xacro + robot.launch.py."""
    return 'True' if lc.perform(context).lower() in ('true', '1', 'yes') else 'False'


def _viewer(context, robot_name):
    """Return the launch action for the chosen viewer, or nothing."""
    choice = LaunchConfiguration('enable_viz').perform(context).strip().lower()
    if not choice:
        return []
    if choice not in ('rerun', 'rviz', 'foxglove'):
        raise RuntimeError(
            f"enable_viz must be rerun, rviz, foxglove or empty, not '{choice}'")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare(f'sobits_viz_{choice}'), 'launch', f'{choice}.launch.py'])),
        launch_arguments={
            'robot_name': robot_name,
            'use_sim_time': 'true',
            'enable_tf_prefix': _bool(
                LaunchConfiguration('enable_tf_prefix'), context),
        }.items(),
    )]


def _worlds_src():
    """sobits_gazebo_worlds source dir: export/ and scripts/ are not installed, symlink-install
    links package.xml back to the source package."""
    pkg_xml = os.path.join(get_package_share_directory('sobits_gazebo_worlds'), 'package.xml')
    return os.path.dirname(os.path.realpath(pkg_xml))


def _asset_root(context):
    """Resolve the directory holding mjcf/ (same rule as isaac_minimal's usd/)."""
    root = LaunchConfiguration('asset_root').perform(context) or os.environ.get('SOBITS_SIM_ASSET_ROOT', '')
    if not root:
        root = os.path.join(_worlds_src(), 'export')
    root = os.path.abspath(os.path.expanduser(root))
    if not os.path.isdir(os.path.join(root, 'mjcf')):
        raise RuntimeError(
            f"No mjcf/ directory under '{root}'. Pass asset_root:=<dir containing mjcf/> "
            'or set SOBITS_SIM_ASSET_ROOT.')
    return root


def _build_scene(world_path, robot_xml, pose, out_path):
    """Merge world + robot MJCF into one scene file at launch time; returns its path."""
    script = os.path.join(_worlds_src(), 'scripts', 'mujoco_scene.py')
    cmd = ['python3', script, '--world', world_path, '--robot', robot_xml,
           '--pose', *pose, '--out', out_path]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
    except OSError as e:
        raise RuntimeError(f'[mujoco_minimal] cannot run {script}: {e}')
    if result.returncode != 0:
        raise RuntimeError(
            f'[mujoco_minimal] scene build failed (exit {result.returncode}): {" ".join(cmd)}\n'
            f'{result.stderr.strip() or result.stdout.strip()}')
    lines = result.stdout.strip().splitlines()
    scene = lines[-1].strip() if lines else out_path
    if not os.path.isfile(scene):
        raise RuntimeError(f"[mujoco_minimal] scene build reported '{scene}', which does not exist")
    return scene


def launch_setup(context, *args, **kwargs):
    robot_name  = LaunchConfiguration('robot_name').perform(context)
    robot_id    = int(LaunchConfiguration('robot_id').perform(context))
    world_model = LaunchConfiguration('world_model').perform(context)
    pose = [LaunchConfiguration(f'robot_coords_{k}').perform(context) for k in ('x', 'y', 'z', 'Y')]

    effective_robot_name = robot_name if robot_id == 0 else f'{robot_name}_{robot_id}'
    asset_root = _asset_root(context)

    if os.path.isabs(world_model):
        world_path = world_model
    else:
        suffix = '_closed' if _bool(LaunchConfiguration('world_closed'), context) == 'True' else ''
        world_path = os.path.join(asset_root, 'mjcf', world_model + suffix, f'{world_model}{suffix}.xml')
    robot_xml = os.path.join(asset_root, 'mjcf', 'robots', robot_name, f'{robot_name}.xml')
    for path in (world_path, robot_xml):
        if not os.path.isfile(path):
            raise RuntimeError(f"MuJoCo asset not found: '{path}'")
    scene = _build_scene(world_path, robot_xml, pose,
                         os.path.join(os.path.dirname(world_path), f'scene_{effective_robot_name}.xml'))

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sobit_home_bringup'), 'launch', 'robot.launch.py'])
        ]),
        launch_arguments={
            'robot_name'                  : effective_robot_name,
            'simulator'                   : 'mujoco',
            'mujoco_model'                : scene,
            'mujoco_headless'             : _bool(LaunchConfiguration('headless'), context),
            'robot_coords_x'              : pose[0],
            'robot_coords_y'              : pose[1],
            'robot_coords_z'              : pose[2],
            'robot_coords_Y'              : pose[3],
            'enable_mobile_base'          : _bool(LaunchConfiguration('enable_mobile_base'), context),
            'enable_body'                 : _bool(LaunchConfiguration('enable_body'), context),
            'enable_arm_left'             : _bool(LaunchConfiguration('enable_arm_left'), context),
            'enable_arm_right'            : _bool(LaunchConfiguration('enable_arm_right'), context),
            'enable_hand_left'            : _bool(LaunchConfiguration('enable_hand_left'), context),
            'enable_hand_right'           : _bool(LaunchConfiguration('enable_hand_right'), context),
            'enable_head'                 : _bool(LaunchConfiguration('enable_head'), context),
            'enable_head_cam_color'       : _bool(LaunchConfiguration('enable_head_cam_color'), context),
            'enable_head_cam_depth'       : _bool(LaunchConfiguration('enable_head_cam_depth'), context),
            'enable_hand_left_cam_color'  : _bool(LaunchConfiguration('enable_hand_left_cam_color'), context),
            'enable_hand_right_cam_color' : _bool(LaunchConfiguration('enable_hand_right_cam_color'), context),
            'head_cam_type'               : LaunchConfiguration('head_cam_type').perform(context),
            'enable_lidar'                : _bool(LaunchConfiguration('enable_lidar'), context),
            'enable_display'              : _bool(LaunchConfiguration('enable_display'), context),
            'enable_teleop'               : _bool(LaunchConfiguration('enable_teleop'), context),
            'enable_moveit'               : _bool(LaunchConfiguration('enable_moveit'), context),
            'enable_moveit_rviz'          : _bool(LaunchConfiguration('enable_moveit_rviz'), context),
            'enable_tf_prefix'            : _bool(LaunchConfiguration('enable_tf_prefix'), context),
        }.items(),
    )

    return [robot] + _viewer(context, 'sobit_home')
