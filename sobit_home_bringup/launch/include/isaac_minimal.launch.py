import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name',                 default_value='sobit_home'),
        DeclareLaunchArgument('robot_id',                   default_value='0'),
        DeclareLaunchArgument('world_model',                default_value='rcjo2026_arena',
                              description='World under <asset_root>/usd (e.g. rcjo2026_arena, rcw2026_arena, '
                                          'precomp2025_arena, empty) or an absolute .usd/.usda path'),
        DeclareLaunchArgument('world_closed',               default_value='false',
                              description='Load the <world_model>_closed variant (walls + ceiling + lights)'),
        DeclareLaunchArgument('robot_coords_x',             default_value='-6.0'),
        DeclareLaunchArgument('robot_coords_y',             default_value='1.5'),
        DeclareLaunchArgument('robot_coords_z',             default_value='0.0'),
        DeclareLaunchArgument('robot_coords_Y',             default_value='0.0'),
        # Isaac runs on the HOST and opens these paths itself; the README's ~/colcon_ws symlink
        # on the host makes the container's ~/colcon_ws/src/... paths resolve there too.
        DeclareLaunchArgument('asset_root',                 default_value='',
                              description='Directory holding usd/. Empty = $SOBITS_SIM_ASSET_ROOT, '
                                          'else the sobits_gazebo_worlds source export/ directory'),
        DeclareLaunchArgument('robot_usd',                  default_value='',
                              description='Robot USD. Empty = <asset_root>/usd/robots/<robot_name>/<robot_name>.usd'),
        DeclareLaunchArgument('spawn_only',                 default_value='false',
                              description='Skip load-world (the world is already open in the Isaac GUI)'),
        DeclareLaunchArgument('wait_timeout',               default_value='120',
                              description='Seconds to wait for the Isaac simulation_interfaces services'),
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


def _asset_root(context):
    """Resolve the directory holding usd/; export/ is not installed, so find the source package."""
    root = LaunchConfiguration('asset_root').perform(context) or os.environ.get('SOBITS_SIM_ASSET_ROOT', '')
    if not root:
        # symlink-install links package.xml back to the source package
        pkg_xml = os.path.join(get_package_share_directory('sobits_gazebo_worlds'), 'package.xml')
        root = os.path.join(os.path.dirname(os.path.realpath(pkg_xml)), 'export')
    root = os.path.abspath(os.path.expanduser(root))
    if not os.path.isdir(os.path.join(root, 'usd')):
        raise RuntimeError(
            f"No usd/ directory under '{root}'. Pass asset_root:=<dir containing usd/> "
            'or set SOBITS_SIM_ASSET_ROOT.')
    return root


def _graphs_off(context):
    """Sensor graphs of the robot USD to deactivate for the disabled enable_* flags (Isaac then neither
    renders nor publishes them, like Gazebo without the sensor). Graph names follow the descriptor sensors."""
    def off(name):
        return _bool(LaunchConfiguration(name), context) == 'False'
    graphs = []
    if off('enable_head_cam_color') and off('enable_head_cam_depth'):
        graphs.append('ROS2_Camera_head_camera')
    elif off('enable_head_cam_color'):
        graphs += ['ROS2_Camera_head_camera/' + n for n in ('HelperRGB', 'HelperCompressed', 'InfoRGB')]
    elif off('enable_head_cam_depth'):
        graphs += ['ROS2_Camera_head_camera/' + n for n in ('HelperDepth', 'HelperPCL', 'InfoDepth')]
    for side in ('left', 'right'):
        if off(f'enable_hand_{side}_cam_color'):
            graphs.append(f'ROS2_Camera_hand_{side}_camera')
    if off('enable_lidar'):
        graphs += ['ROS2_Lidar_lidar_front', 'ROS2_Lidar_lidar_back']
    return graphs


def _sim_control(*args):
    return ExecuteProcess(
        cmd=['ros2', 'run', 'sobits_gazebo_worlds', 'isaac_sim_control.py', *args],
        output='screen',
    )


def _then(target, next_actions, step):
    """Run next_actions after target exits 0; on any other exit stop the whole launch."""
    def on_exit(event, context):
        if event.returncode == 0:
            return next_actions
        return [
            LogInfo(msg=f'[isaac_minimal] {step} failed (exit {event.returncode}); '
                        'is Isaac Sim running with simulation_interfaces? Stopping.'),
            EmitEvent(event=Shutdown(reason=f'isaac_sim_control {step} failed')),
        ]
    return RegisterEventHandler(OnProcessExit(target_action=target, on_exit=on_exit))


def launch_setup(context, *args, **kwargs):
    robot_name   = LaunchConfiguration('robot_name').perform(context)
    robot_id     = int(LaunchConfiguration('robot_id').perform(context))
    world_model  = LaunchConfiguration('world_model').perform(context)
    robot_usd    = LaunchConfiguration('robot_usd').perform(context)
    spawn_only   = _bool(LaunchConfiguration('spawn_only'), context) == 'True'
    wait_timeout = LaunchConfiguration('wait_timeout').perform(context)

    effective_robot_name = robot_name if robot_id == 0 else f'{robot_name}_{robot_id}'
    asset_root = _asset_root(context)

    if os.path.isabs(world_model):
        world_path = world_model
    else:
        suffix = '_closed' if _bool(LaunchConfiguration('world_closed'), context) == 'True' else ''
        world_path = os.path.join(asset_root, 'usd', f'{world_model}{suffix}.usda')
    if not robot_usd:
        robot_usd = os.path.join(asset_root, 'usd', 'robots', robot_name, f'{robot_name}.usd')
    for path in ([] if spawn_only else [world_path]) + [robot_usd]:
        if not os.path.isfile(path):
            raise RuntimeError(f"Isaac asset not found: '{path}'")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sobit_home_bringup'), 'launch', 'robot.launch.py'])
        ]),
        launch_arguments={
            'robot_name'                  : effective_robot_name,
            'simulator'                   : 'isaac',
            'robot_coords_x'              : LaunchConfiguration('robot_coords_x').perform(context),
            'robot_coords_y'              : LaunchConfiguration('robot_coords_y').perform(context),
            'robot_coords_z'              : LaunchConfiguration('robot_coords_z').perform(context),
            'robot_coords_Y'              : LaunchConfiguration('robot_coords_Y').perform(context),
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

    # wait → load-world → stop → spawn → play → ROS side; each step only on a clean exit of the previous.
    # The world is always reloaded: a deleted robot leaves its sensor writers and controller_manager
    # behind in Isaac, so a fresh stage is the only clean way to respawn (spawn_only skips it).
    wait = _sim_control('wait', '--timeout', wait_timeout)
    load_world = _sim_control('load-world', world_path)
    stop = _sim_control('state', 'stop')
    graphs_off = _sim_control('graphs-off', *_graphs_off(context))
    spawn = _sim_control(
        'spawn', effective_robot_name, robot_usd,
        '--pose',
        LaunchConfiguration('robot_coords_x').perform(context),
        LaunchConfiguration('robot_coords_y').perform(context),
        LaunchConfiguration('robot_coords_z').perform(context),
        LaunchConfiguration('robot_coords_Y').perform(context),
        '--allow-renaming')
    play = _sim_control('state', 'play')

    actions = [wait]
    if spawn_only:
        actions.append(_then(wait, [stop], 'wait'))
    else:
        actions.append(_then(wait, [load_world], 'wait'))
        actions.append(_then(load_world, [stop], 'load-world'))
    actions.append(_then(stop, [graphs_off], 'state stop'))
    actions.append(_then(graphs_off, [spawn], 'graphs-off'))
    actions.append(_then(spawn, [play], 'spawn'))
    actions.append(_then(play, [robot] + _viewer(context, 'sobit_home'), 'state play'))
    return actions
