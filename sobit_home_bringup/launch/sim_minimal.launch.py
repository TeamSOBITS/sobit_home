import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SIMULATORS = ('gz', 'isaac', 'mujoco')

# Union of the <sim>_minimal arguments; defaults equal theirs, since every one a launcher declares is forwarded
ARGUMENTS = [
    ('robot_name',                  'sobit_home',     'Robot model (and namespace when robot_id is 0)'),
    ('robot_id',                    '0',              'Non-zero appends _<id> to the namespace (multi-robot)'),
    ('world_model',                 'rcjo2026_arena', 'World name (e.g. rcjo2026_arena, rcw2026_arena, precomp2025_arena, empty)'),
    ('world_closed',                'false',          'Closed variant of the arena: 2.5 m walls + ceiling + per-room lights'),
    ('robot_coords_x',              '-6.0',           'Spawn x [m]'),
    ('robot_coords_y',              '1.5',            'Spawn y [m]'),
    ('robot_coords_z',              '0.0',            'Spawn z [m]'),
    ('robot_coords_Y',              '0.0',            'Spawn yaw [rad]'),
    ('headless',                    'false',          'gz / mujoco: no GUI (gz headless rendering, no MuJoCo Simulate window)'),
    ('asset_root',                  '',               'isaac / mujoco: dir holding usd/ and mjcf/. Empty = $SOBITS_SIM_ASSET_ROOT or sobits_gazebo_worlds export/'),
    ('robot_usd',                   '',               'isaac: robot USD. Empty = <asset_root>/usd/robots/<robot_name>/<robot_name>.usd'),
    ('spawn_only',                  'false',          'isaac: skip load-world (world already open in the Isaac GUI)'),
    ('wait_timeout',                '120',            'isaac: seconds to wait for the simulation_interfaces services'),
    ('enable_viz',                  '',               'Viewer to start: rerun, rviz, foxglove, or empty for none'),
    ('enable_teleop',               'false',          'Start the teleop node'),
    ('enable_mobile_base',          'true',           'Module: swerve base (+ lidars)'),
    ('enable_body',                 'true',           'Module: body lift'),
    ('enable_arm_left',             'true',           'Module: left arm (+ left hand camera)'),
    ('enable_arm_right',            'true',           'Module: right arm (+ right hand camera)'),
    ('enable_hand_left',            'true',           'Module: left hand'),
    ('enable_hand_right',           'true',           'Module: right hand'),
    ('enable_head',                 'true',           'Module: pan-tilt head (+ head camera)'),
    ('enable_head_cam_color',       'true',           'Sensor: head camera color stream'),
    ('enable_head_cam_depth',       'true',           'Sensor: head camera depth stream (+ points)'),
    ('enable_hand_left_cam_color',  'true',           'Sensor: left hand camera'),
    ('enable_hand_right_cam_color', 'true',           'Sensor: right hand camera'),
    ('head_cam_type',               'orbbec',         'Head RGB-D camera model in the URDF: orbbec (Gemini 336L) | realsense (D415)'),
    ('enable_lidar',                'true',           'Sensor: front + back lidars and the scan merger'),
    ('enable_display',              'false',          'Start sobits_display'),
    ('enable_moveit',               'true',           'Start MoveIt'),
    ('enable_moveit_rviz',          'false',          "MoveIt's own planning-scene RViz"),
    ('enable_tf_prefix',            'false',          'Prefix TF frames with <robot_name>/'),
]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulator', default_value='gz', choices=list(SIMULATORS),
                              description='Simulator to run: gz | isaac | mujoco'),
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in ARGUMENTS],
        OpaqueFunction(function=launch_setup),
    ])


def _declared(path):
    """Argument names the launch file declares, read from its own generate_launch_description()."""
    spec = importlib.util.spec_from_file_location(os.path.basename(path).split('.')[0], path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return {arg.name for arg in module.generate_launch_description().get_launch_arguments()}


def launch_setup(context, *args, **kwargs):
    simulator = LaunchConfiguration('simulator').perform(context).strip().lower()
    path = os.path.join(get_package_share_directory('sobit_home_bringup'),
                        'launch', f'{simulator}_minimal.launch.py')
    declared = _declared(path)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=[(name, LaunchConfiguration(name).perform(context))
                          for name, _, _ in ARGUMENTS if name in declared],
    )]
