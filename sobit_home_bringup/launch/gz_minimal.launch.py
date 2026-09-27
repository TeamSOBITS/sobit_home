from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name',                 default_value='sobit_home'),
        DeclareLaunchArgument('robot_id',                   default_value='0'),
        DeclareLaunchArgument('world_model',                default_value='rcjo2026_arena',
                              description='empty | wrs | small_house | precomp2025_arena | rcjo2025_arena | rcjo2026_arena | rcw2026_arena'),
        DeclareLaunchArgument('world_closed',               default_value='false',
                              description='Closed environment: 2.5 m walls + solid ceiling + '
                                          'per-room lights. The sobits_gazebo_worlds arenas '
                                          'implement it; ignored by every other world.'),
        DeclareLaunchArgument('robot_coords_x',             default_value='-6.0'),
        DeclareLaunchArgument('robot_coords_y',             default_value='1.5'),
        DeclareLaunchArgument('robot_coords_z',             default_value='0.0'),
        DeclareLaunchArgument('robot_coords_Y',             default_value='0.0'),
        DeclareLaunchArgument('enable_viz',                 default_value='',
                              description='Viewer to start: rerun, rviz, foxglove, or empty for none'),
        DeclareLaunchArgument('enable_teleop',              default_value='false'),
        DeclareLaunchArgument('enable_gz',                  default_value='true'),
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
        DeclareLaunchArgument('head_cam_type',              default_value='realsense',
                              description='Head RGB-D camera model in the URDF: orbbec (Gemini 336L) | realsense (D415)'),
        DeclareLaunchArgument('enable_lidar',               default_value='true'),
        DeclareLaunchArgument('enable_display',             default_value='false'),
        DeclareLaunchArgument('enable_moveit',              default_value='true'),
        DeclareLaunchArgument('enable_moveit_rviz',         default_value='false',
                              description="MoveIt's own planning-scene RViz"),
        DeclareLaunchArgument('enable_tf_prefix',           default_value='false'),
        DeclareLaunchArgument('headless',                   default_value='false',
                              description='Run Gazebo in headless mode (--headless-rendering). '
                                          'Saves GPU memory when the GUI is not needed.'),
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
    package = f'sobits_viz_{choice}'
    launch_file = {'rerun': 'rerun', 'rviz': 'rviz', 'foxglove': 'foxglove'}.get(choice)
    if launch_file is None:
        raise RuntimeError(
            f"enable_viz must be rerun, rviz, foxglove or empty, not '{choice}'")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare(package), 'launch', f'{launch_file}.launch.py'])),
        launch_arguments={
            'robot_name': robot_name,
            'use_sim_time': 'true',
            'enable_tf_prefix': _bool(
                LaunchConfiguration('enable_tf_prefix'), context),
        }.items(),
    )]


def launch_setup(context, *args, **kwargs):
    robot_name  = LaunchConfiguration('robot_name').perform(context)
    robot_id    = int(LaunchConfiguration('robot_id').perform(context))
    world_model = LaunchConfiguration('world_model').perform(context)
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[gz.msgs.Clock",
            "/tf"    + "@tf2_msgs/msg/TFMessage"  + "[gz.msgs.Pose_V",
        ],
        output='screen',
    )


    effective_robot_name = robot_name if robot_id == 0 else f'{robot_name}_{robot_id}'

    # World resolution, xacro expansion and /world/<name>/* service bridge live there
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sobits_gazebo_worlds'),
                                  'launch', 'world.launch.py'])
        ]),
        launch_arguments={
            'world': world_model,
            'closed': _bool(LaunchConfiguration('world_closed'), context),
            'headless': _bool(LaunchConfiguration('headless'), context),
        }.items(),
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sobit_home_bringup'), 'launch', 'robot.launch.py'])
        ]),
        launch_arguments={
            'robot_name'                  : effective_robot_name,
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
            'enable_gz'                   : _bool(LaunchConfiguration('enable_gz'), context),
            'enable_moveit'               : _bool(LaunchConfiguration('enable_moveit'), context),
            'enable_moveit_rviz'          : _bool(LaunchConfiguration('enable_moveit_rviz'), context),
            'enable_tf_prefix'            : _bool(LaunchConfiguration('enable_tf_prefix'), context),
        }.items(),
    )

    return [gz_sim, gz_bridge_node, robot] + _viewer(context, 'sobit_home')
