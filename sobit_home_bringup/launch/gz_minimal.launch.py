import os

from ament_index_python.packages import get_package_share_directory

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
                              description='empty | wrs | small_house | rcjo2025_arena | rcjo2026_arena | simple_data_collection'),
        DeclareLaunchArgument('robot_coords_x',             default_value='-5.5'),
        DeclareLaunchArgument('robot_coords_y',             default_value='1.5'),
        DeclareLaunchArgument('robot_coords_z',             default_value='0.0'),
        DeclareLaunchArgument('robot_coords_Y',             default_value='0.0'),
        DeclareLaunchArgument('use_rviz',                   default_value='true'),
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
        DeclareLaunchArgument('enable_lidar',               default_value='true'),
        DeclareLaunchArgument('enable_display',             default_value='false'),
        DeclareLaunchArgument('headless',                   default_value='false',
                              description='Run Gazebo in headless mode (--headless-rendering). '
                                          'Saves GPU memory when the GUI is not needed.'),
        OpaqueFunction(function=launch_setup),
    ])


def _bool(lc, context):
    """Normalize CLI true/True/1 → 'True', false/False/0 → 'False' for xacro + robot.launch.py."""
    return 'True' if lc.perform(context).lower() in ('true', '1', 'yes') else 'False'


def launch_setup(context, *args, **kwargs):
    robot_name  = LaunchConfiguration('robot_name').perform(context)
    robot_id    = int(LaunchConfiguration('robot_id').perform(context))
    world_model = LaunchConfiguration('world_model').perform(context)
    headless    = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')

    # Resolve world file from world_model string
    if world_model == 'empty':
        world_file = os.path.join(
            get_package_share_directory('sobit_home_description'),
            'worlds', 'empty_w_physics.sdf')
    elif world_model == 'wrs':
        world_file = os.path.join(
            get_package_share_directory('tmc_wrs_gz_worlds'),
            'worlds', 'wrs2020.world.xacro')
    elif world_model == 'small_house':
        world_file = os.path.join(
            get_package_share_directory('aws_small_house_world'),
            'worlds', 'small_house.world')
    elif world_model == 'rcjo2025_arena':
        world_file = os.path.join(
            get_package_share_directory('sobits_gazebo_worlds'),
            'worlds', 'rcjo2025_arena.world.xacro')
    elif world_model == 'rcjo2026_arena':
        world_file = os.path.join(
            get_package_share_directory('sobits_gazebo_worlds'),
            'worlds', 'rcjo2026_arena.world.xacro')
    elif world_model == 'simple_data_collection':
        world_file = os.path.join(
            get_package_share_directory('sobits_gazebo_worlds'),
            'worlds', 'simple_data_collection.world.xacro')
    else:
        world_file = os.path.join(
            get_package_share_directory('sobit_home_description'),
            'worlds', 'empty_w_physics.sdf')

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[gz.msgs.Clock",
            "/tf"    + "@tf2_msgs/msg/TFMessage"  + "[gz.msgs.Pose_V",
        ],
        output='screen',
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('sobit_home_bringup'), 'rviz', 'sobit_home.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    effective_robot_name = robot_name if robot_id == 0 else f'{robot_name}_{robot_id}'

    headless_flag = ' --headless-rendering -s' if headless else ''
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': f'{headless_flag} -r -v 4 {world_file}'}.items(),
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
            'enable_lidar'                : _bool(LaunchConfiguration('enable_lidar'), context),
            'enable_display'              : _bool(LaunchConfiguration('enable_display'), context),
            'enable_teleop'               : _bool(LaunchConfiguration('enable_teleop'), context),
            'enable_gz'                   : _bool(LaunchConfiguration('enable_gz'), context),
        }.items(),
    )

    return [gz_sim, gz_bridge_node, robot, rviz_node]
