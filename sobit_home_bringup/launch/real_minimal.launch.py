from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def _bool(lc, context):
    """Normalize CLI true/True/1 → 'True', else → 'False' for xacro + robot.launch.py."""
    return 'True' if lc.perform(context).lower() in ('true', '1', 'yes') else 'False'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name',   default_value='sobit_home'),
        DeclareLaunchArgument('robot_id',     default_value='0'),
        DeclareLaunchArgument('use_rviz',     default_value='true'),
        DeclareLaunchArgument('enable_mobile_base',          default_value='true'),
        DeclareLaunchArgument('enable_body',                 default_value='true'),
        DeclareLaunchArgument('enable_arm_left',             default_value='true'),
        DeclareLaunchArgument('enable_arm_right',            default_value='true'),
        DeclareLaunchArgument('enable_hand_left',            default_value='true'),
        DeclareLaunchArgument('enable_hand_right',           default_value='true'),
        DeclareLaunchArgument('enable_head',                 default_value='true'),
        DeclareLaunchArgument('enable_head_cam_color',       default_value='true'),
        DeclareLaunchArgument('enable_head_cam_depth',       default_value='true'),
        DeclareLaunchArgument('enable_hand_left_cam_color',  default_value='true'),
        DeclareLaunchArgument('enable_hand_right_cam_color', default_value='true'),
        DeclareLaunchArgument('enable_lidar',                default_value='true'),
        DeclareLaunchArgument('enable_display',              default_value='true'),
        DeclareLaunchArgument('enable_teleop',               default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_id   = int(LaunchConfiguration('robot_id').perform(context))

    effective_robot_name = robot_name if robot_id == 0 else f'{robot_name}_{robot_id}'

    rviz_config = PathJoinSubstitution([
        FindPackageShare('sobit_home_bringup'), 'rviz', 'sobit_home.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sobit_home_bringup'), 'launch', 'robot.launch.py'])
        ]),
        launch_arguments={
            'robot_name'                  : effective_robot_name,
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
            'enable_gz'                   : 'False',  # never Gazebo on real hardware
        }.items(),
    )

    return [robot, rviz_node]
