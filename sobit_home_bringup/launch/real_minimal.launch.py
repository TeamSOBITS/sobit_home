from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = 'sobit_home'
    robot_id = 0

    rviz_config = PathJoinSubstitution([
            FindPackageShare('sobit_home_bringup'),
            'rviz',
            'real.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        # Launch Robot No. 1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sobit_home_bringup'),
                    'launch',
                    'robot.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_name' : robot_name if robot_id == 0 else robot_name + '_' + str(robot_id),
                'enable_mobile_base'          : 'True',
                'enable_arm_left'             : 'True',
                'enable_arm_right'            : 'True',
                'enable_hand_left'            : 'True',
                'enable_hand_right'           : 'True',
                'enable_head'                 : 'True',
                'enable_body'                 : 'True',
                'enable_head_cam_color'       : 'True',
                'enable_head_cam_depth'       : 'True',
                'enable_hand_left_cam_color'  : 'True',
                'enable_hand_right_cam_color' : 'True',
                'enable_lidar'                : 'True',
                'enable_display'              : 'True',
                'enable_teleop'               : 'True',
                'enable_gz'                   : 'False',
            }.items()
        ),
        rviz_node,
    ])
