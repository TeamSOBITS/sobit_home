import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = 'sobit_home'
    robot_id = 0

    urg_configs = [
        os.path.join(get_package_share_directory(robot_name + "_bringup"), "config", "front_urg_node_param.yaml"),
        os.path.join(get_package_share_directory(robot_name + "_bringup"), "config", "back_urg_node_param.yaml"),
    ]

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
                'enable_gz'  : 'False',
                'enable_mobile_base': 'True',
                'enable_arm_left': 'True',
                'enable_arm_right': 'False',
                'enable_hand_left': 'False',
                'enable_hand_right': 'False',
                'enable_head': 'True',
                'enable_body': 'False',
                'enable_real_head_cam' : 'True', # TODO: toggle head camera
                'enable_real_hand_cam' : 'False', # TODO: toggle hand camera
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('urg_node'),
                    'launch',
                    'multi_urg.launch.py'
                ])
            ]),
            launch_arguments={
                'namespace' : robot_name if robot_id == 0 else robot_name + '_' + str(robot_id),
                'lidar_num' : '2',
                'config_file1': urg_configs[0],
                'config_file2': urg_configs[1],
            }.items()
        ),
    ])
