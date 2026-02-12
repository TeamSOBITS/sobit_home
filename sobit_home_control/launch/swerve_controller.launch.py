import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description():
    arg_namespace    = DeclareLaunchArgument('namespace', default_value='')
    arg_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False')
    arg_config_file  = DeclareLaunchArgument('config_file', 
        default_value=os.path.join(get_package_share_directory('sobit_home_bringup'), 'config', "swerve_config.yaml"
    ))

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    swerve_ctr_node = Node(
        package="sobit_home_control",
        executable="swerve_controller_node",
        name="swerve_controller",
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            config_file
        ],
        output="screen",
    )
    return LaunchDescription([
        arg_namespace,
        arg_use_sim_time,
        arg_config_file,
        swerve_ctr_node,
    ])
