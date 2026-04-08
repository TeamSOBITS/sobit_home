import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_home')
    arg_enable_gz = DeclareLaunchArgument('enable_gz', default_value='True')
    arg_use_rviz = DeclareLaunchArgument('use_rviz', default_value='true')
    arg_linear_kp = DeclareLaunchArgument('linear_kp', default_value='0.1')
    arg_linear_kd = DeclareLaunchArgument('linear_kd', default_value='0.4')
    arg_linear_ki = DeclareLaunchArgument('linear_ki', default_value='0.8')
    arg_rotate_kp = DeclareLaunchArgument('rotate_kp', default_value='0.1')
    arg_rotate_kd = DeclareLaunchArgument('rotate_kd', default_value='0.4')
    arg_rotate_ki = DeclareLaunchArgument('rotate_ki', default_value='0.8')

    return LaunchDescription([
        arg_robot_name,
        arg_enable_gz,
        arg_use_rviz,
        arg_linear_kp,
        arg_linear_kd,
        arg_linear_ki,
        arg_rotate_kp,
        arg_rotate_kd,
        arg_rotate_ki,
        OpaqueFunction(function=launch_gz),
    ])


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    enable_gz  = LaunchConfiguration('enable_gz').perform(context)
    use_rviz   = LaunchConfiguration('use_rviz').perform(context)
    linear_kp  = float(LaunchConfiguration('linear_kp').perform(context))
    linear_kd  = float(LaunchConfiguration('linear_kd').perform(context))
    linear_ki  = float(LaunchConfiguration('linear_ki').perform(context))
    rotate_kp  = float(LaunchConfiguration('rotate_kp').perform(context))
    rotate_kd  = float(LaunchConfiguration('rotate_kd').perform(context))
    rotate_ki  = float(LaunchConfiguration('rotate_ki').perform(context))

    use_sim_time = True if enable_gz == 'True' else False

    pose_config = os.path.join(
        get_package_share_directory("sobit_home_library"), "config", "pose_list.yaml")
    right_hand_pose_config = os.path.join(
        get_package_share_directory("sobit_home_library"), "config", "right_hand_pose_list.yaml")
    left_hand_pose_config = os.path.join(
        get_package_share_directory("sobit_home_library"), "config", "left_hand_pose_list.yaml")

    joint_action_server_node = Node(
        package="sobit_home_library",
        executable="joint_action_server",
        name="joint_action_server",
        namespace=robot_name,
        parameters=[
            pose_config,
            right_hand_pose_config,
            left_hand_pose_config,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    wheel_action_server_node = Node(
        package="sobit_home_library",
        executable="wheel_action_server",
        name="wheel_action_server",
        namespace=robot_name,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"wheel_linear_kp": linear_kp},
            {"wheel_linear_kd": linear_kd},
            {"wheel_linear_ki": linear_ki},
            {"wheel_rotate_kp": rotate_kp},
            {"wheel_rotate_kd": rotate_kd},
            {"wheel_rotate_ki": rotate_ki},
        ],
        output="screen",
    )

    # move_group.launch.py starts:
    #   - move_group node (robot model, planning pipelines)
    #   - MoveitWholeBodyBridge (trajectory execution bridge)
    #   - MoveitServer (plan_to_pose service + execute_plan action)
    #   - RViz (optional)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sobit_home_moveit_config'),
                'launch', 'move_group.launch.py'
            )
        ),
        launch_arguments={
            'robot_name': robot_name,
            'use_sim_time': enable_gz,
            'use_rviz': use_rviz,
        }.items(),
    )

    return [
        joint_action_server_node,
        wheel_action_server_node,
        move_group_launch,
    ]
