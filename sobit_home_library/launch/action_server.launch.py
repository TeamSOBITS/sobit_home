import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    _lib_config = os.path.join(get_package_share_directory("sobit_home_library"), "config")
    _moveit_config = os.path.join(
        get_package_share_directory("sobit_home_moveit_config"), "config")

    arg_robot_name       = DeclareLaunchArgument('robot_name',       default_value='sobit_home')
    arg_use_sim_time     = DeclareLaunchArgument('use_sim_time',     default_value='true')
    arg_use_rviz         = DeclareLaunchArgument('use_rviz',         default_value='false')
    arg_enable_teleop    = DeclareLaunchArgument('enable_teleop',    default_value='false')
    arg_enable_moveit    = DeclareLaunchArgument('enable_moveit',    default_value='true')
    arg_pose_config            = DeclareLaunchArgument(
        'pose_config',
        default_value=os.path.join(_lib_config, 'pose_list.yaml'),
        description='Path to whole-body pose YAML. Override with rc_doinglaundry path for competition.')
    arg_right_hand_pose_config = DeclareLaunchArgument(
        'right_hand_pose_config',
        default_value=os.path.join(_lib_config, 'right_hand_pose_list.yaml'),
        description='Path to right-hand pose YAML.')
    arg_left_hand_pose_config  = DeclareLaunchArgument(
        'left_hand_pose_config',
        default_value=os.path.join(_lib_config, 'left_hand_pose_list.yaml'),
        description='Path to left-hand pose YAML.')
    arg_moveit_server_config   = DeclareLaunchArgument(
        'moveit_server_config',
        default_value=os.path.join(_moveit_config, 'moveit_server.yaml'),
        description='Path to MoveitServer planning-params YAML (plan_time_sec, plan_attempts, '
                    'workspace bounds). Override with rc_doinglaundry path for competition.')
    arg_linear_kp        = DeclareLaunchArgument('linear_kp',        default_value='0.1')
    arg_linear_kd        = DeclareLaunchArgument('linear_kd',        default_value='0.4')
    arg_linear_ki        = DeclareLaunchArgument('linear_ki',        default_value='0.8')
    arg_rotate_kp        = DeclareLaunchArgument('rotate_kp',        default_value='0.1')
    arg_rotate_kd        = DeclareLaunchArgument('rotate_kd',        default_value='0.4')
    arg_rotate_ki        = DeclareLaunchArgument('rotate_ki',        default_value='0.8')
    arg_linear_arr_tol   = DeclareLaunchArgument('linear_arrival_tol', default_value='0.02')
    arg_rotate_arr_tol   = DeclareLaunchArgument('rotate_arrival_tol', default_value='0.02')
    arg_enable_tf_prefix = DeclareLaunchArgument('enable_tf_prefix', default_value='false')

    return LaunchDescription([
        arg_robot_name,
        arg_use_sim_time,
        arg_use_rviz,
        arg_enable_teleop,
        arg_enable_moveit,
        arg_linear_kp,
        arg_linear_kd,
        arg_linear_ki,
        arg_rotate_kp,
        arg_rotate_kd,
        arg_rotate_ki,
        arg_linear_arr_tol,
        arg_rotate_arr_tol,
        arg_enable_tf_prefix,
        arg_pose_config,
        arg_right_hand_pose_config,
        arg_left_hand_pose_config,
        arg_moveit_server_config,
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    robot_name       = LaunchConfiguration('robot_name').perform(context)
    use_sim_time     = LaunchConfiguration('use_sim_time').perform(context)
    use_rviz         = LaunchConfiguration('use_rviz').perform(context)
    enable_teleop    = LaunchConfiguration('enable_teleop').perform(context)
    enable_moveit    = LaunchConfiguration('enable_moveit').perform(context)
    linear_kp        = float(LaunchConfiguration('linear_kp').perform(context))
    linear_kd        = float(LaunchConfiguration('linear_kd').perform(context))
    linear_ki        = float(LaunchConfiguration('linear_ki').perform(context))
    rotate_kp        = float(LaunchConfiguration('rotate_kp').perform(context))
    rotate_kd        = float(LaunchConfiguration('rotate_kd').perform(context))
    rotate_ki        = float(LaunchConfiguration('rotate_ki').perform(context))
    linear_arr_tol   = float(LaunchConfiguration('linear_arrival_tol').perform(context))
    rotate_arr_tol   = float(LaunchConfiguration('rotate_arrival_tol').perform(context))
    enable_tf_prefix = LaunchConfiguration('enable_tf_prefix').perform(context)

    use_sim_time_bool = use_sim_time.lower() == 'true'
    enable_tf_prefix_bool = enable_tf_prefix.lower() in ('true', '1', 'yes')

    pose_config            = LaunchConfiguration('pose_config').perform(context)
    right_hand_pose_config = LaunchConfiguration('right_hand_pose_config').perform(context)
    left_hand_pose_config  = LaunchConfiguration('left_hand_pose_config').perform(context)
    moveit_server_config   = LaunchConfiguration('moveit_server_config').perform(context)

    joint_action_server_node = Node(
        package="sobit_home_library",
        executable="joint_action_server",
        name="joint_action_server",
        namespace=robot_name,
        parameters=[
            pose_config,
            right_hand_pose_config,
            left_hand_pose_config,
            {"use_sim_time": use_sim_time_bool},
            {"enable_tf_prefix": enable_tf_prefix_bool},
        ],
        output="screen",
    )

    wheel_action_server_node = Node(
        package="sobit_home_library",
        executable="wheel_action_server",
        name="wheel_action_server",
        namespace=robot_name,
        parameters=[
            {"use_sim_time": use_sim_time_bool},
            {"wheel_linear_kp": linear_kp},
            {"wheel_linear_kd": linear_kd},
            {"wheel_linear_ki": linear_ki},
            {"wheel_rotate_kp": rotate_kp},
            {"wheel_rotate_kd": rotate_kd},
            {"wheel_rotate_ki": rotate_ki},
            {"wheel_linear_arrival_tol": linear_arr_tol},
            {"wheel_rotate_arrival_tol": rotate_arr_tol},
        ],
        # Route through twist_mux instead of publishing cmd_vel directly.
        remappings=[
            ("cmd_vel", "cmd_vel_action"),
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
            'robot_name':   robot_name,
            'use_sim_time': use_sim_time,
            'use_rviz':     use_rviz,
            'enable_teleop':   enable_teleop,
            'moveit_server_config': moveit_server_config,
        }.items(),
    )

    enable_moveit_bool = enable_moveit.lower() in ('true', '1', 'yes')

    nodes = [
        joint_action_server_node,
        wheel_action_server_node,
    ]

    if enable_moveit_bool:
        nodes.append(move_group_launch)

    return nodes
