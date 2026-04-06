import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_home')
    arg_enable_gz = DeclareLaunchArgument('enable_gz', default_value='True')
    arg_linear_kp = DeclareLaunchArgument('linear_kp', default_value='0.1')
    arg_linear_kd = DeclareLaunchArgument('linear_kd', default_value='0.4')
    arg_linear_ki = DeclareLaunchArgument('linear_ki', default_value='0.8')
    arg_rotate_kp = DeclareLaunchArgument('rotate_kp', default_value='0.1')
    arg_rotate_kd = DeclareLaunchArgument('rotate_kd', default_value='0.4')
    arg_rotate_ki = DeclareLaunchArgument('rotate_ki', default_value='0.8')

    return LaunchDescription([
        arg_robot_name,
        arg_enable_gz,
        arg_linear_kp,
        arg_linear_kd,
        arg_linear_ki,
        arg_rotate_kp,
        arg_rotate_kd,
        arg_rotate_ki,
        OpaqueFunction(function = launch_gz),
    ])


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    enable_gz = LaunchConfiguration('enable_gz').perform(context)
    linear_kp = float(LaunchConfiguration('linear_kp').perform(context))
    linear_kd = float(LaunchConfiguration('linear_kd').perform(context))
    linear_ki = float(LaunchConfiguration('linear_ki').perform(context))
    rotate_kp = float(LaunchConfiguration('rotate_kp').perform(context))
    rotate_kd = float(LaunchConfiguration('rotate_kd').perform(context))
    rotate_ki = float(LaunchConfiguration('rotate_ki').perform(context))


    pose_config = os.path.join(
        get_package_share_directory("sobit_home_library"),
        "config",
        "pose_list.yaml",
    )
    right_hand_pose_config = os.path.join(
        get_package_share_directory("sobit_home_library"),
        "config",
        "right_hand_pose_list.yaml",
    )
    left_hand_pose_config = os.path.join(
        get_package_share_directory("sobit_home_library"),
        "config",
        "left_hand_pose_list.yaml",
    )

    # Find package share directories
    package_name_moveit_config = 'sobit_home_moveit_config'
    pkg_share_moveit = FindPackageShare(package=package_name_moveit_config).find('sobit_home_moveit_config')
    pkg_share_control = FindPackageShare(package='sobit_home_control').find('sobit_home_control')

    # Configuration file paths
    srdf_model_path = os.path.join(pkg_share_moveit, 'config', 'sobit_home.srdf')
    moveit_controllers_file_path = os.path.join(pkg_share_moveit, 'config', 'moveit_controllers.yaml')
    joint_limits_file_path = os.path.join(pkg_share_moveit, 'config', 'joint_limits.yaml')
    kinematics_file_path = os.path.join(pkg_share_moveit, 'config', 'kinematics.yaml')
    pilz_cartesian_limits_file_path = os.path.join(pkg_share_moveit, 'config', 'pilz_cartesian_limits.yaml')
    sensors_file_path = os.path.join(pkg_share_moveit, 'config', 'sensors_3d.yaml')
    bridge_config_file_path = os.path.join(pkg_share_control, 'config', 'moveit_whole_body_bridge.yaml')

    joint_action_server_node = Node(
        package="sobit_home_library",
        executable="joint_action_server",
        name="joint_action_server",
        namespace=robot_name,
        parameters=[
            pose_config,
            right_hand_pose_config,
            left_hand_pose_config,
            {"use_sim_time": True if enable_gz == 'True' else False},
        ],
        output="screen",
    )

    wheel_action_server_node = Node(
        package="sobit_home_library",
        executable="wheel_action_server",
        name="wheel_action_server",
        namespace=robot_name,
        parameters=[
            {"use_sim_time": True if enable_gz == 'True' else False},
            {"wheel_linear_kp": linear_kp},
            {"wheel_linear_kd": linear_kd},
            {"wheel_linear_ki": linear_ki},
            {"wheel_rotate_kp": rotate_kp},
            {"wheel_rotate_kd": rotate_kd},
            {"wheel_rotate_ki": rotate_ki},
        ],
        output="screen",
    )

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("sobit_home", package_name=package_name_moveit_config)
        .robot_description_semantic(file_path=srdf_model_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .joint_limits(file_path=joint_limits_file_path)
        # .moveit_cpp(file_path=moveit_cpp_file_path)
        .trajectory_execution(file_path=moveit_controllers_file_path)
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        # .sensors_3d(file_path=sensors_file_path)
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp", "stomp"],
            load_all=True
        )
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
        .to_moveit_configs()
    )

    # Add frame_prefix so MoveIt maps URDF frames to TF frames
    config_dict = moveit_config.to_dict()
    # config_dict['robot_description_planning.frame_prefix'] = robot_name + '/'

    moveit_action_server_node = ComposableNodeContainer(
        name='moveit_server_container',
        namespace=robot_name,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package="sobit_home_library",
                plugin="sobit_home::MoveitServer",
                name="moveit_server",
                namespace=robot_name,
                parameters=[
                    config_dict,
                    {"use_sim_time": True if enable_gz == 'True' else False},
                    {'trajectory_execution.control_multi_dof_joint_variables': True},
                    # {'robot_description_planning.frame_prefix': robot_name + '/'},
                    # {"active_planning_groups": ["arm_left", "arm_right", "arm_group"]},
                ],
                remappings=[
                    ('/attached_collision_object', 'attached_collision_object'),
                    ('/trajectory_execution_event', 'trajectory_execution_event'),
                    ('/transform_listener', 'transform_listener'),
                    ('/planning_scene', 'planning_scene'),
                    ('/planning_scene_world', 'planning_scene_world'),
                    ('/recognize_objects', 'recognize_objects'),
                    ('/recognized_object_array', 'recognized_object_array'),
                    ('/collision_object', 'collision_object'),
                    ('/joint_states', 'joint_states'),
                    ('tf', '/tf'),
                    ('tf_static', '/tf_static'),
                ]
            )
        ],
        output="screen",
    )

    bridge_container_node = ComposableNodeContainer(
        name='sobit_home_controllers_container',
        namespace=robot_name,
        package='rclcpp_components',
        executable='component_container_mt', # MT = MultiThreaded, required for Action Servers
        composable_node_descriptions=[
            ComposableNode(
                package='sobit_home_control',
                plugin='sobit_home::MoveitWholeBodyBridge',
                name='moveit_whole_body_bridge',
                namespace=robot_name,
                parameters=[
                    bridge_config_file_path,
                    {'use_sim_time': True if enable_gz == 'True' else False},
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('sobit_home_moveit_config'), '/launch/move_group.launch.py']),
        launch_arguments={
            'robot_name': robot_name,
            'use_sim_time': enable_gz,
            'use_rviz': 'true'
        }.items(),
    )

    return [
        joint_action_server_node,
        wheel_action_server_node,
        moveit_action_server_node,
        move_group_launch,
        bridge_container_node,
    ]
