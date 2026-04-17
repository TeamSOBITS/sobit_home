import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder


def _launch_setup(context, *args, **kwargs):

    package_name_moveit_config = 'sobit_home_moveit_config'

    pkg_share_moveit_config = FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config)
    pkg_share_control = FindPackageShare(package='sobit_home_control').find('sobit_home_control')
    pkg_share_library = FindPackageShare(package='sobit_home_library').find('sobit_home_library')
    bridge_config_file = os.path.join(pkg_share_control, 'config', 'moveit_whole_body_bridge.yaml')
    arm_teleop_config_file = os.path.join(pkg_share_library, 'config', 'arm_teleop.yaml')

    enable_teleop = LaunchConfiguration('enable_teleop').perform(context).lower() == 'true'

    # Configuration file paths — SRDF chosen based on enable_teleop
    if enable_teleop:
        srdf_model_path = os.path.join(pkg_share_moveit_config, 'config', 'sobit_home_teleop.srdf')
    else:
        srdf_model_path = os.path.join(pkg_share_moveit_config, 'config', 'sobit_home.srdf')
    moveit_controllers_file_path = os.path.join(pkg_share_moveit_config, 'config', 'moveit_controllers.yaml')
    joint_limits_file_path = os.path.join(pkg_share_moveit_config, 'config', 'joint_limits.yaml')
    kinematics_file_path = os.path.join(pkg_share_moveit_config, 'config', 'kinematics.yaml')
    pilz_cartesian_limits_file_path = os.path.join(pkg_share_moveit_config, 'config', 'pilz_cartesian_limits.yaml')
    rviz_config_file = os.path.join(pkg_share_moveit_config, 'rviz', 'moveit.rviz')
    sensors_file_path = os.path.join(pkg_share_moveit_config, 'config', 'sensors_3d.yaml')

    # Launch configuration variables
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_teleop_cfg = LaunchConfiguration('enable_teleop')

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
        .sensors_3d(file_path=sensors_file_path)
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

    octomap_config = {
        'octomap_frame': 'base_footprint',  # if mobile robot, should be a fixed frame in the world
        'octomap_resolution': 0.05,
        'max_range': 5.0
    }

    # move_group node
    start_move_group_node_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=robot_name,
        output="screen",
        parameters=[
            config_dict,
            octomap_config,
            {'use_sim_time': use_sim_time},
            {'trajectory_execution.control_multi_dof_joint_variables': True},
            # {'robot_description_planning.frame_prefix': robot_name + '/'},
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
            ('tf_static', '/tf_static')
        ]
    )

    # RViz
    start_rviz_node_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        # name="rviz2_moveit",
        arguments=["-d", rviz_config_file],
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.sensors_3d,
            {
                'use_sim_time': use_sim_time,
                # 'robot_description_planning.frame_prefix': robot_name + '/',
            },
        ],
        remappings=[
            ('/attached_collision_object', '/sobit_home/attached_collision_object'),
            ('/trajectory_execution_event', '/sobit_home/trajectory_execution_event'),
            ('/transform_listener', '/sobit_home/transform_listener'),
            ('/planning_scene', '/sobit_home/planning_scene'),
            ('/planning_scene_world', '/sobit_home/planning_scene_world'),
            ('/recognize_objects', '/sobit_home/recognize_objects'),
            ('/recognized_object_array', '/sobit_home/recognized_object_array'),
            ('/collision_object', '/sobit_home/collision_object'),
            ('/joint_states', '/sobit_home/joint_states'),
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ]
    )

    exit_event_handler = RegisterEventHandler(
        condition=IfCondition(use_rviz),
        event_handler=OnProcessExit(
            target_action=start_rviz_node_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
        ),
    )

    bridge_container = ComposableNodeContainer(
        name='sobit_home_controllers_container',
        namespace=robot_name,
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        composable_node_descriptions=[
            ComposableNode(
                package='sobit_home_control',
                plugin='sobit_home::MoveitWholeBodyBridge',
                name='moveit_whole_body_bridge',
                namespace=robot_name,
                parameters=[{'use_sim_time': use_sim_time}, bridge_config_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # MoveIt plan/execute server
            ComposableNode(
                package='sobit_home_library',
                plugin='sobit_home::MoveitServer',
                name='moveit_server',
                namespace=robot_name,
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'active_planning_groups': ['arm_left', 'arm_right']},
                ],
                extra_arguments=[{'use_intra_process_comms': False}]
            ),
        ],
        output='screen',
    )

    # Real-time arm teleop bridge
    load_arm_teleop = LoadComposableNodes(
        condition=IfCondition(enable_teleop_cfg),
        target_container=PythonExpression(["'", robot_name, "' + '/sobit_home_controllers_container'"]),
        composable_node_descriptions=[
            ComposableNode(
                package='sobit_home_library',
                plugin='sobit_home::MoveitArmTeleop',
                name='moveit_arm_teleop',
                namespace=robot_name,
                parameters=[
                    {'use_sim_time': use_sim_time},
                    arm_teleop_config_file,
                ],
                extra_arguments=[{'use_intra_process_comms': False}]
            ),
        ],
    )

    return [
        start_move_group_node_cmd,
        start_rviz_node_cmd,
        bridge_container,
        load_arm_teleop,
        exit_event_handler,
    ]


def generate_launch_description():

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='sobit_home',
        description='Robot name used as namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz')

    declare_enable_teleop_cmd = DeclareLaunchArgument(
        name='enable_teleop',
        default_value='false',
        description='Whether to load the MoveitArmTeleop composable node')

    return LaunchDescription([
        declare_robot_name_cmd,
        declare_use_sim_time_cmd,
        declare_use_rviz_cmd,
        declare_enable_teleop_cmd,
        OpaqueFunction(function=_launch_setup),
    ])
