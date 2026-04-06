import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    package_name_moveit_config = 'sobit_home_moveit_config'

    pkg_share_moveit_config = FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config)
    pkg_share_control = FindPackageShare(package='sobit_home_control').find('sobit_home_control')
    bridge_config_file = os.path.join(pkg_share_control, 'config', 'moveit_whole_body_bridge.yaml')

    # Configuration file paths
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

    # Declare launch arguments
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

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("sobit_home", package_name=package_name_moveit_config)
        .trajectory_execution(file_path=moveit_controllers_file_path)
        .robot_description_semantic(file_path=srdf_model_path)
        .joint_limits(file_path=joint_limits_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp", "stomp"],
            default_planning_pipeline="ompl"
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
        # .sensors_3d(file_path=sensors_file_path)
        .to_moveit_configs()
    )

    # Add frame_prefix so MoveIt maps URDF frames to TF frames
    config_dict = moveit_config.to_dict()
    # config_dict['robot_description_planning.frame_prefix'] = robot_name + '/'

    # move_group node
    start_move_group_node_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=robot_name,
        output="screen",
        parameters=[
            config_dict,
            {'use_sim_time': use_sim_time},
            {'trajectory_execution.control_multi_dof_joint_variables': True},
        ],
        remappings=[
            ('/attached_collision_object', 'attached_collision_object'),
            ('/trajectory_execution_event', 'trajectory_execution_event'),
            ('/transform_listener', 'transform_listener'),
            ('/planning_scene', 'planning_scene'),
            ('/planning_scene_world', 'planning_scene_world'),
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
        name="rviz2_moveit",
        arguments=["-d", rviz_config_file],
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            # moveit_config.sensors_3d,
            {
                'use_sim_time': use_sim_time,
                # 'robot_description_planning.frame_prefix': robot_name + '/',
            },
        ],
        remappings=[
            ('/attached_collision_object', 'attached_collision_object'),
            ('/trajectory_execution_event', 'trajectory_execution_event'),
            ('/transform_listener', 'transform_listener'),
            ('/planning_scene', 'planning_scene'),
            ('/planning_scene_world', 'planning_scene_world'),
            ('/collision_object', 'collision_object'),
            ('/joint_states', 'joint_states'),
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
        executable='component_container_mt', # MT = MultiThreaded, required for Action Servers
        composable_node_descriptions=[
            ComposableNode(
                package='sobit_home_control',
                plugin='sobit_home::MoveitWholeBodyBridge',
                name='moveit_whole_body_bridge',
                namespace=robot_name,
                parameters=[{'use_sim_time': use_sim_time}, bridge_config_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )


    ld = LaunchDescription()

    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(start_move_group_node_cmd)
    ld.add_action(start_rviz_node_cmd)
    ld.add_action(bridge_container)

    return ld
