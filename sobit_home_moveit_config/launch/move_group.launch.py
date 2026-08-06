import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder


def _launch_setup(context, *args, **kwargs):

    package_name_moveit_config = 'sobit_home_moveit_config'

    pkg_share_moveit_config = FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config)
    pkg_share_control = FindPackageShare(package='sobit_home_control').find('sobit_home_control')
    bridge_config_file = os.path.join(pkg_share_control, 'config', 'moveit_whole_body_bridge.yaml')

    # Launch configuration variables (substitutions for node parameters)
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    # One xacro SRDF for every configuration; teleop is just an argument.
    srdf_model_path = os.path.join(pkg_share_moveit_config, 'config', 'sobit_home.srdf.xacro')
    # Module switches shared by the URDF and the SRDF.
    module_mappings = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            'enable_mobile_base',
            'enable_arm_left',
            'enable_arm_right',
            'enable_hand_left',
            'enable_hand_right',
            'enable_head',
            'enable_body',
        )
    }
    # Forward enable_* flags so the SRDF matches the spawned URDF.
    srdf_mappings = dict(
        module_mappings,
        enable_teleop=LaunchConfiguration('enable_teleop').perform(context),
    )
    # Without an explicit robot_description the builder expands the URDF with its
    # xacro defaults (every module on), so move_group would plan against a robot
    # that does not match the one actually spawned.
    urdf_model_path = os.path.join(
        FindPackageShare(package='sobit_home_description').find('sobit_home_description'),
        'robots', 'sobit_home_robot.urdf.xacro')
    # The URDF evaluates its flags as bare Python (${$(arg enable_head)}), so they
    # must be capitalized literals; the SRDF lowercases and compares as strings.
    def _py_bool(value):
        return 'True' if value.lower() in ('true', '1', 'yes') else 'False'

    # enable_gz picks the ros2_control hardware plugin; it tracks use_sim_time,
    # which robot.launch.py already derives from enable_gz.
    urdf_mappings = {k: _py_bool(v) for k, v in module_mappings.items()}
    urdf_mappings.update(
        robot_name=LaunchConfiguration('robot_name').perform(context),
        enable_gz=_py_bool(LaunchConfiguration('use_sim_time').perform(context)),
        enable_tf_prefix=_py_bool(LaunchConfiguration('enable_tf_prefix').perform(context)),
    )
    # Planning params for MoveitServer. Defaults to this package's generic yaml;
    # override with the rc_doinglaundry path at launch for competition tuning.
    moveit_server_params_file_path = LaunchConfiguration('moveit_server_config').perform(context)
    if not moveit_server_params_file_path:
        moveit_server_params_file_path = os.path.join(
            pkg_share_moveit_config, 'config', 'moveit_server.yaml')
    moveit_controllers_file_path = os.path.join(pkg_share_moveit_config, 'config', 'moveit_controllers.yaml')
    joint_limits_file_path = os.path.join(pkg_share_moveit_config, 'config', 'joint_limits.yaml')
    kinematics_file_path = os.path.join(pkg_share_moveit_config, 'config', 'kinematics.yaml')
    pilz_cartesian_limits_file_path = os.path.join(pkg_share_moveit_config, 'config', 'pilz_cartesian_limits.yaml')
    rviz_config_file = os.path.join(pkg_share_moveit_config, 'rviz', 'moveit.rviz')
    sensors_file_path = os.path.join(pkg_share_moveit_config, 'config', 'sensors_3d.yaml')

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("sobit_home", package_name=package_name_moveit_config)
        .robot_description(file_path=urdf_model_path, mappings=urdf_mappings)
        .robot_description_semantic(file_path=srdf_model_path, mappings=srdf_mappings)
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
            {'planning_scene_monitor.transform_buffer_duration': 20.0},
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
                    moveit_server_params_file_path,
                    {'use_sim_time': use_sim_time},
                    {'active_planning_groups': ['arm_left', 'arm_right', 'arm_left_body', 'arm_right_body', 'head_arm_body']},
                ],
                extra_arguments=[{'use_intra_process_comms': False}]
            ),
        ],
        output='screen',
    )

    return [
        start_move_group_node_cmd,
        start_rviz_node_cmd,
        bridge_container,
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

    declare_moveit_server_config_cmd = DeclareLaunchArgument(
        name='moveit_server_config',
        default_value='',
        description='Path to MoveitServer planning-params YAML. Empty = package default. '
                    'Override with the rc_doinglaundry path for competition.')

    declare_enable_tf_prefix_cmd = DeclareLaunchArgument(
        name='enable_tf_prefix',
        default_value='false',
        description='Prefix TF frames with the robot name; must match robot.launch.py')

    # Module switches for the SRDF xacro; names match robot.launch.py.
    declare_module_cmds = [
        DeclareLaunchArgument(
            name=name,
            default_value='true',
            description=f'Whether {label} is present; disables its SRDF planning groups when false')
        for name, label in (
            ('enable_mobile_base', 'the mobile base'),
            ('enable_arm_left', 'the left arm'),
            ('enable_arm_right', 'the right arm'),
            ('enable_hand_left', 'the left hand'),
            ('enable_hand_right', 'the right hand'),
            ('enable_head', 'the head'),
            ('enable_body', 'the body lift'),
        )
    ]

    return LaunchDescription([
        declare_robot_name_cmd,
        declare_use_sim_time_cmd,
        declare_use_rviz_cmd,
        declare_enable_teleop_cmd,
        declare_moveit_server_config_cmd,
        declare_enable_tf_prefix_cmd,
        *declare_module_cmds,
        OpaqueFunction(function=_launch_setup),
    ])
