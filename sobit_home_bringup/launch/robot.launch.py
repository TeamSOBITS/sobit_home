import os
import subprocess
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap

import xacro

def _bool(val):
    """Normalize CLI true/True/1 → True, false/False/0 → False."""
    return val.lower() in ('true', '1', 'yes')


def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_home')

    arg_enable_lidar = DeclareLaunchArgument('enable_lidar', default_value='true')

    arg_robot_coords_x = DeclareLaunchArgument('robot_coords_x', default_value='0')
    arg_robot_coords_y = DeclareLaunchArgument('robot_coords_y', default_value='0')
    arg_robot_coords_z = DeclareLaunchArgument('robot_coords_z', default_value='0')
    arg_robot_coords_Y = DeclareLaunchArgument('robot_coords_Y', default_value='0')

    arg_enable_gz                   = DeclareLaunchArgument('enable_gz', default_value='true')
    arg_enable_display              = DeclareLaunchArgument('enable_display', default_value='false')
    arg_enable_mobile_base          = DeclareLaunchArgument('enable_mobile_base', default_value='true')
    arg_enable_arm_left             = DeclareLaunchArgument('enable_arm_left', default_value='true')
    arg_enable_arm_right            = DeclareLaunchArgument('enable_arm_right', default_value='true')
    arg_enable_hand_left            = DeclareLaunchArgument('enable_hand_left', default_value='true')
    arg_enable_hand_right           = DeclareLaunchArgument('enable_hand_right', default_value='true')
    arg_enable_head                 = DeclareLaunchArgument('enable_head', default_value='true')
    arg_enable_body                 = DeclareLaunchArgument('enable_body', default_value='true')
    arg_enable_head_cam_color       = DeclareLaunchArgument('enable_head_cam_color', default_value='true')
    arg_enable_head_cam_depth       = DeclareLaunchArgument('enable_head_cam_depth', default_value='true')
    arg_enable_hand_left_cam_color  = DeclareLaunchArgument('enable_hand_left_cam_color', default_value='true')
    arg_enable_hand_right_cam_color = DeclareLaunchArgument('enable_hand_right_cam_color', default_value='true')
    arg_head_cam_config_file        = DeclareLaunchArgument(
        'head_cam_config_file',
        default_value=os.path.join(
            get_package_share_directory('sobit_home_bringup'),
            'config',
            'head_camera_orbbec.yaml',
        ),
    )
    arg_enable_teleop          = DeclareLaunchArgument('enable_teleop',          default_value='false')
    arg_enable_rm_motors       = DeclareLaunchArgument('enable_rm_motors',       default_value='true')
    arg_enable_dxl_pro         = DeclareLaunchArgument('enable_dxl_pro',         default_value='true')
    arg_enable_moveit          = DeclareLaunchArgument('enable_moveit',          default_value='true')
    arg_enable_tf_prefix       = DeclareLaunchArgument('enable_tf_prefix',       default_value='false')
    arg_enable_action_server   = DeclareLaunchArgument(
        'enable_action_server', default_value='true',
        description='Set false when the action server (with pose config) is started externally, e.g. from doinglaundry.launch.py on the dev PC.')

    # Pose config: prefer rc_doinglaundry override if the package is installed, else library default.
    _lib_config = os.path.join(get_package_share_directory('sobit_home_library'), 'config')
    try:
        _rc_config = get_package_share_directory('rc_doinglaundry')
        _default_pose_config            = os.path.join(_rc_config, 'config', 'pose_list.yaml')
        _default_right_hand_pose_config = os.path.join(_lib_config, 'right_hand_pose_list.yaml')
        _default_left_hand_pose_config  = os.path.join(_lib_config, 'left_hand_pose_list.yaml')
    except PackageNotFoundError:
        _default_pose_config            = os.path.join(_lib_config, 'pose_list.yaml')
        _default_right_hand_pose_config = os.path.join(_lib_config, 'right_hand_pose_list.yaml')
        _default_left_hand_pose_config  = os.path.join(_lib_config, 'left_hand_pose_list.yaml')

    arg_pose_config            = DeclareLaunchArgument(
        'pose_config', default_value=_default_pose_config,
        description='Path to whole-body pose YAML (overridable).')
    arg_right_hand_pose_config = DeclareLaunchArgument(
        'right_hand_pose_config', default_value=_default_right_hand_pose_config,
        description='Path to right-hand pose YAML (overridable).')
    arg_left_hand_pose_config  = DeclareLaunchArgument(
        'left_hand_pose_config', default_value=_default_left_hand_pose_config,
        description='Path to left-hand pose YAML (overridable).')

    return LaunchDescription([
        arg_robot_name,
        arg_enable_lidar,
        arg_robot_coords_x,
        arg_robot_coords_y,
        arg_robot_coords_z,
        arg_robot_coords_Y,
        arg_enable_gz,
        arg_enable_display,
        arg_enable_mobile_base,
        arg_enable_arm_left,
        arg_enable_arm_right,
        arg_enable_hand_left,
        arg_enable_hand_right,
        arg_enable_head,
        arg_enable_body,
        arg_enable_head_cam_color,
        arg_enable_head_cam_depth,
        arg_enable_hand_left_cam_color,
        arg_enable_hand_right_cam_color,
        arg_head_cam_config_file,
        arg_enable_teleop,
        arg_enable_rm_motors,
        arg_enable_dxl_pro,
        arg_enable_moveit,
        arg_enable_tf_prefix,
        arg_enable_action_server,
        arg_pose_config,
        arg_right_hand_pose_config,
        arg_left_hand_pose_config,
        OpaqueFunction(function = launch_gz),
    ])


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    bringup_package = 'sobit_home_bringup'

    enable_lidar = _bool(LaunchConfiguration('enable_lidar').perform(context))

    robot_coords_x = LaunchConfiguration('robot_coords_x').perform(context)
    robot_coords_y = LaunchConfiguration('robot_coords_y').perform(context)
    robot_coords_z = LaunchConfiguration('robot_coords_z').perform(context)
    robot_coords_Y = LaunchConfiguration('robot_coords_Y').perform(context)

    enable_mobile_base          = _bool(LaunchConfiguration('enable_mobile_base').perform(context))
    enable_body                 = _bool(LaunchConfiguration('enable_body').perform(context))
    enable_arm_left             = _bool(LaunchConfiguration('enable_arm_left').perform(context))
    enable_arm_right            = _bool(LaunchConfiguration('enable_arm_right').perform(context))
    enable_hand_left            = _bool(LaunchConfiguration('enable_hand_left').perform(context))
    enable_hand_right           = _bool(LaunchConfiguration('enable_hand_right').perform(context))
    enable_head                 = _bool(LaunchConfiguration('enable_head').perform(context))
    enable_head_cam_color       = _bool(LaunchConfiguration('enable_head_cam_color').perform(context))
    enable_head_cam_depth       = _bool(LaunchConfiguration('enable_head_cam_depth').perform(context))
    enable_hand_left_cam_color  = _bool(LaunchConfiguration('enable_hand_left_cam_color').perform(context))
    enable_hand_right_cam_color = _bool(LaunchConfiguration('enable_hand_right_cam_color').perform(context))
    head_cam_config_file        = LaunchConfiguration('head_cam_config_file').perform(context)
    enable_display              = _bool(LaunchConfiguration('enable_display').perform(context))
    enable_teleop               = _bool(LaunchConfiguration('enable_teleop').perform(context))
    enable_gz                   = _bool(LaunchConfiguration('enable_gz').perform(context))
    enable_rm_motors            = _bool(LaunchConfiguration('enable_rm_motors').perform(context))
    enable_dxl_pro              = _bool(LaunchConfiguration('enable_dxl_pro').perform(context))
    enable_moveit               = _bool(LaunchConfiguration('enable_moveit').perform(context))
    enable_tf_prefix            = _bool(LaunchConfiguration('enable_tf_prefix').perform(context))
    enable_action_server        = _bool(LaunchConfiguration('enable_action_server').perform(context))
    pose_config                 = LaunchConfiguration('pose_config').perform(context)
    right_hand_pose_config      = LaunchConfiguration('right_hand_pose_config').perform(context)
    left_hand_pose_config       = LaunchConfiguration('left_hand_pose_config').perform(context)

    # Find Dynamixel Port name from DXL_LOWER_PORT/DXL_UPPER_PORT environment variable
    dxl_x_lower_body_port = ''
    dxl_x_upper_body_port = ''
    dxl_p_upper_body_port = ''
    dxl_x_hand_port = ''
    um_body_port = ''
    um_body_id = ''
    # Find USB Cam port name from HOME_CAM_LEFT_PORT/HOME_CAM_RIGHT_PORT environment variable
    cam_left_port = ''
    cam_right_port = ''
    if not enable_gz:
        dxl_x_lower_body_port = str(os.environ.get('DXL_X_LOWER_PORT'))
        dxl_x_upper_body_port = str(os.environ.get('DXL_X_UPPER_PORT'))
        dxl_p_upper_body_port = str(os.environ.get('DXL_P_UPPER_PORT'))
        dxl_x_hand_port = str(os.environ.get('DXL_X_HAND_PORT'))

        um_body_port = str(os.environ.get('UM_PORT'))
        um_body_id = str(os.environ.get('UM_ID', '5'))
        print('Dynamixel Lower Body Port : ' + dxl_x_lower_body_port)
        print('Dynamixel Upper Body Port : ' + dxl_x_upper_body_port)
        print('Dynamixel PRO Upper Port  : ' + dxl_p_upper_body_port)
        print('Dynamixel Hand Port : ' + dxl_x_hand_port)
        print('Uirobot Gateway Port : ' + um_body_port)
        print('Uirobot Body Node ID : ' + um_body_id)

        if enable_mobile_base and enable_rm_motors:
            can_setup_cmds = [
                ['sudo', 'ip', 'link', 'set', 'can0', 'down'],
                ['sudo', 'ip', 'link', 'set', 'can0', 'type', 'can', 'bitrate', '1000000'],
                # The default 10-frame TX queue jams with ENOBUFS if the bus briefly stops draining
                ['sudo', 'ip', 'link', 'set', 'can0', 'txqueuelen', '65536'],
                ['sudo', 'ip', 'link', 'set', 'can0', 'up'],
            ]
            if any(subprocess.run(cmd).returncode != 0 for cmd in can_setup_cmds):
                print('Failed to set up CAN0 interface. Please check CAN adapter connection.')
                exit(1)
            print('CAN0 interface is set up.')

            # Powered RM motors broadcast feedback at 1 kHz; a silent bus cannot ACK
            # our frames and ros2_control write() would fail with ENOBUFS (os error 105)
            try:
                subprocess.run(['candump', '-n', '1', 'can0'],
                               timeout=1.5, stdout=subprocess.DEVNULL, check=True)
                print('CAN0 bus is alive (motor feedback detected).')
            except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
                print('No traffic on CAN0. Is the emergency stop released and motor power on?')
                exit(1)

        # Find USB Cam port
        cam_left_port = str(os.environ.get('HOME_CAM_LEFT_PORT'))
        cam_right_port = str(os.environ.get('HOME_CAM_RIGHT_PORT'))
        print(f'USB Cam (Left) : {cam_left_port}')
        print(f'USB Cam (RIGHT) : {cam_right_port}')

    robot_description = os.path.join(get_package_share_directory(
        'sobit_home_description'),
        'robots',
        'sobit_home_robot.urdf.xacro'
    )

    urg_configs = [
        os.path.join(get_package_share_directory(bringup_package), "config", "front_urg_node_param.yaml"),
        os.path.join(get_package_share_directory(bringup_package), "config", "back_urg_node_param.yaml"),
    ]
    merge_scan_config = os.path.join(get_package_share_directory(bringup_package), "config", "laser_scan_merger.yaml")
    swerve_config = os.path.join(get_package_share_directory(bringup_package), "config", "swerve_config.yaml")
    hand_left_cam_config  = os.path.join(get_package_share_directory(bringup_package), "config", "hand_left_cam.yaml")
    hand_right_cam_config = os.path.join(get_package_share_directory(bringup_package), "config", "hand_right_cam.yaml")

    robot_description_config = xacro.process_file(
        robot_description,
        mappings={
            'robot_name': robot_name,
            'enable_gz' : 'True' if enable_gz else 'False',
            'enable_mobile_base': 'True' if enable_mobile_base else 'False',
            'enable_body': 'True' if enable_body else 'False',
            'enable_arm_left': 'True' if enable_arm_left else 'False',
            'enable_arm_right': 'True' if enable_arm_right else 'False',
            'enable_hand_left': 'True' if enable_hand_left else 'False',
            'enable_hand_right': 'True' if enable_hand_right else 'False',
            'enable_head': 'True' if enable_head else 'False',
            'enable_head_cam_color' : 'True' if enable_head_cam_color else 'False',
            'enable_head_cam_depth' : 'True' if enable_head_cam_depth else 'False',
            'enable_hand_left_cam_color' : 'True' if enable_hand_left_cam_color else 'False',
            'enable_hand_right_cam_color' : 'True' if enable_hand_right_cam_color else 'False',
            'enable_rm_motors': 'True' if enable_rm_motors else 'False',
            'enable_dxl_pro': 'True' if enable_dxl_pro else 'False',
            'enable_tf_prefix': 'True' if enable_tf_prefix else 'False',
            'dxl_x_lower_body_port': dxl_x_lower_body_port,
            'dxl_x_upper_body_port': dxl_x_upper_body_port,
            'dxl_p_upper_body_port': dxl_p_upper_body_port,
            'dxl_x_hand_port': dxl_x_hand_port,
            'um_body_port': um_body_port,
            'um_body_id': um_body_id,
        })


    robot_state_publisher_params = [
        {"robot_description": robot_description_config.toxml()},
        {"use_sim_time": enable_gz},
        {"publish_frequency": 50.0},
    ]
    if enable_tf_prefix:
        robot_state_publisher_params.append({"frame_prefix": robot_name + '/'})

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=robot_state_publisher_params,
        prefix=None if enable_gz else 'taskset -c 8-15',
        output="screen",
    )

    sobits_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sobits_display'),
                'launch',
                'sobits_display.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_type': 'sobit_home',
            'namespace': robot_name,
        }.items(),
        condition=IfCondition('true' if enable_display else 'false'),
    )

    controllers = []
    nodes = []

    controller_config = os.path.join(get_package_share_directory(
        'sobit_home_control'),
        'config',
        'controllers.yaml'
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        namespace=robot_name,
        parameters=[
            controller_config,
            {"use_sim_time": enable_gz},
        ],
        remappings=[
            ("controller_manager/robot_description", "robot_description"),
        ],
        prefix=None if enable_gz else 'taskset -c 2-7 chrt -f 80',
        output="both",
    )

    if (not enable_gz and enable_lidar):
        urg_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('urg_node'),
                    'launch',
                    'multi_urg.launch.py'
                ])
            ]),
            launch_arguments={
                'namespace' : robot_name,
                'lidar_num' : '1',
                'config_file1': urg_configs[0],
                'topic_namespace1': 'lidar_front',
                # 'config_file2': urg_configs[1],
                # 'topic_namespace2': 'lidar_back',
                'enable_tf_prefix': 'true' if enable_tf_prefix else 'false',
            }.items()
        )
        controllers.append(urg_node)

    if enable_lidar:
        merge_lidar_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros2_laser_scan_merger'),
                    'launch',
                    'merge_2_scan.launch.py'
                ])
            ]),
            launch_arguments={
                'config_file' : merge_scan_config,
                'pointcloud_remapping' : '/' + robot_name + '/lidar_scan/points',
                'scan_remapping': '/' + robot_name + '/lidar_scan',
                'use_sim_time': 'true' if enable_gz else 'false',
            }.items()
        )
        controllers.append(merge_lidar_node)

    if enable_mobile_base:
        wheel_steer_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            name='wheel_steer_position_controller',
            namespace=robot_name,
            arguments=[
                'wheel_steer_position_controller',
                '-c', 'controller_manager', '--activate'
                ],
        )
        swerve_controller = Node(
            package='sobit_home_control',
            executable='swerve_controller_node',
            name='swerve_controller',
            namespace=robot_name,
            parameters=[
                {'use_sim_time': enable_gz},
                swerve_config,
            ],
            prefix=None if enable_gz else 'taskset -c 2-7',
            output='screen',
        )
        controllers.append(wheel_steer_position_controller)
        if enable_rm_motors:
            wheel_drive_velocity_controller = Node(
                package='controller_manager',
                executable='spawner',
                name='wheel_drive_velocity_controller',
                namespace=robot_name,
                arguments=[
                    'wheel_drive_velocity_controller',
                    '-c', 'controller_manager', '--activate'
                    ],
            )
            controllers.append(wheel_drive_velocity_controller)

    if enable_arm_left:
        arm_left_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            name='arm_left_position_controller',
            namespace=robot_name,
            arguments=[
                'arm_left_position_controller',
                '-c', 'controller_manager', '--activate'
                ],
        )
        controllers.append(arm_left_position_controller)

    if enable_arm_right:
        arm_right_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            name='arm_right_position_controller',
            namespace=robot_name,
            arguments=[
                'arm_right_position_controller',
                '-c', 'controller_manager', '--activate'
                ],
        )
        controllers.append(arm_right_position_controller)

    if enable_hand_left:
        hand_left_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            name='hand_left_position_controller',
            namespace=robot_name,
            arguments=[
                'hand_left_position_controller',
                '-c', 'controller_manager', '--activate'
                ],
        )
        controllers.append(hand_left_position_controller)

    if enable_hand_right:
        hand_right_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            name='hand_right_position_controller',
            namespace=robot_name,
            arguments=[
                'hand_right_position_controller',
                '-c', 'controller_manager', '--activate'
                ],
        )
        controllers.append(hand_right_position_controller)

    if enable_head:
        head_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            name='head_position_controller',
            namespace=robot_name,
            arguments=[
                'head_position_controller',
                '-c', 'controller_manager', '--activate'
                ],
        )
        controllers.append(head_position_controller)

    if enable_body:
        body_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            name='body_position_controller',
            namespace=robot_name,
            arguments=[
                'body_position_controller',
                '-c', 'controller_manager', '--activate'
                ],
        )
        controllers.append(body_position_controller)

    gz_spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        namespace=robot_name,
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-x', robot_coords_x,
            '-y', robot_coords_y,
            '-z', robot_coords_z,
            '-Y', robot_coords_Y,
        ],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster',
        namespace=robot_name,
        arguments=[
            'joint_state_broadcaster',
            '-c', 'controller_manager',
            ],
    )
    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity_node,
            on_exit=joint_state_broadcaster,
        )
    )

    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=controllers,
        )
    )

    if enable_mobile_base:
        delayed_swerve_controller = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controllers[-1],
                on_exit=[swerve_controller],
            )
        )
        nodes.append(delayed_swerve_controller)

    action_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sobit_home_library'),
                'launch',
                'action_server.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name'             : robot_name,
            'use_sim_time'           : 'true' if enable_gz else 'false',
            'enable_teleop'          : 'true' if enable_teleop else 'false',
            'enable_moveit'          : 'true' if enable_moveit else 'false',
            'enable_tf_prefix'       : 'true' if enable_tf_prefix else 'false',
            # Module switches -> SRDF xacro args.
            'enable_mobile_base'     : 'true' if enable_mobile_base else 'false',
            'enable_arm_left'        : 'true' if enable_arm_left else 'false',
            'enable_arm_right'       : 'true' if enable_arm_right else 'false',
            'enable_hand_left'       : 'true' if enable_hand_left else 'false',
            'enable_hand_right'      : 'true' if enable_hand_right else 'false',
            'enable_head'            : 'true' if enable_head else 'false',
            'enable_body'            : 'true' if enable_body else 'false',
            'pose_config'            : pose_config,
            'right_hand_pose_config' : right_hand_pose_config,
            'left_hand_pose_config'  : left_hand_pose_config,
        }.items(),
    )

    if not enable_gz and (enable_head_cam_color or enable_head_cam_depth):
        head_cam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('orbbec_camera'),
                    'launch',
                    'gemini_330_series.launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': robot_name,
                'camera_name': 'head_camera',
                'config_file_path': head_cam_config_file,
                # 'head_camera_depth_frame_id':    robot_name + '/head_camera_depth_frame',
                # 'depth_optical_frame_id':        robot_name + '/head_camera_depth_optical_frame',
                # 'head_camera_color_frame_id':    robot_name + '/head_camera_color_frame',
                # 'color_optical_frame_id':        robot_name + '/head_camera_color_optical_frame',
                # 'head_camera_left_ir_frame_id':  robot_name + '/head_camera_infra_1_frame',
                # 'left_ir_optical_frame_id':      robot_name + '/head_camera_infra_1_optical_frame',
                # 'head_camera_right_ir_frame_id': robot_name + '/head_camera_infra2_frame',
                # 'right_ir_optical_frame_id':     robot_name + '/head_camera_infra2_optical_frame',
                # 'head_camera_depth_frame_id':    'head_camera_depth_frame',
                'depth_optical_frame_id':        'head_camera_depth_optical_frame',
                # 'head_camera_color_frame_id':    'head_camera_color_frame',
                'color_optical_frame_id':        'head_camera_color_optical_frame',
                # 'head_camera_left_ir_frame_id':  'head_camera_infra_1_frame',
                'left_ir_optical_frame_id':      'head_camera_left_ir_optical_frame',
                # 'head_camera_right_ir_frame_id': 'head_camera_infra2_frame',
                'right_ir_optical_frame_id':     'head_camera_right_ir_optical_frame',
            }.items(),
        )
        nodes.append(head_cam_launch)

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        namespace=robot_name,
        arguments=[
                    "/" + robot_name + "/joint_states" + "@sensor_msgs/msg/JointState" + "[gz.msgs.Model",
                    # "/model/" + robot_name + "/pose" + "@geometry_msgs/msg/Pose" + "[gz.msgs.Pose",
                    "/" + robot_name + "/head_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                    "/" + robot_name + "/head_camera/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                    # NOTE: head_camera/depth/points is NOT bridged from gz. gz emits the
                    # cloud in BODY convention (+X fwd), which does not match the real cam
                    "/" + robot_name + "/hand_left_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                    "/" + robot_name + "/hand_left_camera/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                    "/" + robot_name + "/hand_right_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                    "/" + robot_name + "/hand_right_camera/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                    "/" + robot_name + "/lidar_front/scan" + "@sensor_msgs/msg/LaserScan" + "[gz.msgs.LaserScan",
                    "/" + robot_name + "/lidar_front/scan/points" + "@sensor_msgs/msg/PointCloud2" + "[gz.msgs.PointCloudPacked",
                    "/" + robot_name + "/lidar_back/scan" + "@sensor_msgs/msg/LaserScan" + "[gz.msgs.LaserScan",
                    "/" + robot_name + "/lidar_back/scan/points" + "@sensor_msgs/msg/PointCloud2" + "[gz.msgs.PointCloudPacked",
                    # "/" + robot_name + "/imu" + "@sensor_msgs/msg/Imu" + "[gz.msgs.IMU",
                ],
        remappings=[
                    ("/" + robot_name + "/head_camera/color", "/" + robot_name + "/head_camera/color/image_raw"),
                    ("/" + robot_name + "/hand_left_camera/camera_info", "/" + robot_name + "/hand_left_camera/color/camera_info"),
                    ("/" + robot_name + "/hand_left_camera/color", "/" + robot_name + "/hand_left_camera/color/image_raw"),
                    ("/" + robot_name + "/hand_right_camera/camera_info", "/" + robot_name + "/hand_right_camera/color/camera_info"),
                    ("/" + robot_name + "/hand_right_camera/color", "/" + robot_name + "/hand_right_camera/color/image_raw"),
                ],
        parameters=[{
            'use_sim_time': True,
            f'qos_overrides./{robot_name}/head_camera/color/image_raw.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/head_camera/color/image_raw.publisher.depth': 1,
            f'qos_overrides./{robot_name}/hand_left_camera/color/image_raw.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/hand_left_camera/color/image_raw.publisher.depth': 1,
            f'qos_overrides./{robot_name}/hand_right_camera/color/image_raw.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/hand_right_camera/color/image_raw.publisher.depth': 1,
        }],
        output='screen'
    )

    # Bridge the depth IMAGE and stamp it with the OPTICAL frame_id. This image is the
    # single source of truth for everything 3D in sim:
    #   - MoveIt's DepthImageOctomapUpdater deprojects it via camera_info (optical
    #     convention), so its header MUST be head_camera_depth_optical_frame.
    #   - depth_image_proc regenerates the point cloud from it (below), inheriting the
    #     same optical frame -> matches the real Orbbec driver exactly.
    # The ros_gz_bridge `override_frame_id` param relabels header.frame_id for every
    # topic this node bridges (here: only the depth image). gz's own gz_frame_id is
    # left at head_camera_depth_frame in the SDF and is now irrelevant on the ROS side.
    gz_bridge_depth_image_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge_depth_image',
        namespace=robot_name,
        arguments=[
            "/" + robot_name + "/head_camera/depth" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
            "/" + robot_name + "/head_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/" + robot_name + "/head_camera/depth", "/" + robot_name + "/head_camera/depth/image_raw"),
            ("/" + robot_name + "/head_camera/camera_info", "/" + robot_name + "/head_camera/depth/camera_info"),
        ],
        parameters=[{
            'use_sim_time': True,
            'override_frame_id': 'head_camera_depth_optical_frame',
            f'qos_overrides./{robot_name}/head_camera/depth/image_raw.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/head_camera/depth/image_raw.publisher.depth': 1,
            f'qos_overrides./{robot_name}/head_camera/depth/camera_info.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/head_camera/depth/camera_info.publisher.depth': 1,
        }],
        output='screen'
    )

    # Regenerate the depth point cloud on the ROS side from the depth image +
    # camera_info, exactly as the real cam does.
    head_camera_point_cloud_xyz_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='head_camera_point_cloud_xyz',
        namespace=robot_name,
        parameters=[{
            'use_sim_time': True,
            f'qos_overrides./{robot_name}/head_camera/depth/image_raw.subscription.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/head_camera/depth/camera_info.subscription.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/head_camera/depth/points.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/head_camera/depth/points.publisher.depth': 1,
        }],
        remappings=[
            ("image_rect",  "/" + robot_name + "/head_camera/depth/image_raw"),
            ("camera_info", "/" + robot_name + "/head_camera/depth/camera_info"),
            ("points",      "/" + robot_name + "/head_camera/depth/points"),
        ],
        output='screen'
    )

    # Publish a compressedDepth transport of the raw depth image
    head_camera_depth_compressed_node = Node(
        package='image_transport',
        executable='republish',
        name='head_camera_depth_compressed_republisher',
        namespace=robot_name,
        arguments=['raw', 'compressedDepth'],
        remappings=[
            ("in",                  "/" + robot_name + "/head_camera/depth/image_raw"),
            ("out/compressedDepth", "/" + robot_name + "/head_camera/depth/image_raw/compressedDepth"),
        ],
        parameters=[{
            'use_sim_time': True,
            f'qos_overrides./{robot_name}/head_camera/depth/image_raw.subscription.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/head_camera/depth/image_raw/compressedDepth.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/head_camera/depth/image_raw/compressedDepth.publisher.depth': 1,
        }],
        output='log'
    )

    # Real hardware: ELP wrist cameras via usb_cam
    if not enable_gz:
        hand_left_cam_node = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='hand_left_camera',
            namespace=robot_name + '/hand_left_camera',
            parameters=[
                hand_left_cam_config,
                {"video_device": cam_left_port},
                {"frame_id": robot_name + '/hand_left_camera_optical_frame'},
                {f'qos_overrides./{robot_name}/hand_left_camera/color/image_raw.publisher.reliability': 'best_effort'},
                {f'qos_overrides./{robot_name}/hand_left_camera/color/image_raw.publisher.depth': 1},
                {f'qos_overrides./{robot_name}/hand_left_camera/color/image_raw/compressed.publisher.reliability': 'best_effort'},
                {f'qos_overrides./{robot_name}/hand_left_camera/color/image_raw/compressed.publisher.depth': 1},
                ],
            remappings=[
                ('image_raw', 'color/image_raw'),
                ('image_raw/compressed', 'color/image_raw/compressed'),
                ('image_raw/compressedDepth', 'color/image_raw/compressedDepth'),
                ('image_raw/theora', 'color/image_raw/theora'),
                ('image_raw/zstd', 'color/image_raw/zstd'),
            ],
            prefix='taskset -c 8-15',
            output='log',
        )

        hand_right_cam_node = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='hand_right_camera',
            namespace=robot_name + '/hand_right_camera',
            parameters=[
                hand_right_cam_config,
                {"video_device": cam_right_port},
                {"frame_id": robot_name + '/hand_right_camera_optical_frame'},
                {f'qos_overrides./{robot_name}/hand_right_camera/color/image_raw.publisher.reliability': 'best_effort'},
                {f'qos_overrides./{robot_name}/hand_right_camera/color/image_raw.publisher.depth': 1},
                {f'qos_overrides./{robot_name}/hand_right_camera/color/image_raw/compressed.publisher.reliability': 'best_effort'},
                {f'qos_overrides./{robot_name}/hand_right_camera/color/image_raw/compressed.publisher.depth': 1},
            ],
            remappings=[
                ('image_raw', 'color/image_raw'),
                ('image_raw/compressed', 'color/image_raw/compressed'),
                ('image_raw/compressedDepth', 'color/image_raw/compressedDepth'),
                ('image_raw/theora', 'color/image_raw/theora'),
                ('image_raw/zstd', 'color/image_raw/zstd'),
            ],
            prefix='taskset -c 8-15',
            output='log',
        )

    # action_server_launch is delayed until the drive stack is ready
    if enable_action_server:
        if enable_mobile_base:
            delayed_action_server = RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=swerve_controller,
                    on_start=[action_server_launch],
                )
            )
        elif controllers:
            delayed_action_server = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=controllers[-1],
                    on_exit=[action_server_launch],
                )
            )
        else:
            delayed_action_server = action_server_launch

    if enable_gz:
        nodes.append(gz_bridge_node)
        nodes.append(gz_spawn_entity_node)
        nodes.append(delayed_joint_state_broadcaster)
        nodes.append(delayed_controllers)
        if enable_action_server:
            nodes.append(delayed_action_server)

        if enable_head_cam_depth:
            nodes.append(gz_bridge_depth_image_node)
            nodes.append(head_camera_point_cloud_xyz_node)
            nodes.append(head_camera_depth_compressed_node)

        # Republish raw images as compressed for each camera in Gazebo
        for cam_name, enabled in [
            ('head_camera',       enable_head_cam_color),
            ('hand_left_camera',  enable_hand_left_cam_color),
            ('hand_right_camera', enable_hand_right_cam_color),
        ]:
            if enabled:
                nodes.append(Node(
                    package='image_transport',
                    executable='republish',
                    name=cam_name + '_compressed_republisher',
                    namespace=robot_name,
                    arguments=['raw', 'compressed'],
                    remappings=[
                        ('in',              '/' + robot_name + '/' + cam_name + '/color/image_raw'),
                        ('out/compressed',  '/' + robot_name + '/' + cam_name + '/color/image_raw/compressed'),
                    ],
                    parameters=[{
                        'use_sim_time': True,
                        f'qos_overrides./{robot_name}/{cam_name}/color/image_raw.subscription.reliability': 'best_effort',
                        f'qos_overrides./{robot_name}/{cam_name}/color/image_raw/compressed.publisher.reliability': 'best_effort',
                    }],
                    output='log',
                ))
    else:
        nodes.append(joint_state_broadcaster)
        nodes.append(control_node)
        nodes.extend(controllers)
        if enable_action_server:
            nodes.append(delayed_action_server)
        if enable_hand_left_cam_color:
            nodes.append(hand_left_cam_node)
        if enable_hand_right_cam_color:
            nodes.append(hand_right_cam_node)
    nodes.append(robot_state_publisher_node)
    nodes.append(sobits_display_launch)

    return nodes
