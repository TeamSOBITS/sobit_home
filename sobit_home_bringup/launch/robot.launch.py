import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_home')

    arg_robot_coords_x = DeclareLaunchArgument('robot_coords_x', default_value='0')
    arg_robot_coords_y = DeclareLaunchArgument('robot_coords_y', default_value='0')
    arg_robot_coords_z = DeclareLaunchArgument('robot_coords_z', default_value='0')
    arg_robot_coords_Y = DeclareLaunchArgument('robot_coords_Y', default_value='0')

    arg_enable_gz = DeclareLaunchArgument('enable_gz', default_value='True')
    arg_enable_mb = DeclareLaunchArgument('enable_mb', default_value='True')
    arg_enable_arm = DeclareLaunchArgument('enable_arm', default_value='False')
    arg_enable_head = DeclareLaunchArgument('enable_head', default_value='False')
    arg_enable_gz_front_cam_color = DeclareLaunchArgument('enable_gz_front_cam_color', default_value='True')
    arg_enable_gz_back_cam_color = DeclareLaunchArgument('enable_gz_back_cam_color', default_value='True')
    arg_enable_gz_head_cam_color = DeclareLaunchArgument('enable_gz_head_cam_color', default_value='True')
    arg_enable_gz_head_cam_depth = DeclareLaunchArgument('enable_gz_head_cam_depth', default_value='True')
    arg_enable_gz_hand_cam_color = DeclareLaunchArgument('enable_gz_hand_cam_color', default_value='True')
    arg_enable_gz_hand_cam_depth = DeclareLaunchArgument('enable_gz_hand_cam_depth', default_value='True')
    arg_enable_gz_lidar = DeclareLaunchArgument('enable_gz_lidar', default_value='True')
    arg_enable_gz_imu = DeclareLaunchArgument('enable_gz_imu', default_value='True')

    arg_enable_real_head_cam = DeclareLaunchArgument('enable_real_head_cam', default_value='True')
    arg_enable_real_hand_cam = DeclareLaunchArgument('enable_real_hand_cam', default_value='True')

    return LaunchDescription([
        arg_robot_name,
        arg_robot_coords_x,
        arg_robot_coords_y,
        arg_robot_coords_z,
        arg_robot_coords_Y,
        arg_enable_gz,
        arg_enable_mb,
        arg_enable_arm,
        arg_enable_head,
        arg_enable_gz_front_cam_color,
        arg_enable_gz_back_cam_color,
        arg_enable_gz_head_cam_color,
        arg_enable_gz_head_cam_depth,
        arg_enable_gz_hand_cam_color,
        arg_enable_gz_hand_cam_depth,
        arg_enable_gz_lidar,
        arg_enable_gz_imu,
        arg_enable_real_head_cam,
        arg_enable_real_hand_cam,
        OpaqueFunction(function = launch_gz),
    ])


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    robot_coords_x = LaunchConfiguration('robot_coords_x').perform(context)
    robot_coords_y = LaunchConfiguration('robot_coords_y').perform(context)
    robot_coords_z = LaunchConfiguration('robot_coords_z').perform(context)
    robot_coords_Y = LaunchConfiguration('robot_coords_Y').perform(context)

    enable_gz = LaunchConfiguration('enable_gz').perform(context)
    enable_mb = LaunchConfiguration('enable_mb').perform(context)
    enable_arm = LaunchConfiguration('enable_arm').perform(context)
    enable_head = LaunchConfiguration('enable_head').perform(context)
    enable_gz_front_cam_color = LaunchConfiguration('enable_gz_front_cam_color').perform(context)
    enable_gz_back_cam_color = LaunchConfiguration('enable_gz_back_cam_color').perform(context)
    enable_gz_head_cam_color = LaunchConfiguration('enable_gz_head_cam_color').perform(context)
    enable_gz_head_cam_depth = LaunchConfiguration('enable_gz_head_cam_depth').perform(context)
    enable_gz_hand_cam_color = LaunchConfiguration('enable_gz_hand_cam_color').perform(context)
    enable_gz_hand_cam_depth = LaunchConfiguration('enable_gz_hand_cam_depth').perform(context)
    enable_gz_lidar = LaunchConfiguration('enable_gz_lidar').perform(context)
    enable_gz_imu = LaunchConfiguration('enable_gz_imu').perform(context)

    enable_real_head_cam = LaunchConfiguration('enable_real_head_cam').perform(context) # TODO: Implement
    enable_real_hand_cam = LaunchConfiguration('enable_real_hand_cam').perform(context) # TODO: Implement

    robot_description = os.path.join(get_package_share_directory(
        'sobit_home_description'), 
        'robots',
        'sobit_home_robot.urdf.xacro'
    )
    robot_description_config = xacro.process_file(
        robot_description,
        mappings={
            'enable_gz'  : enable_gz,
            'enable_mb'  : enable_mb,
            'enable_arm' : enable_arm,
            'enable_head': enable_head,
            'robot_namespace': robot_name
        })
    
    if enable_gz == 'False':
        rviz_file = 'real.rviz'
        control_file = 'real_controllers.yaml'
    else:
        rviz_file = 'gazebo.rviz'
        control_file = 'gz_controllers.yaml'

    rviz_config = PathJoinSubstitution([
        FindPackageShare('sobit_home_bringup'),
        'rviz',
        rviz_file
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    controller_config = os.path.join(get_package_share_directory(
        'sobit_home_control'),
        'config',
        control_file
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=robot_name,
        parameters=[controller_config],
        remappings=[
            ("controller_manager/robot_description", "robot_description"),
        ],
        output="screen",
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])


    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
            '--set-state', 'active',
            '--controller-manager', robot_name+'/controller_manager',
            # '--use-sim-time',
            'joint_state_broadcaster'
        ],
        output='screen'
    )
    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster],
        )
    )

    wheel_steer_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
            '--set-state', 'active',
            '--controller-manager', robot_name+'/controller_manager',
            # '--use-sim-time',
            'wheel_steer_position_controller'
        ],
        output='screen'
    )
    delayed_wheel_steer_position_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[wheel_steer_position_controller],
        )
    )

    wheel_drive_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
            '--set-state', 'active',
            '--controller-manager', robot_name+'/controller_manager',
            # '--use-sim-time',
            'wheel_drive_velocity_controller'
        ],
        output='screen'
    )
    delayed_wheel_drive_velocity_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[wheel_drive_velocity_controller],
        )
    )

    linear_actuator_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
            '--set-state', 'active',
            '--controller-manager', robot_name+'/controller_manager',
            # '--use-sim-time',
            'linear_actuator_position_controller'
        ],
        output='screen'
    )
    delayed_linear_actuator_position_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[linear_actuator_position_controller],
        )
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=[
            {"frame_prefix": robot_name + '/'},
            {"robot_description": robot_description_config.toxml()},
            {"use_sim_time": True if enable_gz == 'True' else False},
        ],
        output="screen",
    )

    action_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sobit_home_library'),
                'launch',
                'action_server.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'enable_gz': enable_gz,
        }.items(),
    )

    if enable_gz == 'False':
        delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
        delayed_joint_state_broadcaster = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[joint_state_broadcaster],
            )
        )
        delayed_wheel_steer_position_controller = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[wheel_steer_position_controller],
            )
        )
        delayed_wheel_drive_velocity_controller = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[wheel_drive_velocity_controller],
            )
        )
    else:
        gz_spawn_entity_node = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=robot_name,
            arguments=[
                '-topic', '/' + robot_name + '/robot_description',
                '-name', robot_name,
                '-x', robot_coords_x,
                '-y', robot_coords_y,
                '-z', robot_coords_z,
                '-Y', robot_coords_Y,
            ],
            output='screen',
        )

        gz_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=robot_name,
            arguments=[
                        "/" + robot_name + "/joint_states" + "@sensor_msgs/msg/JointState" + "[ignition.msgs.Model",
                        # "/model/" + robot_name + "/pose" + "@geometry_msgs/msg/Pose" + "[ignition.msgs.Pose",
                        # "/" + robot_name + "/base_front_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        # "/" + robot_name + "/base_front_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/base_front_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/base_back_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        # "/" + robot_name + "/base_back_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/base_back_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/head_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        # "/" + robot_name + "/head_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/head_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/head_camera/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                        # "/" + robot_name + "/hand_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        # "/" + robot_name + "/hand_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/hand_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/hand_camera/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                        "/" + robot_name + "/lidar/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
                        "/" + robot_name + "/lidar/scan/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                        "/" + robot_name + "/imu" + "@sensor_msgs/msg/Imu" + "[ignition.msgs.IMU",
                    ],
            output='screen'
        )

        gz_tf_head_cam_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', robot_name + '/head_camera_depth_optical_frame',
                    '--child-frame-id', robot_name + '/head_pitch_link/head_camera_depth',
                    '--pitch', '-1.57',
                    '--roll', '1.57'],
            output='screen',
        )

        gz_tf_hand_cam_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', robot_name + '/hand_camera_depth_optical_frame',
                    '--child-frame-id', robot_name + '/arm_wrist_roll_link/hand_camera_depth',
                    '--pitch', '-1.57',
                    '--roll', '1.57'],
            output='screen',
        )

    if enable_gz == 'False':
        hand_camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sobit_home_bringup'),
                    'launch',
                    'include',
                    'rs_d405_hand_cam.launch.py'
                ])
            ]),
            condition=LaunchConfigurationEquals('enable_real_hand_cam', 'True')
        )

        head_camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sobit_home_bringup'),
                    'launch',
                    'include',
                    'rs_d415_head_cam.launch.py'
                ])
            ]),
            condition=LaunchConfigurationEquals('enable_real_head_cam', 'True')
        )
    
        return [
            delayed_controller_manager,
            delayed_wheel_steer_position_controller,
            delayed_wheel_drive_velocity_controller,
            delayed_linear_actuator_position_controller,
            delayed_joint_state_broadcaster,
            robot_state_publisher_node,
            rviz_node,
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=joint_state_broadcaster,
            #         on_exit=[hand_camera_launch],
            #     )
            # ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=joint_state_broadcaster,
            #         on_exit=[head_camera_launch],
            #     )
            # ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=joint_state_broadcaster,
            #         on_exit=[action_server_launch],
            #     )
            # ),
        ]
    
    else:
        return [
            gz_spawn_entity_node,
            gz_bridge_node,
            # gz_tf_head_cam_node,
            # gz_tf_hand_cam_node,
            delayed_controller_manager,
            delayed_wheel_steer_position_controller,
            delayed_wheel_drive_velocity_controller,
            delayed_linear_actuator_position_controller,
            delayed_joint_state_broadcaster,
            robot_state_publisher_node,
            rviz_node,
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=gz_spawn_entity_node,
            #         on_exit=[joint_state_broadcaster],
            #     )
            # ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=joint_state_broadcaster,
            #         on_exit=[joint_trajectory_controller],
            #     )
            # ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=joint_state_broadcaster,
            #         on_exit=[velocity_controller],
            #     )
            # ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=joint_state_broadcaster,
            #         on_exit=[diff_controller],
            #     )
            # ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=joint_state_broadcaster,
            #         on_exit=[action_server_launch],
            #     )
            # ),
        ]