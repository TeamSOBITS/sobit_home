import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_gui',                    default_value='true'),
        DeclareLaunchArgument('robot_namespace',            default_value='sobit_home'),
        DeclareLaunchArgument('enable_gz',                  default_value='true'),
        DeclareLaunchArgument('enable_mobile_base',         default_value='true'),
        DeclareLaunchArgument('enable_body',                default_value='true'),
        DeclareLaunchArgument('enable_arm_left',            default_value='true'),
        DeclareLaunchArgument('enable_arm_right',           default_value='true'),
        DeclareLaunchArgument('enable_hand_left',           default_value='true'),
        DeclareLaunchArgument('enable_hand_right',          default_value='true'),
        DeclareLaunchArgument('enable_head',                default_value='true'),
        DeclareLaunchArgument('enable_head_cam_color',      default_value='true'),
        DeclareLaunchArgument('enable_head_cam_depth',      default_value='true'),
        DeclareLaunchArgument('enable_hand_left_cam_color', default_value='true'),
        DeclareLaunchArgument('enable_hand_right_cam_color',default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])

def _bool_str(val):
    return 'True' if val.lower() in ('true', '1', 'yes') else 'False'

def launch_setup(context, *args, **kwargs):
    use_gui          = LaunchConfiguration('use_gui').perform(context)
    robot_namespace  = LaunchConfiguration('robot_namespace').perform(context)
    enable_gz        = _bool_str(LaunchConfiguration('enable_gz').perform(context))
    enable_mobile_base = _bool_str(LaunchConfiguration('enable_mobile_base').perform(context))
    enable_body      = _bool_str(LaunchConfiguration('enable_body').perform(context))
    enable_arm_left  = _bool_str(LaunchConfiguration('enable_arm_left').perform(context))
    enable_arm_right = _bool_str(LaunchConfiguration('enable_arm_right').perform(context))
    enable_hand_left = _bool_str(LaunchConfiguration('enable_hand_left').perform(context))
    enable_hand_right = _bool_str(LaunchConfiguration('enable_hand_right').perform(context))
    enable_head      = _bool_str(LaunchConfiguration('enable_head').perform(context))
    enable_head_cam_color = _bool_str(LaunchConfiguration('enable_head_cam_color').perform(context))
    enable_head_cam_depth = _bool_str(LaunchConfiguration('enable_head_cam_depth').perform(context))
    enable_hand_left_cam_color = _bool_str(LaunchConfiguration('enable_hand_left_cam_color').perform(context))
    enable_hand_right_cam_color = _bool_str(LaunchConfiguration('enable_hand_right_cam_color').perform(context))

    rviz_config = os.path.join(get_package_share_directory(
        'sobit_home_description'), "rviz", "display.rviz")
    
    robot_description = os.path.join(get_package_share_directory(
        'sobit_home_description'), 
        'robots',
        'sobit_home_robot.urdf.xacro'
    )

    xacro_arguments = {
        'robot_namespace': robot_namespace,
        'enable_gz': enable_gz,
        'enable_mobile_base': enable_mobile_base,
        'enable_body': enable_body,
        'enable_arm_left': enable_arm_left,
        'enable_arm_right': enable_arm_right,
        'enable_hand_left': enable_hand_left,
        'enable_hand_right': enable_hand_right,
        'enable_head': enable_head,
        'enable_head_cam_color': enable_head_cam_color,
        'enable_head_cam_depth': enable_head_cam_depth,
        'enable_hand_left_cam_color': enable_hand_left_cam_color,
        'enable_hand_right_cam_color': enable_hand_right_cam_color,
    }
    robot_description_config = xacro.process_file(
        robot_description,
        mappings=xacro_arguments
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_namespace,
        parameters=[{
            "robot_description": robot_description_config.toxml(),
            "use_sim_time": True,
        }],
        output="screen",
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        namespace=robot_namespace,
        condition=UnlessCondition('true' if use_gui.lower() in ('true', '1', 'yes') else 'false')
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        namespace=robot_namespace,
        condition=IfCondition('true' if use_gui.lower() in ('true', '1', 'yes') else 'false')
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return [
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ]