from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from moveit_configs_utils.launches import generate_rsp_launch
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("sobit_home_description"),
            "robots",
            "sobit_home_robot.urdf.xacro",
        ]),
        " enable_gz:=False",
        " enable_mobile_base:=True",
        " enable_body:=True",
        " enable_arm_left:=True",
        " enable_arm_right:=True",
        " enable_hand_left:=True",
        " enable_hand_right:=True",
        " enable_head:=True",
        " enable_head_cam_color:=False",
        " enable_head_cam_depth:=False",
        " enable_hand_left_cam_color:=False",
        " enable_hand_right_cam_color:=False",
    ])

    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    rviz_config = PathJoinSubstitution([
            FindPackageShare('sobit_home_moveit_config'),
            'rviz',
            'moveit.rviz'
    ])

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='sobit_home',
        description='Robot name used as namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_robot_description = DeclareLaunchArgument(
        "loaded_description",
        default_value=robot_description,
        description="URDF generated from xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder("sobit_home", package_name="sobit_home_moveit_config")
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    moveit_config.robot_description = {
        "robot_description": ParameterValue(
            LaunchConfiguration("loaded_description"),
            value_type=str
        )
    }

    config_dict = moveit_config.to_dict()
    config_dict['robot_description_planning.frame_prefix'] = \
        'sobit_home/'

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=robot_name,
        output="screen",
        parameters=[
            config_dict,
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_node_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            # moveit_config.sensors_3d,
            {'use_sim_time': use_sim_time,
             'robot_description_planning.frame_prefix': 'sobit_home/'},
        ],
    )

    return LaunchDescription(
        [
            declare_robot_name_cmd,
            declare_robot_description,
            declare_use_sim_time_cmd,
            declare_use_rviz_cmd,
            move_group_node,
            generate_static_virtual_joint_tfs_launch(moveit_config),
            generate_rsp_launch(moveit_config),
            rviz_node_cmd,
        ]
    )
