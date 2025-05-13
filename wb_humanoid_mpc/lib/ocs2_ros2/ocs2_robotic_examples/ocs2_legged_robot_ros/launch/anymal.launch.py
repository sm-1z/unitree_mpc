from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RVIZ?')

    task_name_arg = DeclareLaunchArgument(
        'task_name', default_value='mpc',
        description='Task name')

    reference_name_arg = DeclareLaunchArgument(
        'reference_name', default_value='command',
        description='Reference name')

    # RVIZ conditional group
    rviz_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ocs2_legged_robot_ros'),
                    'launch',
                    'visualize.launch.py'
                )
            ]),
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    ])

    # Nodes
    legged_robot_mpc_node = Node(
        package='ocs2_legged_robot_ros',
        executable='legged_robot_sqp_mpc',
        name='legged_robot_sqp_mpc',
        output='screen',
        arguments=[LaunchConfiguration('task_name'), LaunchConfiguration('reference_name')],
    )

    legged_robot_dummy_test_node = Node(
        package='ocs2_legged_robot_ros',
        executable='legged_robot_dummy',
        name='legged_robot_dummy',
        output='screen',
        arguments=[LaunchConfiguration('task_name'), LaunchConfiguration('reference_name')],
        prefix='gnome-terminal --'
    )

    legged_robot_target_node = Node(
        package='ocs2_legged_robot_ros',
        executable='legged_robot_target',
        name='legged_robot_target',
        output='screen',
        arguments=[LaunchConfiguration('task_name'), LaunchConfiguration('reference_name')],
        prefix='gnome-terminal --'
    )

    legged_robot_gait_command_node = Node(
        package='ocs2_legged_robot_ros',
        executable='legged_robot_gait_command',
        name='legged_robot_gait_command',
        output='screen',
        arguments=[LaunchConfiguration('task_name'), LaunchConfiguration('reference_name')],
        prefix='gnome-terminal --'
    )

    # Launch description
    return LaunchDescription([
        rviz_arg,
        task_name_arg,
        reference_name_arg,
        rviz_group,
        legged_robot_mpc_node,
        legged_robot_dummy_test_node,
        legged_robot_target_node,
        legged_robot_gait_command_node
    ])
