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

    # RVIZ conditional group
    rviz_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ocs2_ballbot_ros'),
                    'launch',
                    'visualize.launch.py'
                )
            ]),
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    ])

    # Nodes
    ballbot_mpc_node = Node(
        package='ocs2_ballbot_ros',
        executable='ballbot_sqp',
        name='ballbot_sqp',
        output='screen',
        arguments=[LaunchConfiguration('task_name')],
    )

    ballbot_dummy_test_node = Node(
        package='ocs2_ballbot_ros',
        executable='ballbot_dummy_test',
        name='ballbot_dummy_test',
        output='screen',
        arguments=[LaunchConfiguration('task_name')],
        prefix='gnome-terminal --'
    )

    ballbot_target_node = Node(
        package='ocs2_ballbot_ros',
        executable='ballbot_target',
        name='ballbot_target',
        output='screen',
        arguments=[LaunchConfiguration('task_name')],
        prefix='gnome-terminal --'
    )

    # Launch description
    return LaunchDescription([
        rviz_arg,
        task_name_arg,
        rviz_group,
        ballbot_mpc_node,
        ballbot_dummy_test_node,
        ballbot_target_node
    ])
