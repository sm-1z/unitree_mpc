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
                    get_package_share_directory('ocs2_double_integrator_ros'),
                    'launch',
                    'visualize.launch.py'
                )
            ]),
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    ])

    # Nodes
    double_integrator_mpc_node = Node(
        package='ocs2_double_integrator_ros',
        executable='double_integrator_mpc',
        name='double_integrator_mpc',
        output='screen',
        arguments=[LaunchConfiguration('task_name')],
    )

    double_integrator_dummy_test_node = Node(
        package='ocs2_double_integrator_ros',
        executable='double_integrator_dummy_test',
        name='double_integrator_dummy_test',
        output='screen',
        arguments=[LaunchConfiguration('task_name')],
        prefix='gnome-terminal --'
    )

    double_integrator_target_node = Node(
        package='ocs2_double_integrator_ros',
        executable='double_integrator_target',
        name='double_integrator_target',
        output='screen',
        arguments=[LaunchConfiguration('task_name')],
        prefix='gnome-terminal --'
    )

    # Launch description
    return LaunchDescription([
        rviz_arg,
        task_name_arg,
        rviz_group,
        double_integrator_mpc_node,
        double_integrator_dummy_test_node,
        double_integrator_target_node
    ])
