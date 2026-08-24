import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    share_dir = get_package_share_directory('arcs2_bringup')
    params_file = os.path.join(share_dir, 'config', 'params.yaml')
    rviz_config_file = os.path.join(share_dir, 'rviz', 'view.rviz')

    ns_arg = DeclareLaunchArgument('namespace', default_value='', description='Namespace for the nodes')
    ns = LaunchConfiguration('namespace')

    use_node_3_arg = DeclareLaunchArgument('use_node_3', default_value='True')
    use_node_3 = LaunchConfiguration('use_node_3')

    simple_count_node_1 = Node(
        name='simple_count_node_1',
        package='arcs2_bringup',
        executable='simple_count.py',
        parameters=[params_file],
    )

    simple_count_node_2 = Node(
        name='simple_count_node_2',
        namespace=ns,
        package='arcs2_bringup',
        executable='simple_count.py',
        parameters=[params_file],
    )

    simple_count_node_3 = Node(
        name='simple_count_node_3',
        namespace=ns,
        package='arcs2_bringup',
        executable='simple_count.py',
        parameters=[params_file],
        remappings=[('count', 'count_3')],
        condition=IfCondition(use_node_3),
    )

    timer_action = TimerAction(
        period=1.0,
        actions=[simple_count_node_2],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        ns_arg,
        use_node_3_arg,

        simple_count_node_1,
        timer_action,
        simple_count_node_3,
        rviz_node
    ])