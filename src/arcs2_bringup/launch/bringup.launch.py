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
    rviz_file = os.path.join(share_dir, 'config', 'view.rviz')

    ns_arg = DeclareLaunchArgument('namespace', default_value='arcs2')
    ns = LaunchConfiguration('namespace')

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_rviz = LaunchConfiguration('use_rviz')

    simple_count_node_1 = Node(
        name='simple_count_node_1',
        package='arcs2_bringup',
        executable='simple_count.py',
        parameters=[{
            'count': 0,
            'multiplier': 1,
            'publish_rate': 1.0,
            'log': True,
        }],
    )

    simple_count_node_2 = Node(
        name='simple_count_node_2',
        namespace='simple_count',
        package='arcs2_bringup',
        executable='simple_count.py',
        parameters=[params_file],
        remappings=[('count', 'count_2')],
    )

    simple_count_node_3 = Node(
        name='simple_count_node_3',
        namespace=ns,
        package='arcs2_bringup',
        executable='simple_count.py',
        parameters=[params_file],
        remappings=[('count', 'count_3')],
    )

    timer_action = TimerAction(
        period=5.0,
        actions=[simple_count_node_3]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        ns_arg,
        use_rviz_arg,

        simple_count_node_1,
        simple_count_node_2,
        timer_action,
        rviz_node,
    ])
