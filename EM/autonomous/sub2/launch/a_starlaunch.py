from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sub2',
            node_executable='load_map',
            node_name='load_map',
            output='screen'
        ),
        Node(
            package='sub2',
            node_executable='odom',
            node_name='odom',
            output='screen'
        ),
        Node(
            package='sub2',
            node_executable='a_star',
            node_name='a_star',
            output='screen'
        ),
        Node(
            package='sub2',
            node_executable='a_star_local_path',
            node_name='a_star_local_path',
            output='screen'
        ),
        Node(
            package='sub2',
            node_executable='path_tracking',
            node_name='path_tracking',
            output='screen'
        ),
        Node(
            package='sub2',
            node_executable='rotation',
            node_name='rotation',
            output='screen'
        ),
        Node(
            package='sub2',
            node_executable='goal_pub',
            node_name='goal_pub',
            output='screen'
        )
    ])