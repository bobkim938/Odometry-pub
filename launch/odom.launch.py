from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    odom_node = Node(
        package='odom',
        executable='odom_node',
        name='odom_node',
        parameters=['./src/odom/config/params.yaml']
    )
    ld.add_action(odom_node)
    return ld