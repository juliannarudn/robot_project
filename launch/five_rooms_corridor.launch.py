from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=['-s', 'libgazebo_ros_factory.so', '~/robot_project/worlds/five_rooms_corridor.world'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen'
        )
    ])
