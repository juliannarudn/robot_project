from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    world_file = os.path.expanduser('~/robot_project/worlds/five_rooms_corridor_fixed.world')
    model_path = '/opt/ros/' + os.environ.get('ROS_DISTRO', 'humble') + '/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf'
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gzserver', world_file, '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_burger',
                '-file', model_path,
                '-x', '-3', '-y', '2.5', '-z', '0.05'
            ],
            output='screen'
        ),
        Node(
            package='lqg_controller',
            executable='lqg_controller',
            parameters=[{'goal_x': 2.0, 'goal_y': -1.0, 'goal_theta': 0.0}],
            output='screen'
        )
    ])
