from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetParameter

# SetParameter(name='use_sim_time', value=True),


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mobile.urdf')
    world_file = os.path.join(pkg_share, 'world', 'maze')  

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()


    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                world_file,   # load world vào Gazebo
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'

            ],
            output='screen'
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'mobile'],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_desc}
            ]
        ),

    ])