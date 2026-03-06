from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    model_file = '/home/nttien/ros2_ws/src/mobile/models/obstacle_box.sdf'
    model_fil = '/home/nttien/ros2_ws/src/mobile/models/obstacle_bar.sdf'

    spawn_static_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'obstacle_static_1', '-file', model_file, '-x', '9', '-y', '0', '-z', '0.35']
    )

    spawn_static_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'obstacle_static_2', '-file', model_file, '-x', '19', '-y', '0', '-z', '0.35']
    )


    spawn_static_3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'obstacle_static_3', '-file', model_file, '-x', '29', '-y', '0', '-z', '0.35']
    )
    spawn_static_4 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'obstacle_static_4', '-file', model_file, '-x', '39', '-y', '0', '-z', '0.35']
    )

    spawn_static_5 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'obstacle_static_5', '-file', model_file, '-x', '49', '-y', '0', '-z', '0.35']
    )


    spawn_static_6 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'obstacle_static_6', '-file', model_file, '-x', '59', '-y', '0', '-z', '0.35']
    )

    #hang trai B   
    spawn_static_7 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'obstacle_static_7', '-file', model_file, '-x', '9', '-y', '16', '-z', '0.35']
    )

    spawn_static_8 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'obstacle_static_8', '-file', model_file, '-x', '19', '-y', '16', '-z', '0.35']
    )


    spawn_static_9 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'obstacle_static_9', '-file', model_file, '-x', '29', '-y', '16', '-z', '0.35']
    )
    spawn_static_10 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'obstacle_static_10', '-file', model_file, '-x', '39', '-y', '16', '-z', '0.35']
    )

    spawn_static_11 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'obstacle_static_11', '-file', model_file, '-x', '49', '-y', '16', '-z', '0.35']
    )


    spawn_static_12 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'obstacle_static_12', '-file', model_file, '-x', '59', '-y', '16', '-z', '0.35']
    )

    spawn_moving = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'obstacle_moving', '-file', model_fil, '-x', '0', '-y', '5', '-z', '0.35']
    )


    # move_script = Node(
    #     package='mobile',
    #     executable='moving_obstacle.py',
    #     output='screen'
    # )

    return LaunchDescription([
        spawn_static_1,
        spawn_static_2,
        spawn_static_3,
        spawn_static_4,
        spawn_static_5,
        spawn_static_6,
        spawn_static_7,
        spawn_static_8,
        spawn_static_9,
        spawn_static_10,
        spawn_static_11,
        spawn_static_12,
        spawn_moving,
        #move_script
    ])
