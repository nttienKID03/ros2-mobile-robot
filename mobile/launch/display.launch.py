from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mobile.urdf')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')  # đường dẫn tới file EKF

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # --- Node EKF (robot_localization) ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered'),  # có thể chỉnh nếu muốn rename
        ]
    )

    # --- RViz2 node ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # --- Goal Marker Publisher node ---
    goal_marker_node = Node(
        package='mobile',
        executable='goal_markers.py',
        name='goal_marker_publisher',
        output='screen'
    )

    # cam_pc_transform = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='pc2scan',
    #     parameters=[{
    #         'frame_id': 'cam_xa',
    #         'transform_tolerance': 0.05,
    #         'min_height': -1.0,
    #         'max_height': 1.0,
    #         'angle_min': -0.4,
    #         'angle_max': 0.4,
    #         'angle_increment': 0.005,
    #         'use_sim_time': True,
    #         'range_min': 0.2,
    #         'range_max': 15.0,
    #         'use_inf': True,
    #     }],
    #     remappings=[
    #         ('cloud_in', '/cam_xa_sensor/points'),
    #         ('scan', '/scan_camera')
    #     ]
    # )


    return LaunchDescription([
        ekf_node,
        rviz_node,
        goal_marker_node,
        # cam_pc_transform,
        
    ])
