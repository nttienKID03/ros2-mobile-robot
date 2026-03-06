from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_mobile = get_package_share_directory('mobile')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_mobile, 'map', 'map_gui.yaml')
    params_file = os.path.join(pkg_mobile, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ==== Gọi bringup của Nav2 ====
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(nav2_bringup)

    return ld