cmake_minimum_required(VERSION 3.5)
project(mobile)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)


install(DIRECTORY launch urdf meshes config map world rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(
  DIRECTORY world/
  DESTINATION share/${PROJECT_NAME}/world
)

install(PROGRAMS
  scripts/teleop_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/monitor_tab.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/lidar_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/moving_obstacle.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/goal_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/camera_to_costmap_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/lds50cr.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
