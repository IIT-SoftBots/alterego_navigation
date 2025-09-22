cmake_minimum_required(VERSION 3.22)
project(alterego_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(alterego_msgs REQUIRED) # Provides NavService / WhereRUService equivalents if migrated

# Install Python executable scripts
install(PROGRAMS
  src/nav2points.py
  src/initialpose.py
  src/nav2points_service.py
  src/send_key_points.py
  src/where_are_you_service.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration (maps, yaml, launch XML kept for reference)
install(DIRECTORY config launch srv
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
