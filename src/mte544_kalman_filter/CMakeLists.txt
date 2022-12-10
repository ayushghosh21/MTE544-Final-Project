cmake_minimum_required(VERSION 3.5)
project(mte544_kalman_filter)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


find_package(std_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_py REQUIRED)


install(DIRECTORY
  src
  launch
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/mte544_kalman_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
