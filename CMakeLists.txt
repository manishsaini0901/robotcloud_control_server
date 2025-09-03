cmake_minimum_required(VERSION 3.8)
project(robotcloud_control_server)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robotcloud_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    robotcloud_control_server/robot_state_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
