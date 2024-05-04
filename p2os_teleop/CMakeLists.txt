cmake_minimum_required(VERSION 3.9.5)
project(p2os_teleop)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  
)

add_executable(p2os_teleop src/p2os_teleop.cc)
ament_target_dependencies(p2os_teleop rclcpp geometry_msgs sensor_msgs)
ament_export_libraries(lib)
ament_export_dependencies(geometry_msgs sensor_msgs tf2_ros std_msgs rclcpp)
ament_package()

install(TARGETS p2os_teleop
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
