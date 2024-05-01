cmake_minimum_required(VERSION 3.9.5)
project(p2os_driver)
find_package(ament_cmake REQUIRED)
find_package(p2os_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)

## Specify additional locations of header files
include_directories(include
)

## Declare a cpp executable
add_executable(p2os_driver
  src/p2osnode.cpp
  src/p2os.cpp
  src/kinecalc.cpp
  src/packet.cpp
  src/robot_params.cpp
  src/sip.cpp
  src/p2os_ptz.cpp
)

ament_target_dependencies(p2os_driver "tf2_ros" "diagnostic_updater" "geometry_msgs" "nav_msgs" "rclcpp")
ament_export_include_directories(include)
ament_export_libraries(p2os_driver)
ament_export_dependencies(p2os_msgs nav_msgs rclcpp geometry_msgs diagnostic_updater tf2_ros std_msgs kdl_parser)
ament_package()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Mark executables and/or libraries for installation
install(TARGETS p2os_driver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
