cmake_minimum_required(VERSION 3.5)
project(p2os_urdf)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(
  
)

add_executable(p2os_publisher src/publisher.cc)

add_executable(p2os_publisher_3at src/publisher3at.cc)
add_executable(p2os_publisher_3dx src/publisher3dx.cc)
ament_target_dependencies(p2os_publisher)
ament_target_dependencies(p2os_publisher_3at)
ament_target_dependencies(p2os_publisher_3dx)
ament_export_dependencies(tf2_ros urdf geometry_msgs sensor_msgs std_msgs kdl_parser)
ament_package()

install(TARGETS p2os_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS p2os_publisher_3at p2os_publisher_3dx
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
   launch defs meshes models
   DESTINATION share/${PROJECT_NAME}
 
)
