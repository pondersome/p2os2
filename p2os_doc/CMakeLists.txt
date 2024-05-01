cmake_minimum_required(VERSION 3.5)
project(p2os_doc)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY 
   p2os_driver
   DESTINATION share/${PROJECT_NAME}
 
)