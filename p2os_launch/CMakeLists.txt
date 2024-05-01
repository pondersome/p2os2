# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.5)
project(p2os_launch)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY 
   launch
   DESTINATION share/${PROJECT_NAME}
 
)
