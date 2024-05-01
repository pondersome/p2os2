cmake_minimum_required(VERSION 3.5)
project(p2os_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 1 4)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/SonarArray.msg
        msg/ArmState.msg
        msg/AIO.msg
        msg/GripState.msg
        msg/TCMState.msg
        msg/MotorState.msg
        msg/LiftState.msg
        msg/DIO.msg
        msg/PTZState.msg
        msg/GripperState.msg
        msg/BatteryState.msg
    DEPENDENCIES
)

ament_package()
