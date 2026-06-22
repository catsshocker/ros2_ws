cmake_minimum_required(VERSION 3.8)
project(my_robot_base)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# find the package which i need
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(my_robot_interfaces REQUIRED)
#
add_executable(DCspeed_node src/DCspeed.cpp)
add_executable(motion_controller src/Motion.cpp src/motion/MoveTask.cpp)
add_executable(joy_controller src/JoyControl.cpp)

target_include_directories(
    motion_controller
    PUBLIC
    include
)

ament_target_dependencies(
    motion_controller
    rclcpp
    geometry_msgs
    std_msgs
    my_robot_interfaces
)

ament_target_dependencies(
    DCspeed_node
    rclcpp
    geometry_msgs
    std_msgs
)

ament_target_dependencies(
    joy_controller
    rclcpp
    geometry_msgs
    sensor_msgs
    std_msgs
)

install(
    TARGETS
    DCspeed_node
    motion_controller
    joy_controller
    DESTINATION lib/${PROJECT_NAME}
    )

ament_package()
