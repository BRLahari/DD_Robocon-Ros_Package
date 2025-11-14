cmake_minimum_required(VERSION 3.10)
project(test_package)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-common3 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-transport8 REQUIRED)
find_package(ignition-utils1 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(
    include
    ${ignition-gazebo6_INCLUDE_DIRS}
    ${ignition-common3_INCLUDE_DIRS}
    ${ignition-plugin1_INCLUDE_DIRS}
    ${ignition-math6_INCLUDE_DIRS}
    ${ignition-transport8_INCLUDE_DIRS}
    ${ignition-utils1_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rclpy_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
)

# Add the library (shared plugin)
add_library(${PROJECT_NAME}_plugin SHARED 
    src/omniwheel_plugin.cpp
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}_plugin
    rclcpp
    rclpy
    geometry_msgs
    ignition-gazebo6
    ignition-common3
    ignition-plugin1
    ignition-math6
    ignition-transport8
    ignition-utils1
)

# Link additional libraries
target_link_libraries(${PROJECT_NAME}_plugin
    ${rclcpp_LIBRARIES}
    ${rclpy_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${ignition-gazebo6_LIBRARIES}
    ${ignition-common3_LIBRARIES}
    ${ignition-plugin1_LIBRARIES}
    ${ignition-math6_LIBRARIES}
    ${ignition-transport8_LIBRARIES}
    ${ignition-utils1_LIBRARIES}
)

# Install plugin
install(
    TARGETS ${PROJECT_NAME}_plugin
    LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
    script/controller.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch and SDF files
install(DIRECTORY
    launch
    sdf
    DESTINATION share/${PROJECT_NAME}/
)

# Export package
ament_package()

