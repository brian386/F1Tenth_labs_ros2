cmake_minimum_required(VERSION 3.5)
project(lab2_safety_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(safety_node src/SafetyNode.cpp)
ament_target_dependencies(safety_node rclcpp sensor_msgs std_msgs nav_msgs ackermann_msgs)

install(TARGETS
  safety_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
