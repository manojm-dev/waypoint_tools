cmake_minimum_required(VERSION 3.8)
project(waypoint_tools)

#Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)


# Add include directory
include_directories(include)

# Common dependencies
set(COMMON_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  nlohmann_json
)

add_executable(waypoint_recorder src/waypoint_recorder_node.cpp)
ament_target_dependencies(waypoint_recorder ${COMMON_DEPENDENCIES})

install(
  TARGETS waypoint_recorder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
