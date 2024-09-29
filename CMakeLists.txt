cmake_minimum_required(VERSION 3.8)
project(nov_fij_draw_logo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(ament_dependencies
  rclcpp
  std_msgs
  geometry_msgs
)
include_directories(include)
add_executable(logo_turtlesim src/logo_turtlesim.cpp)
ament_target_dependencies(logo_turtlesim rclcpp geometry_msgs)
# ament_target_dependencies(simple_sub_node ${ament_dependencies} )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


install(TARGETS logo_turtlesim.cpp DESTINATION lib/${PROJECT_NAME})

ament_package()