cmake_minimum_required(VERSION 3.10)
project(map_memory)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(map_memory_node src/map_memory_node.cpp)

target_include_directories(map_memory_node
  PUBLIC include)

ament_target_dependencies(map_memory_node 
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  map_memory_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
