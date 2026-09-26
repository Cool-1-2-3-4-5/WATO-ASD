cmake_minimum_required(VERSION 3.10)
project(costmap)
 
# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)      
find_package(std_msgs REQUIRED)    
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 1. Compile source files into a library AND add its dependencies
add_library(costmap_lib src/costmap_core.cpp)
target_include_directories(costmap_lib PUBLIC include)
ament_target_dependencies(costmap_lib 
  rclcpp 
  nav_msgs 
  sensor_msgs 
  std_msgs
)

# 2. Create ROS2 node executable
add_executable(costmap_node src/costmap_node.cpp)

# 3. Link the library and add dependencies
target_link_libraries(costmap_node costmap_lib)
ament_target_dependencies(costmap_node
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
)
 
# Copy executable to installation location
install(TARGETS
  costmap_node
  DESTINATION lib/${PROJECT_NAME})
 
# Copy launch and config files to installation location
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
 
ament_package()
