cmake_minimum_required(VERSION 3.10)
project(control)

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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 1. Create ROS2 node executable from source files FIRST
add_executable(control_node src/control_node.cpp)

# 2. Indicate to compiler where to search for header files
target_include_directories(control_node PUBLIC include)

# 3. Add ROS2 dependencies required by the executable SECOND
ament_target_dependencies(control_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# Copy executable to installation location
install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  # Ignore error if config folder doesn't exist yet
  OPTIONAL 
)

ament_package()
