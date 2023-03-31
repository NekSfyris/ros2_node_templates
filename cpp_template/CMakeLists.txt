cmake_minimum_required(VERSION 3.5)
project(cpp_template)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED) # for timesync callbacks
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)


## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

add_executable(test_node src/test_node.cpp)

ament_target_dependencies(test_node 
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_filters
  px4_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen)


## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(TARGETS test_node
ARCHIVE DESTINATION lib/${PROJECT_NAME}
LIBRARY DESTINATION lib/${PROJECT_NAME}
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()