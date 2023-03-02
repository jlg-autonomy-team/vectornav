cmake_minimum_required(VERSION 3.5)
# Compile options so compiles with ROS Kinetic
# Future releases may want to
add_compile_options(-std=c++17)
project(vectornav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Declare ROS2 messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ins.msg"
  "srv/SetFrameHorizontal.srv"
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

#include(vnproglib-1.1/cpp/CMakeLists.txt)
add_subdirectory(vnproglib-1.2.0.0/cpp)

## Declare a cpp library
## Declare a cpp executable
add_executable(vnpub src/main.cpp)
ament_target_dependencies(vnpub
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs)
target_link_libraries(vnpub
  libvncxx
  ${cpp_typesupport_target})

add_executable(vectornav_node src/vectornav_node.cpp src/vectornav.cpp)
target_include_directories(vectornav_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vectornav_node
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs)
target_link_libraries(vectornav_node
  libvncxx
  ${cpp_typesupport_target})

add_executable(vectornav_lifecycle_node src/vectornav_lifecycle_node.cpp src/vectornav_lifecycle.cpp)
target_include_directories(vectornav_lifecycle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vectornav_lifecycle_node
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  rclcpp_lifecycle)
target_link_libraries(vectornav_lifecycle_node
  libvncxx
  ${cpp_typesupport_target})


## Mark executables and/or libraries for installation
install(TARGETS
  vnpub vectornav_node vectornav_lifecycle_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
