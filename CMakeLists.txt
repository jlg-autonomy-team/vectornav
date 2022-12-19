cmake_minimum_required(VERSION 3.5)
# Compile options so compiles with ROS Kinetic
# Future releases may want to
add_compile_options(-std=c++17)
project(vectornav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vnpub
  libvncxx
  ${cpp_typesupport_target}
)

#add_executable(vectornav_node src/vectornav_node.cpp src/vectornav.cpp)
#target_link_libraries(vectornav_node
#  libvncxx
#  ${catkin_LIBRARIES})


## Mark executables and/or libraries for installation
install(TARGETS
  vnpub #vectornav_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
