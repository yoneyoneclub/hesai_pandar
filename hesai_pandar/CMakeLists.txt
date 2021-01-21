cmake_minimum_required(VERSION 3.0.2)
project(hesai_pandar)

add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  image_transport
  pcl_conversions
  pcl_ros
  pandar_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    pandar_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Target
## autoware_state_monitor_node
add_executable(hesai_pandar
  src/hesai_pandar/main.cpp
  src/hesai_pandar/hesai_pandar_node.cpp
  src/hesai_pandar/socket_input.cpp
  src/hesai_pandar/tcp_command_client.cpp
  src/hesai_pandar/tcp_util.cpp
  src/hesai_pandar/calibration.cpp
  src/hesai_pandar/pcap_input.cpp
  src/hesai_pandar/decoder/pandar40_decoder.cpp
)

target_link_libraries(hesai_pandar
    ${catkin_LIBRARIES}
    pcap
)


# Install
## executables and libraries
install(
  TARGETS
  hesai_pandar
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## directories
install(
  DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
