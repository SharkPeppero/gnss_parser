cmake_minimum_required(VERSION 3.8)
project(gnss_comm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)

set(msg_files
  "msg/GnssBestXYZMsg.msg"
  "msg/GnssEphemMsg.msg"
  "msg/GnssGloEphemMsg.msg"
  "msg/GnssMeasMsg.msg"
  "msg/GnssObsMsg.msg"
  "msg/GnssPVTSolnMsg.msg"
  "msg/GnssSvsMsg.msg"
  "msg/GnssTimeMsg.msg"
  "msg/GnssTimePulseInfoMsg.msg"
  "msg/RtcmMsg.msg"
  "msg/StampedFloat64Array.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

file(GLOB_RECURSE source_files ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME}_core ${source_files})

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR} 
  ${GLOG_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_core
  ${GLOG_LIBRARIES}
  ${cpp_typesupport_target}
)
set_property(TARGET ${PROJECT_NAME}_core PROPERTY INTERFACE_LINK_LIBRARIES "${cpp_typesupport_target}")

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME}_core HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)

ament_package()
