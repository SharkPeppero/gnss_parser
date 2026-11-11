cmake_minimum_required(VERSION 3.8)
project(gnss_comm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gnss_msg REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(fusion_location_utils REQUIRED)

file(GLOB_RECURSE source_files CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME}_core ${source_files})

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR} 
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  std_msgs
  gnss_msg
)

target_link_libraries(${PROJECT_NAME}_core
  glog::glog
)

add_executable(spp_solver_node apps/spp_solver_node.cpp)
target_link_libraries(spp_solver_node ${PROJECT_NAME}_core ${OpenCV_LIBS})
target_include_directories(spp_solver_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(spp_solver_node rclcpp gnss_msg sensor_msgs geometry_msgs fusion_location_utils)
install(TARGETS spp_solver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz_cfg DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME}_core HAS_LIBRARY_TARGET)
ament_export_dependencies(glog)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(gnss_msg)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)

ament_package()
