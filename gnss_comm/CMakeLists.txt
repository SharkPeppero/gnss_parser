cmake_minimum_required(VERSION 3.5)
project(gnss_comm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gnss_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)

# Use modern CMake approach with target_include_directories
file(GLOB_RECURSE source_files ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} ${source_files})

# Set up include directories for the target
target_include_directories(${PROJECT_NAME} PUBLIC 
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIR} 
  ${GLOG_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(${PROJECT_NAME} 
  ${rclpy_LIBRARIES} 
  ${gnss_interfaces_LIBRARIES} 
  ${rclcpp_LIBRARIES} 
  ${ament_cmake_LIBRARIES} 
  ${GLOG_LIBRARIES}
)

# Add dependencies
ament_target_dependencies(${PROJECT_NAME} 
  rclpy 
  gnss_interfaces 
  rclcpp 
  ament_cmake
)

install(
        DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rclpy)
ament_export_dependencies(gnss_interfaces)
ament_export_dependencies(rclcpp)
ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
