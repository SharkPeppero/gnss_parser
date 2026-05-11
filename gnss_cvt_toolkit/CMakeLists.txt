cmake_minimum_required(VERSION 3.8)
project(gnss_cvt_toolkit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gnss_comm REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

add_executable(bag2rinex src/bag2rinex.cpp)
ament_target_dependencies(bag2rinex
  gnss_comm
  rclcpp
  rosbag2_cpp
  rosbag2_storage
)

add_executable(bag2rtk_solution src/bag2rtk_solution.cpp)
ament_target_dependencies(bag2rtk_solution
  gnss_comm
  rclcpp
  rosbag2_cpp
  rosbag2_storage
)

install(TARGETS
  bag2rinex
  bag2rtk_solution
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

ament_package()
