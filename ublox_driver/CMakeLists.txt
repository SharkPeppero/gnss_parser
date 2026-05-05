cmake_minimum_required(VERSION 3.8)
project(ublox_driver)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gnss_comm REQUIRED)
find_package(gnss_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${GLOG_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/include/
)

set(UBLOX_DRIVER_SOURCES
        ${PROJECT_SOURCE_DIR}/apps/ublox_driver_node.cpp
        ${PROJECT_SOURCE_DIR}/src/ublox_driver.cpp
        ${PROJECT_SOURCE_DIR}/src/ros_handler.cpp
        ${PROJECT_SOURCE_DIR}/src/ublox_message_processor.cpp
        ${PROJECT_SOURCE_DIR}/src/serial_handler.cpp
        ${PROJECT_SOURCE_DIR}/src/tcp_client.cpp
        ${PROJECT_SOURCE_DIR}/src/loopback_tcp_client.cpp
        ${PROJECT_SOURCE_DIR}/src/ntrip_client.cpp
        ${PROJECT_SOURCE_DIR}/src/params.cpp
)

add_executable(${PROJECT_NAME} ${UBLOX_DRIVER_SOURCES})
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        sensor_msgs
        gnss_comm
        gnss_interfaces
)
target_link_libraries(${PROJECT_NAME}
        ${GLOG_LIBRARIES}
        Threads::Threads
        yaml-cpp
)

add_executable(sync_system_time ${PROJECT_SOURCE_DIR}/apps/sync_system_time_node.cpp)
ament_target_dependencies(sync_system_time
        rclcpp
        sensor_msgs
        gnss_comm
)
target_link_libraries(sync_system_time
        ${GLOG_LIBRARIES}
        Threads::Threads
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME} sync_system_time
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
