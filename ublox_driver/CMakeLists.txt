cmake_minimum_required(VERSION 3.8)
project(ublox_driver)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gnss_comm REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_library(UBLOX_GLOG_LIBRARY NAMES glog REQUIRED)

set(UBLOX_DRIVER_SOURCES
        apps/ublox_driver_node.cpp
        src/gnss_communication_wrapper/ublox_driver.cpp
        src/gnss_communication_wrapper/ros_handler.cpp
        src/gnss_message_parser/ublox_message_processor.cpp
        src/gnss_message_parser/ublox_message_processor_helper.cpp
        src/serial/serial_handler.cpp
        src/rtcm/tcp_client.cpp
        src/rtcm/loopback_tcp_client.cpp
        src/rtcm/ntrip_client.cpp
        src/params/params.cpp
)

add_executable(${PROJECT_NAME} ${UBLOX_DRIVER_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        sensor_msgs
        gnss_comm
)
target_link_libraries(${PROJECT_NAME}
        ${UBLOX_GLOG_LIBRARY}
        Threads::Threads
        yaml-cpp
)

add_executable(sync_system_time apps/sync_system_time_node.cpp)
target_include_directories(sync_system_time PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sync_system_time
        rclcpp
        sensor_msgs
        gnss_comm
)
target_link_libraries(sync_system_time
        ${UBLOX_GLOG_LIBRARY}
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

ament_export_include_directories(include)

ament_package()
