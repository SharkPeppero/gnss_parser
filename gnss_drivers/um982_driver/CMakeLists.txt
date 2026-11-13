cmake_minimum_required(VERSION 3.14)
project(um982_driver)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gnss_msg REQUIRED)
find_package(gnss_comm REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CURL REQUIRED)
find_package(Threads REQUIRED)

add_subdirectory(third_party/serial)

add_executable(um982_node src/um982_node.cpp src/unicore_messages.cpp src/rtcm_writer.cpp)
target_include_directories(um982_node PRIVATE include)
ament_target_dependencies(um982_node rclcpp ament_index_cpp std_msgs sensor_msgs gnss_msg gnss_comm)
target_link_libraries(um982_node serial::serial yaml-cpp CURL::libcurl Threads::Threads)

# The executable is installed in lib/um982_driver; libserial.so is in lib.
set_target_properties(um982_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

install(TARGETS um982_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
