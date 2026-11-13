cmake_minimum_required(VERSION 3.14)
project(serial LANGUAGES CXX)

find_package(Threads REQUIRED)

# Linux build of wjwwood/serial, embedded in um982_driver without catkin.
add_library(serial SHARED
  src/serial.cc
  src/impl/unix.cc
  src/impl/list_ports/list_ports_linux.cc
)
add_library(serial::serial ALIAS serial)

target_include_directories(serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(serial PUBLIC cxx_std_11)
target_link_libraries(serial PRIVATE Threads::Threads rt)

install(TARGETS serial
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/serial DESTINATION include)
