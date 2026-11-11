cmake_minimum_required(VERSION 3.8)
project(gnss_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

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

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)

ament_package()
