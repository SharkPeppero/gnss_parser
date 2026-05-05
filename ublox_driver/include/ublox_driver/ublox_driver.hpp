/**
 * @brief 负责串口、socket、ROS 与 UBX 解析模块的组装与调度。
 */
#ifndef UBLOX_DRIVER_UBLOX_DRIVER_HPP_
#define UBLOX_DRIVER_UBLOX_DRIVER_HPP_

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ublox_driver/loopback_tcp_client.hpp"
#include "ublox_driver/ntrip_client.hpp"
#include "ublox_driver/params.h"
#include "ublox_driver/ros_handler.hpp"
#include "ublox_driver/serial_handler.hpp"
#include "ublox_driver/tcp_client.hpp"
#include "ublox_driver/ublox_message_processor.hpp"

namespace ublox_driver {

class UbloxDriver {
public:
  UbloxDriver(rclcpp::Node::SharedPtr node, std::string config_filepath, std::string receiver_config_filepath);
  ~UbloxDriver();

private:
  bool configureReceiverAtStartup();
  void handleConfigAck(const uint8_t *data, size_t len);
  void handlePvtSolution(const gnss_comm::PVTSolutionPtr &pvt_soln);
  void sendNtripGga();
  std::string buildNmeaGgaSentence(const gnss_comm::PVTSolution &pvt_soln) const;
  void logRtcmInputChunk(size_t len);
  void logRtcmFrames(const uint8_t *data, size_t len);
  void drainRtcmLogBuffer();
  void setupSerialPipeline();
  void setupRtcmPipeline();

  rclcpp::Node::SharedPtr node_;
  std::string config_filepath_;
  std::string receiver_config_filepath_;
  UbloxDriverParams params_;

  std::shared_ptr<SerialHandler> serial_handler_;                  // 串口模块
  std::shared_ptr<TcpClient> rtcm_client_;                         // RTCM客户端
  std::shared_ptr<UbloxRosHandler> ros_handler_;                   // ROS管理层
  std::shared_ptr<UbloxMessageProcessor> ublox_message_processor_; // Ublox参数解析

  std::mutex ack_mutex_;
  std::condition_variable ack_cv_;
  int ack_flag_ = 0;

  std::mutex latest_pvt_mutex_;
  gnss_comm::PVTSolutionPtr latest_pvt_;
  rclcpp::TimerBase::SharedPtr ntrip_gga_timer_;

  std::mutex rtcm_log_mutex_;
  std::vector<uint8_t> rtcm_log_buffer_;
  uint64_t rtcm_input_chunk_count_ = 0;
  uint64_t rtcm_input_total_bytes_ = 0;
  uint64_t rtcm_frame_count_ = 0;
  uint64_t ntrip_gga_send_count_ = 0;
  uint64_t ntrip_gga_skip_count_ = 0;
};

} // namespace ublox_driver

#endif
