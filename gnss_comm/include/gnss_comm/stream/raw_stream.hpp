//
// Created by yjh on 2026/11/12.
//

#ifndef GNSS_COMM_STREAM_RAW_STREAM_HPP_
#define GNSS_COMM_STREAM_RAW_STREAM_HPP_

#include "gnss_comm/types/ephemeris.hpp"
#include "gnss_comm/types/observation.hpp"
#include <atomic>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <gnss_msg/msg/gnss_ephem_msg.hpp>
#include <gnss_msg/msg/gnss_glo_ephem_msg.hpp>
#include <gnss_msg/msg/gnss_meas_msg.hpp>
#include <gnss_msg/msg/stamped_float64_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <string>
#include <thread>
#include <vector>

namespace gnss_comm {

// 流动站原始数据的输出回调；同一数据源的回调在同一线程中按时间顺序触发。
struct RawStreamCallbacks {
  std::function<void(const EphemBasePtr &)> ephem;
  std::function<void(const std::vector<double> &)> iono;           // GPS 广播电离层 8 参数
  std::function<void(const Eigen::Vector3d &)> reference_lla;      // ENU 原点候选，纬度/经度 deg、高度 m
  std::function<void(const std::vector<ObsPtr> &)> obs;            // 一个历元的全部卫星观测
  std::function<void()> finished;                                  // 数据源结束（仅离线文件）
};

// 流动站原始数据源：星历、电离层参数与逐历元观测。
class RawStream {
public:
  virtual ~RawStream() = default;
  virtual void start(const RawStreamCallbacks &callbacks) = 0;
  virtual void stop() {}
};

// 在线：订阅驱动或 ros2 bag 回放的话题，回调在节点的执行器线程中触发。
struct RosRawTopics {
  std::string range_meas, ephem, glo_ephem, iono_params, receiver_lla;
};

class RosRawStream : public RawStream {
public:
  RosRawStream(rclcpp::Node &node, const RosRawTopics &topics) : node_(node), topics_(topics) {}
  void start(const RawStreamCallbacks &callbacks) override;

private:
  rclcpp::Node &node_;
  RosRawTopics topics_;
  RawStreamCallbacks callbacks_;
  rclcpp::Subscription<gnss_msg::msg::GnssEphemMsg>::SharedPtr ephem_sub_;
  rclcpp::Subscription<gnss_msg::msg::GnssGloEphemMsg>::SharedPtr glo_sub_;
  rclcpp::Subscription<gnss_msg::msg::StampedFloat64Array>::SharedPtr iono_sub_;
  rclcpp::Subscription<gnss_msg::msg::GnssMeasMsg>::SharedPtr meas_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr lla_sub_;
};

// 离线：一次性读取流动站 RINEX 观测与导航文件，在独立线程中先送出全部星历和电离层参数，再按时间顺序送出观测。
// playback_rate 为 0 时尽快处理；> 0 时按观测时间间隔 / playback_rate 节流，便于在 RViz 中观察。
// 观测文件头 APPROX POSITION 非零时作为 ENU 原点候选。
class RinexRawStream : public RawStream {
public:
  RinexRawStream(const std::vector<std::string> &obs_paths, const std::vector<std::string> &nav_paths, double playback_rate)
      : obs_paths_(obs_paths), nav_paths_(nav_paths), playback_rate_(playback_rate) {}
  ~RinexRawStream() override { stop(); }
  void start(const RawStreamCallbacks &callbacks) override;
  void stop() override;

private:
  void run(const RawStreamCallbacks &callbacks);

  std::vector<std::string> obs_paths_, nav_paths_;
  double playback_rate_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

} // namespace gnss_comm

#endif // GNSS_COMM_STREAM_RAW_STREAM_HPP_
