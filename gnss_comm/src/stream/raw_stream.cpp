//
// Created by yjh on 2026/11/12.
//

#include "gnss_comm/stream/raw_stream.hpp"
#include "gnss_comm/rinex/rinex_conversion.hpp"
#include "gnss_comm/ros/conversion.hpp"
#include "gnss_comm/utility/coordinate_utility.hpp"
#include "gnss_comm/utility/time_utility.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <glog/logging.h>
#include <map>

namespace gnss_comm {

void RosRawStream::start(const RawStreamCallbacks &callbacks) {
  callbacks_ = callbacks;
  const auto qos = rclcpp::SensorDataQoS().keep_last(1000);
  ephem_sub_ = node_.create_subscription<gnss_msg::msg::GnssEphemMsg>(topics_.ephem, qos, [this](gnss_msg::msg::GnssEphemMsg::ConstSharedPtr msg) { callbacks_.ephem(msg2ephem(msg)); });
  glo_sub_ = node_.create_subscription<gnss_msg::msg::GnssGloEphemMsg>(topics_.glo_ephem, qos,
                                                                       [this](gnss_msg::msg::GnssGloEphemMsg::ConstSharedPtr msg) { callbacks_.ephem(msg2glo_ephem(msg)); });
  iono_sub_ = node_.create_subscription<gnss_msg::msg::StampedFloat64Array>(topics_.iono_params, qos,
                                                                            [this](gnss_msg::msg::StampedFloat64Array::ConstSharedPtr msg) { callbacks_.iono(msg->data); });
  lla_sub_ = node_.create_subscription<sensor_msgs::msg::NavSatFix>(topics_.receiver_lla, qos, [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
    if (msg->status.status >= 0)
      callbacks_.reference_lla(Eigen::Vector3d(msg->latitude, msg->longitude, msg->altitude));
  });
  meas_sub_ = node_.create_subscription<gnss_msg::msg::GnssMeasMsg>(topics_.range_meas, qos, [this](gnss_msg::msg::GnssMeasMsg::ConstSharedPtr msg) {
    const auto meas = msg2meas(msg);
    if (!meas.empty())
      callbacks_.obs(meas);
  });
}

void RinexRawStream::start(const RawStreamCallbacks &callbacks) {
  stop();
  stop_ = false;
  thread_ = std::thread(&RinexRawStream::run, this, callbacks);
}

void RinexRawStream::stop() {
  stop_ = true;
  if (thread_.joinable())
    thread_.join();
}

void RinexRawStream::run(const RawStreamCallbacks &callbacks) {
  // 星历：全部导航文件读入后一次性送出，算法端按观测时刻挑选
  size_t num_ephems = 0;
  for (const auto &path : nav_paths_) {
    std::map<uint32_t, std::vector<EphemBasePtr>> sat2ephem;
    rinex2ephems(path, sat2ephem);
    for (const auto &sat_ephems : sat2ephem)
      for (const auto &ephem : sat_ephems.second) {
        callbacks.ephem(ephem);
        ++num_ephems;
      }
  }
  LOG(INFO) << "Offline rover: " << num_ephems << " ephemerides from " << nav_paths_.size() << " navigation files";
  LOG_IF(ERROR, num_ephems == 0) << "No ephemeris loaded, all epochs will be skipped (navigation files must be RINEX 3.04)";

  // 电离层：取第一个含 GPSA/GPSB 的导航文件（读取失败时参数全为 0）
  for (const auto &path : nav_paths_) {
    std::vector<double> iono_params;
    rinex2iono_params(path, iono_params);
    if (iono_params.size() == 8 && std::any_of(iono_params.begin(), iono_params.end(), [](double x) { return x != 0; })) {
      callbacks.iono(iono_params);
      break;
    }
  }

  std::vector<std::vector<ObsPtr>> epochs;
  for (const auto &path : obs_paths_) {
    const size_t before = epochs.size();
    rinex2obs(path, epochs);
    LOG(INFO) << "Offline rover: " << epochs.size() - before << " epochs from " << path;
  }
  epochs.erase(std::remove_if(epochs.begin(), epochs.end(), [](const std::vector<ObsPtr> &epoch) { return epoch.empty(); }), epochs.end());
  std::stable_sort(epochs.begin(), epochs.end(), [](const std::vector<ObsPtr> &a, const std::vector<ObsPtr> &b) { return time_diff(a.front()->time, b.front()->time) < 0; });

  RinexObsHeader header;
  if (!obs_paths_.empty() && rinex2obs_header(obs_paths_.front(), header) && header.approx_pos.norm() > 6.0e6)
    callbacks.reference_lla(ecef2geo(header.approx_pos));

  for (size_t k = 0; k < epochs.size() && !stop_; ++k) {
    if (playback_rate_ > 0 && k > 0) {
      // 数据中断时最多等 1 s，避免长时间空等
      const double dt = std::min(1.0, time_diff(epochs[k].front()->time, epochs[k - 1].front()->time)) / playback_rate_;
      if (dt > 0)
        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }
    callbacks.obs(epochs[k]);
  }
  if (!stop_ && callbacks.finished)
    callbacks.finished();
}

} // namespace gnss_comm
