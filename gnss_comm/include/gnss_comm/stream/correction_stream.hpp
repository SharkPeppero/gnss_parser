//
// Created by yjh on 2026/11/12.
//

#ifndef GNSS_COMM_STREAM_CORRECTION_STREAM_HPP_
#define GNSS_COMM_STREAM_CORRECTION_STREAM_HPP_

#include "gnss_comm/types/observation.hpp"
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

namespace gnss_comm {

// 基站测站信息，RINEX 与 RTCM 统一表示为“标石 + 天线偏移”。
struct BaseStationInfo {
  std::string name;
  std::string antenna_type;                                   // 与 ANTEX TYPE 字段格式一致（型号 + 天线罩）
  Eigen::Vector3d marker_ecef{Eigen::Vector3d::Zero()};       // 标石 ECEF，m
  Eigen::Vector3d antenna_delta_hen{Eigen::Vector3d::Zero()}; // 天线参考点（ARP）相对标石的 H/E/N 偏移，m
};

// 离线差分数据：一次性加载基站观测文件（RINEX 3.xx 或 RTCM 3.x MSM），按流动站时刻检索最近的基站历元。
// format 为 "rinex"、"rtcm3" 或 "auto"（文件名含 .rtcm 或以 .rtc 结尾按 RTCM，否则按 RINEX）。
// marker_override 为 3 维时覆盖文件中的标石坐标（RINEX 头为米级近似值，应换成 SINEX 等精确坐标）。
class CorrectionStream {
public:
  CorrectionStream(const std::vector<std::string> &paths, const std::string &format, const std::vector<double> &marker_override);

  // 加载全部文件并按时间排序。RTCM 只含周内秒，ref_time 取任一流动站观测时刻以确定周数；RINEX 忽略。
  bool load(const gtime_t &ref_time);
  bool loaded() const { return loaded_; }

  const BaseStationInfo &station() const { return station_; }
  size_t num_epochs() const { return epochs_.size(); }

  // 时间最近的基站历元（可能晚于流动站），龄期限制由 RTK 解算检查；未加载时返回空指针。
  const std::vector<ObsPtr> *nearest_epoch(const gtime_t &time) const;

private:
  bool is_rtcm(const std::string &path) const;

  std::vector<std::string> paths_;
  std::string format_;
  std::vector<double> marker_override_;
  bool loaded_ = false;
  BaseStationInfo station_;
  std::vector<std::vector<ObsPtr>> epochs_;
};

} // namespace gnss_comm

#endif // GNSS_COMM_STREAM_CORRECTION_STREAM_HPP_
