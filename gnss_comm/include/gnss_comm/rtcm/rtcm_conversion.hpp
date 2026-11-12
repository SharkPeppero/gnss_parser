//
// Created by yjh on 2026/11/12.
//

#ifndef GNSS_COMM_RTCM_CONVERSION_HPP_
#define GNSS_COMM_RTCM_CONVERSION_HPP_

#include "gnss_comm/types/observation.hpp"
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

// RTCM 3.x 文件到工程内部观测的转换接口，实现位于 src/rtcm/rtcm3.cpp。
namespace gnss_comm {

// RTCM 测站信息：1005/1006 给出天线参考点（ARP）坐标与天线高，1007/1008/1033 给出天线型号。
struct RtcmStation {
  uint32_t station_id{};
  bool has_position = false;
  Eigen::Vector3d arp_ecef{Eigen::Vector3d::Zero()}; // ARP ECEF，m
  double antenna_height_m{};                          // 1006：标石到 ARP 的高度，m；1005 为 0
  std::string antenna_type;                           // 天线描述符，与 ANTEX TYPE 字段格式一致（含天线罩）
};

// 读取 RTCM 3.x 二进制文件中的 MSM4/5/6/7 观测（GPS/GLONASS/Galileo/BDS/QZSS）及测站信息，结果追加至 epochs。
// MSM 只携带周内秒（GLONASS 为日内秒），ref_time 为任一接近数据时段的 GPST，用于确定周数，之后按相邻历元连续推算。
// 每个频段按与 RINEX 相同的优先级只保留一个信号；载波的 LLI 由锁定时间指示器减小或归零判定。
// GLONASS 频道号只能从 MSM5/7 的扩展信息获得，MSM4/6 中未知频道号的 GLONASS 观测被丢弃。
// 未处理传统观测电文（1001~1004、1009~1012）。CRC 校验失败的帧被跳过。返回是否解出至少一个历元。
bool rtcm2obs(const std::string &rtcm_filepath, const gtime_t &ref_time, std::vector<std::vector<ObsPtr>> &epochs, RtcmStation &station);

} // namespace gnss_comm

#endif // GNSS_COMM_RTCM_CONVERSION_HPP_
