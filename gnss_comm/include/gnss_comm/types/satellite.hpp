#ifndef GNSS_COMM_TYPES_SATELLITE_HPP_
#define GNSS_COMM_TYPES_SATELLITE_HPP_

#include <cstdint>
#include <memory>
#include <set>
#include <map>
#include <eigen3/Eigen/Dense>
#include "gnss_comm/types/time.hpp"

// 卫星辅助信息及指定历元的卫星状态。
// 星历保存轨道参数；SatState 保存由轨道参数计算出的具体时刻状态。
namespace gnss_comm {

// 单颗卫星在解算中的状态，以及参与解算的信号频率集合。
struct BestSat {
  gtime_t time{};
  uint32_t sat{};
  uint8_t status{};
  std::set<double> freq_used; /* frequency used in solution */
};
typedef std::shared_ptr<BestSat> BestSatPtr;

// 单颗卫星的健康状态、观测方向和信号信息。
struct SvInfo {
  gtime_t time{};
  uint32_t sat{};
  int32_t freqo{}; /* GLONASS frequency number (-7~+6) */
  bool health{};
  double elev_degree{};
  double az_degree{};
  uint32_t sig_mask{};
};
typedef std::shared_ptr<SvInfo> SvInfoPtr;

// 同一历元的多星位置与钟差，映射键为工程内部卫星编号。
struct EphemerideEpoch {
  gtime_t time{};
  std::map<uint32_t, Eigen::Vector3d> sat2pos;
  std::map<uint32_t, double> sat2clk;
};
typedef std::shared_ptr<EphemerideEpoch> EphemerideEpochPtr;

// 单颗卫星在信号发送时刻的运动状态、钟差及群延迟。
struct SatState {
  uint32_t sat_id{};
  gtime_t ttx{};
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
  double satellite_clock_bias_s{}; // 卫星钟差，s
  double satellite_clock_drift_sps{}; // 卫星钟漂，s/s
  double tgd{};
};
typedef std::shared_ptr<SatState> SatStatePtr;

} // namespace gnss_comm

#endif // GNSS_COMM_TYPES_SATELLITE_HPP_
