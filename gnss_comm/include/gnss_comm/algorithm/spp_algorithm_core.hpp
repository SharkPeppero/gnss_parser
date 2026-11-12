//
// Created by yjh on 2026/11/12.
//

#ifndef GNSS_COMM_ALGORITHM_SPP_ALGORITHM_CORE_HPP_
#define GNSS_COMM_ALGORITHM_SPP_ALGORITHM_CORE_HPP_

#include "gnss_comm/solver/spp.hpp"
#include <array>
#include <map>

namespace gnss_comm {

struct SppResult {
  bool valid = false;
  gtime_t time{};
  Eigen::Vector3d pos_ecef{Eigen::Vector3d::Zero()};
  std::array<double, 4> clock_bias_m{}; // GPS/GLO/GAL/BDS 接收机钟差，m；QZSS 共用 GPS，未参与的星座为 NaN
  std::map<uint32_t, int> sys_counts;   // 星座 → 参与解算的卫星数
};

// 单历元伪距单点定位；输入为 AlgorithmManager 筛选后、与星历逐项对应的观测。
class SPPAlgorithmCore {
public:
  explicit SPPAlgorithmCore(const SppSolverPipelineOptions &options) : solver_(options) {}

  SppResult solve(const std::vector<ObsPtr> &obs, const std::vector<EphemBasePtr> &ephems, const std::vector<double> &iono_params) const;

private:
  SppSolverPipeline solver_;
};

} // namespace gnss_comm

#endif // GNSS_COMM_ALGORITHM_SPP_ALGORITHM_CORE_HPP_
