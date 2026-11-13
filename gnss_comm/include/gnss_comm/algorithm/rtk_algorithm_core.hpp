//
// Created by yjh on 2026/11/12.
//

#ifndef GNSS_COMM_ALGORITHM_RTK_ALGORITHM_CORE_HPP_
#define GNSS_COMM_ALGORITHM_RTK_ALGORITHM_CORE_HPP_

#include "gnss_comm/solver/rtk.hpp"
#include "gnss_comm/stream/correction_stream.hpp"
#include <memory>
#include <string>

namespace gnss_comm {

// 单基站 RTK：由基站测站信息算出天线参考点（ARP）与 ANTEX 相位中心偏移，再逐历元调用 RtkSolverPipeline。
// 基站信息要等差分数据加载后才有，因此解算器在 initialize 中创建。
class RTKAlgorithmCore {
public:
  RTKAlgorithmCore(const RtkSolverPipelineOptions &options, //
                   const std::string &antex_path)
      : options_(options), antex_path_(antex_path) {}

  bool initialized() const { return solver_ != nullptr; }

  // 标石 + ANTENNA: DELTA H/E/N → ARP；antex_path 非空时读取基站天线各频段 PCO。
  void initialize(const BaseStationInfo &station);

  RtkSolution solve(const std::vector<ObsPtr> &rover_obs,          //
                    const std::vector<EphemBasePtr> &rover_ephems, //
                    const std::vector<ObsPtr> &base_obs,           //
                    const std::vector<double> &iono_params) {
    return solver_->solve(rover_obs, rover_ephems, base_obs, iono_params);
  }

private:
  RtkSolverPipelineOptions options_;
  std::string antex_path_;
  std::unique_ptr<RtkSolverPipeline> solver_;
};

} // namespace gnss_comm

#endif // GNSS_COMM_ALGORITHM_RTK_ALGORITHM_CORE_HPP_
