//
// Created by yjh on 2026/11/12.
//

#ifndef GNSS_COMM_ALGORITHM_ALGORITHM_MANAGER_HPP_
#define GNSS_COMM_ALGORITHM_ALGORITHM_MANAGER_HPP_

#include "gnss_comm/algorithm/rtk_algorithm_core.hpp"
#include "gnss_comm/algorithm/spp_algorithm_core.hpp"
#include "gnss_comm/stream/correction_stream.hpp"
#include <deque>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

namespace gnss_comm {

struct AlgorithmManagerOptions {
  bool enable_spp = true;
  bool enable_rtk = false;
  SppSolverPipelineOptions spp;
  RtkSolverPipelineOptions rtk;
  std::string antex_path;        // 基站天线 PCO，为空不改正
  size_t max_ephem_history = 16; // 每颗卫星保留的星历版本数；0 不限（离线一次性载入整天星历时使用）

  // 参数加载后打印；未启用的算法不打印其参数
  friend std::ostream &operator<<(std::ostream &os, const AlgorithmManagerOptions &o) {
    const std::ios_base::fmtflags flags = os.flags();
    os << std::boolalpha << "AlgorithmManagerOptions:\n"
       << " enable_spp: " << o.enable_spp << "\n"
       << " enable_rtk: " << o.enable_rtk << "\n"
       << " antex_path: \"" << o.antex_path << "\"\n"
       << " max_ephem_history: " << o.max_ephem_history << "\n";
    if (o.enable_spp)
      os << " spp:\n" << o.spp;
    if (o.enable_rtk)
      os << " rtk:\n" << o.rtk;
    os.flags(flags);
    return os;
  }
};

struct EpochResult {
  std::string skip_reason;          // 非空表示本历元没有进入任何解算
  std::vector<ObsPtr> obs;          // 筛选后参与解算的观测
  std::vector<EphemBasePtr> ephems; // 与 obs 逐项对应的星历
  SppResult spp;                    // enable_spp 时有效与否见 spp.valid
  bool has_rtk = false;
  RtkSolution rtk;
};

// 算法调度：缓存星历与电离层参数，筛选观测，再分发给 SPP / RTK 两个核心。
// RTK 首个历元时以该历元时间加载差分数据（RTCM 需要它确定周数）并初始化基站。
// 非线程安全，所有接口须在同一线程调用。
class AlgorithmManager {
public:
  AlgorithmManager(const AlgorithmManagerOptions &options, CorrectionStream *correction);

  void add_ephemeris(const EphemBasePtr &ephem);

  void set_iono(const std::vector<double> &iono_params);

  EpochResult process(const std::vector<ObsPtr> &meas);

private:
  /// 寻找当前观测是有最佳星历
  EphemBasePtr select_ephemeris(const ObsPtr &obs) const;

  AlgorithmManagerOptions options_;
  CorrectionStream *correction_;
  std::unique_ptr<SPPAlgorithmCore> spp_;
  std::unique_ptr<RTKAlgorithmCore> rtk_;
  bool correction_failed_ = false;
  std::map<uint32_t, std::deque<EphemBasePtr>> ephemerides_;
  std::vector<double> iono_params_;
  bool has_iono_ = false;
};

} // namespace gnss_comm

#endif // GNSS_COMM_ALGORITHM_ALGORITHM_MANAGER_HPP_
