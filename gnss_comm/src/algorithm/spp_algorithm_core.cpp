//
// Created by yjh on 2026/11/12.
//

#include "gnss_comm/algorithm/spp_algorithm_core.hpp"
#include "gnss_comm/gnss_utility.hpp"
#include <limits>
#include <set>

namespace gnss_comm {

SppResult SPPAlgorithmCore::solve(const std::vector<ObsPtr> &obs, const std::vector<EphemBasePtr> &ephems, const std::vector<double> &iono_params) const {
  SppResult result;
  std::set<int> clock_indices;
  for (const auto &entry : obs) {
    const uint32_t sys = satsys(entry->sat, nullptr);
    clock_indices.insert(sys2idx.at(sys));
    ++result.sys_counts[sys];
  }
  // 未知数为 3 维位置 + 每个星座一个钟差
  if (obs.empty() || obs.size() < 3 + clock_indices.size())
    return result;

  const auto state = solver_.psr_pos(obs, ephems, iono_params);
  if (!state.allFinite() || state.head<3>().norm() < 1.0)
    return result;
  result.valid = true;
  result.time = obs.front()->time;
  result.pos_ecef = state.head<3>();
  result.clock_bias_m.fill(std::numeric_limits<double>::quiet_NaN());
  for (const auto index : clock_indices)
    result.clock_bias_m[index] = state[3 + index];
  return result;
}

} // namespace gnss_comm
