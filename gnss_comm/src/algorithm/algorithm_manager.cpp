//
// Created by yjh on 2026/11/12.
//

#include "gnss_comm/algorithm/algorithm_manager.hpp"
#include "gnss_comm/gnss_utility.hpp"
#include <algorithm>
#include <cmath>
#include <glog/logging.h>
#include <set>
#include <stdexcept>

namespace gnss_comm {

AlgorithmManager::AlgorithmManager(const AlgorithmManagerOptions &options, CorrectionStream *correction) : options_(options), correction_(correction) {
  if (options_.enable_spp)
    spp_ = std::make_unique<SPPAlgorithmCore>(options_.spp);
  if (options_.enable_rtk) {
    if (!correction_)
      throw std::invalid_argument("RTK enabled but no correction stream");
    rtk_ = std::make_unique<RTKAlgorithmCore>(options_.rtk, options_.antex_path);
  }
}

void AlgorithmManager::add_ephemeris(const EphemBasePtr &ephem) {
  if (!ephem)
    return;
  auto &history = ephemerides_[ephem->sat];
  // 同一期号、同一 toe 的重复广播覆盖即可；保留多个版本供观测时刻匹配。
  for (auto &entry : history) {
    if (entry->iode == ephem->iode && time_diff(entry->toe, ephem->toe) == 0) {
      entry = ephem;
      return;
    }
  }
  history.push_back(ephem);
  if (options_.max_ephem_history > 0 && history.size() > options_.max_ephem_history)
    history.pop_front();
}

void AlgorithmManager::set_iono(const std::vector<double> &iono_params) {
  if (iono_params.size() != 8 || !std::all_of(iono_params.begin(), iono_params.end(), [](double x) { return std::isfinite(x); }))
    return;
  iono_params_ = iono_params;
  has_iono_ = true;
}

EphemBasePtr AlgorithmManager::select_ephemeris(const ObsPtr &obs) const {
  const auto it = ephemerides_.find(obs->sat);
  if (it == ephemerides_.end())
    return nullptr;
  double best_age = satsys(obs->sat, nullptr) == SYS_GLO ? GLO_EPH_VALID_SECONDS : EPH_VALID_SECONDS;
  EphemBasePtr best;
  for (const auto &entry : it->second) {
    const double age = std::abs(time_diff(obs->time, entry->toe));
    if (age <= best_age) {
      best_age = age;
      best = entry;
    }
  }
  // 最近版本不健康时直接剔除，不回退到更旧的健康版本。
  return best && best->health == 0 ? best : nullptr;
}

EpochResult AlgorithmManager::process(const std::vector<ObsPtr> &meas) {
  EpochResult result;
  if (!has_iono_) {
    result.skip_reason = "waiting for ionosphere parameters";
    return result;
  }

  // 观测筛选（SPP / RTK 共用）：L1 类伪距有效、同一历元、星历健康、URA 权重有效
  std::set<uint32_t> satellites;
  for (const auto &entry : meas) {
    const uint32_t sys = satsys(entry->sat, nullptr);
    if (sys != SYS_GPS && sys != SYS_GLO && sys != SYS_GAL && sys != SYS_BDS && sys != SYS_QZS)
      continue;
    int index = -1;
    if (L1_freq(entry, &index) <= 0 || index < 0)
      continue;
    const auto i = static_cast<size_t>(index);
    if (i >= entry->psr.size() || i >= entry->psr_std.size() || i >= entry->status.size() || !(entry->status[i] & 1) || !std::isfinite(entry->psr[i]) || entry->psr[i] <= 0 ||
        !std::isfinite(entry->psr_std[i]) || entry->psr_std[i] < 0 || std::abs(time_diff(entry->time, meas.front()->time)) > 0.001)
      continue;
    const auto ephem = select_ephemeris(entry);
    if (!ephem)
      continue;
    // 沿用现有 solver 的 URA 权重表达式，拒绝会产生非正/无穷权重的输入。
    const double denominator = sys == SYS_GLO ? 4.0 : ephem->ura - (sys == SYS_GAL ? 2.0 : 1.0);
    if (!std::isfinite(denominator) || denominator <= 0 || !satellites.insert(entry->sat).second)
      continue;
    result.obs.push_back(entry);
    result.ephems.push_back(ephem);
  }
  if (result.obs.size() < 4) {
    result.skip_reason = "insufficient usable observations / ephemerides (" + std::to_string(result.obs.size()) + ")";
    return result;
  }

  if (spp_)
    result.spp = spp_->solve(result.obs, result.ephems, iono_params_);

  if (rtk_ && !rtk_->initialized() && !correction_failed_) {
    if (correction_->load(result.obs.front()->time))
      rtk_->initialize(correction_->station());
    else
      correction_failed_ = true; // 只尝试一次，避免每个历元重复读文件
  }
  if (rtk_ && rtk_->initialized() && result.obs.size() >= 5) {
    result.rtk = rtk_->solve(result.obs, result.ephems, *correction_->nearest_epoch(result.obs.front()->time), iono_params_);
    result.has_rtk = true;
  }
  return result;
}

} // namespace gnss_comm
