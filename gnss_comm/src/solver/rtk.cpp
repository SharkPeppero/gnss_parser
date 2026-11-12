#include "gnss_comm/solver/rtk.hpp"
#include "gnss_comm/gnss_utility.hpp"
#include "gnss_comm/solver/lambda.hpp"
#include <algorithm>
#include <cmath>
#include <glog/logging.h>
#include <map>

namespace gnss_comm {

namespace {

// 频段编号：0 为 L1/G1/E1/B1I，1 为 L2/G2/E5b/B2I，其它信号返回 -1。
int band_index(const uint32_t sys, const double freq) {
  const auto near = [freq](double f) { return std::abs(freq - f) < 1e3; };
  if (sys == SYS_GPS || sys == SYS_QZS)
    return near(FREQ1) ? 0 : near(FREQ2) ? 1 : -1;
  if (sys == SYS_GLO)
    return std::abs(freq - FREQ1_GLO) < 8 * DFRQ1_GLO ? 0 : std::abs(freq - FREQ2_GLO) < 8 * DFRQ2_GLO ? 1 : -1;
  if (sys == SYS_GAL)
    return near(FREQ1) ? 0 : near(FREQ7) ? 1 : -1;
  if (sys == SYS_BDS)
    return near(FREQ1_BDS) ? 0 : near(FREQ2_BDS) ? 1 : -1;
  return -1;
}

// 观测中属于指定频段、且各字段长度足够的信号下标，没有返回 -1。
int signal_index(const ObsPtr &obs, const uint32_t sys, const int band) {
  for (size_t i = 0; i < obs->freqs.size(); ++i) {
    if (band_index(sys, obs->freqs[i]) != band)
      continue;
    if (i < obs->psr.size() && i < obs->cp.size() && i < obs->lli.size() && i < obs->status.size())
      return static_cast<int>(i);
  }
  return -1;
}

// 几何距离加 Sagnac 改正，m。
double geo_range(const Eigen::Vector3d &sat_pos, const Eigen::Vector3d &rcv_pos) {
  return (sat_pos - rcv_pos).norm() + EARTH_OMG_GPS * (sat_pos.x() * rcv_pos.y() - sat_pos.y() * rcv_pos.x()) / LIGHT_SPEED;
}

// 模糊度所属的双差分组（星座, 频段），QZSS 并入 GPS。
std::pair<uint32_t, int> amb_group(const std::pair<uint32_t, int> &key) {
  const uint32_t sys = satsys(key.first, NULL);
  return {sys == SYS_QZS ? SYS_GPS : sys, key.second};
}

} // namespace

namespace {

// 无几何组合 GF = λ1·φ1 − λ2·φ2（m），两个频段载波都有效时返回 true。
bool gf_combination(const ObsPtr &obs, double &gf_m) {
  const uint32_t sys = satsys(obs->sat, NULL);
  const int i0 = signal_index(obs, sys, 0), i1 = signal_index(obs, sys, 1);
  if (i0 < 0 || i1 < 0 || obs->cp[i0] == 0 || obs->cp[i1] == 0)
    return false;
  if ((obs->status[i0] & 0x06) != 0x06 || (obs->status[i1] & 0x06) != 0x06)
    return false;
  gf_m = LIGHT_SPEED / obs->freqs[i0] * obs->cp[i0] - LIGHT_SPEED / obs->freqs[i1] * obs->cp[i1];
  return true;
}

} // namespace

RtkSolverPipeline::RtkSolverPipeline(const Eigen::Vector3d &base_arp_ecef, const RtkSolverPipelineOptions &options, const BasePcoMap &base_pco)
    : base_ecef_(base_arp_ecef), base_lla_(ecef2geo(base_arp_ecef)), base_pco_(base_pco), options_(options), spp_(options.spp) {}

Eigen::Vector3d RtkSolverPipeline::base_phase_center(const uint32_t sys, const int band) const {
  const auto it = base_pco_.find({sys == SYS_QZS ? SYS_GPS : sys, band});
  return it == base_pco_.end() ? base_ecef_ : Eigen::Vector3d(base_ecef_ + it->second);
}

RtkSolution RtkSolverPipeline::solve(const std::vector<ObsPtr> &rover_obs,          //
                                     const std::vector<EphemBasePtr> &rover_ephems, //
                                     const std::vector<ObsPtr> &base_obs,           //
                                     const std::vector<double> &iono_params) {
  RtkSolution sol;
  if (rover_obs.empty())
    return sol;
  sol.time = rover_obs.front()->time;

  // 1. 流动站 SPP 作为初值，也是双差不可用时的退化结果。
  const Eigen::Matrix<double, 7, 1> spp_state = spp_.psr_pos(rover_obs, rover_ephems, iono_params);
  if (spp_state.head<3>().norm() < 1.0)
    return sol;
  sol.pos_ecef = spp_state.head<3>();
  sol.pos_type = PV_SINGLE;
  if (base_obs.empty())
    return sol;

  sol.base_age_s = time_diff(sol.time, base_obs.front()->time);
  if (std::abs(sol.base_age_s) > options_.max_base_age_s)
    return sol;

  // 2. 双差构建；滤波器已运行时用其位置计算高度角，比 SPP 更准。
  const Eigen::Vector3d approx_pos = initialized_ ? Eigen::Vector3d(x_.head<3>()) : sol.pos_ecef;
  std::vector<DdMeasurement> dds = build_dd(rover_obs, rover_ephems, base_obs, approx_pos);

  // 3. 双差伪距定位：滤波器初值，也是载波不可用时的结果。
  Eigen::Vector3d dgnss_pos = sol.pos_ecef;
  Eigen::Matrix3d dgnss_cov;
  const bool has_dgnss = dd_psr_pos(dds, dgnss_pos, dgnss_cov);
  if (has_dgnss) {
    sol.pos_ecef = dgnss_pos;
    sol.pos_cov_ecef = dgnss_cov;
    sol.pos_type = PV_PSR_DIFF;
    sol.num_dd_psr = dds.size();
  }

  // 4. EKF 预测；首历元、时间倒退或中断过长时用伪距差分结果重置。
  const double dt = initialized_ ? time_diff(sol.time, last_time_) : 0.0;
  if (!initialized_ || dt < 0 || dt > options_.max_gap_s) {
    if (!has_dgnss) {
      initialized_ = false;
      return sol;
    }
    reset_filter(dgnss_pos, dgnss_cov);
  } else {
    predict(dt);
  }
  last_time_ = sol.time;
  const gtime_t base_time = base_obs.front()->time;
  const bool base_changed = time_diff(base_time, last_base_time_) != 0;
  last_base_time_ = base_time;

  // 5. GF 周跳并入双差的周跳位，与 LLI 一起交给模糊度管理。
  const std::map<uint32_t, uint8_t> gf_slips = detect_slips_gf(rover_obs, base_obs, base_changed);
  for (auto &dd : dds) {
    const auto sat_it = gf_slips.find(dd.sat), ref_it = gf_slips.find(dd.ref_sat);
    if (sat_it != gf_slips.end())
      dd.sat_slip |= sat_it->second;
    if (ref_it != gf_slips.end())
      dd.ref_slip |= ref_it->second;
  }

  // 6. 模糊度管理 + 量测更新。
  update_ambiguities(dds, sol.time, base_changed);
  std::vector<bool> phase_used;
  sol.num_dd_cp = ekf_update(dds, phase_used);
  if (sol.num_dd_cp < 4)
    return sol;
  sol.pos_ecef = x_.head<3>();
  sol.pos_cov_ecef = P_.topLeftCorner<3, 3>();
  sol.pos_type = PV_NARROW_FLOAT;

  // 7. LAMBDA 固定（含部分固定）；成功且开启 fix-and-hold 时把整数解约束回滤波器。
  Eigen::Vector3d fix_pos;
  Eigen::Matrix3d fix_cov;
  std::vector<std::pair<int, int>> fixed_pairs;
  Eigen::VectorXd fixed_amb;
  if (fix_ambiguity(dds, phase_used, fix_pos, fix_cov, sol.ratio, fixed_pairs, fixed_amb)) {
    sol.pos_ecef = fix_pos;
    sol.pos_cov_ecef = fix_cov;
    sol.pos_type = PV_NARROW_INT;
    sol.num_fix_dd = fixed_pairs.size();
    if (options_.fix_and_hold)
      hold_ambiguity(fixed_pairs, fixed_amb);
  }
  return sol;
}

std::vector<DdMeasurement> RtkSolverPipeline::build_dd(const std::vector<ObsPtr> &rover_obs,          //
                                                       const std::vector<EphemBasePtr> &rover_ephems, //
                                                       const std::vector<ObsPtr> &base_obs,           //
                                                       const Eigen::Vector3d &rover_ecef) const {
  std::vector<DdMeasurement> dds;

  // 1. 两站共视：都有 L1 类伪距（sat_states 用它估计发射时刻），星历取自流动站。
  std::map<uint32_t, ObsPtr> sat2base;
  for (const auto &obs : base_obs) {
    int l1_idx = -1;
    if (L1_freq(obs, &l1_idx) > 0 && l1_idx >= 0 && obs->psr[l1_idx] > 0)
      sat2base[obs->sat] = obs;
  }
  std::vector<ObsPtr> rov_common, base_common;
  std::vector<EphemBasePtr> ephem_common;
  for (size_t i = 0; i < rover_obs.size(); ++i) {
    const uint32_t sys = satsys(rover_obs[i]->sat, NULL);
    if (sys != SYS_GPS && sys != SYS_GLO && sys != SYS_GAL && sys != SYS_BDS && sys != SYS_QZS)
      continue;
    if (sys == SYS_GLO && !options_.use_glonass)
      continue;
    int l1_idx = -1;
    if (L1_freq(rover_obs[i], &l1_idx) <= 0 || l1_idx < 0 || rover_obs[i]->psr[l1_idx] <= 0)
      continue;
    const auto it = sat2base.find(rover_obs[i]->sat);
    if (it == sat2base.end())
      continue;
    rov_common.push_back(rover_obs[i]);
    base_common.push_back(it->second);
    ephem_common.push_back(rover_ephems[i]);
  }
  if (rov_common.empty())
    return dds;

  // 两站各自信号发射时刻的卫星状态；时间差（龄期）期间的卫星运动和钟差变化由此吸收。
  const std::vector<SatStatePtr> rov_states = spp_.sat_states(rov_common, ephem_common);
  const std::vector<SatStatePtr> base_states = spp_.sat_states(base_common, ephem_common);

  // 2. 按星座/频段分组的单差观测。
  const Eigen::Vector3d rover_lla = ecef2geo(rover_ecef);
  struct SdLeg {
    size_t k;        // 共视卫星下标
    double el;       // 流动站处高度角，rad
    double sd_psr_m; // 站间单差伪距（已改正卫星钟差与对流层）
    double sd_cp_m;  // 站间单差载波（已改正卫星钟差与对流层）
    bool cp_valid;
    uint8_t slip;
    double wavelength_m;
  };
  std::map<std::pair<uint32_t, int>, std::vector<SdLeg>> groups;
  for (size_t k = 0; k < rov_common.size(); ++k) {
    const ObsPtr &rov = rov_common[k], &base = base_common[k];
    const uint32_t sys = satsys(rov->sat, NULL);
    double azel[2] = {0, 0};
    sat_azel(rover_ecef, rov_states[k]->pos, azel);
    if (azel[1] < options_.elevation_cutoff_degree * D2R)
      continue;
    // 两站各自的“卫星钟差 − 对流层延迟”改正：卫星钟差在龄期内会变化，对流层随两站高差与高度角不同，双差中都不能完全抵消。
    double base_azel[2] = {0, 0};
    sat_azel(base_ecef_, base_states[k]->pos, base_azel);
    const double rov_corr_m = rov_states[k]->satellite_clock_bias_s * LIGHT_SPEED - calculate_trop_delay(rov_states[k]->ttx, rover_lla, azel);
    const double base_corr_m = base_states[k]->satellite_clock_bias_s * LIGHT_SPEED - calculate_trop_delay(base_states[k]->ttx, base_lla_, base_azel);

    for (int band = 0; band < static_cast<int>(options_.num_bands); ++band) {
      const int ri = signal_index(rov, sys, band), bi = signal_index(base, sys, band);
      if (ri < 0 || bi < 0)
        continue;
      if (!(rov->status[ri] & 0x01) || !(base->status[bi] & 0x01) || rov->psr[ri] <= 0 || base->psr[bi] <= 0)
        continue;
      SdLeg leg;
      leg.k = k;
      leg.el = azel[1];
      leg.wavelength_m = LIGHT_SPEED / rov->freqs[ri];
      leg.sd_psr_m = (rov->psr[ri] + rov_corr_m) - (base->psr[bi] + base_corr_m);
      // 载波需半周已解决（status bit1/bit2）；GLONASS 各星波长不同，暂不构建载波双差。
      // TODO: 两站信号码不一致（如 L2C 与 L2W）时需改正 1/4 周相位偏差，可比对 code 字段。
      leg.cp_valid = sys != SYS_GLO && (rov->status[ri] & 0x06) == 0x06 && (base->status[bi] & 0x06) == 0x06 && rov->cp[ri] != 0 && base->cp[bi] != 0;
      leg.slip = (rov->lli[ri] & 0x01 ? 0x01 : 0x00) | (base->lli[bi] & 0x01 ? 0x02 : 0x00);
      leg.sd_cp_m = leg.cp_valid ? (rov->cp[ri] * leg.wavelength_m + rov_corr_m) - (base->cp[bi] * leg.wavelength_m + base_corr_m) : 0;
      groups[{sys == SYS_QZS ? SYS_GPS : sys, band}].push_back(leg);
    }
  }

  // 3. 每组取最高高度角卫星作参考星，其余卫星与之作差。
  for (const auto &group : groups) {
    const std::vector<SdLeg> &legs = group.second;
    if (legs.size() < 2)
      continue;
    const SdLeg &ref = *std::max_element(legs.begin(), legs.end(), [](const SdLeg &a, const SdLeg &b) { return a.el < b.el; });
    // 基站几何距离取到该频段的相位中心（ARP + PCO）。
    const Eigen::Vector3d base_pc = base_phase_center(group.first.first, group.first.second);
    const double ref_base_range = geo_range(base_states[ref.k]->pos, base_pc);
    for (const auto &leg : legs) {
      if (leg.k == ref.k)
        continue;
      DdMeasurement dd;
      dd.sat = rov_common[leg.k]->sat;
      dd.ref_sat = rov_common[ref.k]->sat;
      dd.sys = group.first.first;
      dd.band = group.first.second;
      dd.wavelength_m = leg.wavelength_m;
      dd.sat_pos = rov_states[leg.k]->pos;
      dd.ref_sat_pos = rov_states[ref.k]->pos;
      dd.base_sd_range_m = geo_range(base_states[leg.k]->pos, base_pc) - ref_base_range;
      dd.dd_psr_m = leg.sd_psr_m - ref.sd_psr_m;
      dd.cp_valid = leg.cp_valid && ref.cp_valid;
      dd.dd_cp_m = dd.cp_valid ? leg.sd_cp_m - ref.sd_cp_m : 0;
      dd.var_factor_sat = 1.0 / (std::sin(leg.el) * std::sin(leg.el));
      dd.var_factor_ref = 1.0 / (std::sin(ref.el) * std::sin(ref.el));
      dd.sat_sd_psr_m = leg.sd_psr_m;
      dd.ref_sd_psr_m = ref.sd_psr_m;
      dd.sat_sd_cp_m = leg.sd_cp_m;
      dd.ref_sd_cp_m = ref.sd_cp_m;
      dd.sat_slip = leg.slip;
      dd.ref_slip = ref.slip;
      dds.push_back(dd);
    }
  }
  return dds;
}

bool RtkSolverPipeline::dd_psr_pos(const std::vector<DdMeasurement> &dds, Eigen::Vector3d &pos, Eigen::Matrix3d &cov) const {
  const size_t n = dds.size();
  if (n < 4)
    return false;

  // 双差协方差：对角 σ_j² + σ_r²，同组观测共享参考星项 σ_r²；系数 2 对应流动站与基站各一份非差误差。
  const double var0 = 2.0 * options_.code_std_m * options_.code_std_m;
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(n, n);
  for (size_t i = 0; i < n; ++i) {
    R(i, i) = var0 * (dds[i].var_factor_sat + dds[i].var_factor_ref);
    for (size_t k = i + 1; k < n; ++k)
      if (dds[k].sys == dds[i].sys && dds[k].band == dds[i].band)
        R(i, k) = R(k, i) = var0 * dds[i].var_factor_ref;
  }
  const Eigen::MatrixXd W = R.inverse();

  Eigen::Matrix3d N = Eigen::Matrix3d::Identity();
  double dx_norm = 1.0;
  uint32_t num_iter = 0;
  while (num_iter < options_.max_iterations && dx_norm > options_.convergence_threshold) {
    // “预测值−观测值”残差，与 SPP 相同的符号约定。
    Eigen::VectorXd res(n);
    Eigen::MatrixXd J(n, 3);
    for (size_t i = 0; i < n; ++i) {
      const DdMeasurement &dd = dds[i];
      res(i) = geo_range(dd.sat_pos, pos) - geo_range(dd.ref_sat_pos, pos) - dd.base_sd_range_m - dd.dd_psr_m;
      J.row(i) = -((dd.sat_pos - pos).normalized() - (dd.ref_sat_pos - pos).normalized()).transpose();
    }
    N = J.transpose() * W * J;
    const Eigen::Vector3d dx = -N.ldlt().solve(J.transpose() * W * res);
    dx_norm = dx.norm();
    pos += dx;
    ++num_iter;
  }
  if (dx_norm > options_.convergence_threshold) {
    LOG(WARNING) << "[gnss_comm::RtkSolverPipeline::dd_psr_pos] DD pseudorange solver reached maximum iterations.\n";
    return false;
  }
  cov = N.inverse();
  return pos.allFinite();
}

void RtkSolverPipeline::reset_filter(const Eigen::Vector3d &pos, const Eigen::Matrix3d &cov) {
  x_ = Eigen::VectorXd::Zero(6);
  x_.head<3>() = pos;
  P_ = Eigen::MatrixXd::Zero(6, 6);
  P_.topLeftCorner<3, 3>() = cov;
  P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 100.0; // 速度未知，10 m/s
  ambs_.clear();
  initialized_ = true;
}

void RtkSolverPipeline::predict(const double dt) {
  const int n = static_cast<int>(x_.size());
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(n, n);
  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  x_.head<3>() += dt * x_.segment<3>(3);
  P_ = F * P_ * F.transpose();

  // 白噪声加速度的离散化过程噪声。
  const double qa = options_.accel_std_mps2 * options_.accel_std_mps2;
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  P_.block<3, 3>(0, 0) += I * qa * dt * dt * dt / 3.0;
  P_.block<3, 3>(0, 3) += I * qa * dt * dt / 2.0;
  P_.block<3, 3>(3, 0) += I * qa * dt * dt / 2.0;
  P_.block<3, 3>(3, 3) += I * qa * dt;
  const double qn = options_.amb_process_std_cycles * options_.amb_process_std_cycles * dt;
  for (int i = 6; i < n; ++i)
    P_(i, i) += qn;
}

void RtkSolverPipeline::update_ambiguities(const std::vector<DdMeasurement> &dds, const gtime_t &time, const bool base_changed) {
  // 本历元每颗卫星/频段的单差载波与伪距（参考星会在多个双差里重复出现，值相同）。
  struct SdInfo {
    double sd_cp_m, sd_psr_m, wavelength_m;
    uint8_t slip;
  };
  std::map<AmbKey, SdInfo> sds;
  for (const auto &dd : dds) {
    if (!dd.cp_valid)
      continue;
    sds[{dd.sat, dd.band}] = {dd.sat_sd_cp_m, dd.sat_sd_psr_m, dd.wavelength_m, dd.sat_slip};
    sds[{dd.ref_sat, dd.band}] = {dd.ref_sd_cp_m, dd.ref_sd_psr_m, dd.wavelength_m, dd.ref_slip};
  }

  // 1. 删除长时间未观测或发生周跳的模糊度。
  std::vector<AmbKey> to_remove;
  for (const auto &amb : ambs_) {
    const auto it = sds.find(amb.first);
    if (it == sds.end()) {
      if (time_diff(time, amb.second.last_seen) > options_.max_amb_outage_s)
        to_remove.push_back(amb.first);
      continue;
    }
    if ((it->second.slip & 0x01) || (base_changed && (it->second.slip & 0x02)))
      to_remove.push_back(amb.first);
  }
  for (const auto &key : to_remove)
    remove_ambiguity(key);

  // 2. 相位-伪距一致性：接收机钟差变化会使所有单差模糊度整体漂移（双差中抵消），
  //    按组把已有状态平移到当前 (载波−伪距) 的水平，保证新旧模糊度的公共部分一致。
  std::map<std::pair<uint32_t, int>, std::pair<double, int>> offsets;
  for (const auto &sd : sds) {
    const auto it = ambs_.find(sd.first);
    if (it == ambs_.end())
      continue;
    auto &offset = offsets[amb_group(sd.first)];
    offset.first += (sd.second.sd_cp_m - sd.second.sd_psr_m) / sd.second.wavelength_m - x_(it->second.idx);
    ++offset.second;
  }
  for (const auto &amb : ambs_) {
    const auto it = offsets.find(amb_group(amb.first));
    if (it != offsets.end())
      x_(amb.second.idx) += it->second.first / it->second.second;
  }

  // 3. 初始化新模糊度，刷新观测时间。
  const double var = options_.amb_init_std_cycles * options_.amb_init_std_cycles;
  for (const auto &sd : sds) {
    const auto it = ambs_.find(sd.first);
    if (it == ambs_.end())
      add_ambiguity(sd.first, (sd.second.sd_cp_m - sd.second.sd_psr_m) / sd.second.wavelength_m, var, time);
    else
      it->second.last_seen = time;
  }
}

uint32_t RtkSolverPipeline::ekf_update(const std::vector<DdMeasurement> &dds, std::vector<bool> &phase_used) {
  phase_used.assign(dds.size(), false);

  // 量测行：每个双差一行伪距，载波有效且两颗星都有模糊度状态时再加一行载波。
  struct Row {
    size_t dd;
    bool phase;
  };
  std::vector<Row> rows;
  for (size_t i = 0; i < dds.size(); ++i) {
    rows.push_back({i, false});
    if (dds[i].cp_valid && ambs_.count({dds[i].sat, dds[i].band}) && ambs_.count({dds[i].ref_sat, dds[i].band}))
      rows.push_back({i, true});
  }
  if (rows.empty())
    return 0;

  const int n = static_cast<int>(x_.size()), m = static_cast<int>(rows.size());
  const Eigen::Vector3d pos = x_.head<3>();
  const double baseline_std = options_.baseline_std_ppm * 1e-6 * (pos - base_ecef_).norm();
  const double baseline_var = baseline_std * baseline_std;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, n);
  Eigen::VectorXd y(m);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);
  for (int r = 0; r < m; ++r) {
    const DdMeasurement &dd = dds[rows[r].dd];
    const double rho = geo_range(dd.sat_pos, pos) - geo_range(dd.ref_sat_pos, pos) - dd.base_sd_range_m;
    H.block<1, 3>(r, 0) = -((dd.sat_pos - pos).normalized() - (dd.ref_sat_pos - pos).normalized()).transpose();
    if (rows[r].phase) {
      const int ia = ambs_.at({dd.sat, dd.band}).idx, ir = ambs_.at({dd.ref_sat, dd.band}).idx;
      y(r) = dd.dd_cp_m - (rho + dd.wavelength_m * (x_(ia) - x_(ir)));
      H(r, ia) = dd.wavelength_m;
      H(r, ir) = -dd.wavelength_m;
    } else {
      y(r) = dd.dd_psr_m - rho;
    }
    // 双差协方差：同类型、同组的观测共享参考星方差项；基线比例项近似未模型化的电离层/对流层/轨道残差。
    const double std0 = rows[r].phase ? options_.phase_std_m : options_.code_std_m;
    const double var0 = 2.0 * (std0 * std0 + baseline_var);
    R(r, r) = var0 * (dd.var_factor_sat + dd.var_factor_ref);
    for (int q = 0; q < r; ++q) {
      const DdMeasurement &other = dds[rows[q].dd];
      if (rows[q].phase == rows[r].phase && other.sys == dd.sys && other.band == dd.band)
        R(r, q) = R(q, r) = var0 * dd.var_factor_ref;
    }
  }

  // 新息门限：按先验 S 的对角元检验；载波超限多为周跳，重置该星（非参考星）模糊度。
  const Eigen::MatrixXd PHt = P_ * H.transpose();
  std::vector<int> keep;
  std::vector<AmbKey> to_reset;
  for (int r = 0; r < m; ++r) {
    const double S_rr = H.row(r).dot(PHt.col(r)) + R(r, r);
    if (std::abs(y(r)) > options_.innov_gate_sigma * std::sqrt(S_rr)) {
      if (rows[r].phase)
        to_reset.push_back({dds[rows[r].dd].sat, dds[rows[r].dd].band});
      continue;
    }
    keep.push_back(r);
  }

  uint32_t num_phase = 0;
  if (!keep.empty()) {
    const int k = static_cast<int>(keep.size());
    Eigen::MatrixXd Hk(k, n), Rk(k, k);
    Eigen::VectorXd yk(k);
    for (int i = 0; i < k; ++i) {
      Hk.row(i) = H.row(keep[i]);
      yk(i) = y(keep[i]);
      for (int j = 0; j < k; ++j)
        Rk(i, j) = R(keep[i], keep[j]);
    }
    const Eigen::MatrixXd S = Hk * P_ * Hk.transpose() + Rk;
    // K = P Hᵀ S⁻¹ = (S⁻¹ H P)ᵀ，P、S 均对称。
    const Eigen::MatrixXd K = S.ldlt().solve(Hk * P_).transpose();
    x_ += K * yk;
    const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(n, n) - K * Hk;
    P_ = IKH * P_ * IKH.transpose() + K * Rk * K.transpose(); // Joseph 形式，保持对称正定
    for (int i = 0; i < k; ++i) {
      if (!rows[keep[i]].phase)
        continue;
      phase_used[rows[keep[i]].dd] = true;
      ++num_phase;
    }
  }
  for (const auto &key : to_reset)
    remove_ambiguity(key);
  return num_phase;
}

bool RtkSolverPipeline::fix_ambiguity(const std::vector<DdMeasurement> &dds, const std::vector<bool> &phase_used, //
                                      Eigen::Vector3d &pos, Eigen::Matrix3d &cov, double &ratio,                 //
                                      std::vector<std::pair<int, int>> &fixed_pairs, Eigen::VectorXd &fixed_amb) const {
  ratio = 0;
  // 候选双差模糊度：(本星状态下标, 参考星状态下标, 本星定权系数 1/sin²E)；低高度角排在前面，部分固定时先剔除。
  struct Candidate {
    int sat_idx, ref_idx;
    double var_factor;
  };
  std::vector<Candidate> cands;
  for (size_t i = 0; i < dds.size(); ++i) {
    if (!phase_used[i])
      continue;
    const auto sat_it = ambs_.find({dds[i].sat, dds[i].band}), ref_it = ambs_.find({dds[i].ref_sat, dds[i].band});
    if (sat_it != ambs_.end() && ref_it != ambs_.end())
      cands.push_back({sat_it->second.idx, ref_it->second.idx, dds[i].var_factor_sat});
  }
  std::sort(cands.begin(), cands.end(), [](const Candidate &a, const Candidate &b) { return a.var_factor > b.var_factor; });

  const int n = static_cast<int>(x_.size());
  for (size_t drop = 0; drop < cands.size(); ++drop) {
    const int na = static_cast<int>(cands.size() - drop);
    if (na < static_cast<int>(options_.min_fix_dd))
      return false;
    // 双差模糊度 a = D·x，D 每行在本星处 +1、参考星处 −1。
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(na, n);
    for (int k = 0; k < na; ++k) {
      D(k, cands[drop + k].sat_idx) = 1.0;
      D(k, cands[drop + k].ref_idx) = -1.0;
    }
    const Eigen::VectorXd a = D * x_;
    const Eigen::MatrixXd Qa = D * P_ * D.transpose();
    Eigen::MatrixXd F;
    Eigen::VectorXd s;
    const bool searched = lambda(a, Qa, 2, F, s);
    const double this_ratio = searched ? s(1) / std::max(s(0), 1e-12) : 0.0;
    if (drop == 0)
      ratio = this_ratio; // 未能固定时报告全集 ratio
    if (searched && this_ratio >= options_.min_ratio) {
      // 固定解回代：x_b = x_b − P_ba·Q_a⁻¹·(a − ǎ)，P_b = P_bb − P_ba·Q_a⁻¹·P_ab，b 为位置。
      const Eigen::MatrixXd Pba = P_.topRows(3) * D.transpose();
      const Eigen::LDLT<Eigen::MatrixXd> Qa_ldlt(Qa);
      pos = x_.head<3>() - Pba * Qa_ldlt.solve(a - F.col(0));
      cov = P_.topLeftCorner<3, 3>() - Pba * Qa_ldlt.solve(Pba.transpose());
      ratio = this_ratio;
      fixed_pairs.clear();
      for (int k = 0; k < na; ++k)
        fixed_pairs.emplace_back(cands[drop + k].sat_idx, cands[drop + k].ref_idx);
      fixed_amb = F.col(0);
      return pos.allFinite();
    }
    if (!options_.par_enabled)
      return false;
  }
  return false;
}

void RtkSolverPipeline::hold_ambiguity(const std::vector<std::pair<int, int>> &fixed_pairs, const Eigen::VectorXd &fixed_amb) {
  const int na = static_cast<int>(fixed_pairs.size()), n = static_cast<int>(x_.size());
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(na, n);
  for (int k = 0; k < na; ++k) {
    H(k, fixed_pairs[k].first) = 1.0;
    H(k, fixed_pairs[k].second) = -1.0;
  }
  const Eigen::VectorXd y = fixed_amb - H * x_;
  const Eigen::MatrixXd R = Eigen::MatrixXd::Identity(na, na) * options_.hold_std_cycles * options_.hold_std_cycles;
  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  const Eigen::MatrixXd K = S.ldlt().solve(H * P_).transpose();
  x_ += K * y;
  const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(n, n) - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
}

std::map<uint32_t, uint8_t> RtkSolverPipeline::detect_slips_gf(const std::vector<ObsPtr> &rover_obs, const std::vector<ObsPtr> &base_obs, const bool base_changed) {
  std::map<uint32_t, uint8_t> slips;
  // 相邻两次 GF 之差主要是电离层变化（cm 级）；超过阈值视为周跳。间隔超过 max_dt 时只更新记录、不比较。
  const auto check = [&](const std::vector<ObsPtr> &obs_list, std::map<uint32_t, GfRecord> &history, const uint8_t bit, const double max_dt) {
    for (const auto &obs : obs_list) {
      double gf_m = 0;
      if (!gf_combination(obs, gf_m)) {
        history.erase(obs->sat);
        continue;
      }
      const auto it = history.find(obs->sat);
      if (it != history.end() && time_diff(obs->time, it->second.time) <= max_dt && std::abs(gf_m - it->second.gf_m) > options_.gf_slip_threshold_m)
        slips[obs->sat] |= bit;
      history[obs->sat] = {obs->time, gf_m};
    }
  };
  check(rover_obs, rover_gf_, 0x01, options_.max_gap_s);
  // 同一基站历元会被多个流动站历元复用，只在切换时与上一基站历元比较；基站间隔按不超过两倍最大龄期处理。
  if (base_changed)
    check(base_obs, base_gf_, 0x02, 2.0 * options_.max_base_age_s);
  return slips;
}

void RtkSolverPipeline::add_ambiguity(const AmbKey &key, const double value_cycles, const double var_cycles2, const gtime_t &time) {
  const int n = static_cast<int>(x_.size());
  x_.conservativeResize(n + 1);
  x_(n) = value_cycles;
  P_.conservativeResize(n + 1, n + 1);
  P_.row(n).setZero();
  P_.col(n).setZero();
  P_(n, n) = var_cycles2;
  ambs_[key] = {n, time};
}

void RtkSolverPipeline::remove_ambiguity(const AmbKey &key) {
  const auto it = ambs_.find(key);
  if (it == ambs_.end())
    return;
  const int i = it->second.idx, n = static_cast<int>(x_.size()), tail = n - i - 1;
  Eigen::VectorXd x(n - 1);
  Eigen::MatrixXd P(n - 1, n - 1);
  x.head(i) = x_.head(i);
  x.tail(tail) = x_.tail(tail);
  P.topLeftCorner(i, i) = P_.topLeftCorner(i, i);
  P.topRightCorner(i, tail) = P_.topRightCorner(i, tail);
  P.bottomLeftCorner(tail, i) = P_.bottomLeftCorner(tail, i);
  P.bottomRightCorner(tail, tail) = P_.bottomRightCorner(tail, tail);
  x_ = x;
  P_ = P;
  ambs_.erase(it);
  for (auto &amb : ambs_)
    if (amb.second.idx > i)
      --amb.second.idx;
}

} // namespace gnss_comm
