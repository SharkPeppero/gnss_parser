//
// Created by yjh on 2026/11/10.
//

#ifndef GNSS_COMM_SOLVER_RTK_HPP_
#define GNSS_COMM_SOLVER_RTK_HPP_

#include "gnss_comm/constants/solution.hpp"
#include "gnss_comm/solver/spp.hpp"
#include "gnss_comm/types/ephemeris.hpp"
#include "gnss_comm/types/observation.hpp"
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <map>
#include <utility>
#include <vector>

namespace gnss_comm {

struct RtkSolverPipelineOptions {
  double elevation_cutoff_degree = 15.0; // 卫星高度角截止值，度
  uint32_t num_bands = 2;                // 参与双差的频段数：1 只用 L1/G1/E1/B1I；2 再加 L2/G2/E5b/B2I
  bool use_glonass = false;              // GLONASS FDMA 双差需要处理频间偏差，当前默认不用
  double code_std_m = 0.3;               // 天顶方向非差伪距标准差，m
  double phase_std_m = 0.003;            // 天顶方向非差载波标准差，m
  double max_base_age_s = 30.0;          // 流动站与基站观测的最大时间差（差分龄期），s
  uint32_t max_iterations = 10;
  double convergence_threshold = 1e-4; // 位置增量范数阈值，m
  SppSolverPipelineOptions spp;        // 流动站初值 SPP 的参数

  // 浮点 EKF
  double baseline_std_ppm = 1.0;        // 与基线长度成正比的非差误差，ppm（1 ppm × 13 km ≈ 1.3 cm）
  double accel_std_mps2 = 3.0;          // 匀速模型的白噪声加速度标准差，m/s²
  double amb_init_std_cycles = 10.0;    // 新模糊度初值标准差，周
  double amb_process_std_cycles = 1e-4; // 模糊度随机游走，周/√s
  double innov_gate_sigma = 5.0;        // 新息 |y| > k·√S 时剔除该观测；载波同时重置该星模糊度
  double max_gap_s = 10.0;              // 相邻历元间隔超过该值时重置滤波器，s
  double max_amb_outage_s = 10.0;       // 模糊度连续未观测超过该值时删除，s
  double gf_slip_threshold_m = 0.05;    // 无几何组合 GF 相邻历元跳变超过该值判为周跳，m

  // 模糊度固定
  double min_ratio = 3.0;       // LAMBDA ratio 检验阈值
  uint32_t min_fix_dd = 5;      // 参与固定的最少双差模糊度个数
  bool par_enabled = true;      // 全集固定失败时，按高度角从低到高逐个剔除再试（部分模糊度固定）
  // 部分固定时参与固定的双差数至少占候选总数的比例；剔得过多时只剩少数双差，误固定概率很高，残差检验也查不出
  double par_min_fix_fraction = 0.9;
  bool fix_and_hold = true;     // 固定成功后把整数双差作为强约束回写滤波器
  double hold_std_cycles = 1e-3; // fix-and-hold 伪观测标准差，周
  // hold 之后浮点模糊度被整数约束，ratio 会虚高而失去检验意义：连续固定达到该历元数才开始 hold，避免一次误固定被锁死
  uint32_t hold_min_fix_epochs = 10;
  double fix_validate_sigma = 4.0; // 固定解回代后各双差载波残差超过 k·σ 则拒绝该组固定（不依赖 ratio）

  // 滤波器保护
  double reset_pos_diff_m = 30.0;  // 浮点位置与伪距差分解（不可用时为 SPP）相差超过该值……
  uint32_t reset_diverge_epochs = 5; // ……且连续达到该历元数时重置滤波器；伪距差分解单历元可能被多路径拉偏
  double psr_outlier_sigma = 4.0;  // 双差伪距定位迭代剔除标准化残差超过 k·σ 的观测
  uint32_t reset_min_phase_dd = 5; // 参与更新的载波双差少于该值……
  double reset_low_phase_s = 10.0; // ……且持续超过该时长时重置滤波器，s
};

// 基站各（星座, 频段）的天线相位中心偏移，ECEF m，相对天线参考点；QZSS 按 GPS 查。
using BasePcoMap = std::map<std::pair<uint32_t, int>, Eigen::Vector3d>;

// 一个双差观测：(rover^j − rover^r) − (base^j − base^r)，r 为同星座、同频段的参考星。
// 已扣除两站各自发射时刻的卫星钟差，伪距与载波均为米。
struct DdMeasurement {
  uint32_t sat{}, ref_sat{};
  uint32_t sys{}; // 分组星座，QZSS 并入 GPS
  int band{};     // 0: L1/G1/E1/B1I，1: L2/G2/E5b/B2I
  double wavelength_m{};
  Eigen::Vector3d sat_pos{Eigen::Vector3d::Zero()};     // 流动站信号发射时刻的卫星位置，ECEF m
  Eigen::Vector3d ref_sat_pos{Eigen::Vector3d::Zero()}; // 同上，参考星
  double base_sd_range_m{};                             // 基站几何距离单差 ρ_b^j − ρ_b^r（含 Sagnac），m
  double dd_psr_m{};
  double dd_cp_m{}; // 双差载波，m；含双差整周模糊度 × 波长
  bool cp_valid{};  // 两站两星载波都有效（半周已解决），周跳由 *_slip 单独给出
  double var_factor_sat{}, var_factor_ref{}; // 高度角定权系数 1/sin²(el)，乘以 σ² 得方差
  // 站间单差，用于初始化单差模糊度状态及相位-伪距一致性校正，m
  double sat_sd_psr_m{}, ref_sd_psr_m{};
  double sat_sd_cp_m{}, ref_sd_cp_m{};
  uint8_t sat_slip{}, ref_slip{}; // 周跳标志：bit0 流动站（LLI 或 GF），bit1 基站（LLI 或 GF）
};

struct RtkSolution {
  gtime_t time{};
  uint32_t pos_type = PV_NONE; // PV_SINGLE / PV_PSR_DIFF / PV_NARROW_FLOAT / PV_NARROW_INT
  Eigen::Vector3d pos_ecef{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d pos_cov_ecef{Eigen::Matrix3d::Zero()}; // m²
  uint32_t num_dd_psr{}, num_dd_cp{};                    // 伪距双差个数；通过检验、参与更新的载波双差个数
  double base_age_s{};                                   // 流动站时刻 − 基站时刻，s
  double ratio{};                                        // 模糊度固定 ratio，未尝试或失败为 0
  uint32_t num_fix_dd{};                                 // 固定的双差模糊度个数（部分固定时少于 num_dd_cp）
};

// 单基站 RTK 计算流程，不负责文件读取、星历检索或基站历元匹配。
// 输入时间统一为 GPST；base_arp_ecef 为基站天线参考点（ARP）的精确 ECEF 坐标，误差会整体带入流动站结果；
// base_pco 为基站各频段相位中心相对 ARP 的偏移，为空时按 ARP 计算。流动站结果对应流动站天线的相位中心。
// 内部 EKF 跨历元保留状态，流动站历元须按时间顺序输入。
class RtkSolverPipeline {
public:
  RtkSolverPipeline(const Eigen::Vector3d &base_arp_ecef, const RtkSolverPipelineOptions &options = {}, const BasePcoMap &base_pco = {});

  ~RtkSolverPipeline() = default;

  // 流动站 SPP → 双差构建（含两站对流层改正）→ 双差伪距定位（初值/退化解）→ EKF 预测 → GF 周跳探测
  // → 模糊度管理 → 双差伪距+载波更新 → 发散/载波不足检查（必要时重置）
  // → LAMBDA（失败时部分固定）与 ratio + 残差检验 → 固定解回代 → 连续固定足够后 fix-and-hold。
  // rover_obs 与 rover_ephems 等长且按卫星逐项对应；base_obs 为匹配好的基站历元，基站卫星星历取自流动站。
  // 失败时 pos_type 为 PV_NONE；双差不可用时退回 PV_SINGLE。
  // TODO: GLONASS 频间偏差、接收机天线 PCV。
  RtkSolution solve(const std::vector<ObsPtr> &rover_obs,          //
                    const std::vector<EphemBasePtr> &rover_ephems, //
                    const std::vector<ObsPtr> &base_obs,           //
                    const std::vector<double> &iono_params);

  // 构建双差：两站共视卫星 → 高度角筛选 → 每个星座/频段取最高高度角卫星作参考星。
  // rover_ecef 为流动站近似位置，用于高度角和发射时刻相关的几何计算。
  std::vector<DdMeasurement> build_dd(const std::vector<ObsPtr> &rover_obs,          //
                                      const std::vector<EphemBasePtr> &rover_ephems, //
                                      const std::vector<ObsPtr> &base_obs,           //
                                      const Eigen::Vector3d &rover_ecef) const;

private:
  using AmbKey = std::pair<uint32_t, int>; // (卫星编号, 频段)
  struct AmbState {
    int idx{};          // 在 x_ 中的下标
    gtime_t last_seen{}; // 最近一次有有效载波的时刻
  };

  // 双差伪距加权最小二乘；权阵为双差协方差的逆（同组共享参考星，非对角）。
  // 收敛后逐个剔除标准化残差最大且超过 psr_outlier_sigma 的观测再解，最多剔除 1/3 且至少保留 5 个。
  // pos 输入为初值、输出为结果；cov 为位置协方差，m²。
  bool dd_psr_pos(const std::vector<DdMeasurement> &dds, Eigen::Vector3d &pos, Eigen::Matrix3d &cov) const;

  // 以伪距差分结果重置滤波器：位置取其协方差，速度置零，清空模糊度。
  void reset_filter(const Eigen::Vector3d &pos, const Eigen::Matrix3d &cov);

  // 匀速模型预测：位置 += 速度·dt；模糊度加随机游走噪声。
  void predict(double dt);

  // 模糊度管理：删除长期未观测/周跳的状态 → 按组校正相位-伪距偏移 → 初始化新模糊度。
  // base_changed 表示基站历元已切换，此时基站 LLI 才视为新周跳（同一基站历元会被多个流动站历元复用）。
  void update_ambiguities(const std::vector<DdMeasurement> &dds, const gtime_t &time, bool base_changed);

  // 双差伪距 + 双差载波量测更新，含新息门限；phase_used 标记各 dds 的载波是否参与更新。
  // 返回参与更新的载波双差个数。
  uint32_t ekf_update(const std::vector<DdMeasurement> &dds, std::vector<bool> &phase_used);

  // 无几何组合 GF = λ1φ1 − λ2φ2 周跳探测：与该站上一历元比较，跳变超过阈值即判周跳。
  // 基站只在 base_changed 时比较。返回 卫星 → 周跳位（bit0 流动站，bit1 基站）。
  std::map<uint32_t, uint8_t> detect_slips_gf(const std::vector<ObsPtr> &rover_obs, const std::vector<ObsPtr> &base_obs, bool base_changed);

  // 对本历元参与更新的双差模糊度做 LAMBDA；ratio 或残差检验不通过且允许部分固定时，按高度角从低到高逐个剔除再试。
  // 通过时输出固定后的位置及协方差，以及固定所用的模糊度状态下标对（本星, 参考星）与整数解，供 fix-and-hold 使用。
  // searched 表示至少对一组 >= min_fix_dd 的模糊度做过搜索（区分“检验不通过”与“模糊度太少”）。
  bool fix_ambiguity(const std::vector<DdMeasurement> &dds, const std::vector<bool> &phase_used, //
                     Eigen::Vector3d &pos, Eigen::Matrix3d &cov, double &ratio,                 //
                     std::vector<std::pair<int, int>> &fixed_pairs, Eigen::VectorXd &fixed_amb, bool &searched) const;

  // 固定解检验：用固定位置和整数双差模糊度计算各双差载波残差，全部在 fix_validate_sigma·σ 内才通过。
  bool validate_fix(const std::vector<DdMeasurement> &dds, const std::vector<size_t> &dd_indices, const Eigen::Vector3d &pos, const Eigen::VectorXd &dd_amb) const;

  // fix-and-hold：以固定的整数双差为伪观测（标准差 hold_std_cycles）更新滤波器。
  void hold_ambiguity(const std::vector<std::pair<int, int>> &fixed_pairs, const Eigen::VectorXd &fixed_amb);

  void add_ambiguity(const AmbKey &key, double value_cycles, double var_cycles2, const gtime_t &time);
  void remove_ambiguity(const AmbKey &key);
  // 删除全部模糊度状态，保留位置/速度，让模糊度从浮点重新收敛。
  void clear_ambiguities();

  // 基站在某（分组星座, 频段）上的相位中心 = ARP + PCO。
  Eigen::Vector3d base_phase_center(uint32_t sys, int band) const;

  Eigen::Vector3d base_ecef_; // 天线参考点 ARP
  Eigen::Vector3d base_lla_;  // ARP 经纬高，对流层模型使用
  BasePcoMap base_pco_;
  RtkSolverPipelineOptions options_;
  SppSolverPipeline spp_;

  // GF 周跳探测的上一历元记录：卫星 → (时刻, GF m)
  struct GfRecord {
    gtime_t time{};
    double gf_m{};
  };
  std::map<uint32_t, GfRecord> rover_gf_, base_gf_;

  // 滤波状态：x_ = [位置(3) m, 速度(3) m/s, 单差模糊度(周)...]，单差模糊度的公共部分不可观，只有双差有意义。
  bool initialized_ = false;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  std::map<AmbKey, AmbState> ambs_;
  gtime_t last_time_{}, last_base_time_{};

  // 固定与保护状态，reset_filter 时清零
  uint32_t fix_count_ = 0;      // 连续固定历元数
  bool held_ = false;           // 模糊度已被 fix-and-hold 约束过
  uint32_t diverge_count_ = 0;  // 浮点位置连续超出 reset_pos_diff_m 的历元数
  bool low_phase_ = false;      // 载波双差不足计时中
  gtime_t low_phase_since_{};   // 载波双差开始不足的时刻
};

} // namespace gnss_comm

#endif // GNSS_COMM_SOLVER_RTK_HPP_
