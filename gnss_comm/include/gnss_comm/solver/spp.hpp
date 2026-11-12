#ifndef GNSS_COMM_SOLVER_SPP_HPP_
#define GNSS_COMM_SOLVER_SPP_HPP_

#include "gnss_comm/constants/solution.hpp"
#include "gnss_comm/types/ephemeris.hpp"
#include "gnss_comm/types/observation.hpp"
#include "gnss_comm/types/satellite.hpp"
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace gnss_comm {

struct SppSolverPipelineOptions {
  double elevation_cutoff_degree = 15.0; // 卫星高度角截止值，度
  uint32_t max_iterations = MAX_ITER_PVT;
  double convergence_threshold = EPSILON_PVT; // 状态增量范数阈值，单位随位置/速度求解状态而定
};

// 单历元 SPP 计算流程，不负责文件读取、星历检索或跨历元滤波。
// 输入 obs/ephems 应等长且按卫星逐项对应，指针有效，观测数组按信号索引对齐。
// 输入时间统一为 GPST；内部卫星钟差使用 s，接收机钟差状态使用 m。
class SppSolverPipeline {
public:
  explicit SppSolverPipeline(const SppSolverPipelineOptions &options = {});

  ~SppSolverPipeline() = default;

  // 筛选 L1 类观测 → 计算卫星状态 → 迭代伪距加权最小二乘。
  // 北斗沿用 B1I 选择；iono_params 为原模型所需的 8 项电离层参数。
  // 返回 [ECEF位置(m), receiver_clock_bias_m(GPS/GLO/GAL/BDS)]，QZSS 共用 GPS 钟差。
  // 沿用既有失败约定：观测不足或达到迭代上限时返回零向量。
  Eigen::Matrix<double, 7, 1> psr_pos(const std::vector<ObsPtr> &obs,          //
                                      const std::vector<EphemBasePtr> &ephems, //
                                      const std::vector<double> &iono_params) const;

  // 筛选观测 → 计算卫星状态及高度角 → 迭代多普勒加权最小二乘。
  // ref_ecef 为接收机 ECEF 位置(m)；若为零，先以零电离层参数调用 psr_pos 并回写位置。
  // 返回 [ECEF速度(m/s), receiver_clock_drift_mps]。
  Eigen::Matrix<double, 4, 1> dopp_vel(const std::vector<ObsPtr> &obs,          // 所有卫星的观测
                                       const std::vector<EphemBasePtr> &ephems, // 所有卫星的星历
                                       Eigen::Vector3d &ref_ecef                // 参考的ECEF控制点
  ) const;

  // 步骤二：利用伪距估计发送时刻，计算位置、速度、钟差(s)和钟漂(s/s)。
  // 每颗卫星需有 L1 类伪距；RTK 复用它分别计算流动站与基站对应的卫星状态。
  std::vector<SatStatePtr> sat_states(const std::vector<ObsPtr> &obs, //
                                      const std::vector<EphemBasePtr> &ephems) const;

private:
  // 步骤一：筛选支持的星座及 L1 类频率，同时保持观测与星历一一对应。
  void filter_L1(const std::vector<ObsPtr> &obs,          //
                 const std::vector<EphemBasePtr> &ephems, //
                 std::vector<ObsPtr> &L1_obs,             //
                 std::vector<EphemBasePtr> &L1_ephems) const;

  // 位置迭代步骤：构建“预测值−观测值”的伪距残差(m)和雅可比。
  // atmos_delay 输出电离层/对流层延迟(m)，all_sv_azel 输出方位角/高度角(rad)。
  void psr_res(const Eigen::Matrix<double, 7, 1> &rcv_state,  //
               const std::vector<ObsPtr> &obs,                //
               const std::vector<SatStatePtr> &all_sv_states, //
               const std::vector<double> &iono_params,        //
               Eigen::VectorXd &res, Eigen::MatrixXd &J,      //
               std::vector<Eigen::Vector2d> &atmos_delay,     //
               std::vector<Eigen::Vector2d> &all_sv_azel) const;

  // 速度迭代步骤：构建距离率残差(m/s)，当前多普勒约定为距离率观测 = -波长×多普勒。
  void dopp_res(const Eigen::Matrix<double, 4, 1> &rcv_state,  //
                const Eigen::Vector3d &rcv_ecef,               //
                const std::vector<ObsPtr> &obs,                //
                const std::vector<SatStatePtr> &all_sv_states, //
                Eigen::VectorXd &res,                          //
                Eigen::MatrixXd &J) const;

  // 由对象拥有配置值，不依赖外部配置对象的生命周期；每次调用的解算状态保存在局部变量中。
  SppSolverPipelineOptions options_;
};

} // namespace gnss_comm

#endif // GNSS_COMM_SOLVER_SPP_HPP_
