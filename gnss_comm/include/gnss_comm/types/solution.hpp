#ifndef GNSS_COMM_TYPES_SOLUTION_HPP_
#define GNSS_COMM_TYPES_SOLUTION_HPP_

#include <cstdint>
#include <memory>
#include "gnss_comm/types/time.hpp"

// 接收机定位结果：位置、速度、精度指标与解状态。
// 各类型保留原消息字段和布局；相似字段不表示消息来源或解类型相同。
namespace gnss_comm {

// 接收机输出的位置结果及质量信息。
struct BestPos {
  gtime_t time{};
  uint32_t sol_status{};
  uint32_t pos_type{};
  double lat{}, lon{}, hgt{};
  double lat_sigma{}, lon_sigma{}, hgt_sigma{};
  double undulation{};
  double diff_age{};
  double sol_age{};
  uint8_t num_svs{};
  uint8_t num_soln_svs{};
  uint8_t ext_sol_stat{};
  uint8_t sig_mask{};
};
typedef std::shared_ptr<BestPos> BestPosPtr;

// 接收机输出的速度结果及运动方向。
struct BestVel {
  gtime_t time{};
  uint32_t sol_status{};
  uint32_t vel_type{};
  double latency{};
  double age{};
  double hor_speed{}, vel_speed{};
  double direction2north_degree{};
};
typedef std::shared_ptr<BestVel> BestVelPtr;

// 以 XYZ 分量存储的位置、速度及其标准差。
struct BestXYZ {
  gtime_t time{};
  uint32_t pos_sol_status{};
  uint32_t pos_type{};
  double pos[3]{}, pos_sigma[3]{};
  uint32_t vel_sol_status{};
  uint32_t vel_type{};
  double vel[3]{}, vel_sigma[3]{};
  double vel_latency{};
  double diff_age{};
  double sol_age{};
  uint8_t num_svs{};
  uint8_t num_soln_svs{};
  uint8_t ext_sol_stat{};
  uint8_t sig_mask{};
};
typedef std::shared_ptr<BestXYZ> BestXYZPtr;

// PVT 综合解：位置、速度、时间与定位状态。
struct PVTSolution {
  gtime_t time{};
  uint8_t fix_type{}; // 0-no fix, 1-dead reckoning only, 2-2D, 3-3D, 4-GNSS+(1), 5-time only fix
  bool valid_fix{};
  bool diff_soln{};    // whether differential applied
  uint8_t carr_soln{}; // 0-no cp solution, 1-float, 2-fixed
  uint8_t num_sv{};
  double lat{}, lon{}, hgt{};
  double hgt_msl{};                 // height above mean sea level
  double h_acc{}, v_acc{};          // horizontal and vertical accuracy
  double p_dop{};                   // position DOP
  double vel_n{}, vel_e{}, vel_d{}; // velocity in NED frame
  double vel_acc{};                 // velocity accuracy
};
typedef std::shared_ptr<PVTSolution> PVTSolutionPtr;

// 伪距定位结果；保留独立类型以对应原消息接口。
struct PsrPos {
  gtime_t time{};
  uint32_t sol_status{};
  uint32_t pos_type{};
  double lat{}, lon{}, hgt{};
  double lat_sigma{}, lon_sigma{}, hgt_sigma{};
  double undulation{};
  double diff_age{};
  double sol_age{};
  uint8_t num_svs{};
  uint8_t num_soln_svs{};
  uint8_t ext_sol_stat{};
  uint8_t sig_mask{};
};
typedef std::shared_ptr<PsrPos> PsrPosPtr;

// 伪距定位相关的速度结果记录，具体算法来源由接收机协议决定。
struct PsrVel {
  gtime_t time{};
  uint32_t sol_status{};
  uint32_t vel_type{};
  double latency{};
  double age{}; /* differential age */
  double hor_spd{};
  double trk_gnd{};
  double vert_spd{};
};
typedef std::shared_ptr<PsrVel> PsrVelPtr;

} // namespace gnss_comm

#endif // GNSS_COMM_TYPES_SOLUTION_HPP_
