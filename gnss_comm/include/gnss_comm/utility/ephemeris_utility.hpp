#ifndef GNSS_COMM_UTILITY_EPHEMERIS_HPP_
#define GNSS_COMM_UTILITY_EPHEMERIS_HPP_

#include "gnss_comm/types/ephemeris.hpp"
#include <eigen3/Eigen/Dense>

// 广播星历轨道及钟差计算；开普勒参数模型与 GLONASS 数值积分模型分别处理。
namespace gnss_comm {

/**
 * @brief 根据平近点角 mk（rad）与偏心率 es 求偏近点角（rad）。
 * @param mk 平近点角，rad。
 * @param es 椭圆轨道偏心率，无量纲，适用 0<=es<1。
 * @return 偏近点角 E（rad），满足 E-es*sin(E)=mk。
 * @note 使用迭代计算；达到次数上限会记录警告，返回当前迭代值。
 */
double Kepler(const double mk, const double es);

/**
 * @brief 由广播星历钟差参数计算卫星钟差（s），不是钟漂变化率。
 * @param curr_time 计算时刻，使用工程内部 GPST 表示。
 * @param ephem_ptr 有效星历指针，参考时刻应已统一为 GPST。
 * @return 卫星钟差，s；不应把它当作位置误差或钟漂。
 * @note 此接口计算钟差多项式；轨道相关的相对论改正在 eph2pos 中另行加入。
 */
double eph2svdt(const gtime_t &curr_time, const EphemPtr ephem_ptr);

/**
 * @brief 由广播星历计算 ECEF 位置（m），satellite_clock_bias_s 可为空，否则输出钟差（s）。
 * @param curr_time 计算时刻，使用工程内部 GPST 表示。
 * @param ephem_ptr 有效星历指针，参考时刻应已统一为 GPST。
 * @param[out] satellite_clock_bias_s 可为空；非空时输出卫星钟差，s。
 * @return ECEF 卫星位置，m。
 */
Eigen::Vector3d eph2pos(const gtime_t &curr_time, const EphemPtr ephem_ptr, double *satellite_clock_bias_s);

/**
 * @brief 由广播星历计算 ECEF 速度（m/s），可输出钟差变化率（s/s）。
 * @param curr_time 计算时刻，使用工程内部 GPST 表示。
 * @param ephem 有效星历指针，参考时刻应已统一为 GPST。
 * @param[out] satellite_clock_drift_sps 可为空；非空时输出卫星钟差变化率，s/s。
 * @return ECEF 卫星速度，m/s。
 */
Eigen::Vector3d eph2vel(const gtime_t &curr_time, const EphemPtr ephem, double *satellite_clock_drift_sps);

/**
 * @brief GLONASS 轨道微分方程：由状态与广播摄动加速度计算位置、速度导数。
 * @param pos 地固位置，m。
 * @param vel 地固速度，m/s。
 * @param acc 广播摄动加速度，m/s²，不是总加速度。
 * @param[out] pos_dot 位置导数，m/s。
 * @param[out] vel_dot 速度导数，m/s²，含模型中的地球引力与旋转相关项。
 */
void deq(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const Eigen::Vector3d &acc, Eigen::Vector3d &pos_dot, Eigen::Vector3d &vel_dot);

/**
 * @brief 按 dt 秒执行四阶龙格库塔积分，原地更新 GLONASS 位置与速度。
 * @param dt 本次数值积分步长，s；可正向或反向。
 * @param[in,out] pos 地固位置，m，更新为积分后的值。
 * @param[in,out] vel 地固速度，m/s，更新为积分后的值。
 * @param acc 广播摄动加速度，m/s²。
 * @note 仅执行一个 RK4 步骤；长时间间隔由上层分步调用。
 */
void glo_orbit(double dt, Eigen::Vector3d &pos, Eigen::Vector3d &vel, const Eigen::Vector3d &acc);

/**
 * @brief 由 GLONASS 星历计算卫星钟差（s），时间使用 GPST。
 * @param curr_time 计算时刻，使用工程内部 GPST 表示。
 * @param geph_ptr 有效星历指针，参考时刻应已统一为 GPST。
 * @return 卫星钟差，s；不应把它当作位置误差或钟漂。
 */
double geph2svdt(const gtime_t &curr_time, const GloEphemPtr geph_ptr);

/**
 * @brief GLONASS 轨道积分得到 ECEF 位置（m），可输出钟差（s）。
 * @param curr_time 计算时刻，使用工程内部 GPST 表示。
 * @param geph_ptr 有效星历指针，参考时刻应已统一为 GPST。
 * @param[out] satellite_clock_bias_s 可为空；非空时输出卫星钟差，s。
 * @return ECEF 卫星位置，m。
 */
Eigen::Vector3d geph2pos(const gtime_t &curr_time, const GloEphemPtr geph_ptr, double *satellite_clock_bias_s);

/**
 * @brief GLONASS 轨道积分得到 ECEF 速度（m/s），可输出钟漂（s/s）。
 * @param curr_time 计算时刻，使用工程内部 GPST 表示。
 * @param geph_ptr 有效星历指针，参考时刻应已统一为 GPST。
 * @param[out] satellite_clock_drift_sps 可为空；非空时输出卫星钟差变化率，s/s。
 * @return ECEF 卫星速度，m/s。
 */
Eigen::Vector3d geph2vel(const gtime_t &curr_time, const GloEphemPtr geph_ptr, double *satellite_clock_drift_sps);

} // namespace gnss_comm

#endif // GNSS_COMM_UTILITY_EPHEMERIS_HPP_
