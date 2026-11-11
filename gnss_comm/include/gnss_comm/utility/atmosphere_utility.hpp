#ifndef GNSS_COMM_UTILITY_ATMOSPHERE_HPP_
#define GNSS_COMM_UTILITY_ATMOSPHERE_HPP_

#include "gnss_comm/types/time.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>

// 信号传播的大气延迟模型；返回距离改正量，单位为米。
// 延迟类型1：对流层延迟
// 延迟类型2：电离层延迟
namespace gnss_comm {

/**
 * @brief 计算对流层斜路径延迟（m）；位置为纬度/经度（度）/高程（m），方位高度角为 rad。
 * @param time 观测时刻，GPST。
 * @param rev_lla 接收机纬度、经度（度）及椭球高（m）。
 * @param azel 非空的 {方位角,高度角} 数组，单位 rad。
 * @return 延迟对应的传播距离，m，不是秒。
 * @note 当前使用标准大气与 Saastamoinen 模型，并进行映射；高度角非正或高程超出模型范围时返回 0。
 */
double calculate_trop_delay(gtime_t time, const Eigen::Vector3d &rev_lla, const double *azel);

/**
 * @brief 由 8 个广播电离层参数计算 L1 电离层延迟（m）；经纬度为度，方位高度角为 rad。
 * @param t 观测时刻，GPST。
 * @param rev_lla 接收机纬度、经度（度）及椭球高（m）。
 * @param azel 非空的 {方位角,高度角} 数组，单位 rad。
 * @return 延迟对应的传播距离，m，不是秒。
 * @param ion_parameters 8 个广播模型参数，依次为 alpha0~3、beta0~3。
 * @note 输出为 L1 频率对应的模型延迟，不能直接当作所有频段的延迟。
 */
double calculate_ion_delay(gtime_t t, const std::vector<double> &ion_parameters, const Eigen::Vector3d &rev_lla, const double *azel);

} // namespace gnss_comm

#endif // GNSS_COMM_UTILITY_ATMOSPHERE_HPP_
