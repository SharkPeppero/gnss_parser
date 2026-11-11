#ifndef GNSS_COMM_UTILITY_COORDINATE_HPP_
#define GNSS_COMM_UTILITY_COORDINATE_HPP_

#include <eigen3/Eigen/Dense>

// WGS-84 经纬高、地固坐标 ECEF 与局部东北天 ENU 的转换。
namespace gnss_comm {

/**
 * @brief 纬度、经度（度）及椭球高（m）转为地固 XYZ（m）。
 * @param lla {纬度,经度,椭球高}，单位分别为度、度、m。
 * @return WGS-84 椭球模型下的 ECEF {x,y,z}，单位 m。
 */
Eigen::Vector3d geo2ecef(const Eigen::Vector3d &lla);

/**
 * @brief 地固 XYZ（m）转为纬度、经度（度）、椭球高（m），顺序为纬度在前。
 * @param xyz ECEF {x,y,z}，单位 m。
 * @return {纬度,经度,椭球高}，单位为度、度、m。
 * @note 当前实现未处理 x=y=0 的极轴情况，不应将此类点作为正常输入。
 */
Eigen::Vector3d ecef2geo(const Eigen::Vector3d &xyz);

/**
 * @brief 按参考经纬度将 ECEF 向量旋转到东、北、天；不自动减去参考点位置。
 * @param pos_lla 参考点经纬高，顺序为纬度、经度、高程，角度使用度。
 * @param v_ecef 待旋转的 ECEF 向量，例如两点位置差或速度。
 * @return {东,北,天} 分量，保持输入向量的单位。
 * @note 转换目标点的位置时，先计算 target_ecef-reference_ecef，再传入本函数。
 */
Eigen::Vector3d ecef2enu(const Eigen::Vector3d &pos_lla, const Eigen::Vector3d &v_ecef);

/**
 * @brief 参考经纬高生成 ENU 到 ECEF 的旋转矩阵，角度单位为度。
 * @param ref_geo 参考点纬度、经度（度）及高程；旋转只由经纬度决定。
 * @return R，满足 v_ecef=R*v_enu；反向旋转使用 R.transpose()。
 */
Eigen::Matrix3d geo2rotation(const Eigen::Vector3d &ref_geo);

/**
 * @brief 通过参考点 ECEF 位置生成 ENU 到 ECEF 的旋转矩阵。
 * @param ref_ecef 参考点的地固 XYZ，单位 m。
 * @return ENU 到 ECEF 的旋转矩阵，内部先求参考点经纬度。
 */
Eigen::Matrix3d ecef2rotation(const Eigen::Vector3d &ref_ecef);

/**
 * @brief 接收机与卫星 ECEF 位置转为方位角、高度角，输出顺序 az/el，单位 rad。
 * @param rev_pos 接收机 ECEF 位置，m。
 * @param sat_pos 卫星 ECEF 位置，m。
 * @param[out] azel 非空、至少含两个元素；输出 {方位角,高度角}，单位 rad。
 * @note 方位角从北向东计量，范围 [0,2π)；高度角相对水平面，负值表示位于地平线下。
 */
void sat_azel(const Eigen::Vector3d &rev_pos, const Eigen::Vector3d &sat_pos, double *azel);

} // namespace gnss_comm

#endif // GNSS_COMM_UTILITY_COORDINATE_HPP_
