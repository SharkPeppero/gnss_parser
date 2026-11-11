#ifndef GNSS_COMM_ROS_CONVERSION_HPP_
#define GNSS_COMM_ROS_CONVERSION_HPP_

#include "rclcpp/rclcpp.hpp"
#include <gnss_msg/msg/gnss_best_xyz_msg.hpp>
#include <gnss_msg/msg/gnss_ephem_msg.hpp>
#include <gnss_msg/msg/gnss_glo_ephem_msg.hpp>
#include <gnss_msg/msg/gnss_meas_msg.hpp>
#include <gnss_msg/msg/gnss_obs_msg.hpp>
#include <gnss_msg/msg/gnss_pvt_soln_msg.hpp>
#include <gnss_msg/msg/gnss_svs_msg.hpp>
#include <gnss_msg/msg/gnss_time_msg.hpp>
#include <gnss_msg/msg/gnss_time_pulse_info_msg.hpp>
#include <gnss_msg/msg/stamped_float64_array.hpp>

#include "gnss_comm/gnss_constant.hpp"
#include "gnss_comm/gnss_utility.hpp"

/// Noted：
/// Ephem含义：            “轨道是什么形状、朝哪个方向、卫星走到哪里”   | 开普勒方程 + 轨道修正 | 目标时刻卫星位置和速度
/// Glonass Ephem含义：    “参考时刻卫星在哪里、速度多少、受到什么摄动” | 动力学方程 + 数值积分 | 目标时刻卫星位置和速度

// 工程内部 GNSS 数据结构与 gnss_msg ROS 2 消息之间的转换接口。
// msg2*：从只读 ROS 消息复制字段，构造新的内部数据对象。
// *2msg：从内部数据复制字段，返回可供发布的 ROS 消息值；函数本身不发布话题。
// 指针参数及观测集合中的指针应有效；这里不进行定位求解、观测筛选或坐标系变换。
// week/tow 时间字段按 GPST 编解码，tow 的单位为秒；不自动执行 UTC/BDT 到 GPST 的换算。
namespace gnss_comm {

/**
 * @brief ROS 广播星历消息 → 内部开普勒参数星历。
 * @param gnss_ephem_msg 输入星历消息，适用于工程中的 GPS、Galileo、北斗等星历类型。
 * @return 新建的 Ephem 共享指针，包含轨道参数、钟差、群延迟及健康状态。
 * @note ttr/toe/toc 从 GPST 周数与周内秒恢复；独立的 week/toe_tow 字段原样复制。
 */
EphemPtr msg2ephem(const gnss_msg::msg::GnssEphemMsg::ConstSharedPtr &gnss_ephem_msg);

/**
 * @brief ROS GLONASS 星历消息 → 内部 GLONASS 状态向量星历。
 * @param gnss_glo_ephem_msg 输入消息，包含位置、速度、加速度、频道号与钟差参数。
 * @return 新建的 GloEphem 共享指针；ttr/toe 按 GPST 恢复，其他字段保持原单位。
 */
GloEphemPtr msg2glo_ephem(const gnss_msg::msg::GnssGloEphemMsg::ConstSharedPtr &gnss_glo_ephem_msg);

/**
 * @brief ROS 观测集合消息 → 内部卫星观测列表。
 * @param gnss_meas_msg 输入观测消息，通常包含同一历元的多颗卫星。
 * @return 每条消息观测对应一个新建的 Obs 对象，列表顺序与输入一致。
 * @note 保留各观测自身的时间和信号向量，不检查是否同历元，也不剔除无效观测。
 */
std::vector<ObsPtr> msg2meas(const gnss_msg::msg::GnssMeasMsg::ConstSharedPtr &gnss_meas_msg);

/**
 * @brief ROS 时间脉冲消息 → 内部 PPS 信息。
 * @param gnss_tp_info_msg 输入脉冲时间及参考系统标志。
 * @return 新建的 TimePulseInfo 共享指针。
 * @note time 按 GPST 周数/周内秒解码；utc_based 与 time_sys 仅复制，不触发时间系统转换。
 */
TimePulseInfoPtr msg2tp_info(const gnss_msg::msg::GnssTimePulseInfoMsg::ConstSharedPtr &gnss_tp_info_msg);

/**
 * @brief ROS PVT 消息 → 内部位置、速度和时间解。
 * @param pvt_msg 输入定位结果，包含解类型、有效性、位置、速度及精度指标。
 * @return 新建的 PVTSolution 共享指针；时间按 GPST 恢复，速度保持北/东/地分量。
 */
PVTSolutionPtr msg2pvt(const gnss_msg::msg::GnssPVTSolnMsg::ConstSharedPtr &pvt_msg);

/**
 * @brief 内部开普勒参数星历 → ROS 广播星历消息。
 * @param ephem_ptr 输入星历；ttr/toe/toc 应已统一为 GPST。
 * @return 字段复制后的消息，三个参考时间分别编码为 GPS 周数与周内秒。
 * @note 不进行北斗时间加 14 秒等修正；调用前应完成时间系统统一。
 */
gnss_msg::msg::GnssEphemMsg ephem2msg(const EphemPtr &ephem_ptr);

/**
 * @brief 内部 GLONASS 星历 → ROS GLONASS 星历消息。
 * @param glo_ephem_ptr 输入星历；ttr/toe 应使用工程内部的 GPST 表示。
 * @return 位置、速度、加速度按 x/y/z 分量展开的消息，频道号和钟差参数原样复制。
 */
gnss_msg::msg::GnssGloEphemMsg glo_ephem2msg(const GloEphemPtr &glo_ephem_ptr);

/**
 * @brief 内部卫星观测列表 → ROS 观测集合消息。
 * @param meas 通常为同一历元的多颗卫星观测；每颗卫星可包含多个信号。
 * @return GnssMeasMsg；空列表对应空观测消息，各条记录保持输入顺序。
 * @note 频率为 Hz，伪距为 m，载波相位为周，多普勒为 Hz；不进行单位换算或重采样。
 */
gnss_msg::msg::GnssMeasMsg meas2msg(const std::vector<ObsPtr> &meas);

/**
 * @brief 内部 PPS 信息 → ROS 时间脉冲消息。
 * @param tp_info 输入脉冲信息，包含时间、是否基于 UTC 及参考系统标识。
 * @return time 经 time2gpst 编码后的消息，utc_based/time_sys 原样复制。
 * @note UTC 标志不会触发 UTC→GPST 换算，时间字段与标志的语义由调用方保证。
 */
gnss_msg::msg::GnssTimePulseInfoMsg tp_info2msg(const TimePulseInfoPtr &tp_info);

/**
 * @brief 内部 XYZ 定位结果 → ROS XYZ 结果消息。
 * @param best_xyz 输入位置、速度、对应标准差及参与解算的卫星数。
 * @return frame_id 为 ECEF 的消息；num_svs 使用 num_soln_svs，而非全部跟踪卫星数。
 * @note 仅复制实现中对应的字段，不包含 BestXYZ 的所有状态信息，也不执行坐标变换。
 * @note 当前 header.stamp 使用 rclcpp::Time(time2sec(...))；秒与 ROS 构造参数的单位需单独核查。
 */
gnss_msg::msg::GnssBestXYZMsg best_xyz2msg(const BestXYZPtr &best_xyz);

/**
 * @brief 内部 PVT 定位解 → ROS PVT 消息。
 * @param pvt_soln 输入定位结果；包括经纬高、海拔高、北东地速度和精度指标。
 * @return 时间编码为 GPST 周数/周内秒的消息；定位状态及数值字段保持不变。
 */
gnss_msg::msg::GnssPVTSolnMsg pvt2msg(const PVTSolutionPtr &pvt_soln);

/**
 * @brief 内部卫星信息列表 → ROS 卫星状态消息。
 * @param svs 卫星列表，通常应来自同一历元；包含健康状态、频道号、方位角和高度角等。
 * @return 各字段按输入顺序展开为平行数组；空列表返回默认消息。
 * @note 非空列表使用第一颗卫星的时间生成 header.stamp，不进行历元一致性检查。
 * @note 当前 header.stamp 使用 rclcpp::Time(time2sec(...))；秒与 ROS 构造参数的单位需单独核查。
 */
gnss_msg::msg::GnssSvsMsg svs2msg(const std::vector<SvInfo> &svs);

} // namespace gnss_comm

#endif // GNSS_COMM_ROS_CONVERSION_HPP_
