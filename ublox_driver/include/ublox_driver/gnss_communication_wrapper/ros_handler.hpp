/**
 * @brief 管理 ublox_driver 的 ROS topic 发布逻辑。
 */
#ifndef UBLOX_DRIVER_ROS_HANDLER_HPP_
#define UBLOX_DRIVER_ROS_HANDLER_HPP_

#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/msg/rtcm_msg.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace ublox_driver {

inline constexpr uint32_t kDefaultRawObservationSystemMask = SYS_GPS | SYS_BDS;
inline constexpr uint32_t kDefaultEphemerisSystemMask = SYS_GPS | SYS_BDS;

class UbloxRosHandler {
public:
  explicit UbloxRosHandler(const rclcpp::Node::SharedPtr &node,                                     //
                           uint32_t raw_observation_system_mask = kDefaultRawObservationSystemMask, //
                           uint32_t ephemeris_system_mask = kDefaultEphemerisSystemMask);

  /** @brief 发布每个卫星的观测 */
  void publishObservations(const std::vector<gnss_comm::ObsPtr> &meas);

  /** @brief 发布星历消息 */
  void publishEphemeris(const gnss_comm::EphemBasePtr &ephem);

  /** @brief 发布电离层消息 */
  void publishIonoParams(const std::vector<double> &iono_params, const gnss_comm::gtime_t &stamp);

  /** @brief 发布时间脉冲消息 */
  void publishTimePulse(const gnss_comm::TimePulseInfoPtr &time_pulse);

  /** @brief 发布PVT消息 */
  void publishPvt(const gnss_comm::PVTSolutionPtr &pvt_soln);

  /** @brief 发布RTCM差分帧 */
  void publishRtcm(const uint8_t *data, size_t len, uint16_t message_type, uint16_t payload_length);

  /** @brief  */
  void registerPvtCallback(std::function<void(const gnss_comm::PVTSolutionPtr &)> callback);

private:
  rclcpp::Node::SharedPtr node_;

  //
  rclcpp::CallbackGroup::SharedPtr cbg_pvt_;
  rclcpp::PublisherOptions pub_pvt_options_;
  rclcpp::Publisher<gnss_comm::msg::GnssPVTSolnMsg>::SharedPtr pub_pvt_;

  //
  rclcpp::CallbackGroup::SharedPtr cbg_lla_;
  rclcpp::PublisherOptions pub_lla_options_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_lla_;

  //
  rclcpp::CallbackGroup::SharedPtr cbg_tp_info_;
  rclcpp::PublisherOptions pub_tp_info_options_;
  rclcpp::Publisher<gnss_comm::msg::GnssTimePulseInfoMsg>::SharedPtr pub_tp_info_;

  //
  rclcpp::CallbackGroup::SharedPtr cbg_range_meas_;
  rclcpp::PublisherOptions pub_range_meas_options_;
  rclcpp::Publisher<gnss_comm::msg::GnssMeasMsg>::SharedPtr pub_range_meas_;

  //
  rclcpp::CallbackGroup::SharedPtr cbg_ephem_;
  rclcpp::PublisherOptions pub_ephem_options_;
  rclcpp::Publisher<gnss_comm::msg::GnssEphemMsg>::SharedPtr pub_ephem_;

  //
  rclcpp::CallbackGroup::SharedPtr cbg_glo_ephem_;
  rclcpp::PublisherOptions pub_glo_ephem_options_;
  rclcpp::Publisher<gnss_comm::msg::GnssGloEphemMsg>::SharedPtr pub_glo_ephem_;

  //
  rclcpp::CallbackGroup::SharedPtr cbg_iono_;
  rclcpp::PublisherOptions pub_iono_options_;
  rclcpp::Publisher<gnss_comm::msg::StampedFloat64Array>::SharedPtr pub_iono_;

  //
  rclcpp::CallbackGroup::SharedPtr cbg_rtcm_;
  rclcpp::PublisherOptions pub_rtcm_options_;
  rclcpp::Publisher<gnss_comm::msg::RtcmMsg>::SharedPtr pub_rtcm_;

  //
  std::vector<std::function<void(const gnss_comm::PVTSolutionPtr &)>> pvt_callbacks_;

  //
  uint32_t raw_observation_system_mask_; // 原始观测的Mask
  uint32_t ephemeris_system_mask_;       // 星历的Mask
};

} // namespace ublox_driver

#endif
