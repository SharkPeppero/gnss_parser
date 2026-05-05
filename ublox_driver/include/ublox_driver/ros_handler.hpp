/**
 * @brief 管理 ublox_driver 的 ROS topic 发布逻辑。
 */
#ifndef UBLOX_DRIVER_ROS_HANDLER_HPP_
#define UBLOX_DRIVER_ROS_HANDLER_HPP_

#include <functional>
#include <memory>
#include <vector>

#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace ublox_driver {

class UbloxRosHandler {
public:
  explicit UbloxRosHandler(const rclcpp::Node::SharedPtr &node,
                           uint32_t raw_observation_system_mask = SYS_GPS | SYS_BDS,
                           uint32_t ephemeris_system_mask = SYS_GPS | SYS_BDS);

  void publishObservations(const std::vector<gnss_comm::ObsPtr> &meas);
  void publishEphemeris(const gnss_comm::EphemBasePtr &ephem);
  void publishIonoParams(const std::vector<double> &iono_params, const gnss_comm::gtime_t &stamp);
  void publishTimePulse(const gnss_comm::TimePulseInfoPtr &time_pulse);
  void publishPvt(const gnss_comm::PVTSolutionPtr &pvt_soln);
  void registerPvtCallback(std::function<void(const gnss_comm::PVTSolutionPtr &)> callback);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cbg_pvt_;
  rclcpp::PublisherOptions pub_pvt_options_;
  rclcpp::Publisher<gnss_interfaces::msg::GnssPVTSolnMsg>::SharedPtr pub_pvt_;
  rclcpp::CallbackGroup::SharedPtr cbg_lla_;
  rclcpp::PublisherOptions pub_lla_options_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_lla_;
  rclcpp::CallbackGroup::SharedPtr cbg_tp_info_;
  rclcpp::PublisherOptions pub_tp_info_options_;
  rclcpp::Publisher<gnss_interfaces::msg::GnssTimePulseInfoMsg>::SharedPtr pub_tp_info_;
  rclcpp::CallbackGroup::SharedPtr cbg_range_meas_;
  rclcpp::PublisherOptions pub_range_meas_options_;
  rclcpp::Publisher<gnss_interfaces::msg::GnssMeasMsg>::SharedPtr pub_range_meas_;
  rclcpp::CallbackGroup::SharedPtr cbg_ephem_;
  rclcpp::PublisherOptions pub_ephem_options_;
  rclcpp::Publisher<gnss_interfaces::msg::GnssEphemMsg>::SharedPtr pub_ephem_;
  rclcpp::CallbackGroup::SharedPtr cbg_glo_ephem_;
  rclcpp::PublisherOptions pub_glo_ephem_options_;
  rclcpp::Publisher<gnss_interfaces::msg::GnssGloEphemMsg>::SharedPtr pub_glo_ephem_;
  rclcpp::CallbackGroup::SharedPtr cbg_iono_;
  rclcpp::PublisherOptions pub_iono_options_;
  rclcpp::Publisher<gnss_interfaces::msg::StampedFloat64Array>::SharedPtr pub_iono_;
  std::vector<std::function<void(const gnss_comm::PVTSolutionPtr &)>> pvt_callbacks_;
  uint32_t raw_observation_system_mask_;
  uint32_t ephemeris_system_mask_;
};

} // namespace ublox_driver

#endif
