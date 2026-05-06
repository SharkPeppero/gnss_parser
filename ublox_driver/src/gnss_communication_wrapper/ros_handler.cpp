#include "ublox_driver/gnss_communication_wrapper/ros_handler.hpp"

#include <algorithm>
#include <cmath>

namespace {

rclcpp::Time cvtTimestamp(const gnss_comm::gtime_t &time) {
  return rclcpp::Time(static_cast<int64_t>(std::llround(gnss_comm::time2sec(time) * 1e9)));
}

} // namespace

namespace ublox_driver {

UbloxRosHandler::UbloxRosHandler(const rclcpp::Node::SharedPtr &node, uint32_t raw_observation_system_mask, uint32_t ephemeris_system_mask)
    : node_(node), raw_observation_system_mask_(raw_observation_system_mask), ephemeris_system_mask_(ephemeris_system_mask) {
  const auto qos = rclcpp::QoS(100);

  cbg_pvt_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_pvt_options_.callback_group = cbg_pvt_;
  pub_pvt_ = node_->create_publisher<gnss_interfaces::msg::GnssPVTSolnMsg>("~/receiver_pvt", qos, pub_pvt_options_);

  cbg_lla_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_lla_options_.callback_group = cbg_lla_;
  pub_lla_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("~/receiver_lla", qos, pub_lla_options_);

  cbg_tp_info_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_tp_info_options_.callback_group = cbg_tp_info_;
  pub_tp_info_ = node_->create_publisher<gnss_interfaces::msg::GnssTimePulseInfoMsg>("~/time_pulse_info", qos, pub_tp_info_options_);

  cbg_range_meas_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_range_meas_options_.callback_group = cbg_range_meas_;
  pub_range_meas_ = node_->create_publisher<gnss_interfaces::msg::GnssMeasMsg>("~/range_meas", qos, pub_range_meas_options_);

  cbg_ephem_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_ephem_options_.callback_group = cbg_ephem_;
  pub_ephem_ = node_->create_publisher<gnss_interfaces::msg::GnssEphemMsg>("~/ephem", qos, pub_ephem_options_);

  cbg_glo_ephem_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_glo_ephem_options_.callback_group = cbg_glo_ephem_;
  pub_glo_ephem_ = node_->create_publisher<gnss_interfaces::msg::GnssGloEphemMsg>("~/glo_ephem", qos, pub_glo_ephem_options_);

  cbg_iono_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_iono_options_.callback_group = cbg_iono_;
  pub_iono_ = node_->create_publisher<gnss_interfaces::msg::StampedFloat64Array>("~/iono_params", qos, pub_iono_options_);
}

void UbloxRosHandler::publishObservations(const std::vector<gnss_comm::ObsPtr> &meas) {
  if (meas.empty()) {
    return;
  }

  std::vector<gnss_comm::ObsPtr> filtered_meas;
  filtered_meas.reserve(meas.size());
  for (const auto &obs : meas) {
    if (!obs) {
      continue;
    }

    const uint32_t sys = gnss_comm::satsys(obs->sat, nullptr);
    if ((sys & raw_observation_system_mask_) == 0U) {
      continue;
    }
    filtered_meas.push_back(obs);
  }

  if (filtered_meas.empty()) {
    return;
  }
  pub_range_meas_->publish(gnss_comm::meas2msg(filtered_meas));
}

void UbloxRosHandler::publishEphemeris(const gnss_comm::EphemBasePtr &ephem) {
  if (!ephem || ephem->ttr.time == 0) {
    return;
  }

  const uint32_t sys = gnss_comm::satsys(ephem->sat, nullptr);
  if ((sys & ephemeris_system_mask_) == 0U) {
    return;
  }

  if (sys == SYS_GLO) {
    pub_glo_ephem_->publish(gnss_comm::glo_ephem2msg(std::dynamic_pointer_cast<gnss_comm::GloEphem>(ephem)));
    return;
  }

  pub_ephem_->publish(gnss_comm::ephem2msg(std::dynamic_pointer_cast<gnss_comm::Ephem>(ephem)));
}

void UbloxRosHandler::publishIonoParams(const std::vector<double> &iono_params, const gnss_comm::gtime_t &stamp) {
  if (iono_params.size() != 8) {
    return;
  }

  gnss_interfaces::msg::StampedFloat64Array iono_msg;
  if (stamp.time != 0) {
    iono_msg.header.stamp = cvtTimestamp(stamp);
  }
  std::copy(iono_params.begin(), iono_params.end(), std::back_inserter(iono_msg.data));
  pub_iono_->publish(iono_msg);
}

void UbloxRosHandler::publishTimePulse(const gnss_comm::TimePulseInfoPtr &time_pulse) {
  if (!time_pulse || time_pulse->time.time == 0) {
    return;
  }
  pub_tp_info_->publish(gnss_comm::tp_info2msg(time_pulse));
}

void UbloxRosHandler::publishPvt(const gnss_comm::PVTSolutionPtr &pvt_soln) {
  if (!pvt_soln || pvt_soln->time.time == 0) {
    return;
  }

  for (const auto &callback : pvt_callbacks_) {
    callback(pvt_soln);
  }

  pub_pvt_->publish(gnss_comm::pvt2msg(pvt_soln));

  sensor_msgs::msg::NavSatFix lla_msg;
  lla_msg.header.stamp = cvtTimestamp(pvt_soln->time);
  lla_msg.latitude = pvt_soln->lat;
  lla_msg.longitude = pvt_soln->lon;
  lla_msg.altitude = pvt_soln->hgt;
  lla_msg.status.status = static_cast<int8_t>(pvt_soln->fix_type);
  lla_msg.status.service = static_cast<uint16_t>(pvt_soln->carr_soln);
  pub_lla_->publish(lla_msg);
}

void UbloxRosHandler::registerPvtCallback(std::function<void(const gnss_comm::PVTSolutionPtr &)> callback) { pvt_callbacks_.push_back(std::move(callback)); }

} // namespace ublox_driver
