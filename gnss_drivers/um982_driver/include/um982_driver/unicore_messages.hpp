#pragma once

#include <rclcpp/rclcpp.hpp>
#include <gnss_msg/msg/gnss_meas_msg.hpp>
#include <gnss_msg/msg/gnss_ephem_msg.hpp>
#include <gnss_msg/msg/gnss_glo_ephem_msg.hpp>
#include <gnss_msg/msg/stamped_float64_array.hpp>
#include <map>
#include <string>
#include <vector>

// Unicore N4 R1.14 ASCII logs. All published navigation times are GPST.
class UnicoreMessages
{
public:
    UnicoreMessages(rclcpp::Node& node, const std::string& frame);
    void parse(const std::string& line);

private:
    using Fields = std::vector<std::string>;
    void observations(const Fields& fields, const gnss_msg::msg::GnssTimeMsg& time);
    void ephemeris(const std::string& name, const Fields& fields,
                   const gnss_msg::msg::GnssTimeMsg& time);
    void ionosphere(const std::string& name, const Fields& fields, const Fields& header);
    rclcpp::Node& node_;
    std::string frame_;
    rclcpp::Publisher<gnss_msg::msg::GnssMeasMsg>::SharedPtr obs_;
    rclcpp::Publisher<gnss_msg::msg::GnssEphemMsg>::SharedPtr ephem_;
    rclcpp::Publisher<gnss_msg::msg::GnssGloEphemMsg>::SharedPtr glo_ephem_;
    std::map<std::string, rclcpp::Publisher<gnss_msg::msg::StampedFloat64Array>::SharedPtr> iono_;
    struct Lock { double time; double duration; bool valid; };
    std::map<std::pair<unsigned, unsigned>, Lock> locks_;
};
