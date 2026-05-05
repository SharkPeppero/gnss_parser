#include <iostream>
#include <unistd.h>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_ros.hpp>

constexpr int UTC_OFFSET = 8;

using namespace gnss_comm;

static std::atomic<bool> done(false);

void set_datetime(const gtime_t t, const int timezone) {
  gtime_t local_t = time_add(t, timezone * 3600);
  gtime_t gtime_utc = gpst2utc(local_t);
  std::vector<double> utc_t;
  utc_t.resize(6);
  utc_t.clear();
  time2epoch(gtime_utc, &(utc_t[0]));
  std::cout << "set system time to: ";
  for (size_t i = 0; i < utc_t.size(); ++i)
    std::cout << utc_t[i] << " ";
  std::cout << std::endl;
  char buf[100];
  sprintf(buf, "%4d-%02d-%02d,%02d:%02d:%02d", int(utc_t[0]), int(utc_t[1]),
          int(utc_t[2]), int(utc_t[3]), int(utc_t[4]), int(utc_t[5]));
  struct tm s_tm;
  strptime(buf, "%Y-%m-%d,%H:%M:%S", &s_tm);
  struct timespec s_timespec;
  s_timespec.tv_sec = mktime(&s_tm);
  s_timespec.tv_nsec = static_cast<long>(t.sec * 1e9 + 0.5);
  // LOG(INFO) << "tv_sec is " << s_timespec.tv_sec << ", and nsec is " << s_timespec.tv_nsec;
  if (clock_settime(CLOCK_REALTIME, &s_timespec) == -1)
    std::cerr << "clock setting error: " << strerror(errno);
}

void NavSatFixCallBack(const sensor_msgs::msg::NavSatFix::ConstSharedPtr lla_msg) {
  const gtime_t time = sec2time(lla_msg->header.stamp.sec + static_cast<double>(lla_msg->header.stamp.nanosec) * 1e-9);
  if (time.time != 0) {
    set_datetime(time, UTC_OFFSET);
    done = true;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("set_system_time");
  auto sub_lla =
      node->create_subscription<sensor_msgs::msg::NavSatFix>("/ublox_driver/receiver_lla", 10,
                                                             std::bind(&NavSatFixCallBack, std::placeholders::_1));

  rclcpp::WallRate rate(1.0);
  while (rclcpp::ok() && !done) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  std::cout << "System time updated" << std::endl;
  rclcpp::shutdown();
  return 0;
}
