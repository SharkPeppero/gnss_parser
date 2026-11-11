#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

#include "csv_io_helper/csv_io_helper.h"
#include "gnss_comm/ros/conversion.hpp"
#include "gnss_comm/solver/spp.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <glog/logging.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace gnss_comm {

// 单线程订阅节点：星历/电离层回调更新缓存，观测回调触发单历元 SPP。
// receiver_lla 仅提供局部 ENU 原点，不作为伪距观测，也不修改 SPP 迭代初值。
class SppSolverNode : public rclcpp::Node {
public:
  SppSolverNode() : Node("spp_solver_node") {
    SppSolverPipelineOptions options;
    options.elevation_cutoff_degree = declare_parameter("elevation_cutoff_degree", 15.0);
    const int iterations = declare_parameter("max_iterations", static_cast<int>(MAX_ITER_PVT));
    options.convergence_threshold = declare_parameter("convergence_threshold", EPSILON_PVT);
    if (!std::isfinite(options.elevation_cutoff_degree) || options.elevation_cutoff_degree < 0 || options.elevation_cutoff_degree >= 90 || iterations <= 0 ||
        !std::isfinite(options.convergence_threshold) || options.convergence_threshold <= 0 || options.convergence_threshold >= 1.0)
      throw std::invalid_argument("Invalid SPP options (convergence_threshold must be in (0,1))");
    options.max_iterations = static_cast<uint32_t>(iterations);
    elevation_cutoff_degree_ = options.elevation_cutoff_degree;
    skyplot_size_ = declare_parameter("skyplot_size", 600);
    if (skyplot_size_ < 200)
      throw std::invalid_argument("skyplot_size must be >= 200");
    solver_ = std::make_unique<SppSolverPipeline>(options);

    // best_effort 订阅兼容可靠及 best_effort 发布者；适用于驱动和 bag 回放。
    const auto qos = rclcpp::SensorDataQoS().keep_last(1000);
    ephem_sub_ = create_subscription<gnss_msg::msg::GnssEphemMsg>(declare_parameter<std::string>("subscribe.ephem", "/ublox_driver/ephem"), qos, [this](gnss_msg::msg::GnssEphemMsg::ConstSharedPtr msg) { cache_ephemeris(msg2ephem(msg)); });
    glo_sub_ = create_subscription<gnss_msg::msg::GnssGloEphemMsg>(declare_parameter<std::string>("subscribe.glo_ephem", "/ublox_driver/glo_ephem"), qos, [this](gnss_msg::msg::GnssGloEphemMsg::ConstSharedPtr msg) { cache_ephemeris(msg2glo_ephem(msg)); });
    iono_sub_ = create_subscription<gnss_msg::msg::StampedFloat64Array>(declare_parameter<std::string>("subscribe.iono_params", "/ublox_driver/iono_params"), qos, [this](gnss_msg::msg::StampedFloat64Array::ConstSharedPtr msg) {
      if (msg->data.size() == 8 && std::all_of(msg->data.begin(), msg->data.end(), [](double x) { return std::isfinite(x); })) {
        iono_params_ = msg->data;
        has_iono_ = true;
      }
    });
    lla_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(declare_parameter<std::string>("subscribe.receiver_lla", "/ublox_driver/receiver_lla"), qos, [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
      if (has_reference_ || msg->status.status < 0 || !std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) || !std::isfinite(msg->altitude) || std::abs(msg->latitude) >= 90 ||
          std::abs(msg->longitude) > 180)
        return;
      reference_lla_ = Eigen::Vector3d(msg->latitude, msg->longitude, msg->altitude);
      reference_ecef_ = geo2ecef(reference_lla_);
      has_reference_ = true;
      reference_pub_->publish(*msg);
      RCLCPP_INFO(get_logger(), "ENU origin: lat=%.9f lon=%.9f h=%.3f m", msg->latitude, msg->longitude, msg->altitude);
    });
    // 接收机 PVT 与 PPS 作为辅助数据缓存；不混入 SPP 的测量方程。
    pvt_sub_ = create_subscription<gnss_msg::msg::GnssPVTSolnMsg>(declare_parameter<std::string>("subscribe.receiver_pvt", "/ublox_driver/receiver_pvt"), qos, [this](gnss_msg::msg::GnssPVTSolnMsg::ConstSharedPtr msg) { receiver_pvt_ = msg; });
    pulse_sub_ = create_subscription<gnss_msg::msg::GnssTimePulseInfoMsg>(declare_parameter<std::string>("subscribe.time_pulse_info", "/ublox_driver/time_pulse_info"), qos, [this](gnss_msg::msg::GnssTimePulseInfoMsg::ConstSharedPtr msg) { time_pulse_ = msg; });
    meas_sub_ = create_subscription<gnss_msg::msg::GnssMeasMsg>(declare_parameter<std::string>("subscribe.range_meas", "/ublox_driver/range_meas"), qos, [this](gnss_msg::msg::GnssMeasMsg::ConstSharedPtr msg) { solve_epoch(msg); });

    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(declare_parameter<std::string>("publish.fix", "spp/fix"), 10);
    ecef_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(declare_parameter<std::string>("publish.ecef", "spp/ecef"), 10);
    enu_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(declare_parameter<std::string>("publish.enu", "spp/enu"), 10);
    clock_pub_ = create_publisher<gnss_msg::msg::StampedFloat64Array>(declare_parameter<std::string>("publish.receiver_clock_bias", "spp/receiver_clock_bias_m"), 10);
    skyplot_pub_ = create_publisher<sensor_msgs::msg::Image>(declare_parameter<std::string>("publish.skyplot", "spp/skyplot"), 10);
    reference_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(declare_parameter<std::string>("publish.reference_lla", "spp/reference_lla"), rclcpp::QoS(1).transient_local());

    // output_path 为空则不写 CSV；每次启动按时间戳新建文件，CORE 保留策略避免长 bag 的早期分片被轮转删除。
    const auto output_path = declare_parameter<std::string>("output_path", "");
    if (!output_path.empty()) {
      std::filesystem::create_directories(output_path);
      const std::vector<std::string> header = {"timestamp", "x", "y", "z", "lat", "lon", "alt", "ecef_x", "ecef_y", "ecef_z",
                                               "num_sats", "num_gps", "num_glo", "num_gal", "num_bds", "num_qzs",
                                               "clk_gps_m", "clk_glo_m", "clk_gal_m", "clk_bds_m"};
      csv_enabled_ = common::CsvIOHelper::GetInstance().RegisterFileWithHeader(output_path, "spp_enu", header, true, common::CsvIOHelper::RetentionClass::CORE);
      if (!csv_enabled_)
        RCLCPP_ERROR(get_logger(), "CSV registration failed: %s", output_path.c_str());
    }
    RCLCPP_INFO(get_logger(), "Waiting for receiver_lla, ionosphere parameters and ephemerides.");
  }

  ~SppSolverNode() override {
    if (csv_enabled_)
      common::CsvIOHelper::GetInstance().Flush();
  }

private:
  void cache_ephemeris(const EphemBasePtr &ephem) {
    auto &history = ephemerides_[ephem->sat];
    // 同一期号、同一 toe 的重复广播覆盖即可；保留多个版本供观测时刻匹配。
    for (auto &entry : history) {
      if (entry->iode == ephem->iode && time_diff(entry->toe, ephem->toe) == 0) {
        entry = ephem;
        return;
      }
    }
    history.push_back(ephem);
    if (history.size() > 16)
      history.pop_front();
  }

  EphemBasePtr select_ephemeris(const ObsPtr &obs) const {
    const auto it = ephemerides_.find(obs->sat);
    if (it == ephemerides_.end())
      return nullptr;
    double best_age = satsys(obs->sat, nullptr) == SYS_GLO ? GLO_EPH_VALID_SECONDS : EPH_VALID_SECONDS;
    EphemBasePtr best;
    for (const auto &entry : it->second) {
      const double age = std::abs(time_diff(obs->time, entry->toe));
      if (age <= best_age) {
        best_age = age;
        best = entry;
      }
    }
    // 最近版本不健康时直接剔除，不回退到更旧的健康版本。
    return best && best->health == 0 ? best : nullptr;
  }

  void solve_epoch(const gnss_msg::msg::GnssMeasMsg::ConstSharedPtr &msg) {
    if (!has_reference_ || !has_iono_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Skipping epoch: waiting for reference LLA / ionosphere parameters.");
      return;
    }
    const auto measurements = msg2meas(msg);
    if (measurements.empty())
      return;
    std::vector<ObsPtr> obs;
    std::vector<EphemBasePtr> ephems;
    std::set<uint32_t> satellites;
    std::set<int> clock_indices;
    uint16_t services = 0;
    for (const auto &entry : measurements) {
      const uint32_t sys = satsys(entry->sat, nullptr);
      if (sys != SYS_GPS && sys != SYS_GLO && sys != SYS_GAL && sys != SYS_BDS && sys != SYS_QZS)
        continue;
      int index = -1;
      if (L1_freq(entry, &index) <= 0 || index < 0)
        continue;
      const auto i = static_cast<size_t>(index);
      // 当前 solver 会访问选中频点的伪距及标准差；先检查长度、有效位和数值。
      if (i >= entry->psr.size() || i >= entry->psr_std.size() || i >= entry->status.size() || !(entry->status[i] & 1) || !std::isfinite(entry->psr[i]) || entry->psr[i] <= 0 ||
          !std::isfinite(entry->psr_std[i]) || entry->psr_std[i] < 0 || std::abs(time_diff(entry->time, measurements.front()->time)) > 0.001)
        continue;
      const auto ephem = select_ephemeris(entry);
      if (!ephem)
        continue;
      // 沿用现有 solver 的 URA 权重表达式，拒绝会产生非正/无穷权重的输入。
      const double denominator = sys == SYS_GLO ? 4.0 : ephem->ura - (sys == SYS_GAL ? 2.0 : 1.0);
      if (!std::isfinite(denominator) || denominator <= 0 || !satellites.insert(entry->sat).second)
        continue;
      obs.push_back(entry);
      ephems.push_back(ephem);
      clock_indices.insert(sys2idx.at(sys));
      if (sys == SYS_GPS || sys == SYS_QZS)
        services |= sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      if (sys == SYS_GLO)
        services |= sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
      if (sys == SYS_GAL)
        services |= sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
      if (sys == SYS_BDS)
        services |= sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
    }
    if (obs.size() < 3 + clock_indices.size() || obs.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Skipping epoch: insufficient usable observations / ephemerides (%zu).", obs.size());
      return;
    }
    const auto state = solver_->psr_pos(obs, ephems, iono_params_);
    if (!state.allFinite() || state.head<3>().norm() < 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "SPP failed; no solution published.");
      return;
    }
    const Eigen::Vector3d ecef = state.head<3>();
    const Eigen::Vector3d lla = ecef2geo(ecef);
    const Eigen::Vector3d enu = ecef2enu(reference_lla_, ecef - reference_ecef_);
    if (!lla.allFinite() || !enu.allFinite())
      return;

    // 观测时间是 GPST，ROS stamp 使用 UTC Unix 时间；显式转纳秒，避免秒/纳秒混用。
    const auto utc = gpst2utc(obs.front()->time);
    const builtin_interfaces::msg::Time stamp = rclcpp::Time(static_cast<int64_t>(utc.time) * 1000000000LL + static_cast<int64_t>(std::llround(utc.sec * 1e9)));
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = stamp;
    fix.header.frame_id = "gnss";
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = services;
    fix.latitude = lla.x();
    fix.longitude = lla.y();
    fix.altitude = lla.z();
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    fix_pub_->publish(fix);

    geometry_msgs::msg::PointStamped point;
    point.header.stamp = stamp;
    point.header.frame_id = "ecef";
    point.point.x = ecef.x();
    point.point.y = ecef.y();
    point.point.z = ecef.z();
    ecef_pub_->publish(point);
    point.header.frame_id = "enu";
    point.point.x = enu.x();
    point.point.y = enu.y();
    point.point.z = enu.z();
    enu_pub_->publish(point);

    gnss_msg::msg::StampedFloat64Array clocks;
    clocks.header.stamp = stamp;
    // 固定顺序 GPS/GLO/GAL/BDS，QZSS 共用 GPS；未参与的星座填 NaN，避免误读为零钟差。
    clocks.data.resize(4, std::numeric_limits<double>::quiet_NaN());
    for (const auto index : clock_indices)
      clocks.data[index] = state[3 + index];
    clock_pub_->publish(clocks);

    if (csv_enabled_) {
      std::map<uint32_t, int> counts;
      for (const auto &entry : obs)
        ++counts[satsys(entry->sat, nullptr)];
      // CsvIOHelper 对 double 使用 std::to_string（6 位小数），经纬度需要更高精度，单独格式化。
      char lat[32], lon[32];
      std::snprintf(lat, sizeof(lat), "%.9f", lla.x());
      std::snprintf(lon, sizeof(lon), "%.9f", lla.y());
      const double timestamp = static_cast<double>(utc.time) + utc.sec;
      common::CsvIOHelper::GetInstance().write_line("spp_enu", timestamp, enu.x(), enu.y(), enu.z(), std::string(lat), std::string(lon), lla.z(), ecef.x(), ecef.y(), ecef.z(),
                                                    obs.size(), counts[SYS_GPS], counts[SYS_GLO], counts[SYS_GAL], counts[SYS_BDS], counts[SYS_QZS],
                                                    clocks.data[0], clocks.data[1], clocks.data[2], clocks.data[3]);
    }

    if (skyplot_pub_->get_subscription_count() > 0)
      publish_skyplot(obs, ephems, ecef, stamp);
  }

  // 虚拟鱼眼天空图：等距投影，天顶为圆心、地平线为外圆，北向上、东向右（常规 skyplot 约定）。
  // 仅绘制本历元有可用伪距和健康星历的卫星；实心=高于截止角，空心=低于截止角（被 solver 剔除）。
  void publish_skyplot(const std::vector<ObsPtr> &obs, const std::vector<EphemBasePtr> &ephems, const Eigen::Vector3d &ecef, const builtin_interfaces::msg::Time &stamp) const {
    struct Style {
      uint32_t sys;
      const char *name;
      cv::Scalar color; // BGR
    };
    static const Style styles[] = {{SYS_GPS, "GPS", {80, 200, 80}}, {SYS_GLO, "GLONASS", {80, 80, 240}}, {SYS_GAL, "Galileo", {240, 160, 40}}, {SYS_BDS, "BeiDou", {0, 170, 255}}, {SYS_QZS, "QZSS", {200, 80, 200}}};
    const int size = skyplot_size_;
    const cv::Point center(size / 2, size / 2);
    const double radius = 0.42 * size;
    const auto project = [&](double az, double el_deg) {
      const double r = radius * (90.0 - el_deg) / 90.0;
      return cv::Point(center.x + static_cast<int>(std::lround(r * std::sin(az))), center.y - static_cast<int>(std::lround(r * std::cos(az))));
    };
    const cv::Scalar grid(90, 90, 90), text(220, 220, 220);
    const double font = 0.45 * size / 600.0;
    cv::Mat img(size, size, CV_8UC3, cv::Scalar(25, 25, 25));

    for (int el = 0; el < 90; el += 30) {
      const int r = static_cast<int>(radius * (90 - el) / 90.0);
      cv::circle(img, center, r, grid, 1, cv::LINE_AA);
      cv::putText(img, std::to_string(el), center + cv::Point(3, -r + 14), cv::FONT_HERSHEY_SIMPLEX, font, grid, 1, cv::LINE_AA);
    }
    cv::circle(img, center, static_cast<int>(radius * (90.0 - elevation_cutoff_degree_) / 90.0), cv::Scalar(60, 60, 150), 1, cv::LINE_AA);
    for (int az = 0; az < 360; az += 30)
      cv::line(img, center, project(az * D2R, 0), grid, 1, cv::LINE_AA);
    const char *directions[] = {"N", "E", "S", "W"};
    for (int k = 0; k < 4; ++k)
      cv::putText(img, directions[k], project(k * M_PI / 2, -6) + cv::Point(-6, 6), cv::FONT_HERSHEY_SIMPLEX, 1.4 * font, text, 2, cv::LINE_AA);

    std::map<uint32_t, int> counts;
    for (size_t i = 0; i < obs.size(); ++i) {
      const uint32_t sys = satsys(obs[i]->sat, nullptr);
      const auto style = std::find_if(std::begin(styles), std::end(styles), [sys](const Style &s) { return s.sys == sys; });
      if (style == std::end(styles))
        continue;
      // 显示用途：直接取接收时刻的卫星位置，忽略 ~70 ms 信号传播时间带来的微小角度差。
      double clock_bias_s = 0;
      const Eigen::Vector3d sat_pos = sys == SYS_GLO ? geph2pos(obs[i]->time, std::dynamic_pointer_cast<GloEphem>(ephems[i]), &clock_bias_s)
                                                     : eph2pos(obs[i]->time, std::dynamic_pointer_cast<Ephem>(ephems[i]), &clock_bias_s);
      double azel[2] = {0, 0};
      sat_azel(ecef, sat_pos, azel);
      if (!std::isfinite(azel[0]) || !std::isfinite(azel[1]) || azel[1] < 0)
        continue;
      const double el_deg = azel[1] * R2D;
      const cv::Point p = project(azel[0], el_deg);
      const int dot = std::max(5, size / 70);
      cv::circle(img, p, dot, style->color, el_deg >= elevation_cutoff_degree_ ? cv::FILLED : 2, cv::LINE_AA);
      cv::putText(img, sat2str(obs[i]->sat), p + cv::Point(dot + 2, dot / 2), cv::FONT_HERSHEY_SIMPLEX, font, text, 1, cv::LINE_AA);
      ++counts[sys];
    }

    // 左上角图例：星座颜色与本历元卫星数。
    int row = 0;
    for (const auto &style : styles) {
      const cv::Point p(14, 18 + 20 * row++);
      cv::circle(img, p, 6, style.color, cv::FILLED, cv::LINE_AA);
      cv::putText(img, std::string(style.name) + " " + std::to_string(counts[style.sys]), p + cv::Point(12, 5), cv::FONT_HERSHEY_SIMPLEX, font, text, 1, cv::LINE_AA);
    }
    cv::putText(img, "hollow: el < " + std::to_string(static_cast<int>(elevation_cutoff_degree_)) + " deg", cv::Point(8, size - 10), cv::FONT_HERSHEY_SIMPLEX, font, text, 1, cv::LINE_AA);

    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = "skyplot";
    image.height = img.rows;
    image.width = img.cols;
    image.encoding = "bgr8";
    image.step = static_cast<uint32_t>(img.step);
    image.data.assign(img.datastart, img.dataend);
    skyplot_pub_->publish(image);
  }

  std::unique_ptr<SppSolverPipeline> solver_;
  std::map<uint32_t, std::deque<EphemBasePtr>> ephemerides_;
  std::vector<double> iono_params_;
  bool has_iono_ = false, has_reference_ = false, csv_enabled_ = false;
  double elevation_cutoff_degree_ = 15.0;
  int skyplot_size_ = 600;
  Eigen::Vector3d reference_lla_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d reference_ecef_ = Eigen::Vector3d::Zero();
  gnss_msg::msg::GnssPVTSolnMsg::ConstSharedPtr receiver_pvt_;
  gnss_msg::msg::GnssTimePulseInfoMsg::ConstSharedPtr time_pulse_;
  rclcpp::Subscription<gnss_msg::msg::GnssEphemMsg>::SharedPtr ephem_sub_;
  rclcpp::Subscription<gnss_msg::msg::GnssGloEphemMsg>::SharedPtr glo_sub_;
  rclcpp::Subscription<gnss_msg::msg::StampedFloat64Array>::SharedPtr iono_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr lla_sub_;
  rclcpp::Subscription<gnss_msg::msg::GnssPVTSolnMsg>::SharedPtr pvt_sub_;
  rclcpp::Subscription<gnss_msg::msg::GnssTimePulseInfoMsg>::SharedPtr pulse_sub_;
  rclcpp::Subscription<gnss_msg::msg::GnssMeasMsg>::SharedPtr meas_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_, reference_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ecef_pub_, enu_pub_;
  rclcpp::Publisher<gnss_msg::msg::StampedFloat64Array>::SharedPtr clock_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr skyplot_pub_;
};
} // namespace gnss_comm

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_comm::SppSolverNode>());
  rclcpp::shutdown();
  google::ShutdownGoogleLogging();
  return 0;
}
