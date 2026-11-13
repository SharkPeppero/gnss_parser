//
// Created by yjh on 2026/11/12.
//
// GNSS 解算节点：StreamWrapper 提供流动站原始数据（在线话题 / 离线 RINEX）与离线差分数据（RINEX / RTCM），
// AlgorithmManager 调度 SPP 与 RTK，本节点负责参数、发布、CSV 与天空图。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

#include "csv_io_helper/csv_io_helper.h"
#include "gnss_comm/algorithm/algorithm_manager.hpp"
#include "gnss_comm/gnss_utility.hpp"
#include "gnss_comm/stream/stream_wrapper.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <glog/logging.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace gnss_comm {

// ROS 参数 YAML 不展开 ~，这里手动替换为 $HOME。
static std::string expand_home(std::string path) {
  if (path.rfind("~/", 0) == 0 && std::getenv("HOME"))
    path = std::string(std::getenv("HOME")) + path.substr(1);
  return path;
}

static std::vector<std::string> expand_home(std::vector<std::string> paths) {
  for (auto &path : paths)
    path = expand_home(path);
  return paths;
}

// 启动时检查输入文件，路径写错时直接报出参数名和路径，而不是在读取线程里崩溃。
static void require_files(const std::string &param, const std::vector<std::string> &paths) {
  for (const auto &path : paths)
    if (!std::filesystem::is_regular_file(path))
      throw std::invalid_argument(param + ": file not found: " + path);
}

// 观测时间是 GPST，ROS stamp 使用 UTC Unix 时间；显式转纳秒，避免秒/纳秒混用。
static builtin_interfaces::msg::Time gpst2stamp(const gtime_t &time) {
  const auto utc = gpst2utc(time);
  return rclcpp::Time(static_cast<int64_t>(utc.time) * 1000000000LL + static_cast<int64_t>(std::llround(utc.sec * 1e9)));
}

class GnssSolverNode : public rclcpp::Node {
public:
  GnssSolverNode() : Node("gnss_solver_node") {
    const auto mode = declare_parameter<std::string>("mode", "online");
    if (mode != "online" && mode != "offline")
      throw std::invalid_argument("mode must be online or offline, got " + mode);
    offline_ = mode == "offline";

    AlgorithmManagerOptions options;
    options.enable_spp = declare_parameter("spp.enable", true);
    options.enable_rtk = declare_parameter("rtk.enable", false);
    if (!options.enable_spp && !options.enable_rtk)
      throw std::invalid_argument("spp.enable and rtk.enable are both false");

    // SPP
    options.spp.elevation_cutoff_degree = declare_parameter("spp.elevation_cutoff_degree", 15.0);
    const int spp_iterations = declare_parameter("spp.max_iterations", static_cast<int>(MAX_ITER_PVT));
    options.spp.convergence_threshold = declare_parameter("spp.convergence_threshold", EPSILON_PVT);
    if (!std::isfinite(options.spp.elevation_cutoff_degree) || options.spp.elevation_cutoff_degree < 0 || options.spp.elevation_cutoff_degree >= 90 || spp_iterations <= 0 ||
        !std::isfinite(options.spp.convergence_threshold) || options.spp.convergence_threshold <= 0 || options.spp.convergence_threshold >= 1.0)
      throw std::invalid_argument("Invalid SPP options (convergence_threshold must be in (0,1))");
    options.spp.max_iterations = static_cast<uint32_t>(spp_iterations);
    elevation_cutoff_degree_ = options.spp.elevation_cutoff_degree;
    skyplot_size_ = declare_parameter("skyplot_size", 600);
    if (skyplot_size_ < 200)
      throw std::invalid_argument("skyplot_size must be >= 200");

    // RTK
    auto &rtk = options.rtk;
    rtk.elevation_cutoff_degree = declare_parameter("rtk.elevation_cutoff_degree", 15.0);
    rtk.num_bands = static_cast<uint32_t>(declare_parameter("rtk.num_bands", 2));
    rtk.use_glonass = declare_parameter("rtk.use_glonass", false);
    rtk.code_std_m = declare_parameter("rtk.code_std_m", 0.3);
    rtk.phase_std_m = declare_parameter("rtk.phase_std_m", 0.003);
    rtk.max_base_age_s = declare_parameter("rtk.max_base_age_s", 30.0);
    rtk.baseline_std_ppm = declare_parameter("rtk.baseline_std_ppm", 1.0);
    rtk.accel_std_mps2 = declare_parameter("rtk.accel_std_mps2", 3.0);
    rtk.amb_init_std_cycles = declare_parameter("rtk.amb_init_std_cycles", 10.0);
    rtk.amb_process_std_cycles = declare_parameter("rtk.amb_process_std_cycles", 1e-4);
    rtk.innov_gate_sigma = declare_parameter("rtk.innov_gate_sigma", 5.0);
    rtk.max_gap_s = declare_parameter("rtk.max_gap_s", 10.0);
    rtk.max_amb_outage_s = declare_parameter("rtk.max_amb_outage_s", 10.0);
    rtk.gf_slip_threshold_m = declare_parameter("rtk.gf_slip_threshold_m", 0.05);
    rtk.min_ratio = declare_parameter("rtk.min_ratio", 3.0);
    const int min_fix_dd = declare_parameter("rtk.min_fix_dd", 5);
    rtk.par_enabled = declare_parameter("rtk.par_enabled", true);
    rtk.par_min_fix_fraction = declare_parameter("rtk.par_min_fix_fraction", 0.9);
    rtk.fix_and_hold = declare_parameter("rtk.fix_and_hold", true);
    rtk.hold_std_cycles = declare_parameter("rtk.hold_std_cycles", 1e-3);
    const int hold_min_fix_epochs = declare_parameter("rtk.hold_min_fix_epochs", 10);
    rtk.fix_validate_sigma = declare_parameter("rtk.fix_validate_sigma", 4.0);
    rtk.reset_pos_diff_m = declare_parameter("rtk.reset_pos_diff_m", 30.0);
    const int reset_diverge_epochs = declare_parameter("rtk.reset_diverge_epochs", 5);
    rtk.psr_outlier_sigma = declare_parameter("rtk.psr_outlier_sigma", 4.0);
    rtk.reset_spp_diff_m = declare_parameter("rtk.reset_spp_diff_m", 300.0);
    const int reset_min_phase_dd = declare_parameter("rtk.reset_min_phase_dd", 5);
    rtk.reset_low_phase_s = declare_parameter("rtk.reset_low_phase_s", 10.0);
    const int stall_min_phase_dd = declare_parameter("rtk.stall_min_phase_dd", 8);
    rtk.stall_reset_s = declare_parameter("rtk.stall_reset_s", 30.0);
    rtk.spp = options.spp;
    rtk.spp.elevation_cutoff_degree = rtk.elevation_cutoff_degree;
    if (rtk.elevation_cutoff_degree < 0 || rtk.elevation_cutoff_degree >= 90 || rtk.num_bands < 1 || rtk.num_bands > 2 || rtk.code_std_m <= 0 || rtk.phase_std_m <= 0 ||
        rtk.max_base_age_s <= 0 || rtk.baseline_std_ppm < 0 || rtk.accel_std_mps2 <= 0 || rtk.amb_init_std_cycles <= 0 || rtk.amb_process_std_cycles < 0 || rtk.innov_gate_sigma <= 0 ||
        rtk.max_gap_s <= 0 || rtk.max_amb_outage_s <= 0 || rtk.gf_slip_threshold_m <= 0 || rtk.min_ratio < 1.0 || min_fix_dd < 1 || rtk.hold_std_cycles <= 0 ||
        hold_min_fix_epochs < 1 || rtk.fix_validate_sigma <= 0 || rtk.reset_pos_diff_m <= 0 || reset_min_phase_dd < 0 || rtk.reset_low_phase_s <= 0 ||
        rtk.par_min_fix_fraction < 0 || rtk.par_min_fix_fraction > 1 || reset_diverge_epochs < 1 || rtk.psr_outlier_sigma <= 0 ||
        rtk.reset_spp_diff_m <= 0 || stall_min_phase_dd < 0 || rtk.stall_reset_s <= 0)
      throw std::invalid_argument("Invalid RTK options");
    rtk.min_fix_dd = static_cast<uint32_t>(min_fix_dd);
    rtk.hold_min_fix_epochs = static_cast<uint32_t>(hold_min_fix_epochs);
    rtk.reset_min_phase_dd = static_cast<uint32_t>(reset_min_phase_dd);
    rtk.reset_diverge_epochs = static_cast<uint32_t>(reset_diverge_epochs);
    rtk.stall_min_phase_dd = static_cast<uint32_t>(stall_min_phase_dd);
    options.antex_path = expand_home(declare_parameter<std::string>("correction.antex_path", ""));
    // 离线一次性载入整天星历，不能只保留最近 16 个版本
    options.max_ephem_history = offline_ ? 0 : 16;

    // 原始数据流
    std::unique_ptr<RawStream> raw;
    if (offline_) {
      const auto obs_paths = expand_home(declare_parameter<std::vector<std::string>>("offline.rover_obs_paths", std::vector<std::string>{}));
      const auto nav_paths = expand_home(declare_parameter<std::vector<std::string>>("offline.rover_nav_paths", std::vector<std::string>{}));
      const double playback_rate = declare_parameter("offline.playback_rate", 0.0);
      if (obs_paths.empty() || nav_paths.empty() || playback_rate < 0)
        throw std::invalid_argument("offline mode needs offline.rover_obs_paths, offline.rover_nav_paths and playback_rate >= 0");
      require_files("offline.rover_obs_paths", obs_paths);
      require_files("offline.rover_nav_paths", nav_paths);
      raw = std::make_unique<RinexRawStream>(obs_paths, nav_paths, playback_rate);
    } else {
      RosRawTopics topics;
      topics.range_meas = declare_parameter<std::string>("subscribe.range_meas", "/ublox_driver/range_meas");
      topics.ephem = declare_parameter<std::string>("subscribe.ephem", "/ublox_driver/ephem");
      topics.glo_ephem = declare_parameter<std::string>("subscribe.glo_ephem", "/ublox_driver/glo_ephem");
      topics.iono_params = declare_parameter<std::string>("subscribe.iono_params", "/ublox_driver/iono_params");
      topics.receiver_lla = declare_parameter<std::string>("subscribe.receiver_lla", "/ublox_driver/receiver_lla");
      raw = std::make_unique<RosRawStream>(*this, topics);
    }

    // 差分数据流（仅 RTK）
    std::unique_ptr<CorrectionStream> correction;
    if (options.enable_rtk) {
      const auto paths = expand_home(declare_parameter<std::vector<std::string>>("correction.paths", std::vector<std::string>{}));
      if (paths.empty())
        throw std::invalid_argument("rtk.enable is true but correction.paths is empty");
      require_files("correction.paths", paths);
      if (!options.antex_path.empty())
        require_files("correction.antex_path", {options.antex_path});
      correction = std::make_unique<CorrectionStream>(paths, declare_parameter<std::string>("correction.format", "auto"),
                                                      declare_parameter<std::vector<double>>("correction.base_ecef", std::vector<double>{}));
    }
    streams_ = std::make_unique<StreamWrapper>(std::move(raw), std::move(correction));
    manager_ = std::make_unique<AlgorithmManager>(options, streams_->correction());
    std::ostringstream options_text;
    options_text << options;
    RCLCPP_INFO(get_logger(), "%s", options_text.str().c_str());

    reference_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(declare_parameter<std::string>("publish.reference_lla", "gnss/reference_lla"), rclcpp::QoS(1).transient_local());
    if (options.enable_spp) {
      spp_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(declare_parameter<std::string>("publish.spp.fix", "spp/fix"), 10);
      spp_ecef_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(declare_parameter<std::string>("publish.spp.ecef", "spp/ecef"), 10);
      spp_enu_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(declare_parameter<std::string>("publish.spp.enu", "spp/enu"), 10);
      clock_pub_ = create_publisher<gnss_msg::msg::StampedFloat64Array>(declare_parameter<std::string>("publish.spp.receiver_clock_bias", "spp/receiver_clock_bias_m"), 10);
      skyplot_pub_ = create_publisher<sensor_msgs::msg::Image>(declare_parameter<std::string>("publish.spp.skyplot", "spp/skyplot"), 10);
    }
    if (options.enable_rtk) {
      rtk_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(declare_parameter<std::string>("publish.rtk.fix", "rtk/fix"), 10);
      rtk_ecef_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(declare_parameter<std::string>("publish.rtk.ecef", "rtk/ecef"), 10);
      rtk_enu_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(declare_parameter<std::string>("publish.rtk.enu", "rtk/enu"), 10);
    }

    // output_path 为空则不写 CSV；每次启动在其下新建本地时间命名的子目录（如 2026-11-12_15_23_15），
    // CORE 保留策略避免长数据的早期分片被轮转删除。
    auto output_path = expand_home(declare_parameter<std::string>("output_path", ""));
    if (!output_path.empty()) {
      const std::time_t now = std::time(nullptr);
      std::tm local_time{};
      localtime_r(&now, &local_time);
      char run_dir[32];
      std::strftime(run_dir, sizeof(run_dir), "%Y-%m-%d_%H_%M_%S", &local_time);
      output_path = (std::filesystem::path(output_path) / run_dir).string();
      std::filesystem::create_directories(output_path);
      auto &csv = common::CsvIOHelper::GetInstance();
      bool ok = true;
      if (options.enable_spp)
        ok &= csv.RegisterFileWithHeader(output_path, "spp_enu",
                                         {"timestamp", "x", "y", "z", "lat", "lon", "alt", "ecef_x", "ecef_y", "ecef_z", "num_sats", "num_gps", "num_glo", "num_gal", "num_bds",
                                          "num_qzs", "clk_gps_m", "clk_glo_m", "clk_gal_m", "clk_bds_m"},
                                         true, common::CsvIOHelper::RetentionClass::CORE);
      if (options.enable_rtk)
        ok &= csv.RegisterFileWithHeader(output_path, "rtk_enu",
                                         {"timestamp", "x", "y", "z", "lat", "lon", "alt", "pos_type", "num_dd_psr", "num_dd_cp", "base_age_s", "ratio", "num_fix_dd", "std_e_m",
                                          "std_n_m", "std_u_m"},
                                         true, common::CsvIOHelper::RetentionClass::CORE);
      csv_enabled_ = ok;
      if (!ok)
        RCLCPP_ERROR(get_logger(), "CSV registration failed: %s", output_path.c_str());
    }

    RawStreamCallbacks callbacks;
    callbacks.ephem = [this](const EphemBasePtr &ephem) { manager_->add_ephemeris(ephem); };
    callbacks.iono = [this](const std::vector<double> &iono_params) { manager_->set_iono(iono_params); };
    callbacks.reference_lla = [this](const Eigen::Vector3d &lla) { set_reference(lla); };
    callbacks.obs = [this](const std::vector<ObsPtr> &meas) { process_epoch(meas); };
    callbacks.finished = [this]() {
      if (csv_enabled_)
        common::CsvIOHelper::GetInstance().Flush();
      RCLCPP_INFO(get_logger(), "Offline rover data finished.");
    };
    RCLCPP_INFO(get_logger(), "Mode %s, SPP %s, RTK %s.", mode.c_str(), options.enable_spp ? "on" : "off", options.enable_rtk ? "on" : "off");
    streams_->start(callbacks);
  }

  ~GnssSolverNode() override {
    // 先停离线读取线程，再释放它回调里用到的成员
    streams_->stop();
    if (csv_enabled_)
      common::CsvIOHelper::GetInstance().Flush();
  }

private:
  // ENU 原点：在线取驱动 receiver_lla 首帧；离线取观测文件头 APPROX POSITION，没有时取首个解。
  void set_reference(const Eigen::Vector3d &lla) {
    if (has_reference_ || !lla.allFinite() || std::abs(lla.x()) >= 90 || std::abs(lla.y()) > 180)
      return;
    reference_lla_ = lla;
    reference_ecef_ = geo2ecef(lla);
    has_reference_ = true;
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = now();
    msg.header.frame_id = "gnss";
    msg.latitude = lla.x();
    msg.longitude = lla.y();
    msg.altitude = lla.z();
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    reference_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "ENU origin: lat=%.9f lon=%.9f h=%.3f m", lla.x(), lla.y(), lla.z());
  }

  void process_epoch(const std::vector<ObsPtr> &meas) {
    const EpochResult result = manager_->process(meas);
    if (!result.skip_reason.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Skipping epoch: %s.", result.skip_reason.c_str());
      return;
    }
    if (!has_reference_ && offline_) {
      if (result.spp.valid)
        set_reference(ecef2geo(result.spp.pos_ecef));
      else if (result.has_rtk && result.rtk.pos_type != PV_NONE)
        set_reference(ecef2geo(result.rtk.pos_ecef));
    }
    if (!has_reference_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Skipping epoch: waiting for receiver_lla (ENU origin).");
      return;
    }

    if (spp_fix_pub_) {
      if (result.spp.valid)
        publish_spp(result);
      else
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "SPP failed (%zu usable satellites); no solution published.", result.obs.size());
    }
    if (rtk_fix_pub_) {
      if (!result.has_rtk || result.rtk.pos_type == PV_NONE || !result.rtk.pos_ecef.allFinite())
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "RTK failed; no solution published.");
      else
        publish_rtk(result.rtk);
    }
  }

  void publish_point(const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr &pub, const builtin_interfaces::msg::Time &stamp, const char *frame,
                     const Eigen::Vector3d &p) const {
    geometry_msgs::msg::PointStamped point;
    point.header.stamp = stamp;
    point.header.frame_id = frame;
    point.point.x = p.x();
    point.point.y = p.y();
    point.point.z = p.z();
    pub->publish(point);
  }

  void publish_spp(const EpochResult &result) {
    const SppResult &spp = result.spp;
    const Eigen::Vector3d &ecef = spp.pos_ecef;
    const Eigen::Vector3d lla = ecef2geo(ecef);
    const Eigen::Vector3d enu = ecef2enu(reference_lla_, ecef - reference_ecef_);
    if (!lla.allFinite() || !enu.allFinite())
      return;
    const auto stamp = gpst2stamp(spp.time);
    const auto count = [&spp](uint32_t sys) {
      const auto it = spp.sys_counts.find(sys);
      return it == spp.sys_counts.end() ? 0 : it->second;
    };

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = stamp;
    fix.header.frame_id = "gnss";
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    if (count(SYS_GPS) || count(SYS_QZS))
      fix.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    if (count(SYS_GLO))
      fix.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
    if (count(SYS_GAL))
      fix.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
    if (count(SYS_BDS))
      fix.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
    fix.latitude = lla.x();
    fix.longitude = lla.y();
    fix.altitude = lla.z();
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    spp_fix_pub_->publish(fix);
    publish_point(spp_ecef_pub_, stamp, "ecef", ecef);
    publish_point(spp_enu_pub_, stamp, "enu", enu);

    // 固定顺序 GPS/GLO/GAL/BDS，QZSS 共用 GPS；未参与的星座为 NaN，避免误读为零钟差。
    gnss_msg::msg::StampedFloat64Array clocks;
    clocks.header.stamp = stamp;
    clocks.data.assign(spp.clock_bias_m.begin(), spp.clock_bias_m.end());
    clock_pub_->publish(clocks);

    if (csv_enabled_) {
      // CsvIOHelper 对 double 使用 std::to_string（6 位小数），经纬度需要更高精度，单独格式化。
      char lat[32], lon[32];
      std::snprintf(lat, sizeof(lat), "%.9f", lla.x());
      std::snprintf(lon, sizeof(lon), "%.9f", lla.y());
      const auto utc = gpst2utc(spp.time);
      common::CsvIOHelper::GetInstance().write_line("spp_enu", static_cast<double>(utc.time) + utc.sec, enu.x(), enu.y(), enu.z(), std::string(lat), std::string(lon), lla.z(), ecef.x(),
                                                    ecef.y(), ecef.z(), result.obs.size(), count(SYS_GPS), count(SYS_GLO), count(SYS_GAL), count(SYS_BDS), count(SYS_QZS),
                                                    spp.clock_bias_m[0], spp.clock_bias_m[1], spp.clock_bias_m[2], spp.clock_bias_m[3]);
    }

    if (skyplot_pub_->get_subscription_count() > 0)
      publish_skyplot(result.obs, result.ephems, ecef, stamp);
  }

  void publish_rtk(const RtkSolution &sol) {
    if (sol.pos_type == PV_SINGLE)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Differential solution unavailable (base age %.1f s, %u DD), publishing SPP.", sol.base_age_s, sol.num_dd_psr);
    const Eigen::Vector3d &ecef = sol.pos_ecef;
    const Eigen::Vector3d lla = ecef2geo(ecef);
    const Eigen::Vector3d enu = ecef2enu(reference_lla_, ecef - reference_ecef_);
    // NavSatFix 协方差为 ENU 顺序；R 为 ENU→ECEF 旋转，故 cov_enu = Rᵀ cov_ecef R。
    const Eigen::Matrix3d R = geo2rotation(lla);
    const Eigen::Matrix3d cov_enu = R.transpose() * sol.pos_cov_ecef * R;
    const auto stamp = gpst2stamp(sol.time);

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = stamp;
    fix.header.frame_id = "gnss";
    // NavSatStatus 没有 RTK 细分：差分类解统一为 GBAS_FIX，具体类型见 CSV 的 pos_type
    // （16 单点 / 17 伪距差分 / 34 浮点 / 50 固定）。
    fix.status.status = sol.pos_type == PV_SINGLE ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    fix.latitude = lla.x();
    fix.longitude = lla.y();
    fix.altitude = lla.z();
    if (sol.pos_type == PV_SINGLE) {
      fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    } else {
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          fix.position_covariance[3 * r + c] = cov_enu(r, c);
      fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
    }
    rtk_fix_pub_->publish(fix);
    publish_point(rtk_ecef_pub_, stamp, "ecef", ecef);
    publish_point(rtk_enu_pub_, stamp, "enu", enu);

    if (csv_enabled_) {
      char lat[32], lon[32];
      std::snprintf(lat, sizeof(lat), "%.9f", lla.x());
      std::snprintf(lon, sizeof(lon), "%.9f", lla.y());
      const double nan = std::numeric_limits<double>::quiet_NaN();
      const bool has_cov = sol.pos_type != PV_SINGLE;
      const auto utc = gpst2utc(sol.time);
      common::CsvIOHelper::GetInstance().write_line("rtk_enu", static_cast<double>(utc.time) + utc.sec, enu.x(), enu.y(), enu.z(), std::string(lat), std::string(lon), lla.z(), sol.pos_type,
                                                    sol.num_dd_psr, sol.num_dd_cp, sol.base_age_s, sol.ratio, sol.num_fix_dd, has_cov ? std::sqrt(cov_enu(0, 0)) : nan,
                                                    has_cov ? std::sqrt(cov_enu(1, 1)) : nan, has_cov ? std::sqrt(cov_enu(2, 2)) : nan);
    }
  }

  // 虚拟鱼眼天空图：等距投影，天顶为圆心、地平线为外圆，北向上、东向右（常规 skyplot 约定）。
  // 仅绘制本历元有可用伪距和健康星历的卫星；实心=高于截止角，空心=低于截止角（被 solver 剔除）。
  void publish_skyplot(const std::vector<ObsPtr> &obs, const std::vector<EphemBasePtr> &ephems, const Eigen::Vector3d &ecef, const builtin_interfaces::msg::Time &stamp) const {
    struct Style {
      uint32_t sys;
      const char *name;
      cv::Scalar color; // BGR
    };
    static const Style styles[] = {
        {SYS_GPS, "GPS", {80, 200, 80}}, {SYS_GLO, "GLONASS", {80, 80, 240}}, {SYS_GAL, "Galileo", {240, 160, 40}}, {SYS_BDS, "BeiDou", {0, 170, 255}}, {SYS_QZS, "QZSS", {200, 80, 200}}};
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
      const Eigen::Vector3d sat_pos =
          sys == SYS_GLO ? geph2pos(obs[i]->time, std::dynamic_pointer_cast<GloEphem>(ephems[i]), &clock_bias_s) : eph2pos(obs[i]->time, std::dynamic_pointer_cast<Ephem>(ephems[i]), &clock_bias_s);
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

  bool offline_ = false;
  bool has_reference_ = false, csv_enabled_ = false;
  double elevation_cutoff_degree_ = 15.0;
  int skyplot_size_ = 600;
  Eigen::Vector3d reference_lla_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d reference_ecef_ = Eigen::Vector3d::Zero();
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr reference_pub_, spp_fix_pub_, rtk_fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr spp_ecef_pub_, spp_enu_pub_, rtk_ecef_pub_, rtk_enu_pub_;
  rclcpp::Publisher<gnss_msg::msg::StampedFloat64Array>::SharedPtr clock_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr skyplot_pub_;
  std::unique_ptr<AlgorithmManager> manager_;
  std::unique_ptr<StreamWrapper> streams_;
};
} // namespace gnss_comm

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_comm::GnssSolverNode>());
  rclcpp::shutdown();
  google::ShutdownGoogleLogging();
  return 0;
}
