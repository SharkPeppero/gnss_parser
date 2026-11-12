//
// Created by yjh on 2026/11/12.
//

#include "gnss_comm/stream/correction_stream.hpp"
#include "gnss_comm/rinex/rinex_conversion.hpp"
#include "gnss_comm/rtcm/rtcm_conversion.hpp"
#include "gnss_comm/utility/coordinate_utility.hpp"
#include "gnss_comm/utility/time_utility.hpp"
#include <algorithm>
#include <cmath>
#include <glog/logging.h>
#include <iterator>
#include <stdexcept>

namespace gnss_comm {

CorrectionStream::CorrectionStream(const std::vector<std::string> &paths, const std::string &format, const std::vector<double> &marker_override)
    : paths_(paths), format_(format), marker_override_(marker_override) {
  if (format_ != "auto" && format_ != "rinex" && format_ != "rtcm3")
    throw std::invalid_argument("correction format must be auto / rinex / rtcm3, got " + format_);
  if (!marker_override_.empty() && marker_override_.size() != 3)
    throw std::invalid_argument("base_ecef must be [] or [x, y, z]");
}

bool CorrectionStream::is_rtcm(const std::string &path) const {
  if (format_ != "auto")
    return format_ == "rtcm3";
  return path.find(".rtcm") != std::string::npos || (path.size() >= 4 && path.compare(path.size() - 4, 4, ".rtc") == 0);
}

bool CorrectionStream::load(const gtime_t &ref_time) {
  if (loaded_)
    return true;
  bool has_station = false;
  for (const auto &path : paths_) {
    const size_t before = epochs_.size();
    if (is_rtcm(path)) {
      RtcmStation rtcm;
      rtcm2obs(path, ref_time, epochs_, rtcm);
      // 1005/1006 给的是 ARP，换算回标石以与 RINEX 统一：标石 = ARP − 天线高 × 天顶方向
      if (!has_station && rtcm.has_position) {
        const Eigen::Matrix3d R_enu2ecef = geo2rotation(ecef2geo(rtcm.arp_ecef));
        station_.name = "RTCM " + std::to_string(rtcm.station_id);
        station_.antenna_type = rtcm.antenna_type;
        station_.antenna_delta_hen = Eigen::Vector3d(rtcm.antenna_height_m, 0, 0);
        station_.marker_ecef = rtcm.arp_ecef - R_enu2ecef.col(2) * rtcm.antenna_height_m;
        has_station = true;
      }
    } else {
      RinexObsHeader header;
      if (!has_station && rinex2obs_header(path, header)) {
        station_.name = header.marker_name;
        station_.antenna_type = header.antenna_type;
        station_.antenna_delta_hen = header.antenna_delta_hen;
        station_.marker_ecef = header.approx_pos;
        has_station = header.approx_pos.norm() > 6.0e6;
      }
      rinex2obs(path, epochs_);
    }
    LOG(INFO) << "Loaded " << epochs_.size() - before << " base epochs from " << path;
  }
  epochs_.erase(std::remove_if(epochs_.begin(), epochs_.end(), [](const std::vector<ObsPtr> &epoch) { return epoch.empty(); }), epochs_.end());
  std::stable_sort(epochs_.begin(), epochs_.end(), [](const std::vector<ObsPtr> &a, const std::vector<ObsPtr> &b) { return time_diff(a.front()->time, b.front()->time) < 0; });

  if (marker_override_.size() == 3) {
    station_.marker_ecef = Eigen::Vector3d(marker_override_[0], marker_override_[1], marker_override_[2]);
    has_station = true;
  } else {
    LOG(WARNING) << "base_ecef not set, using base position from the correction file (RINEX APPROX POSITION is meter-level).";
  }
  if (epochs_.empty() || !has_station || station_.marker_ecef.norm() < 6.0e6) {
    LOG(ERROR) << "Correction data unusable: " << epochs_.size() << " epochs, base position " << (has_station ? "ok" : "missing")
               << " (decompress .crx with crx2rnx first; RTCM needs 1005/1006 or base_ecef)";
    return false;
  }
  loaded_ = true;
  return true;
}

const std::vector<ObsPtr> *CorrectionStream::nearest_epoch(const gtime_t &time) const {
  if (epochs_.empty())
    return nullptr;
  const auto it = std::lower_bound(epochs_.begin(), epochs_.end(), time, [](const std::vector<ObsPtr> &epoch, const gtime_t &t) { return time_diff(epoch.front()->time, t) < 0; });
  if (it == epochs_.end())
    return &epochs_.back();
  if (it == epochs_.begin())
    return &epochs_.front();
  const auto prev = std::prev(it);
  return std::abs(time_diff(time, prev->front()->time)) <= std::abs(time_diff(it->front()->time, time)) ? &(*prev) : &(*it);
}

} // namespace gnss_comm
