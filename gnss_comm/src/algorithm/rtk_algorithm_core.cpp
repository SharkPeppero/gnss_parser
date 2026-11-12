//
// Created by yjh on 2026/11/12.
//

#include "gnss_comm/algorithm/rtk_algorithm_core.hpp"
#include "gnss_comm/gnss_utility.hpp"
#include "gnss_comm/rinex/rinex_conversion.hpp"
#include <glog/logging.h>

namespace gnss_comm {

void RTKAlgorithmCore::initialize(const BaseStationInfo &station) {
  // 标石 → 天线参考点 ARP：ANTENNA: DELTA H/E/N 是当地 ENU 下的 (E, N, H) 偏移。
  const Eigen::Matrix3d R_enu2ecef = geo2rotation(ecef2geo(station.marker_ecef));
  const Eigen::Vector3d &hen = station.antenna_delta_hen;
  const Eigen::Vector3d base_arp = station.marker_ecef + R_enu2ecef * Eigen::Vector3d(hen.y(), hen.z(), hen.x());
  LOG(INFO) << "Base " << station.name << " antenna '" << station.antenna_type << "', ARP offset H/E/N = " << hen.transpose() << " m";

  // 基站天线 PCO（ANTEX，相对 ARP）：ANTEX 频率代码 → (星座, 频段)；缺对应频率时退用 GPS 同频段的值。
  BasePcoMap base_pco;
  std::map<std::string, Eigen::Vector3d> freq2pco_neu;
  if (antex_path_.empty()) {
    LOG(WARNING) << "antex_path not set, base antenna PCO is not applied.";
  } else if (!antex2rcv_pco(antex_path_, station.antenna_type, freq2pco_neu)) {
    LOG(WARNING) << "Antenna '" << station.antenna_type << "' not found in " << antex_path_ << ", base antenna PCO is not applied.";
  } else {
    struct PcoCode {
      uint32_t sys;
      int band;
      const char *code, *fallback;
    };
    static const PcoCode codes[] = {{SYS_GPS, 0, "G01", "G01"}, {SYS_GPS, 1, "G02", "G02"}, {SYS_GLO, 0, "R01", "G01"}, {SYS_GLO, 1, "R02", "G02"},
                                    {SYS_GAL, 0, "E01", "G01"}, {SYS_GAL, 1, "E07", "G02"}, {SYS_BDS, 0, "C02", "G01"}, {SYS_BDS, 1, "C07", "G02"}};
    for (const auto &code : codes) {
      auto it = freq2pco_neu.find(code.code);
      if (it == freq2pco_neu.end())
        it = freq2pco_neu.find(code.fallback);
      if (it == freq2pco_neu.end())
        continue;
      const Eigen::Vector3d &neu = it->second;
      base_pco[{code.sys, code.band}] = R_enu2ecef * Eigen::Vector3d(neu.y(), neu.x(), neu.z());
      LOG(INFO) << "Base PCO " << code.code << " (from " << it->first << "): N/E/U = " << neu.transpose() << " m";
    }
  }
  solver_ = std::make_unique<RtkSolverPipeline>(base_arp, options_, base_pco);
}

} // namespace gnss_comm
