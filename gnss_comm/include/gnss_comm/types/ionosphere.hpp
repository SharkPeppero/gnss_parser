#ifndef GNSS_COMM_TYPES_IONOSPHERE_HPP_
#define GNSS_COMM_TYPES_IONOSPHERE_HPP_

#include <memory>
#include <vector>
#include "gnss_comm/types/time.hpp"

// 电离层总电子含量（TEC）网格数据，用于描述电离层及辅助传播延迟改正。
namespace gnss_comm {

// 单历元的 TEC 网格及误差数据，网格范围由起点、终点和间隔描述。
struct TECEpoch {
  gtime_t time{}; // epoch time
  double rb{};    // earth radius (km)
  double lla_start[3]{};
  double lla_end[3]{};
  double lla_interval[3]{};
  std::vector<double> data; // num_hgt * num_lat * num_lon
  std::vector<double> rms;  // num_hgt * num_lat * num_lon
};
typedef std::shared_ptr<TECEpoch> TECEpochPtr;

} // namespace gnss_comm

#endif // GNSS_COMM_TYPES_IONOSPHERE_HPP_
