#ifndef GNSS_COMM_TYPES_OBSERVATION_HPP_
#define GNSS_COMM_TYPES_OBSERVATION_HPP_

#include <cstdint>
#include <memory>
#include <vector>
#include "gnss_comm/types/time.hpp"

namespace gnss_comm {

// 单颗卫星在一个接收机采样历元的多信号观测，不是整个历元的全部卫星。
// 各观测向量的相同下标对应同一信号；填充哪些辅助向量由解析器决定，使用前需确认长度。
struct GNSSObservation {
  gtime_t time{};               // 接收机采样时刻，工程内部约定使用 GPST
  uint32_t sat{};               // 工程内部卫星编号，由星座与星座内编号转换得到，不是原始 PRN
  std::vector<double> freqs;    // 各信号实际载波频率，Hz；GLONASS FDMA 需计入频道偏移
  std::vector<double> cn0;      // 载噪密度比 C/N0，dB-Hz，用于描述信号强度
  std::vector<uint8_t> lli;     // 失锁指示，具体位含义由输入格式/解析器约定，不是失锁次数
  std::vector<uint8_t> code;    // 工程 CODE_* 信号编码，表示频段与调制方式
  std::vector<double> psr;      // 伪距观测，m，包含接收机/卫星钟差及传播误差等影响
  std::vector<double> psr_std;  // 伪距标准差，m（不是方差）
  std::vector<double> cp;       // 载波相位观测，周；乘以对应波长可转换为米
  std::vector<double> cp_std;   // 载波相位标准差，周
  std::vector<double> dopp;     // 多普勒观测，Hz；转换速度时按解析器和算法的符号约定处理
  std::vector<double> dopp_std; // 多普勒标准差，Hz
  // 观测状态位：bit0 伪距有效，bit1 载波相位有效；
  // u-blox 还使用 bit2 半周有效、bit3 已扣除半周；不是单一的周跳标志。
  std::vector<uint8_t> status;
};
typedef std::shared_ptr<GNSSObservation> GNSSObservationPtr;

// 保留既有解析器、定位算法和 ROS 转换接口使用的类型名。
using Obs = GNSSObservation;
using ObsPtr = GNSSObservationPtr;

} // namespace gnss_comm

#endif // GNSS_COMM_TYPES_OBSERVATION_HPP_
