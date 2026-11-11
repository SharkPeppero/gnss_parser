#ifndef GNSS_COMM_TYPES_TIME_PULSE_HPP_
#define GNSS_COMM_TYPES_TIME_PULSE_HPP_

#include <cstdint>
#include <memory>
#include "gnss_comm/types/time.hpp"

namespace gnss_comm {

// 接收机时间脉冲（PPS，秒脉冲）的时间及参考系统信息。
// 保存协议解析结果，不执行时间系统转换，也不表示脉冲已与主机时钟同步。
struct TimePulseInfo {
  gtime_t time{};    // 脉冲关联时刻；当前 u-blox 解析器通过 gpst2time 构造，需结合协议时间基准解释
  bool utc_based{}; // 是否以 UTC 为时间参考：true 表示 UTC，false 表示 GNSS 时间参考
  uint32_t time_sys{}; // 当前工程使用 SYS_* 星座标识或 SYS_NONE，不是 T_SYS_* 时间系统编号
};
typedef std::shared_ptr<TimePulseInfo> TimePulseInfoPtr;

} // namespace gnss_comm

#endif // GNSS_COMM_TYPES_TIME_PULSE_HPP_
