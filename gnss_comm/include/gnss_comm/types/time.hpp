#ifndef GNSS_COMM_TYPES_TIME_HPP_
#define GNSS_COMM_TYPES_TIME_HPP_

#include <ctime>

namespace gnss_comm {

// 通用时间容器：整数秒与小数秒分开保存，避免大数值时间戳损失小数精度。
// 容器本身不标记 GPST/UTC/BDT，时间系统由所在字段和解析流程约定。
struct gtime_t {
  time_t time{}; // 整数秒部分，沿用工程 time_t 表示
  double sec{}; // 小数秒部分；规范化后满足 0 <= sec < 1
};

} // namespace gnss_comm

#endif // GNSS_COMM_TYPES_TIME_HPP_
