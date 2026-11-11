/**
 * @brief 轻量日志封装，用于替代 glog。
 */
#ifndef UBLOX_DRIVER_LOGGING_HPP_
#define UBLOX_DRIVER_LOGGING_HPP_

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>

namespace ublox_driver {

enum class LogSeverity {
  INFO,
  WARNING,
  ERROR,
  FATAL,
};

inline std::string CurrentTimestampString() {
  const auto now = std::chrono::system_clock::now();
  const auto now_time_t = std::chrono::system_clock::to_time_t(now);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) %
                  1000;

  std::tm local_tm {};
  localtime_r(&now_time_t, &local_tm);

  std::ostringstream oss;
  oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
      << '.'
      << std::setw(3) << std::setfill('0') << ms.count();
  return oss.str();
}

class LogMessage {
public:
  LogMessage(const char *severity_name,
             const char *file,
             int line,
             LogSeverity severity)
      : severity_name_(severity_name),
        severity_(severity) {
    (void)file;
    (void)line;
  }

  ~LogMessage() {
    std::cerr << '[' << CurrentTimestampString() << "] "
              << '[' << severity_name_ << "] "
              << stream_.str() << std::endl;
    if (severity_ == LogSeverity::FATAL) {
      std::abort();
    }
  }

  std::ostream &stream() { return stream_; }

private:
  const char *severity_name_;
  LogSeverity severity_;
  std::ostringstream stream_;
};

} // namespace ublox_driver

#ifdef LOG
#undef LOG
#endif

#ifdef LOG_IF
#undef LOG_IF
#endif

#define LOG(severity)                                                          \
  ::ublox_driver::LogMessage(                                                  \
      #severity, __FILE__, __LINE__, ::ublox_driver::LogSeverity::severity)    \
      .stream()

#define LOG_IF(severity, condition)                                            \
  if (!(condition))                                                            \
  ;                                                                            \
  else                                                                         \
  LOG(severity)

#endif
