//
// Created by yjh on 2026/11/12.
//

#ifndef GNSS_COMM_STREAM_STREAM_WRAPPER_HPP_
#define GNSS_COMM_STREAM_STREAM_WRAPPER_HPP_

#include "gnss_comm/stream/correction_stream.hpp"
#include "gnss_comm/stream/raw_stream.hpp"
#include <memory>

namespace gnss_comm {

// 数据流汇总：
//   原始数据：在线 RosRawStream（订阅话题 / ros2 bag 回放）或离线 RinexRawStream（一次性读取文件）；
//   差分数据：CorrectionStream（离线 RINEX / RTCM 文件），不需要 RTK 时为空。
class StreamWrapper {
public:
  StreamWrapper(std::unique_ptr<RawStream> raw, std::unique_ptr<CorrectionStream> correction) : raw_(std::move(raw)), correction_(std::move(correction)) {}
  ~StreamWrapper() { stop(); }

  void start(const RawStreamCallbacks &callbacks) { raw_->start(callbacks); }
  void stop() {
    if (raw_)
      raw_->stop();
  }

  CorrectionStream *correction() const { return correction_.get(); }

private:
  std::unique_ptr<RawStream> raw_;
  std::unique_ptr<CorrectionStream> correction_;
};

} // namespace gnss_comm

#endif // GNSS_COMM_STREAM_STREAM_WRAPPER_HPP_
