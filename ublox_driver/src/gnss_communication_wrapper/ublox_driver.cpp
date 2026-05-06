#include "ublox_driver/gnss_communication_wrapper/ublox_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

#include "ublox_driver/common/logging.hpp"

namespace {

constexpr uint32_t kReceiverConfigBufferCapacity = 8192;
constexpr auto kReceiverConfigAckTimeout = std::chrono::seconds(3);
constexpr auto kReceiverConfigAckPollInterval = std::chrono::milliseconds(100);
constexpr auto kNtripGgaInterval = std::chrono::seconds(1);
constexpr uint64_t kNtripGgaLogAlwaysCount = 5;
constexpr uint64_t kNtripGgaLogPeriodicInterval = 30;
constexpr uint64_t kRtcmChunkLogAlwaysCount = 5;
constexpr uint64_t kRtcmChunkLogPeriodicInterval = 50;
constexpr uint8_t kRtcmPreamble = 0xD3;
constexpr uint16_t kRtcmMaxPayloadLength = 1023;

uint32_t ComputeRtcmCrc24q(const uint8_t *data, size_t len) {
  constexpr uint32_t kPolynomial = 0x1864CFB;
  uint32_t crc = 0;

  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint32_t>(data[i]) << 16U;
    for (int bit = 0; bit < 8; ++bit) {
      crc <<= 1U;
      if ((crc & 0x1000000U) != 0U) {
        crc ^= kPolynomial;
      }
    }
  }

  return crc & 0xFFFFFFU;
}

uint16_t GetRtcmPayloadLength(const uint8_t *frame) { return static_cast<uint16_t>(((frame[1] & 0x03U) << 8U) | frame[2]); }

uint16_t GetRtcmMessageType(const uint8_t *frame, uint16_t payload_length) {
  if (payload_length < 2U) {
    return 0;
  }

  return static_cast<uint16_t>((static_cast<uint16_t>(frame[3]) << 4U) | (static_cast<uint16_t>(frame[4]) >> 4U));
}

int GetNmeaFixQuality(const gnss_comm::PVTSolution &pvt_soln) {
  if (!pvt_soln.valid_fix || pvt_soln.fix_type < 2U) {
    return 0;
  }
  if (pvt_soln.carr_soln == 2U) {
    return 4;
  }
  if (pvt_soln.carr_soln == 1U) {
    return 5;
  }
  if (pvt_soln.diff_soln) {
    return 2;
  }
  return 1;
}

std::string FormatNmeaDegrees(double value, bool is_latitude) {
  const double abs_value = std::fabs(value);
  const int degrees = static_cast<int>(abs_value);
  const double minutes = (abs_value - static_cast<double>(degrees)) * 60.0;

  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(is_latitude ? 2 : 3) << degrees << std::fixed << std::setprecision(5) << std::setw(8) << minutes;
  return oss.str();
}

std::string AppendNmeaChecksum(const std::string &sentence_body) {
  uint8_t checksum = 0;
  for (unsigned char ch : sentence_body) {
    checksum ^= ch;
  }

  std::ostringstream oss;
  oss << '$' << sentence_body << '*' << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(checksum) << "\r\n";
  return oss.str();
}

} // namespace

namespace ublox_driver {

GNSSDriverManager::GNSSDriverManager(rclcpp::Node::SharedPtr node, //
                                     std::string config_filepath,  //
                                     std::string receiver_config_filepath)
    : node_(std::move(node)),                       //
      config_filepath_(std::move(config_filepath)), //
      receiver_config_filepath_(std::move(receiver_config_filepath)) {
  // 加载参数
  params_ = LoadUbloxDriverParams(config_filepath_, receiver_config_filepath_);

  // 初始化ROS的句柄
  ros_handler_ = std::make_shared<UbloxRosHandler>(node_, params_.raw_observation_system_mask, params_.ephemeris_system_mask);
  ros_handler_->registerPvtCallback([this](const gnss_comm::PVTSolutionPtr &pvt_soln) { handlePvtSolution(pvt_soln); });

  // 初始化Ublox消息的解析对象
  ublox_message_processor_ = std::make_shared<UbloxMessageProcessor>(ros_handler_);

  // 初始化串口Pipline
  setupSerialPipeline();

  // 初始化RTCM的Pipline
  setupRtcmPipeline();
}

GNSSDriverManager::~GNSSDriverManager() {
  if (rtcm_client_) {
    rtcm_client_->close();
  }
  if (serial_handler_) {
    serial_handler_->close();
  }
}

void GNSSDriverManager::handleConfigAck(const uint8_t *data, size_t len) {
  const int ack_result = UbloxMessageProcessor::check_ack(data, len);
  if (ack_result == 0) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    ack_flag_ = ack_result;
  }
  ack_cv_.notify_one();
}

void GNSSDriverManager::handlePvtSolution(const gnss_comm::PVTSolutionPtr &pvt_soln) {
  if (!pvt_soln) {
    return;
  }

  std::lock_guard<std::mutex> lock(latest_pvt_mutex_);
  latest_pvt_ = std::make_shared<gnss_comm::PVTSolution>(*pvt_soln);
}

std::string GNSSDriverManager::buildNmeaGgaSentence(const gnss_comm::PVTSolution &pvt_soln) const {
  const int fix_quality = GetNmeaFixQuality(pvt_soln);
  if (fix_quality == 0) {
    return "";
  }

  gnss_comm::gtime_t utc_time = gnss_comm::gpst2utc(pvt_soln.time);
  double epoch[6] = {};
  gnss_comm::time2epoch(utc_time, epoch);

  const double geoid_separation = pvt_soln.hgt - pvt_soln.hgt_msl;

  std::ostringstream body;
  body << "GPGGA," << std::setfill('0') << std::setw(2) << static_cast<int>(epoch[3]) << std::setw(2) << static_cast<int>(epoch[4]) << std::fixed << std::setprecision(3) << std::setw(6) << epoch[5]
       << ',' << FormatNmeaDegrees(pvt_soln.lat, true) << ',' << (pvt_soln.lat >= 0.0 ? 'N' : 'S') << ',' << FormatNmeaDegrees(pvt_soln.lon, false) << ',' << (pvt_soln.lon >= 0.0 ? 'E' : 'W') << ','
       << fix_quality << ',' << std::setw(2) << static_cast<int>(pvt_soln.num_sv) << ',' << std::setprecision(1) << (pvt_soln.p_dop > 0.0 ? pvt_soln.p_dop : 0.0) << ',' << std::setprecision(3)
       << pvt_soln.hgt_msl << ",M," << geoid_separation << ",M,,";

  return AppendNmeaChecksum(body.str());
}

void GNSSDriverManager::sendNtripGga() {
  if (!params_.enable_ntrip_rtcm || !rtcm_client_ || !rtcm_client_->is_open()) {
    return;
  }

  gnss_comm::PVTSolutionPtr latest_pvt;
  {
    std::lock_guard<std::mutex> lock(latest_pvt_mutex_);
    latest_pvt = latest_pvt_;
  }

  if (!latest_pvt || latest_pvt->time.time == 0) {
    ++ntrip_gga_skip_count_;
    if (ntrip_gga_skip_count_ <= kNtripGgaLogAlwaysCount || (ntrip_gga_skip_count_ % kNtripGgaLogPeriodicInterval) == 0U) {
      LOG(WARNING) << "NTRIP GGA skipped because no current PVT solution is available yet."
                   << " skip_count=" << ntrip_gga_skip_count_;
    }
    return;
  }

  const std::string gga_sentence = buildNmeaGgaSentence(*latest_pvt);
  if (gga_sentence.empty()) {
    ++ntrip_gga_skip_count_;
    if (ntrip_gga_skip_count_ <= kNtripGgaLogAlwaysCount || (ntrip_gga_skip_count_ % kNtripGgaLogPeriodicInterval) == 0U) {
      LOG(WARNING) << "NTRIP GGA skipped because the current PVT fix is not valid enough."
                   << " skip_count=" << ntrip_gga_skip_count_ << " fix_type=" << static_cast<int>(latest_pvt->fix_type) << " valid_fix=" << latest_pvt->valid_fix
                   << " diff_soln=" << latest_pvt->diff_soln << " carr_soln=" << static_cast<int>(latest_pvt->carr_soln);
    }
    return;
  }

  if (!rtcm_client_->write(gga_sentence, kIoTimeoutMs)) {
    LOG(WARNING) << "Failed to send NTRIP GGA sentence to caster.";
    return;
  }

  ++ntrip_gga_send_count_;
  if (ntrip_gga_send_count_ <= kNtripGgaLogAlwaysCount || (ntrip_gga_send_count_ % kNtripGgaLogPeriodicInterval) == 0U) {
    std::string gga_preview = gga_sentence;
    while (!gga_preview.empty() && (gga_preview.back() == '\r' || gga_preview.back() == '\n')) {
      gga_preview.pop_back();
    }
    LOG(INFO) << "NTRIP GGA sent:"
              << " send_index=" << ntrip_gga_send_count_ << " sentence=" << gga_preview;
  }
}

void GNSSDriverManager::logRtcmInputChunk(size_t len) {
  ++rtcm_input_chunk_count_;
  rtcm_input_total_bytes_ += len;

  if (rtcm_input_chunk_count_ <= kRtcmChunkLogAlwaysCount || (rtcm_input_chunk_count_ % kRtcmChunkLogPeriodicInterval) == 0U) {
    LOG(INFO) << "RTCM input chunk received:"
              << " chunk_index=" << rtcm_input_chunk_count_ << " chunk_bytes=" << len << " total_bytes=" << rtcm_input_total_bytes_;
  }
}

void GNSSDriverManager::logRtcmFrames(const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0U) {
    return;
  }

  std::lock_guard<std::mutex> lock(rtcm_log_mutex_);
  logRtcmInputChunk(len);
  rtcm_log_buffer_.insert(rtcm_log_buffer_.end(), data, data + len);
  drainRtcmLogBuffer();
}

void GNSSDriverManager::drainRtcmLogBuffer() {
  while (true) {
    const auto preamble_it = std::find(rtcm_log_buffer_.begin(), rtcm_log_buffer_.end(), kRtcmPreamble);
    if (preamble_it == rtcm_log_buffer_.end()) {
      rtcm_log_buffer_.clear();
      return;
    }

    if (preamble_it != rtcm_log_buffer_.begin()) {
      rtcm_log_buffer_.erase(rtcm_log_buffer_.begin(), preamble_it);
    }

    if (rtcm_log_buffer_.size() < 3U) {
      return;
    }

    if ((rtcm_log_buffer_[1] & 0xFCU) != 0U) {
      rtcm_log_buffer_.erase(rtcm_log_buffer_.begin());
      continue;
    }

    const uint16_t payload_length = GetRtcmPayloadLength(rtcm_log_buffer_.data());
    if (payload_length > kRtcmMaxPayloadLength) {
      rtcm_log_buffer_.erase(rtcm_log_buffer_.begin());
      continue;
    }

    const size_t frame_length = static_cast<size_t>(payload_length) + 6U;
    if (rtcm_log_buffer_.size() < frame_length) {
      return;
    }

    const uint32_t expected_crc = (static_cast<uint32_t>(rtcm_log_buffer_[frame_length - 3U]) << 16U) | (static_cast<uint32_t>(rtcm_log_buffer_[frame_length - 2U]) << 8U) |
                                  static_cast<uint32_t>(rtcm_log_buffer_[frame_length - 1U]);
    const uint32_t computed_crc = ComputeRtcmCrc24q(rtcm_log_buffer_.data(), frame_length - 3U);
    if (computed_crc != expected_crc) {
      LOG(WARNING) << "Discarding RTCM frame with CRC mismatch.";
      rtcm_log_buffer_.erase(rtcm_log_buffer_.begin());
      continue;
    }

    const uint16_t message_type = GetRtcmMessageType(rtcm_log_buffer_.data(), payload_length);
    ++rtcm_frame_count_;
    LOG(INFO) << "RTCM message received:"
              << " frame_index=" << rtcm_frame_count_ << " type=" << message_type << " payload_length=" << payload_length << " frame_length=" << frame_length
              << " total_input_bytes=" << rtcm_input_total_bytes_;

    rtcm_log_buffer_.erase(rtcm_log_buffer_.begin(), rtcm_log_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_length));
  }
}

bool GNSSDriverManager::configureReceiverAtStartup() {
  if (!serial_handler_ || !serial_handler_->is_open()) {
    LOG(ERROR) << "Serial port is not available, skip receiver startup configuration.";
    return false;
  }

  if (params_.receiver_configs.empty()) {
    LOG(WARNING) << "Receiver config list is empty, skip startup configuration.";
    return true;
  }

  std::unique_ptr<uint8_t[]> config_buffer(new uint8_t[kReceiverConfigBufferCapacity]);
  std::memset(config_buffer.get(), 0, kReceiverConfigBufferCapacity);

  uint32_t msg_len = 0;
  if (UbloxMessageProcessor::build_config_msg(params_.receiver_configs, config_buffer.get(), msg_len) != 0 || msg_len == 0) {
    LOG(ERROR) << "Failed to build receiver startup configuration message.";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    ack_flag_ = 0;
  }

  serial_handler_->clearDataCallbacks();
  serial_handler_->registerDataCallback([this](const uint8_t *data, size_t len) { handleConfigAck(data, len); });
  serial_handler_->startRead();

  if (!serial_handler_->writeRaw(config_buffer.get(), msg_len)) {
    serial_handler_->stopRead();
    serial_handler_->clearDataCallbacks();
    return false;
  }

  std::unique_lock<std::mutex> lock(ack_mutex_);
  const auto deadline = std::chrono::steady_clock::now() + kReceiverConfigAckTimeout;
  while (ack_flag_ == 0 && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    ack_cv_.wait_for(lock, kReceiverConfigAckPollInterval);
  }
  lock.unlock();

  serial_handler_->stopRead();
  serial_handler_->clearDataCallbacks();

  if (ack_flag_ == 0) {
    if (!rclcpp::ok()) {
      LOG(WARNING) << "Receiver startup configuration interrupted by shutdown signal.";
    } else {
      LOG(ERROR) << "Timed out waiting for receiver configuration ACK.";
    }
    return false;
  }

  return ack_flag_ == 1;
}

void GNSSDriverManager::setupSerialPipeline() {
  if (!params_.enable_serial) {
    LOG(WARNING) << "Serial communication module is disabled.";
    return;
  }

  serial_handler_ = std::make_shared<SerialHandler>(params_.serial_port, static_cast<unsigned int>(params_.serial_baud_rate));
  if (!serial_handler_->is_open()) {
    LOG(ERROR) << "Failed to initialize serial communication module.";
    return;
  }

  if (params_.enable_config_bring && !configureReceiverAtStartup()) {
    LOG(WARNING) << "Receiver startup configuration did not complete successfully.";
  }

  serial_handler_->registerDataCallback([this](const uint8_t *data, size_t len) {
    if (ublox_message_processor_) {
      ublox_message_processor_->process_data(data, len);
    }
  });
  serial_handler_->startRead();
}

void GNSSDriverManager::setupRtcmPipeline() {
  if (!params_.enable_rtcm_tcp_loopback && !params_.enable_ntrip_rtcm) {
    return;
  }

  if (!serial_handler_ || !serial_handler_->is_open()) {
    LOG(WARNING) << "RTCM TCP client skipped because serial module is unavailable.";
    return;
  }

  if (params_.enable_rtcm_tcp_loopback) {
    rtcm_client_ = std::make_shared<LoopbackTcpClient>(params_.rtcm_tcp_port);
  } else {
    rtcm_client_ = std::make_shared<NtripClient>(params_.ntrip_server, params_.ntrip_port, params_.ntrip_mountpoint, params_.ntrip_username, params_.ntrip_password);
  }

  rtcm_client_->registerDataCallback([this](const uint8_t *data, size_t len) {
    logRtcmFrames(data, len);
    if (serial_handler_) {
      serial_handler_->writeRaw(data, len, kIoTimeoutMs);
    }
  });

  rtcm_client_->startRead();
  LOG(INFO) << "RTCM client started with automatic reconnect enabled.";

  if (params_.enable_ntrip_rtcm) {
    ntrip_gga_timer_ = node_->create_wall_timer(kNtripGgaInterval, [this]() { sendNtripGga(); });
    LOG(INFO) << "NTRIP GGA uplink timer started with period=" << std::chrono::duration_cast<std::chrono::milliseconds>(kNtripGgaInterval).count() << "ms.";
  }
}

} // namespace ublox_driver
