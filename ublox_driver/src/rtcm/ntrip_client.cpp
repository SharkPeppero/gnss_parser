#include "ublox_driver/rtcm/ntrip_client.hpp"

#include <cstdint>
#include <sstream>
#include <utility>
#include <vector>

#include "ublox_driver/common/logging.hpp"
#include "ublox_driver/params/params.hpp"

namespace {

constexpr size_t kMaxNtripHeaderBytes = 8192;
constexpr int kNtripHandshakeTimeoutMs = 3000;

std::string Base64Encode(const std::string &input) {
  static constexpr char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  std::string output;
  output.reserve(((input.size() + 2) / 3) * 4);

  uint32_t accumulator = 0;
  int bit_count = 0;
  for (unsigned char ch : input) {
    accumulator = (accumulator << 8U) | ch;
    bit_count += 8;
    while (bit_count >= 6) {
      bit_count -= 6;
      output.push_back(kAlphabet[(accumulator >> bit_count) & 0x3F]);
    }
  }

  if (bit_count > 0) {
    accumulator <<= static_cast<uint32_t>(6 - bit_count);
    output.push_back(kAlphabet[accumulator & 0x3F]);
  }

  while (output.size() % 4 != 0) {
    output.push_back('=');
  }

  return output;
}

bool StartsWith(const std::string &value, const std::string &prefix) { return value.rfind(prefix, 0) == 0; }

} // namespace

NtripClient::NtripClient(std::string host, uint64_t port, std::string mountpoint, std::string username, std::string password, unsigned int buf_size)
    : TcpClient(std::move(host), port, buf_size), mountpoint_(std::move(mountpoint)), username_(std::move(username)), password_(std::move(password)) {}

std::string NtripClient::buildRequest() const {
  std::string mountpoint = mountpoint_;
  if (!mountpoint.empty() && mountpoint.front() != '/') {
    mountpoint.insert(mountpoint.begin(), '/');
  }

  std::ostringstream request;
  request << "GET " << mountpoint << " HTTP/1.0\r\n";
  request << "User-Agent: NTRIP ublox_driver/1.0\r\n";
  request << "Accept: */*\r\n";
  request << "Connection: close\r\n";
  request << "Ntrip-Version: Ntrip/2.0\r\n";
  if (!username_.empty() || !password_.empty()) {
    request << "Authorization: Basic " << Base64Encode(username_ + ":" + password_) << "\r\n";
  }
  request << "\r\n";
  return request.str();
}

bool NtripClient::parseResponse(std::string response) {
  size_t header_end = response.find("\r\n\r\n");
  size_t delimiter_len = 4;
  if (header_end == std::string::npos) {
    header_end = response.find("\n\n");
    delimiter_len = 2;
  }
  if (header_end == std::string::npos) {
    LOG(ERROR) << "NTRIP handshake response does not contain a complete header.";
    return false;
  }

  const std::string header = response.substr(0, header_end);
  const size_t first_line_end = header.find_first_of("\r\n");
  const std::string first_line = header.substr(0, first_line_end);
  if (!StartsWith(first_line, "ICY 200") && !StartsWith(first_line, "HTTP/1.0 200") && !StartsWith(first_line, "HTTP/1.1 200")) {
    LOG(ERROR) << "NTRIP server rejected request: " << first_line;
    return false;
  }

  const size_t payload_offset = header_end + delimiter_len;
  if (payload_offset < response.size()) {
    const auto *payload = reinterpret_cast<const uint8_t *>(response.data() + static_cast<std::ptrdiff_t>(payload_offset));
    dispatchDataCallbacks(payload, response.size() - payload_offset);
  }

  LOG(INFO) << "NTRIP mountpoint attached successfully: " << mountpoint_ << " via " << host() << ':' << port();

  return true;
}

bool NtripClient::onConnected() {
  const std::string request = buildRequest();
  if (!writeAll(reinterpret_cast<const uint8_t *>(request.data()), request.size(), kIoTimeoutMs)) {
    LOG(ERROR) << "Failed to send NTRIP request to " << host() << ':' << port();
    return false;
  }

  std::string response;
  if (!readUntil(response, "\r\n\r\n", kMaxNtripHeaderBytes, kNtripHandshakeTimeoutMs) && !readUntil(response, "\n\n", kMaxNtripHeaderBytes, kNtripHandshakeTimeoutMs)) {
    LOG(ERROR) << "Timed out waiting for NTRIP handshake response from " << host() << ':' << port();
    return false;
  }

  return parseResponse(std::move(response));
}
