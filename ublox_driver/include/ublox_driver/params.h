/**
 * @brief 解析驱动配置文件并集中管理运行时参数。
 */
#ifndef UBLOX_DRIVER_PARAMS_H_
#define UBLOX_DRIVER_PARAMS_H_

#include <gnss_comm/gnss_constant.hpp>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

struct RcvConfigRecord {
  RcvConfigRecord() = default;

  RcvConfigRecord(std::string name, std::string value) : key_name(std::move(name)), value_text(std::move(value)) {}

  RcvConfigRecord(uint32_t key, std::vector<uint8_t> value) : key_id(key), value_bytes(std::move(value)), use_raw_key_value(true) {}

  std::string key_name;
  std::string value_text;
  uint32_t key_id = 0;
  std::vector<uint8_t> value_bytes;
  bool use_raw_key_value = false;
};

struct UbloxDriverParams {
  bool enable_serial = true;
  std::string serial_port;
  long serial_baud_rate = 921600;

  bool enable_rtcm_tcp_loopback = false;
  uint64_t rtcm_tcp_port = 0;

  bool enable_ntrip_rtcm = false;
  std::string ntrip_server;
  uint64_t ntrip_port = 0;
  std::string ntrip_mountpoint;
  std::string ntrip_username;
  std::string ntrip_password;
  uint32_t raw_observation_system_mask = SYS_GPS | SYS_BDS;
  uint32_t ephemeris_system_mask = SYS_GPS | SYS_BDS;

  bool enable_config_bring = false;
  std::string receiver_config_path;
  std::vector<RcvConfigRecord> receiver_configs;
};

constexpr uint32_t kUbxMsgHeaderLen = 6;
constexpr uint32_t kIoTimeoutMs = 50;

UbloxDriverParams LoadUbloxDriverParams(const std::string &driver_config_file, const std::string &receiver_config_override = "");

#endif
