//
// Created by yjh on 2026/4/30.
//
#include "ublox_driver/params.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "ublox_driver/logging.hpp"

namespace {

std::string ToLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

std::string Trim(const std::string &value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }

  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

bool ParseBoolScalar(const std::string &raw_value, const std::string &field_name) {
  const std::string normalized = ToLower(Trim(raw_value));
  if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on") {
    return true;
  }
  if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off") {
    return false;
  }

  throw std::runtime_error("Invalid boolean config value for '" + field_name + "': " + raw_value);
}

YAML::Node FindFirstNode(const YAML::Node &root, std::initializer_list<const char *> keys) {
  if (!root.IsMap()) {
    return YAML::Node();
  }

  for (auto it = root.begin(); it != root.end(); ++it) {
    if (!it->first.IsScalar()) {
      continue;
    }

    const std::string current_key = it->first.as<std::string>();
    for (const char *key : keys) {
      if (current_key == key) {
        return it->second;
      }
    }
  }

  return YAML::Node();
}

template <typename T> T ReadRequiredAny(const YAML::Node &root, std::initializer_list<const char *> keys, const std::string &field_name) {
  const YAML::Node value = FindFirstNode(root, keys);
  if (!value.IsDefined()) {
    throw std::runtime_error("Missing required config field: " + field_name);
  }
  return value.as<T>();
}

template <typename T> T ReadOptionalAny(const YAML::Node &root, std::initializer_list<const char *> keys, const T &default_value) {
  const YAML::Node value = FindFirstNode(root, keys);
  if (!value.IsDefined()) {
    return default_value;
  }
  return value.as<T>();
}

template <typename T> T ReadRequired(const YAML::Node &root, const std::string &key) { return ReadRequiredAny<T>(root, {key.c_str()}, key); }

bool ReadBoolAny(const YAML::Node &root, std::initializer_list<const char *> keys, bool default_value, bool required = false, const std::string &field_name = "");

bool ReadBool(const YAML::Node &root, const std::string &key, bool default_value, bool required = false) { return ReadBoolAny(root, {key.c_str()}, default_value, required, key); }

bool ReadBoolNode(const YAML::Node &value, const std::string &field_name) {
  if (!value.IsDefined()) {
    return false;
  }

  if (!value.IsScalar()) {
    throw std::runtime_error("Config field '" + field_name + "' must be a scalar boolean value.");
  }

  return ParseBoolScalar(value.Scalar(), field_name);
}

bool ReadBoolAny(const YAML::Node &root, std::initializer_list<const char *> keys, bool default_value, bool required, const std::string &field_name) {
  const YAML::Node value = FindFirstNode(root, keys);
  if (!value.IsDefined()) {
    if (required) {
      throw std::runtime_error("Missing required config field: " + field_name);
    }
    return default_value;
  }
  return ReadBoolNode(value, field_name.empty() ? *(keys.begin()) : field_name);
}

uint8_t GetValueByteCount(uint32_t key_id) {
  const uint8_t value_size = static_cast<uint8_t>((key_id >> 28) & 0x07);
  return value_size > 2 ? static_cast<uint8_t>(1U << (value_size - 2)) : 1U;
}

std::vector<uint8_t> ParseHexBytes(const std::string &hex_payload, const std::string &config_path, size_t line_number) {
  std::stringstream ss(hex_payload);
  std::string token;
  std::vector<uint8_t> bytes;
  while (ss >> token) {
    const unsigned long value = std::stoul(token, nullptr, 16);
    if (value > 0xFF) {
      throw std::runtime_error("Hex byte out of range in " + config_path + ":" + std::to_string(line_number));
    }
    bytes.push_back(static_cast<uint8_t>(value));
  }
  return bytes;
}

std::vector<RcvConfigRecord> ParseUCenterConfig(const std::string &receiver_config_path) {
  std::ifstream ifs(receiver_config_path);
  if (!ifs.good()) {
    throw std::runtime_error("Unable to open receiver config file: " + receiver_config_path);
  }

  std::vector<RcvConfigRecord> records;
  std::string line;
  size_t line_number = 0;
  while (std::getline(ifs, line)) {
    ++line_number;
    const std::string trimmed_line = Trim(line);
    if (trimmed_line.empty()) {
      continue;
    }

    const size_t separator = trimmed_line.find(" - ");
    if (separator == std::string::npos) {
      continue;
    }

    const std::string message_name = trimmed_line.substr(0, separator);
    if (message_name != "CFG-VALGET" && message_name != "CFG-VALSET") {
      continue;
    }

    const std::vector<uint8_t> bytes = ParseHexBytes(trimmed_line.substr(separator + 3), receiver_config_path, line_number);
    if (bytes.size() < 8 || bytes[0] != 0x06 || (bytes[1] != 0x8A && bytes[1] != 0x8B)) {
      throw std::runtime_error("Unsupported CFG line format in " + receiver_config_path + ":" + std::to_string(line_number));
    }

    const uint16_t payload_len = static_cast<uint16_t>(bytes[2]) | static_cast<uint16_t>(bytes[3]) << 8;
    if (bytes.size() != static_cast<size_t>(payload_len) + 4U) {
      throw std::runtime_error("Payload length mismatch in " + receiver_config_path + ":" + std::to_string(line_number));
    }
    if (payload_len < 4U) {
      continue;
    }

    size_t offset = 8;
    const size_t payload_end = static_cast<size_t>(payload_len) + 4U;
    while (offset + 4U <= payload_end) {
      const uint32_t key_id =
          static_cast<uint32_t>(bytes[offset]) | static_cast<uint32_t>(bytes[offset + 1]) << 8 | static_cast<uint32_t>(bytes[offset + 2]) << 16 | static_cast<uint32_t>(bytes[offset + 3]) << 24;
      offset += 4;

      const uint8_t value_byte_count = GetValueByteCount(key_id);
      if (offset + value_byte_count > payload_end) {
        throw std::runtime_error("Truncated key/value payload in " + receiver_config_path + ":" + std::to_string(line_number));
      }

      records.emplace_back(key_id, std::vector<uint8_t>(bytes.begin() + static_cast<long>(offset), bytes.begin() + static_cast<long>(offset + value_byte_count)));
      offset += value_byte_count;
    }

    if (offset != payload_end) {
      throw std::runtime_error("Unexpected trailing bytes in " + receiver_config_path + ":" + std::to_string(line_number));
    }
  }

  if (records.empty()) {
    throw std::runtime_error("No CFG-VALGET/CFG-VALSET records found in " + receiver_config_path);
  }
  return records;
}

std::string ResolvePath(const std::string &path, const std::string &base_dir) {
  if (path.empty()) {
    throw std::runtime_error("Config path value cannot be empty.");
  }

  std::string expanded_path = path;
  if (expanded_path[0] == '~') {
    const char *home_dir = std::getenv("HOME");
    if (!home_dir) {
      throw std::runtime_error("HOME is not set, cannot expand '~' in path: " + path);
    }
    expanded_path.replace(0, 1, home_dir);
  }

  std::filesystem::path resolved_path(expanded_path);
  if (resolved_path.is_relative()) {
    resolved_path = std::filesystem::path(base_dir) / resolved_path;
  }

  resolved_path = resolved_path.lexically_normal();
  if (!std::filesystem::exists(resolved_path)) {
    throw std::runtime_error("Configured path does not exist: " + resolved_path.string());
  }

  return resolved_path.string();
}

std::string MaskSecret(const std::string &value) {
  if (value.empty()) {
    return "<empty>";
  }
  if (value.size() <= 2) {
    return std::string(value.size(), '*');
  }
  return value.substr(0, 1) + std::string(value.size() - 2, '*') + value.substr(value.size() - 1);
}

uint32_t ParseSupportedGnssSystemName(const std::string &raw_value,
                                      const std::string &field_name) {
  const std::string normalized = ToLower(Trim(raw_value));
  if (normalized == "gps") {
    return SYS_GPS;
  }
  if (normalized == "bds" || normalized == "beidou") {
    return SYS_BDS;
  }

  throw std::runtime_error("Config field '" + field_name +
                           "' currently only supports GPS and BDS, but got: " +
                           raw_value);
}

uint32_t ParseGnssSystemMask(const YAML::Node &root,
                             const std::string &key,
                             uint32_t default_mask) {
  const YAML::Node value = FindFirstNode(root, {key.c_str()});
  if (!value.IsDefined()) {
    return default_mask;
  }

  uint32_t mask = 0;
  if (value.IsScalar()) {
    mask |= ParseSupportedGnssSystemName(value.as<std::string>(), key);
  } else if (value.IsSequence()) {
    for (const auto &item : value) {
      if (!item.IsScalar()) {
        throw std::runtime_error("Config field '" + key +
                                 "' must contain scalar GNSS system names.");
      }
      mask |= ParseSupportedGnssSystemName(item.as<std::string>(), key);
    }
  } else {
    throw std::runtime_error("Config field '" + key +
                             "' must be a scalar or sequence.");
  }

  if (mask == 0) {
    throw std::runtime_error("Config field '" + key +
                             "' cannot be empty.");
  }
  return mask;
}

std::string GnssSystemMaskToString(uint32_t mask) {
  std::vector<std::string> names;
  if ((mask & SYS_GPS) != 0U) {
    names.emplace_back("GPS");
  }
  if ((mask & SYS_BDS) != 0U) {
    names.emplace_back("BDS");
  }

  if (names.empty()) {
    return "<none>";
  }

  std::ostringstream oss;
  for (size_t i = 0; i < names.size(); ++i) {
    if (i != 0U) {
      oss << ',';
    }
    oss << names[i];
  }
  return oss.str();
}

void LogLoadedParams(const UbloxDriverParams &params) {
  LOG(INFO) << "Loaded ublox driver params:" << "\n"                            //
            << "  enable_serial=" << params.enable_serial << "\n"               //
            << "  serial_port=" << params.serial_port << "\n"                   //
            << "  serial_baud_rate=" << params.serial_baud_rate << "\n"         //
            << "  raw_observation_systems=" << GnssSystemMaskToString(params.raw_observation_system_mask) << "\n" //
            << "  ephemeris_systems=" << GnssSystemMaskToString(params.ephemeris_system_mask) << "\n" //
            << "  enable_config_bring=" << params.enable_config_bring << "\n"   //
            << "  receiver_config_path=" << params.receiver_config_path << "\n" //
            << "  receiver_config_count=" << params.receiver_configs.size();

  LOG(INFO) << "Loaded RTCM TCP loopback params:" << "\n"                               //
            << "  enable_rtcm_tcp_loopback=" << params.enable_rtcm_tcp_loopback << "\n" //
            << "  rtcm_tcp_port=" << params.rtcm_tcp_port;

  LOG(INFO) << "Loaded NTRIP params:" << "\n"                             //
            << "  enable_ntrip_rtcm=" << params.enable_ntrip_rtcm << "\n" //
            << "  ntrip_server=" << params.ntrip_server << "\n"           //
            << "  ntrip_port=" << params.ntrip_port << "\n"               //
            << "  ntrip_mountpoint=" << params.ntrip_mountpoint << "\n"   //
            << "  ntrip_username=" << params.ntrip_username << "\n"       //
            << "  ntrip_password=" << MaskSecret(params.ntrip_password);
}

} // namespace

UbloxDriverParams LoadUbloxDriverParams(const std::string &driver_config_file, const std::string &receiver_config_override) {
  const YAML::Node driver_config_root = YAML::LoadFile(driver_config_file);
  const std::filesystem::path config_path = std::filesystem::absolute(driver_config_file);
  const std::string config_dir = config_path.parent_path().string();

  UbloxDriverParams params;

  params.enable_serial = ReadBool(driver_config_root, "enable_serial", true, true);
  if (!params.enable_serial) {
    throw std::runtime_error("Current driver_config.yaml only supports serial input. "
                             "Please keep 'enable_serial' set to true/1.");
  }
  params.serial_port = ReadRequired<std::string>(driver_config_root, "serial_port");
  params.serial_baud_rate = ReadRequired<long>(driver_config_root, "serial_baud_rate");

  params.enable_rtcm_tcp_loopback = ReadBoolAny(driver_config_root, {"enable_rtcm_tcp_loopback"}, false);
  if (params.enable_rtcm_tcp_loopback) {
    params.rtcm_tcp_port = ReadRequiredAny<uint64_t>(driver_config_root, {"rtcm_tcp_port"}, "rtcm_tcp_port");
  }

  params.enable_ntrip_rtcm = ReadBoolAny(driver_config_root, {"enable_ntrip_rtcm"}, false);
  if (params.enable_ntrip_rtcm) {
    params.ntrip_server = ReadRequiredAny<std::string>(driver_config_root, {"ntrip_server"}, "ntrip_server");
    params.ntrip_port = ReadRequiredAny<uint64_t>(driver_config_root, {"ntrip_port"}, "ntrip_port");
    params.ntrip_mountpoint = ReadRequiredAny<std::string>(driver_config_root, {"ntrip_mountpoint"}, "ntrip_mountpoint");
    params.ntrip_username = ReadOptionalAny<std::string>(driver_config_root, {"ntrip_username"}, "");
    params.ntrip_password = ReadOptionalAny<std::string>(driver_config_root, {"ntrip_password"}, "");
  }

  params.raw_observation_system_mask = ParseGnssSystemMask(
      driver_config_root,
      "raw_observation_systems",
      SYS_GPS | SYS_BDS);
  params.ephemeris_system_mask = ParseGnssSystemMask(
      driver_config_root,
      "ephemeris_systems",
      SYS_GPS | SYS_BDS);

  if (params.enable_rtcm_tcp_loopback && params.enable_ntrip_rtcm) {
    throw std::runtime_error("RTCM input cannot enable both loopback TCP and NTRIP at the same time.");
  }

  params.enable_config_bring = ReadBool(driver_config_root, "enable_config_bring", false);
  if (params.enable_config_bring) {
    const std::string receiver_config_path = receiver_config_override.empty() ? ReadRequired<std::string>(driver_config_root, "config_path") : receiver_config_override;
    params.receiver_config_path = ResolvePath(receiver_config_path, config_dir);

    const std::string extension = ToLower(std::filesystem::path(params.receiver_config_path).extension().string());
    if (extension == ".yaml" || extension == ".yml") {
      const YAML::Node rcv_config_root = YAML::LoadFile(params.receiver_config_path);
      for (auto it = rcv_config_root.begin(); it != rcv_config_root.end(); ++it) {
        params.receiver_configs.emplace_back(it->first.as<std::string>(), it->second.as<std::string>());
      }
    } else {
      params.receiver_configs = ParseUCenterConfig(params.receiver_config_path);
    }
  }

  LogLoadedParams(params);
  return params;
}
