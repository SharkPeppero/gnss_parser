#include "gnss_comm/rinex/rinex_conversion.hpp"
#include <fstream>

namespace gnss_comm {

namespace {

// ANTEX / RINEX 的天线类型字段固定 20 字符，行尾空格可能被截掉。
std::string pad20(std::string str) {
  str.resize(20, ' ');
  return str;
}

} // namespace

bool antex2rcv_pco(const std::string &antex_filepath, const std::string &antenna_type, std::map<std::string, Eigen::Vector3d> &freq2pco_neu) {
  std::ifstream antex_file(antex_filepath);
  if (!antex_file)
    return false;
  const std::string type_exact = pad20(antenna_type);
  const std::string type_none = type_exact.substr(0, 16) + "NONE";

  std::map<std::string, Eigen::Vector3d> exact, none;
  std::map<std::string, Eigen::Vector3d> *current = nullptr; // 当前天线块匹配上时指向对应结果，否则为空（含卫星天线）
  std::string freq;
  std::string line;
  while (std::getline(antex_file, line)) {
    if (line.size() <= 60)
      continue;
    const std::string label = line.substr(60);
    if (label.find("START OF ANTENNA") == 0) {
      current = nullptr;
    } else if (label.find("TYPE / SERIAL NO") == 0) {
      const std::string type = pad20(line.substr(0, 20));
      current = type == type_exact ? &exact : type == type_none ? &none : nullptr;
    } else if (label.find("START OF FREQUENCY") == 0) {
      freq = line.substr(3, 3);
    } else if (label.find("NORTH / EAST / UP") == 0) {
      // 3F10.2，单位 mm
      if (current && !freq.empty())
        (*current)[freq] = Eigen::Vector3d(std::stod(line.substr(0, 10)), std::stod(line.substr(10, 10)), std::stod(line.substr(20, 10))) * 1e-3;
    } else if (label.find("END OF FREQUENCY") == 0) {
      freq.clear();
    } else if (label.find("END OF ANTENNA") == 0) {
      if (!exact.empty())
        break;
      current = nullptr;
    }
  }
  freq2pco_neu = exact.empty() ? none : exact;
  return !freq2pco_neu.empty();
}

} // namespace gnss_comm
