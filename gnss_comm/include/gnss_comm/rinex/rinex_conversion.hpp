#ifndef GNSS_COMM_RINEX_CONVERSION_HPP_
#define GNSS_COMM_RINEX_CONVERSION_HPP_

#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <map>
#include <string>
#include <vector>

#include "gnss_comm/types/ephemeris.hpp"
#include "gnss_comm/types/observation.hpp"

// RINEX 文件与工程内部星历、观测及电离层参数之间的转换接口。
// 导航相关实现位于 src/rinex/navigation.cpp，观测相关实现位于 src/rinex/observation.cpp。
namespace gnss_comm {

// 1. 导航文件读取
// 读取导航星历，按工程内部卫星编号组织记录；一颗卫星可以对应多个参考时刻的星历。
void rinex2ephems(const std::string &rinex_filepath, std::map<uint32_t, std::vector<EphemBasePtr>> &sat2ephem);

// 提取 GPSA/GPSB 电离层模型参数，依次为 alpha 的 4 项和 beta 的 4 项。
// 当前实现要求 RINEX 3.04；iono_params 为输出向量，不是 TEC 网格。
void rinex2iono_params(const std::string &rinex_filepath, std::vector<double> &iono_params);

// 2. 观测文件读写
// 读取 RINEX 3.xx 观测文件（Hatanaka 压缩需先 crx2rnx）；外层向量对应历元，内层对应各颗卫星观测。
// 每个频段按优先级只保留一个信号并读取 LLI；GLONASS 频率按头文件频道号修正。
// 事件标志 0/1 的历元作为观测读取，2~6 的事件与周跳记录被跳过；结果追加至 rinex_meas。
void rinex2obs(const std::string &rinex_filepath, std::vector<std::vector<ObsPtr>> &rinex_meas);

// RINEX 观测文件头中与测站相关的字段。
struct RinexObsHeader {
  std::string marker_name;
  std::string antenna_type;                                   // 20 字符：天线型号(16) + 天线罩(4)，与 ANTEX TYPE 字段格式一致
  Eigen::Vector3d approx_pos{Eigen::Vector3d::Zero()};        // APPROX POSITION XYZ，标石点 ECEF，m
  Eigen::Vector3d antenna_delta_hen{Eigen::Vector3d::Zero()}; // ANTENNA: DELTA H/E/N，天线参考点(ARP)相对标石，m
};

// 只读取观测文件头；读到 END OF HEADER 返回 true。
bool rinex2obs_header(const std::string &rinex_filepath, RinexObsHeader &header);

// 读取 ANTEX 中接收机天线的相位中心偏移（PCO，相对 ARP）；键为 ANTEX 频率代码（G01、G02、R01、E01、C02 …），
// 值为 {北, 东, 天}，m。先按型号+天线罩完全匹配，找不到再取天线罩为 NONE 的条目；未读取 PCV。
bool antex2rcv_pco(const std::string &antex_filepath, const std::string &antenna_type, std::map<std::string, Eigen::Vector3d> &freq2pco_neu);

// RINEX 信号代码工具，RTCM MSM 解码复用同一套编码与选择优先级。
// band_attr 为 "1C"、"2W" 这类频段+属性，返回 CODE_*，未知返回 CODE_NONE。
uint8_t rinex_obs_code(const std::string &band_attr);
// 同一频段多个信号的优先级，越小越优先；sys_char 为 'G'/'R'/'E'/'C'/'J'。
int rinex_attr_rank(uint8_t sys_char, char band, char attr);

// 将多历元观测写入 RINEX 文件；沿用实验性实现，不保证覆盖全部信号和版本。
void obs2rinex(const std::string &rinex_filepath, const std::vector<std::vector<ObsPtr>> &gnss_meas);

// 3. 待实现的导航写出接口
// 以下接口目前只有声明，尚无实现；直接调用会缺少链接定义。
// 预留：将按卫星编号组织的星历写入导航文件。
void ephems2rinex(const std::string &rinex_filepath, const std::map<uint32_t, std::vector<EphemBasePtr>> &sat2ephem);

// 预留：电离层参数写出；保留原拼写 parmas2rinex，返回字符串的语义尚待实现时明确。
std::string parmas2rinex(const std::string &rinex_filepath, const std::vector<double> &iono_params);

} // namespace gnss_comm

#endif // GNSS_COMM_RINEX_CONVERSION_HPP_
