#ifndef GNSS_COMM_RINEX_CONVERSION_HPP_
#define GNSS_COMM_RINEX_CONVERSION_HPP_

#include <cstdint>
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
// 读取 RINEX 3.04 观测文件；外层向量对应历元，内层对应各颗卫星观测。
// 当前实现仅接受普通观测历元（事件标志为 0），将结果追加至 rinex_meas。
void rinex2obs(const std::string &rinex_filepath, std::vector<std::vector<ObsPtr>> &rinex_meas);

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
