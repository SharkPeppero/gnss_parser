#ifndef GNSS_COMM_UTILITY_SATELLITE_HPP_
#define GNSS_COMM_UTILITY_SATELLITE_HPP_

#include <cstdint>
#include <string>
#include "gnss_comm/types/observation.hpp"

// 卫星编号、RINEX 标识与观测频率查询。
namespace gnss_comm {

/**
 * @brief 星座 SYS_* 与星座内 PRN 转为内部编号，失败返回 0。
 * @param sys 单个星座标识 SYS_*，不是多个系统的位或组合。
 * @param prn 星座内编号，使用本工程规定的编号范围。
 * @return 内部统一卫星编号；不支持的星座或超范围编号返回 0。
 */
uint32_t sat_no(uint32_t sys, uint32_t prn);

/**
 * @brief 内部编号转为星座；prn 可为空，否则输出星座内编号。
 * @param sat 工程内部卫星编号。
 * @param[out] prn 可为空；非空时写入星座内编号。
 * @return 星座 SYS_*；无效卫星返回 SYS_NONE。
 */
uint32_t satsys(uint32_t sat, uint32_t *prn);

/**
 * @brief 内部编号转为 RINEX 卫星标识字符串。
 * @param sat_no 内部卫星编号。
 * @return 例如 G01、C06；无法映射时返回空字符串并记录警告。
 */
std::string sat2str(uint32_t sat_no);

/**
 * @brief RINEX 卫星标识转为内部编号，失败返回 0。
 * @param sat_str RINEX 星座字符与两位卫星编号，例如 G01。
 * @return 内部编号；不支持或编号越界时返回 0。
 * @note 数字字段通过 std::stoul 解析，非数字输入可能抛出异常。
 */
uint32_t str2sat(const std::string &sat_str);

/**
 * @brief 查找观测中的 L1 类频率（Hz）；可输出向量下标，不存在时返回负值。
 * @param obs 有效的单星观测指针。
 * @param[out] l1_idx 可为空；非空时输出首个匹配项下标，未找到时为 -1。
 * @return 匹配频率（Hz），未找到时为 -1.0。
 * @note 当前北斗分支匹配 B1I，不是 B1C；GLONASS 匹配 G1 的频道频率范围。
 */
double L1_freq(const ObsPtr &obs, int *l1_idx);

} // namespace gnss_comm

#endif // GNSS_COMM_UTILITY_SATELLITE_HPP_
