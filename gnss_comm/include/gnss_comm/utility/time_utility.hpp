#ifndef GNSS_COMM_UTILITY_TIME_HPP_
#define GNSS_COMM_UTILITY_TIME_HPP_

#include <cstdint>
#include <vector>
#include "gnss_comm/types/time.hpp"

// 时间表示、时间系统换算与时间运算；gtime_t 本身不带时间系统标签。
namespace gnss_comm {

/**
 * @brief 年月日时分秒数组转为时间容器，不自动转换时间系统。
 * @param ep 六元素数组 {年,月,日,时,分,秒}，秒可带小数。
 * @return 整数秒与小数秒分离的时间；当前实现对超出 1970~2099 的年份或非法月份返回零值。
 * @note 不执行时区转换；实际日期范围还受平台 time_t 表示能力限制。
 */
gtime_t epoch2time(const double *ep);

/**
 * @brief 时间容器转为六元素年月日时分秒数组。
 * @param t 输入时间，时间系统由调用方约定。
 * @param[out] ep 至少可写入 6 个 double，顺序为年、月、日、时、分、秒。
 */
void time2epoch(gtime_t t, double *ep);

/**
 * @brief GPS 周数与周内秒转为 GPST 时间容器。
 * @param week GPS 完整周数，以 1980-01-06 为起始历元。
 * @param tow 周内秒，可含小数；不是毫秒。
 * @return GPST 时间表示，不执行截断周号的周翻转恢复。
 */
gtime_t gpst2time(uint32_t week, double tow);

/**
 * @brief GPST 时间转为周内秒，可通过 week 输出 GPS 周数。
 * @param t GPST 输入时间。
 * @param[out] week 可为空；非空时写入 GPS 周数。
 * @return GPS 周内秒，单位 s。
 */
double time2gpst(gtime_t t, uint32_t *week);

/**
 * @brief 按 Galileo 起始历元将周数与周内秒转为时间容器。
 * @param week Galileo 周数，以 1999-08-22 为起始历元。
 * @param tow GST 周内秒，可含小数。
 * @return GST 时间表示；本函数只按对应起始历元构造时间。
 */
gtime_t gst2time(int week, double tow);

/**
 * @brief 按 Galileo 起始历元提取周数与周内秒。
 * @param t GST 输入时间。
 * @param[out] week 可为空；非空时写入 Galileo 周数。
 * @return GST 周内秒，单位 s。
 */
double time2gst(gtime_t t, int *week);

/**
 * @brief 按北斗起始历元解析周数与周内秒；不自动加 14 秒转为 GPST。
 * @param week 北斗周数，以 2006-01-01 为起始历元。
 * @param tow BDT 周内秒，可含小数。
 * @return BDT 时间表示，不是 GPST；需要 GPST 时应另行转换。
 */
gtime_t bdt2time(int week, double tow);

/**
 * @brief 从 BDT 时间提取北斗周数与周内秒，输入需已处于 BDT。
 * @param t BDT 输入时间，不能直接传入尚未转换的 GPST。
 * @param[out] week 可为空；非空时写入北斗周数。
 * @return BDT 周内秒，单位 s。
 */
double time2bdt(gtime_t t, int *week);

/**
 * @brief 依据闰秒表将 GPST 转为 UTC。
 * @param t GPST 输入时间。
 * @return UTC 时间表示。
 * @note 使用项目中的闰秒表，不自动获取外部更新；不包含亚秒级时间系统精细改正。
 */
gtime_t gpst2utc(gtime_t t);

/**
 * @brief 依据闰秒表将 UTC 转为 GPST。
 * @param t UTC 输入时间。
 * @return GPST 时间表示；闰秒偏移取决于日期，不能对所有历史数据固定加 18 秒。
 */
gtime_t utc2gpst(gtime_t t);

/**
 * @brief 年月日时分秒转为儒略日。
 * @param datetime 六元素年月日时分秒向量。
 * @return 儒略日，单位为天，包含日内小数。
 * @note 当前实现检查年份为 1901~2099；不执行时间系统转换。
 */
double julian_day(std::vector<double> datetime);

/**
 * @brief 输入为 GPST 的年月日时分秒，返回对应的 GPST-UTC 整数秒偏移。
 * @note 当前实现未匹配到记录时返回 uint32_t(-1)，调用方应识别该无效值。
 * @param datetime GPST 年月日时分秒，共 6 个元素。
 * @return GPST-UTC 的整数秒偏移，未匹配到表项时为 uint32_t(-1)，不是负的有符号整数。
 */
uint32_t leap_seconds_from_GPS_epoch(std::vector<double> datetime);

/**
 * @brief 返回年内日序，含一天内的小数部分。
 * @param time 输入时间。
 * @return 年内日序，1 月 1 日零时为 1.0；一天内时间折算为小数。
 */
double time2doy(gtime_t time);

/**
 * @brief 返回 t1 - t2，单位秒；两者应使用同一时间系统。
 * @param t1 被减时间。
 * @param t2 减数时间，需与 t1 属于同一时间系统。
 * @return 带符号的时间差，单位 s。
 */
double time_diff(gtime_t t1, gtime_t t2);

/**
 * @brief 在原时间上加 sec 秒并规范化小数秒，不改变时间系统。
 * @param t 基准时间。
 * @param sec 增加的秒数，可为负数或小数。
 * @return 新时间，不修改输入 t。
 */
gtime_t time_add(gtime_t t, double sec);

/**
 * @brief 合并整数秒和小数秒为 double，时间基准保持不变。
 * @param time 输入的整数秒与小数秒时间。
 * @return 秒数，不是 ROS 使用的纳秒；合并为 double 后精度受浮点表示限制。
 */
double time2sec(gtime_t time);

/**
 * @brief 将 double 秒数拆为时间容器，时间系统由调用方约定。
 * @param sec 总秒数，可带小数。
 * @return 分离整数秒和小数秒后的时间，不自动赋予 UTC 或 GPST 语义。
 */
gtime_t sec2time(const double sec);

} // namespace gnss_comm

#endif // GNSS_COMM_UTILITY_TIME_HPP_
