#ifndef GNSS_COMM_CONSTANTS_TIME_HPP_
#define GNSS_COMM_CONSTANTS_TIME_HPP_

// 时间系统标识、历元、时间偏移和闰秒表。

namespace gnss_comm {

// 1. 时间系统标识：普通编号，不是 SYS_* 星座位掩码，不可按位组合。
#define T_SYS_GPS 0 // GPS 时间（GPST）
#define T_SYS_UTC 1 // 协调世界时（UTC）
#define T_SYS_GLO 2 // GLONASS 时间（GLOT）
#define T_SYS_GAL 3 // Galileo 时间
#define T_SYS_QZS 4 // QZSS 时间
#define T_SYS_BDS 5 // 北斗时间（BDT）

// 2. 时间单位与起始历元
#define WEEK_SECONDS 604800 // 一周的秒数，7 * 24 * 60 * 60
// GPS 起始历元的儒略日；保留历史名称 EPHCH（意为 EPOCH），避免破坏既有引用。
#define GPS_EPHCH_JD 2444244.5

// 3. 固定时间偏移（单位：秒）；转换完整时间时还需正确处理日期跨越。
#define BDT_GPST_OFFSET 14
#define BDT_TO_GPST(x) x + BDT_GPST_OFFSET
#define GPST_TO_BDT(x) x - BDT_GPST_OFFSET

#define UTC_GLOT_OFFSET 10800
#define UTC_TO_GLOT(x) x + UTC_GLOT_OFFSET
#define GLOT_TO_UTC(x) x - UTC_GLOT_OFFSET

//   导航系统 | 时间体系
//    GPS       GPST
//    BDS       BDT
//  Galileo     GST
//  Glonass    与UTC有关联
//   QZSS    与GPST保持协调的实践体系

// 4. 闰秒表：按偏移生效的 UTC 日期倒序排列。
// 每行 {年, 月, 日, 时, 分, 秒, GPST_TO_UTC}；
// 前六列供 epoch2time 使用，第七列为累计时间偏移（秒），不是该次新增闰秒数。
//  UTC = GPST  + (GPST_TO_UTC)
//  GPST = UTC - (GPST_TO_UTC)
#define MAXLEAPS 64
const double leaps[MAXLEAPS + 1][7] = {
    {2017, 1, 1, 0, 0, 0, -18},
    {2015, 7, 1, 0, 0, 0, -17},
    {2012, 7, 1, 0, 0, 0, -16},
    {2009, 1, 1, 0, 0, 0, -15},
    {2006, 1, 1, 0, 0, 0, -14},
    {1999, 1, 1, 0, 0, 0, -13},
    {1997, 7, 1, 0, 0, 0, -12},
    {1996, 1, 1, 0, 0, 0, -11},
    {1994, 7, 1, 0, 0, 0, -10},
    {1993, 7, 1, 0, 0, 0, -9},
    {1992, 7, 1, 0, 0, 0, -8},
    {1991, 1, 1, 0, 0, 0, -7},
    {1990, 1, 1, 0, 0, 0, -6},
    {1988, 1, 1, 0, 0, 0, -5},
    {1985, 7, 1, 0, 0, 0, -4},
    {1983, 7, 1, 0, 0, 0, -3},
    {1982, 7, 1, 0, 0, 0, -2},
    {1981, 7, 1, 0, 0, 0, -1},
    {0} // 结束标记
};

} // namespace gnss_comm

#endif // GNSS_COMM_CONSTANTS_TIME_HPP_
