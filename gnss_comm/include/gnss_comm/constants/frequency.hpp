//
// Created by yjh on 2026/11/10.
//

#ifndef GNSS_COMM_CONSTANTS_FREQUENCY_HPP_
#define GNSS_COMM_CONSTANTS_FREQUENCY_HPP_

#include "gnss_comm/constants/physical.hpp"
#include <map>
#include <string>

namespace gnss_comm {
// clang-format off
// 部分定义沿用 RTKLIB 命名；所有 FREQ*/DFRQ* 的单位均为 Hz。
// 名称中的数字是历史频段标记，不是数组下标。

// 1. 多星座共用的载波频率
// 格式：导航系统 信号 | 导航系统 信号
#define FREQ1 1.57542E9  // GPS L1     | Galileo E1  | BeiDou B1C | QZSS L1
#define FREQ2 1.22760E9  // GPS L2     | QZSS L2
#define FREQ5 1.17645E9  // GPS L5     | Galileo E5a | BeiDou B2a | QZSS L5
#define FREQ6 1.27875E9  // Galileo E6 | QZSS L6 (LEX)
#define FREQ7 1.20714E9  // Galileo E5b | BeiDou B2I | BeiDou B2b
#define FREQ8 1.191795E9 // Galileo E5 (E5a+E5b，AltBOC)
#define FREQ9 2.492028E9 // NavIC/IRNSS S频段

// 2. GLONASS 专用频率
// 传统 G1/G2 采用 FDMA，实际频率 = 基准频率 + 频道号 k × 频率间隔。
#define FREQ1_GLO 1.60200E9  // GLONASS G1 | L1基准频率（1602.000 MHz）
#define FREQ2_GLO 1.24600E9  // GLONASS G2 | L2基准频率（1246.000 MHz）
#define FREQ3_GLO 1.202025E9 // GLONASS G3 | L3OC（CDMA信号）

#define FREQ4_GLO 1.600995E9 // RINEX R4 对应频率，沿用原映射值
#define FREQ6_GLO 1.248060E9 // RINEX R6 对应频率，沿用原映射值

// GLONASS G1/G2 的 FDMA 频道间隔
#define DFRQ1_GLO 0.56250E6 // G1：0.5625 MHz
#define DFRQ2_GLO 0.43750E6 // G2：0.4375 MHz

// 3. 北斗传统频段；B1I 使用 FREQ1_BDS，B1C 使用公共频率 FREQ1。
#define FREQ1_BDS 1.561098E9 // BeiDou B1I | 北斗二号/三号传统B1信号
#define FREQ2_BDS 1.20714E9  // BeiDou B2I | 北斗二号传统B2信号
#define FREQ3_BDS 1.26852E9  // BeiDou B3I | 北斗二号/三号B3信号

// 4. 工程处理配置与传统频段分组数量（不表示星座的全部信号数量）
// N_FREQ 保留既有默认值，驱动用它限制内部频率编号，并构造扩展编号。
// 修改该值需同步检查驱动编号与查表逻辑，不能直接设为星座频段总数。
#ifndef N_FREQ
#define N_FREQ 3
#endif
// 以下分组数量目前没有调用方，不控制 type2freq 的条目数或驱动支持范围。
#define N_FREQ_GPS 3 // 传统分组：L1、L2、L5
#define N_FREQ_BDS 3 // 传统分组：B1I、B2I、B3I
#define N_FREQ_GLO 2 // 传统 FDMA 分组：G1、G2；不包含 G3 等信号
#define MAXFREQ 7   // 下方 lam_carr 的表长，不是单颗卫星观测数量

// 5. RINEX 频率映射
// RINEX 星座+频段到频率（Hz）的映射；GLONASS G1/G2 为基准频率。
/* RINEX frequency encoding to frequency value */
const std::map<std::string, double> type2freq = {
    // GPS
    {"G1", FREQ1}, {"G2", FREQ2}, {"G5", FREQ5},
    // GLONASS：R1/R2 还需结合频道号计算实际频率
    {"R1", FREQ1_GLO}, {"R2", FREQ2_GLO}, {"R3", FREQ3_GLO},
    {"R4", FREQ4_GLO}, {"R6", FREQ6_GLO},
    // Galileo
    {"E1", FREQ1}, {"E5", FREQ5}, {"E6", FREQ6}, {"E7", FREQ7}, {"E8", FREQ8},
    // BeiDou：C1 与 C2 对应不同频率，不能统一按“B1”处理
    {"C1", FREQ1}, {"C2", FREQ1_BDS}, {"C5", FREQ5},
    {"C6", FREQ3_BDS}, {"C7", FREQ2_BDS}, {"C8", FREQ8},
    // QZSS
    {"J1", FREQ1}, {"J2", FREQ2}, {"J5", FREQ5}, {"J6", FREQ6},
};

// 6. 公共频率的派生波长（m）：lambda = c / f
// 数组下标 0~6 对应 FREQ1、FREQ2、FREQ5、FREQ6、FREQ7、FREQ8、FREQ9。
// 保持现有顺序，驱动通过 lam_carr[sid - 1] 访问；sid 是内部编号。
// 此表不包含北斗 B1I/B3I 或 GLONASS FDMA 实际频道的波长。
static constexpr double lam_carr[MAXFREQ] = {
    LIGHT_SPEED / FREQ1, // [0]
    LIGHT_SPEED / FREQ2, // [1]
    LIGHT_SPEED / FREQ5, // [2]
    LIGHT_SPEED / FREQ6, // [3]
    LIGHT_SPEED / FREQ7, // [4]
    LIGHT_SPEED / FREQ8, // [5]
    LIGHT_SPEED / FREQ9, // [6]
};
// clang-format on
} // namespace gnss_comm

#endif // GNSS_COMM_CONSTANTS_FREQUENCY_HPP_
