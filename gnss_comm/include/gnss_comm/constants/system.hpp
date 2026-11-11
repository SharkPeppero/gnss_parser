#ifndef GNSS_COMM_CONSTANTS_SYSTEM_HPP_
#define GNSS_COMM_CONSTANTS_SYSTEM_HPP_

// 导航系统位掩码、卫星编号范围和系统索引。SYS_* 可按位组合。
#include <cstdint>
#include <map>

namespace gnss_comm {

/// GNSS的系统编号
#define SYS_NONE 0x00 /* navigation system: none */
#define SYS_GPS 0x01  /* navigation system: GPS */
#define SYS_SBS 0x02  /* navigation system: SBAS */
#define SYS_GLO 0x04  /* navigation system: GLONASS */
#define SYS_GAL 0x08  /* navigation system: Galileo */
#define SYS_QZS 0x10  /* navigation system: QZSS */
#define SYS_BDS 0x20  /* navigation system: BeiDou */
#define SYS_IRN 0x40  /* navigation system: IRNSS */
#define SYS_LEO 0x80  /* navigation system: LEO */
#define SYS_ALL 0xFF  /* navigation system: all */

/// RINEX 星座字符映射；
/* System identifier to system code */
const std::map<uint8_t, uint32_t> char2sys = {
    {'G', SYS_GPS}, //
    {'C', SYS_BDS}, //
    {'R', SYS_GLO}, //
    {'E', SYS_GAL}, //
    {'J', SYS_QZS}, //
};

/* System map to index */
const std::map<uint32_t, uint32_t> sys2idx = {
    {SYS_GPS, 0}, //
    {SYS_GLO, 1}, //
    {SYS_GAL, 2}, //
    {SYS_BDS, 3}, //
    {SYS_QZS, 0}, //
};

/// PRN伪随机噪声码编号
// GPS PRN范围以及有效卫星数量
#define MIN_PRN_GPS 1                             /* min satellite PRN number of GPS */
#define MAX_PRN_GPS 32                            /* max satellite PRN number of GPS */
#define N_SAT_GPS (MAX_PRN_GPS - MIN_PRN_GPS + 1) /* number of GPS satellites */

// GLONASS PRN范围以及有效卫星数量
#define MIN_PRN_GLO 1                             /* min satellite slot number of GLONASS */
#define MAX_PRN_GLO 27                            /* max satellite slot number of GLONASS */
#define N_SAT_GLO (MAX_PRN_GLO - MIN_PRN_GLO + 1) /* number of GLONASS satellites */

// Galileo PRN范围以及有效卫星数量
#define MIN_PRN_GAL 1                             /* min satellite PRN number of Galileo */
#define MAX_PRN_GAL 38                            /* max satellite PRN number of Galileo */
#define N_SAT_GAL (MAX_PRN_GAL - MIN_PRN_GAL + 1) /* number of Galileo satellites */

// BDS PRN范围以及有效卫星数量
#define MIN_PRN_BDS 1                             /* min satellite sat number of BeiDou */
#define MAX_PRN_BDS 63                            /* max satellite sat number of BeiDou */
#define N_SAT_BDS (MAX_PRN_BDS - MIN_PRN_BDS + 1) /* number of BeiDou satellites */
// 北斗导航系统中的混合星座
//  GEO：地球静止轨道        15颗  |
//    GEO卫星相对地面几乎静止，长期提供的几何方向变化很小，
//    可以参与伪距、载波相位定位，但是几何特性可能影响模糊度估计和卫星筛选策略。
//  IGSO：倾斜地球同步轨道    12颗
//  MEO：中圆地球轨道        32颗
#define MAX_PRN_BDS_GEO_LOW 5   /* BeiDou GEO PRN: 1~5 (BDS-2) */
#define MIN_PRN_BDS_GEO_HIGH 59 /* BeiDou GEO PRN: 59~63 (BDS-3) */
#define IS_BDS_GEO(prn) ((prn) <= MAX_PRN_BDS_GEO_LOW || (prn) >= MIN_PRN_BDS_GEO_HIGH)

// QZS PRN范围以及有效卫星数量
#define MIN_PRN_QZS 1                             /* min satellite PRN number of QZSS (RINEX Jnn, PRN-192) */
#define MAX_PRN_QZS 10                            /* max satellite PRN number of QZSS */
#define N_SAT_QZS (MAX_PRN_QZS - MIN_PRN_QZS + 1) /* number of QZSS satellites */

/// 支持最大的卫星数量
#define MAX_SAT (N_SAT_GPS + N_SAT_GLO + N_SAT_GAL + N_SAT_BDS + N_SAT_QZS)

} // namespace gnss_comm

#endif // GNSS_COMM_CONSTANTS_SYSTEM_HPP_
