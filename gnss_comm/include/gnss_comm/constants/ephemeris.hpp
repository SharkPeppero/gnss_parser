#ifndef GNSS_COMM_CONSTANTS_EPHEMERIS_HPP_
#define GNSS_COMM_CONSTANTS_EPHEMERIS_HPP_

// 星历计算参数：时间范围检查、轨道积分、开普勒方程迭代及北斗 GEO 坐标变换。
// 本文件保存算法常量，具体卫星的轨道和钟差数据由星历结构体保存。
namespace gnss_comm {

// 1. 星历计算的时间范围阈值
// 用于检查计算时刻与星历参考时刻的时间差绝对值，单位：秒。
// 这是本工程采用的检查阈值，不等同于所有导航电文的更新周期或精度保证期限。
#define EPH_VALID_SECONDS 7200     // 非 GLONASS 星历计算的时间差阈值：7200 秒（2 小时）
#define GLO_EPH_VALID_SECONDS 1800 // GLONASS 星历计算的时间差阈值：1800 秒（30 分钟）

// 2. GLONASS 轨道数值积分
// 从星历参考时刻的状态向目标时刻积分；前向取正步长，反向取负步长。
// 最后一步若不足该长度，使用剩余时间作为步长。
#define TSTEP 60.0 // 积分步长的大小，单位：秒；不是观测采样周期

// 3. 开普勒方程迭代控制
// 通过迭代求偏近点角；相邻两次结果之差足够小，或达到次数上限时结束迭代。
#define EPSILON_KEPLER 1e-14 // 相邻两次偏近点角之差的收敛阈值，单位：rad
#define MAX_ITER_KEPLER 30   // 最大迭代次数；达到上限不表示已经收敛

// 4. 北斗 GEO（地球静止轨道）卫星坐标变换
// 在本工程的北斗 GEO 位置和速度计算中使用固定 -5° 旋转的三角函数值。
// 这是坐标变换参数，不是从星历读取的卫星实际轨道倾角；两个值均无量纲。
#define SIN_N5 -0.0871557427476582 // sin(-5°)，负号不可省略
#define COS_N5 0.9961946980917456  // cos(-5°)

// 5. 无效钟差标记
// 沿用原工程定义的特殊值，用于表达无效钟差；不是物理常量或正常钟差改正量。
#define INVALID_CLK 999999.999999

} // namespace gnss_comm

#endif // GNSS_COMM_CONSTANTS_EPHEMERIS_HPP_
