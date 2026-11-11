#ifndef GNSS_COMM_CONSTANTS_SOLUTION_HPP_
#define GNSS_COMM_CONSTANTS_SOLUTION_HPP_

// 位置/速度解类型及 PVT 迭代参数；PV_* 数值为既有协议编码。

namespace gnss_comm {

// 1. 历史解状态定义（保留为注释，当前未启用）
// #define SOL_COMPUTED            0 // 解已计算
// #define SOL_INSUFFICIENT_OBS    1 // 观测数量不足
// #define NO_CONVERGENCE          2 // 未收敛
// #define COV_TRACE               4 // 协方差迹相关状态，具体判据需结合原协议

// 历史扩展解状态/电离层改正方式（保留为注释，当前未启用）
// #define EXT_NONE                0x00 // 无扩展状态
// #define EXT_RTK                 0x01 // RTK 相关标志
// #define EXT_IONO_BRDC           0x02 // 广播电离层模型改正
// #define EXT_IONO_SBAS           0x04 // 星基增强电离层改正
// #define EXT_IONO_IFLC           0x06 // 无电离层线性组合
// #define EXT_IONO_DIFF           0x08 // 差分电离层改正

// 2. 位置/速度解类型
// PV_* 是既有接收机协议编码，不是按精度排序的等级，也不是可按位组合的标志。
// FLOAT 表示模糊度按实数估计；INT/FIXED 表示整数模糊度已固定。
// 宽巷/窄巷是载波观测组合；类型编码本身不保证实际定位精度。
// 协议参考：https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm
#define PV_NONE 0 // 无有效解
#define PV_FIXED_POS 1 // 固定位置模式；不是 RTK 整数固定解
#define PV_FIXED_HEIGHT 2 // 固定高度模式
#define PV_DOPPLER_VELOCITY 8 // 通过多普勒观测计算速度
#define PV_SINGLE 16 // 单点定位解
#define PV_PSR_DIFF 17 // 伪距差分定位解
#define PV_WAAS 18 // 使用 SBAS 星基增强改正的解，沿用 WAAS 名称
#define PV_L1_FLOAT 32 // L1 载波相位模糊度浮点解
#define PV_IONO_FREE_FLOAT 33 // 无电离层组合模糊度浮点解
#define PV_NARROW_FLOAT 34 // 窄巷组合模糊度浮点解
#define PV_L1_INT 48 // L1 载波相位整数模糊度固定解
#define PV_WIDE_INT 49 // 宽巷组合整数模糊度固定解
#define PV_NARROW_INT 50 // 窄巷组合整数模糊度固定解
#define PV_INS 52 // 惯性导航相关解，沿用历史命名；编码 52 的细分含义需按接收机协议版本确认
#define PV_INS_PSR_SP 53 // 惯导与 GNSS 单点定位组合解
#define PV_INS_PSR_DIFF 54 // 惯导与伪距差分定位组合解
#define PV_INS_RTK_FLOAT 55 // 惯导与 RTK 浮点解组合
#define PV_INS_RTK_FIXED 56 // 惯导与 RTK 整数固定解组合

// 3. PVT（位置、速度、时间）求解的迭代控制
// 当前伪距定位和多普勒测速均比较状态增量 dx 的范数 dx.norm()。
// 阈值对应各求解器的状态增量单位，不是统一的定位精度指标或时间误差阈值。
#define EPSILON_PVT 1e-8 // 状态增量范数的收敛阈值
#define MAX_ITER_PVT 30  // 最大迭代次数；达到上限不代表已收敛

// 4. 保留的测距相关参数
// 当前 src 中未找到使用处，原注释仅称“测距频率”；保留数值，不能据此解释为载波 Hz 或采样率。
#define RANGE_FREQ 1

} // namespace gnss_comm

#endif // GNSS_COMM_CONSTANTS_SOLUTION_HPP_
