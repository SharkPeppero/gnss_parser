#ifndef GNSS_COMM_CONSTANTS_PHYSICAL_HPP_
#define GNSS_COMM_CONSTANTS_PHYSICAL_HPP_

#include <cmath>

// GNSS 计算使用的物理常量、地球模型参数及角度换算系数。
// 不同系统的模型参数略有差异，应按对应算法选取，不能仅因数值接近而混用。
namespace gnss_comm {

// 1. 真空光速
// 用于传播时间与距离的换算（距离 = 光速 × 时间），以及载波波长计算（波长 = 光速 / 频率）。
#define LIGHT_SPEED 2.99792458e8 // 真空光速 c，单位：m/s；不是某个星座专有的常量

// 2. 地球几何参数
// 平均半径用于球体近似；椭球长半轴与偏心率描述地球椭球形状，两者不可直接替代。
#define EARTH_MEAN_RADIUS 6371009     // 地球平均半径，单位：m；沿用工程中的 IUGG 近似值
#define EARTH_SEMI_MAJOR 6378137      // WGS-84 椭球长半轴 a（赤道半径），单位：m
#define EARTH_ECCE_2 6.69437999014e-3 // WGS-84 第一偏心率的平方 e² = (a²-b²)/a²，无量纲；b 为短半轴
#define EARTH_SEMI_MAJOR_GLO 6378136.0 // GLONASS 轨道模型使用的地球赤道半径，单位：m

// 3. 地球自转角速度
// 描述地固坐标系随地球旋转的速率，用于卫星轨道及地球自转相关计算；不是卫星的轨道角速度。
#define EARTH_OMG_GPS 7.2921151467e-5 // GPS/Galileo 模型使用的地球自转角速度，单位：rad/s
#define EARTH_OMG_GLO 7.2921150000e-5 // GLONASS 模型使用的地球自转角速度，单位：rad/s
#define EARTH_OMG_BDS 7.2921150000e-5 // 北斗模型使用的地球自转角速度，单位：rad/s

// 4. 地球引力参数
// μ = G × M，其中 G 为万有引力常数，M 为地球质量；MU_* 不是单独的 G。
// 用于计算卫星轨道运动，单位均为 m³/s²。
#define MU_GPS 3.9860050000e14 // GPS 模型的地球引力参数
#define MU 3.9860044180e14     // Galileo/北斗模型的地球引力参数；保留既有名称 MU
#define MU_GLO 3.9860044000e14 // GLONASS（PZ-90 模型）的地球引力参数

// 5. 轨道摄动与相对论钟差改正
// J2 描述地球非球形引力场的主要二阶带谐项，用于 GLONASS 轨道积分中的摄动加速度。
#define J2_GLO 1.0826257E-3 // 地球引力场二阶带谐系数，无量纲
// F = -2*sqrt(μ)/c²；配合轨道偏心率 e、长半轴 a 和偏近点角 E，计算 Δt = F*e*sqrt(a)*sin(E)。
// 注意：这里的 e 是卫星轨道偏心率，不是上方地球椭球偏心率；F_REL 本身不是完整钟差。
#define F_REL -4.442807633e-10 // 相对论钟差改正系数，单位：s/√m；保留原工程取值

// 6. 角度单位换算
#define R2D (180.0 / M_PI)     // 弧度转度：角度[°] = 角度[rad] × R2D
#define D2R (M_PI / 180.0)     // 度转弧度：角度[rad] = 角度[°] × D2R
#define SC2RAD 3.1415926535898 // 半周单位转弧度：1 半周 = 180° = π rad；沿用导航电文换算值

} // namespace gnss_comm

#endif // GNSS_COMM_CONSTANTS_PHYSICAL_HPP_
