#ifndef GNSS_COMM_TYPES_EPHEMERIS_HPP_
#define GNSS_COMM_TYPES_EPHEMERIS_HPP_

#include <cstdint>
#include <memory>
#include "gnss_comm/types/time.hpp"

namespace gnss_comm {

// 广播星历公共字段；以基类指针统一保存不同轨道模型。
struct EphemBase {
  virtual ~EphemBase() = default; // 支持通过基类指针正确析构派生星历
  uint32_t sat{};    // 工程内部卫星编号，不是单独的星座内 PRN
  gtime_t ttr{};     // 电文发送时刻，GPST
  gtime_t toe{};     // 轨道参考时刻，GPST
  uint32_t health{}; // 卫星健康状态原始编码，解释方式取决于星座/电文
  double ura{};      // 信号测距精度指标，通常已由解析器换算为米；部分解析器用 -1 表示未知
  uint32_t iode{};   // 星历数据期号/版本标识，用于区分星历更新，不是时间戳
};
typedef std::shared_ptr<EphemBase> EphemBasePtr;

// GLONASS 广播星历：以参考时刻的位置、速度和加速度进行轨道积分。
struct GloEphem : EphemBase {
  int freqo{};             // FDMA 频道号 k，实际频率 = 基准频率 + k × 频道间隔；不是 Hz
  uint32_t age{};          // 电文中的星历龄期信息，沿用输入编码，不是卫星服役年龄
  double pos[3]{};         // 参考时刻地固坐标系位置，按 x/y/z 排列，m
  double vel[3]{};         // 参考时刻地固坐标系速度，m/s
  double acc[3]{};         // 广播的日月等摄动加速度分量，m/s²；不是包含地球引力的总加速度
  double tau_n{}, gamma{}; // 卫星钟差参数 tau_n（s）与相对频率偏差 gamma（无量纲），符号按模型使用
  double delta_tau_n{};    // L1/L2 信号间延迟参数，s
};
typedef std::shared_ptr<GloEphem> GloEphemPtr;

// GPS、Galileo、北斗、QZSS 等使用的开普勒轨道参数形式星历。
// toe/toc/ttr 在工程内部统一为 GPST；week/toe_tow 保留解析流程采用的系统周数和周内秒，
// 尤其不能把北斗周数直接当作 GPS 周数。
struct Ephem : EphemBase {
  gtime_t toc{};    // 卫星钟差多项式参考时刻，GPST
  double toe_tow{}; // 轨道参考时刻的周内秒，s
  uint32_t week{};  // 星历周数，起始历元取决于所属系统及解析流程
  uint32_t iodc{};  // 钟差数据期号/版本标识
  uint32_t code{};  // 导航电文中的码/数据源标志，按星座解释；不是 Obs::code 的统一信号编码

  // 基本轨道参数；角度在解析后以弧度保存。
  double a{};         // 轨道长半轴，m；不是电文常见的 sqrt(a)
  double e{};         // 轨道偏心率，无量纲
  double i0{};        // 参考时刻轨道倾角，rad
  double omg{};       // 近地点幅角，rad
  double omg0{};      // 升交点赤经参数，rad
  double m0{};        // 参考时刻平近点角，rad
  double delta_n{};   // 平均角速度改正量，rad/s
  double omg_dot{};   // 升交点赤经变化率，rad/s
  double i_dot{};     // 轨道倾角变化率，rad/s

  // 二次谐波改正系数：c/s 分别表示余弦项与正弦项。
  double cuc{}, cus{}; // 纬度幅角改正系数，rad
  double crc{}, crs{}; // 轨道半径改正系数，m
  double cic{}, cis{}; // 轨道倾角改正系数，rad

  // 钟差多项式 af0 + af1*dt + af2*dt²，dt 为距 toc 的秒数。
  double af0{}, af1{}, af2{}; // 钟差（s）、钟漂（s/s）、二次项系数（s/s²）
  // 信号群延迟参数，s；应按星座和信号组合使用。
  // GPS：tgd[0]=TGD；Galileo：tgd[0]=BGD E5a/E1，tgd[1]=BGD E5b/E1；
  // 北斗：tgd[0]=BGD1，tgd[1]=BGD2，沿用工程存储约定。
  double tgd[2]{};
  double a_dot{}, n_dot{}; // CNAV 扩展：长半轴变化率（m/s）、平均角速度变化率（rad/s²）
};
typedef std::shared_ptr<Ephem> EphemPtr;

} // namespace gnss_comm

#endif // GNSS_COMM_TYPES_EPHEMERIS_HPP_
