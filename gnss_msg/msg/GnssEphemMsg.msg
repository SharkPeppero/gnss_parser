# 开普勒广播星历（GPS、Galileo、北斗、QZSS），所有时间均为 GPST

uint32 sat          # 卫星编号（由 gnss_utility.hpp 中的 sat_no 函数定义）
GnssTimeMsg ttr     # 电文发射时刻（GPST）
GnssTimeMsg toe     # 星历参考时刻（GPST），轨道外推起点
GnssTimeMsg toc     # 钟差参考时刻（GPST），钟差多项式起点
float64 toe_tow     # toe 的周内秒 [s]（北斗已换算到 GPST）
uint32 week         # toe 的周数（北斗已换算到 GPST）
uint32 iode         # 星历数据龄期
uint32 iodc         # 钟差数据龄期
uint32 health       # 卫星健康状态（0=健康）
uint32 code         # 数据来源标识；UM982 BD3 CNAV 为 0x100|FreqType (0=B1C,1=B2a,2=B2b)
float64 ura         # 用户测距精度（SPP 中用于定权）
float64 a           # 轨道长半轴 [m]（注意是 a 不是 sqrt(a)）
float64 e           # 轨道偏心率
float64 i0          # toe 时刻的轨道倾角 [rad]
float64 omg         # 近地点角距 [rad]
float64 omg0        # 周起始时刻的升交点赤经 [rad]
float64 m0          # toe 时刻的平近点角 [rad]
float64 delta_n     # 平均角速度改正值 [rad/s]
float64 omg_dot     # 升交点赤经变化率 [rad/s]
float64 i_dot       # 轨道倾角变化率 [rad/s]
float64 cuc         # 纬度幅角余弦调和改正振幅 [rad]
float64 cus         # 纬度幅角正弦调和改正振幅 [rad]
float64 crc         # 轨道半径余弦调和改正振幅 [m]
float64 crs         # 轨道半径正弦调和改正振幅 [m]
float64 cic         # 轨道倾角余弦调和改正振幅 [rad]
float64 cis         # 轨道倾角正弦调和改正振幅 [rad]
float64 af0         # 卫星钟差 [s]
float64 af1         # 卫星钟漂 [s/s]
float64 af2         # 卫星钟漂变化率 [s/s^2]
float64 tgd0        # 群延迟 [s]：GPS=TGD，北斗=TGD1(B1/B3)，Galileo=BGD E5a/E1，UM982 BD3 CNAV=B1Cp TGD
float64 tgd1        # 群延迟 [s]：北斗=TGD2(B2/B3)，Galileo=BGD E5b/E1，GPS 未用，UM982 BD3 CNAV=B2ap TGD
float64 a_dot       # 长半轴变化率 [m/s]（仅 CNAV 电文）
float64 n_dot       # 平均角速度变化率 [rad/s^2]（仅 CNAV 电文）
