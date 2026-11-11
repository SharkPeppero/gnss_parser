# GLONASS 广播星历：直接给出 PZ-90 坐标系下的位置/速度/加速度，由 geph2pos 数值积分外推

uint32 sat          # 卫星编号（由 gnss_utility.hpp 中的 sat_no 函数定义）
GnssTimeMsg ttr     # 电文发射时刻（GPST）
GnssTimeMsg toe     # 星历参考时刻（GPST）
int32  freqo        # 频率号（-7~+6），G1 频率 = 1602MHz + k*0.5625MHz
uint32 iode         # 星历数据龄期（取 tb）
uint32 health       # 卫星健康状态（0=健康）
uint32 age          # 星历龄期 [天]
float64 ura         # 用户测距精度
float64 pos_x       # toe 时刻卫星位置 X（ECEF）[m]
float64 pos_y       # toe 时刻卫星位置 Y（ECEF）[m]
float64 pos_z       # toe 时刻卫星位置 Z（ECEF）[m]
float64 vel_x       # toe 时刻卫星速度 X（ECEF）[m/s]
float64 vel_y       # toe 时刻卫星速度 Y（ECEF）[m/s]
float64 vel_z       # toe 时刻卫星速度 Z（ECEF）[m/s]
float64 acc_x       # 日月引力摄动加速度 X（ECEF）[m/s^2]
float64 acc_y       # 日月引力摄动加速度 Y（ECEF）[m/s^2]
float64 acc_z       # 日月引力摄动加速度 Z（ECEF）[m/s^2]
float64 tau_n       # 卫星钟差 [s]（卫星钟差 = -tau_n + gamma*dt）
float64 gamma       # 相对频率偏差
float64 delta_tau_n # L1 与 L2 之间的时延差 [s]
