# 单颗卫星在一个历元的观测数据
# 除 time 和 sat 外均为数组，下标表示频点槽位，同一下标在各数组中对应同一频点

GnssTimeMsg time    # 接收机采样时刻（GPST）
uint32 sat          # 卫星编号（由 gnss_utility.hpp 中的 sat_no 函数定义，satsys 可反解系统和 PRN）
float64[] freqs     # 观测频点的载波频率 [Hz]（GLONASS 为 FDMA，每颗卫星不同）
float64[] cn0       # 载噪比（信号强度）[dB-Hz]
uint8[] lli         # 失锁标志（1=失锁，可能存在周跳）
uint8[] code        # 信号码型（对应 gnss_constant.hpp 中的 CODE_xxx）
float64[] psr       # 伪距观测值 [m]
float64[] psr_std   # 伪距标准差 [m]
float64[] cp        # 载波相位观测值 [周]，乘以波长得到米
float64[] cp_std    # 载波相位标准差 [周]
float64[] dopp      # 多普勒观测值 [Hz]，乘以 -波长 得到伪距变化率 [m/s]
float64[] dopp_std  # 多普勒标准差 [Hz]
# 跟踪状态位：bit_0 伪距有效，bit_1 载波相位有效，
# bit_2 半周模糊度有效（ublox），bit_3 半周已扣除（ublox）
uint8[]   status
