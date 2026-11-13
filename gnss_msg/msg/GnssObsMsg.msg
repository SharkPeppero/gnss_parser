# 单颗卫星在一个历元的观测数据
# 观测数组的相同下标对应同一信号；time、sat、status_version 为每颗卫星的标量

GnssTimeMsg time    # 接收机采样时刻（GPST）
uint32 sat          # 卫星编号（由 gnss_utility.hpp 中的 sat_no 函数定义，satsys 可反解系统和 PRN）
float64[] freqs     # 观测频点的载波频率 [Hz]（GLONASS 为 FDMA，每颗卫星不同）
float64[] cn0       # 载噪比（信号强度）[dB-Hz]
uint8[] lli         # bit0 周跳/失锁/相位不连续；bit1 已知半周未解决（未知由 status 表达）
uint8[] code        # 信号码型（对应 gnss_constant.hpp 中的 CODE_xxx）
float64[] psr       # 伪距观测值 [m]
float64[] psr_std   # 伪距标准差 [m]
float64[] cp        # 载波相位观测值 [周]，乘以波长得到米
float64[] cp_std    # 载波相位标准差 [周]
float64[] dopp      # 多普勒观测值 [Hz]，乘以 -波长 得到伪距变化率 [m/s]
float64[] dopp_std  # 多普勒标准差 [Hz]
# 接收机无关的观测状态，禁止直接复制设备原始状态。
uint8 STATUS_VERSION=1
uint8 PSEUDORANGE_VALID=1
uint8 CARRIER_PHASE_VALID=2
uint8 HALF_CYCLE_RESOLVED=4
# bit3 保留；UBX subHalfCyc 不是统一状态位。
uint8 HALF_CYCLE_KNOWN=16
# 0 表示旧版/未声明语义，消费者不得按版本 1 解释。
uint8 status_version
# bit2 仅在 bit4=1 时有意义；bit4=0 表示未知，而不是已知未解决。
uint8[] status
