# 可见卫星信息，各数组按卫星一一对应（ublox 驱动目前不发布）

std_msgs/Header header      # 时间戳
uint32[]    sat             # 卫星编号（由 sat_no 函数定义）
int32[]     freqo           # GLONASS 频率号（-7~+6），其他系统无意义
bool[]      health          # 卫星是否健康
float64[]   elev_degree     # 高度角 [deg]
float64[]   az_degree       # 方位角 [deg]
uint32[]    sig_mask        # 已跟踪信号的掩码
