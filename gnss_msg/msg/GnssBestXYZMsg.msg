# ECEF 位置速度解，对应 NovAtel BESTXYZ 格式（ublox 驱动不发布）

std_msgs/Header header  # 时间戳
float64[] pos           # ECEF 位置 [x, y, z] [m]
float64[] pos_sigma     # 位置标准差 [m]
float64[] vel           # ECEF 速度 [vx, vy, vz] [m/s]
float64[] vel_sigma     # 速度标准差 [m/s]
uint32    num_svs       # 跟踪卫星数
