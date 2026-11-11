# 接收机内部解算的定位结果，对应 UBX-NAV-PVT 消息
# 参考：UBX-18010854-R08，第 132 页

GnssTimeMsg time    # 导航历元时刻（GPST）
# 定位类型（0=无解，1=仅航位推算，2=二维解，3=三维解，
# 4=GNSS+航位推算组合，5=仅授时）
uint8   fix_type
bool    valid_fix   # 定位是否有效（1=有效）
bool    diff_soln   # 是否使用了差分改正（1=使用）
uint8   carr_soln   # 载波相位解状态（0=无载波解，1=RTK 浮点解，2=RTK 固定解）
uint8   num_sv      # 参与解算的卫星数
float64 latitude    # 纬度 [deg]
float64 longitude   # 经度 [deg]
float64 altitude    # 椭球高 [m]
float64 height_msl  # 海拔高（平均海平面以上）[m]
float64 h_acc       # 水平精度估计 [m]
float64 v_acc       # 垂直精度估计 [m]
float64 p_dop       # 位置精度因子（PDOP）
float64 vel_n       # NED 北向速度 [m/s]
float64 vel_e       # NED 东向速度 [m/s]
float64 vel_d       # NED 地向速度 [m/s]
float64 vel_acc     # 速度精度估计 [m/s]
