# GNSS 时间，用 "周数 + 周内秒" 表示，统一为 GPST（GPS 时）
# 北斗时/Galileo 时在转换时已换算到 GPST，见 gnss_ros.cpp 的 time2gpst/gpst2time

uint32 week     # GPS 周数
float64 tow     # 周内秒 [s]
