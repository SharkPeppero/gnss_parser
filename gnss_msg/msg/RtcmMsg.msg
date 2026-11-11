# 校验通过的 RTCM 差分数据帧，包含元信息和原始字节（来源：NTRIP 或本地 TCP 回环）

std_msgs/Header header  # 收到该帧时的 ROS 时间
uint16 message_type     # RTCM 消息号（如 1005 基站坐标，1074/1124 MSM4 观测）
uint16 payload_length   # 负载长度 [字节]
uint32 frame_length     # 整帧长度 [字节]
uint8[] data            # 完整原始帧字节
