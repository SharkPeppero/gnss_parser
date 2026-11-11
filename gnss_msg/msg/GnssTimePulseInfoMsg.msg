# 秒脉冲（PPS）时间信息，对应 UBX-TIM-TP 消息，用于 GNSS 时间与本机时钟对齐
# 参考：UBX-18010854-R08，第 156 页

GnssTimeMsg time    # 下一个 PPS 脉冲对应的 GNSS 时间
bool utc_based      # 时间基准（1=UTC，0=GNSS 时）
uint32 time_sys     # 时间基准为 GNSS 时（utc_based=0）时，具体的 GNSS 时间系统
