# 带时间戳的浮点数组
# 电离层模型由话题区分：GPS/BDS 为 [alpha0~alpha3, beta0~beta3]；
# UM982 iono/bd3 为 BDGIM [A1~A9]，iono/gal 为 NeQuick [ai0~ai2, SF1~SF5]。
# 后两者不能作为 Klobuchar 8 参数直接输入解算器。

std_msgs/Header header  # 时间戳
float64[] data          # 数据
