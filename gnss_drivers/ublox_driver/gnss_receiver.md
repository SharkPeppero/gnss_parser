# GNSS接收机驱动设计

## Part1 GNSS原始数据接收层

- [x] 添加Enable的星系列表

- [ ] GNSS原始观测数据观测（来自UBX-RXM-RAWX字段消息）
  - [ ] Pseudo Range Observation（伪距观测）
  - [ ] Doppler Observation（多普勒观测）
  - [ ] Carrier Phase Observation（载波观测）

- 观测时间 time
- 卫星号 sat
- 频点 freqs
- 信号强度 cn0
- 失锁/周跳标志 lli
- 信号码型 code
- 伪距 psr
- 伪距标准差 psr_std
- 载波相位 cp
- 载波相位标准差 cp_std
- 多普勒 dopp
- 多普勒标准差 dopp_std
- 跟踪状态 status



------



- [ ] GNSS导航电文、星历、电离层参数（来自UBX-RXM-SFRBX字段消息）
  - [ ] GPS星历
  - [ ] BDS星历
  - [ ] GAL星历
  - [ ] GLONASS星历
  - [ ] 电离层参数

- [ ] 时间脉冲信息（来自UBX-TIM-TP字段消息）

- [ ] TODO：GNSS导航解（来自UBX-NAV-PVT字段消息）



- [ ] UBLOX驱动串口挂载类
- [ ] UBLOX的GNSS串口消息解析类
  - [ ] UBX-RXM-RAWX（UBLOX的原始数据字段）
  - [ ] UBX-RXM-SFRBX（UBLOX导航电文、星历、电离层字段）



------



## Part2 RTCM差分数据接收层

- [ ] RTCM差分数据ROS封装发布层
- [ ] NTRIP服务注册挂载类
- [ ] RTCM差分数据串口解析功能类



------



- [ ] RTCM數據內容組織額
- [ ] RawMesage數據解析以及Block操作
- [ ] RTCM消息如何獲取的Pipline