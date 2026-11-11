# UBlox RXM-RAWX 消息解析说明

## 1. 消息概述

`RXM-RAWX` 是 u-blox 接收机输出的 **原始卫星观测数据**，包括伪距、载波相位、多普勒、信噪比等信息，用于 GNSS 定位、RTK 或精密测量。

本 README 说明该消息的二进制布局及代码对应的解析方法。

---

## 2. UBX 协议结构

```
+--------+--------+--------+--------+-----------+------------+------------+
| SYNC1  | SYNC2  | CLASS  |  ID    | LENGTH_L  | LENGTH_H   |  PAYLOAD   |
+--------+--------+--------+--------+-----------+------------+------------+
| 0xB5   | 0x62   | 0x02   | 0x15   |  uint8_t  |  uint8_t   | n 字节     |
+--------+--------+--------+--------+-----------+------------+------------+
```

- `SYNC1/SYNC2`：固定同步头 `0xB5 0x62`
- `CLASS`：消息类别，`0x02`（RXM）
- `ID`：消息 ID，`0x15`（RAWX）
- `LENGTH_L/H`：payload 长度（小端）
- `PAYLOAD`：原始观测数据
- 最后两字节为校验和 `CK_A/CK_B`

---

## 3. Payload 结构

### 3.1 头部信息

| 偏移 | 字段 | 类型 | 描述 |
|------|------|------|------|
| 0-7  | `rcvTow` | double | 接收机周内秒（TOW，单位：秒） |
| 8-9  | `week` | uint16_t | GPS 周编号 |
| 10   | `leapS` | int8_t | 闰秒信息（可选） |
| 11   | `numMeas` | uint8_t | 本帧卫星观测数量 |
| 12-15 | reserved | uint8_t[4] | 保留 / 对齐 |

### 3.2 每颗卫星观测数据（32 字节）

| 偏移 | 字段 | 类型 | 描述 |
|------|------|------|------|
| 0-7  | `Pseudorange` | double | 伪距（m） |
| 8-15 | `CarrierPhase` | double | 载波相位（cycle） |
| 16-19 | `Doppler` | float | 多普勒频率（Hz） |
| 20   | `gnssId` | uint8_t | 卫星系统 ID（GPS/BDS/GLO/GAL） |
| 21   | `svId` | uint8_t | 卫星编号（PRN） |
| 22   | `sigId` | uint8_t | 信号类型 ID |
| 23   | `freqId` | uint8_t | 频点 ID（GLONASS 特别用） |
| 24-25 | `lockTime` | uint16_t | 锁定时间（ms），用于周跳检测 |
| 26   | `cno` | uint8_t | 信噪比 dB-Hz |
| 27   | `psr_std` | uint8_t | 伪距标准差指数（低 4 位有效） |
| 28   | `cp_std` | uint8_t | 载波标准差指数（低 4 位有效） |
| 29   | `dopp_std` | uint8_t | 多普勒标准差指数（低 4 位有效） |
| 30   | `tck_sta` | uint8_t | 跟踪状态（bit0: psr有效, bit1: cp有效, bit2: half-cycle, bit3: carrier valid） |
| 31   | reserved | uint8_t | 对齐保留 |

---

## 4. 代码解析流程

1. **跳过 UBX header**
```cpp
const uint8_t *p = msg_data + 6; // skip 6-byte header
```

2. **读取头部信息**
```cpp
double tow = *reinterpret_cast<const double *>(p);
uint16_t week = *reinterpret_cast<const uint16_t *>(p + 8);
uint8_t num_meas = p[11];
```

3. **遍历每颗卫星观测**
```cpp
p += 16; // 跳到第一个卫星观测数据
for (size_t i = 0; i < num_meas; ++i, p += 32) {
    double psr = *reinterpret_cast<const double *>(p);
    double cp = *reinterpret_cast<const double *>(p + 8);
    float dopp = *reinterpret_cast<const float *>(p + 16);
    uint8_t gnss_id = p[20];
    uint8_t svid = p[21];
    ...
}
```

4. **观测有效性判断**
- 如果 `tck_sta & 0x01 == 0` → 伪距无效
- 如果 `tck_sta & 0x02 == 0` 或 `cp == -0.5` 或 `cp_std > CPSTD_VALID` → 载波相位无效

5. **周跳和半周状态检测**
```cpp
uint8_t slip = lock_time == 0 ||
               lock_time*1e-3 < lock_time_rec[sat-1][sid-1] ||
               halfc != halfc_rec[sat-1][sid-1];
uint8_t lli = (slip ? LLI_SLIP : 0) | (!halfv ? LLI_HALFC : 0);
```

6. **多频观测合并**：同一颗卫星不同信号放在同一个 `Obs` 对象中。

7. **返回结果**
```cpp
return obs_meas; // std::vector<ObsPtr>
```

---

## 5. 数据流示意

```
UBX RXM-RAWX message
       ↓ parse header
       ↓ extract numMeas
       ↓ loop over numMeas
           ↓ extract psr, cp, doppler, gnssId, svId, sigId, freqId, lockTime, cn0
           ↓ check validity, compute LLI
           ↓ append to Obs object
       ↓ return vector<ObsPtr>
```

---

> 注：本 README 对应解析函数 `UbloxMessageProcessor::parse_meas_msg()`，用于解析 u-blox 原始观测消息，并组织成 ROS `ObsPtr` 数据结构。