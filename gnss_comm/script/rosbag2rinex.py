#!/usr/bin/env python3
"""把 ROS 2 bag 中的流动站 GNSS 观测与导航数据转换成 RINEX 3.04 观测文件 + 导航文件（含 LEAP SECONDS）。

输入话题（gnss_msg，与 gnss_solver_node 在线模式订阅的话题一致）：
    range_meas   gnss_msg/GnssMeasMsg          每条消息一个历元 -> 观测文件
    ephem        gnss_msg/GnssEphemMsg         GPS/Galileo/北斗/QZSS 星历 -> 导航文件
    glo_ephem    gnss_msg/GnssGloEphemMsg      GLONASS 星历 -> 导航文件
    iono_params  gnss_msg/StampedFloat64Array  GPS Klobuchar 8 参数 -> 导航文件头 GPSA/GPSB
    receiver_lla sensor_msgs/NavSatFix         首个有效定位 -> 观测文件头 APPROX POSITION XYZ

输出格式按 gnss_comm 的 C++ 读取器（src/rinex/observation.cpp、navigation.cpp）的列位置写出，
可直接填入 alg_cfg.yaml 的 offline.rover_obs_paths / offline.rover_nav_paths。

约定：
    - 观测时间系为 GPS；GLONASS 星历历元按 RINEX 约定写 UTC，北斗星历写 BDT。
    - 每颗卫星每个频段一个信号，频段按载波频率判定、属性取自消息 code（北斗 B1I 写成 RINEX 3.04 的 "2I"）。
    - 只写半周已解决（status bit1 与 bit2 同时置位）的载波；LLI 取消息的 lli（bit0 失锁，bit1 半周未定）。
    - 没有闰秒参数时按 GPST 日期查表。

用法（需先 source 工作空间，以便加载 gnss_msg 的 Python 消息类型）：
    source /opt/ros/humble/setup.bash && source ~/happy_room/vins_ws/install/setup.bash
    python3 rosbag2rinex.py ~/happy_room/vins_ws/bags/complex_environment_ros2 -o ~/happy_room/vins_ws/bags/rover_rinex
"""

import argparse
import datetime
import getpass
import math
import os
import sys

import yaml

GPS_EPOCH = datetime.datetime(1980, 1, 6)
WEEK_SECONDS = 604800
BDT_GPST_OFFSET = 14  # GPST = BDT + 14 s
BDT_EPOCH_GPS_WEEK = 1356  # BDT 起点 2006-01-01 对应的 GPS 周

# (生效日期 GPST, GPST−UTC 闰秒数)，从新到旧
LEAP_SECONDS_TABLE = [
    (datetime.datetime(2017, 1, 1), 18),
    (datetime.datetime(2015, 7, 1), 17),
    (datetime.datetime(2012, 7, 1), 16),
    (datetime.datetime(2009, 1, 1), 15),
    (datetime.datetime(2006, 1, 1), 14),
]

# 工程卫星编号区间（constants/system.hpp）：(RINEX 星座字符, 起始编号, 卫星数)
SAT_RANGES = [("G", 1, 32), ("R", 33, 27), ("E", 60, 38), ("C", 98, 63), ("J", 161, 10)]

# CODE_* 到 RINEX 频段+属性，下标即 CODE_* 值（与 observation.cpp 的 code_strs 一致）
CODE_STRS = [
    "", "1C", "1P", "1W", "1Y", "1M", "1N", "1S", "1L", "1E",
    "1A", "1B", "1X", "1Z", "2C", "2D", "2S", "2L", "2X", "2P",
    "2W", "2Y", "2M", "2N", "5I", "5Q", "5X", "7I", "7Q", "7X",
    "6A", "6B", "6C", "6X", "6Z", "6S", "6L", "8I", "8Q", "8X",
    "2I", "2Q", "6I", "6Q", "3I", "3Q", "3X", "1I", "1Q",
]

# 各星座 RINEX 频段与载波频率 [Hz]；GLONASS G1/G2 另按 FDMA 范围判断
BAND_FREQS = {
    "G": {"1": 1575.42e6, "2": 1227.60e6, "5": 1176.45e6},
    "E": {"1": 1575.42e6, "5": 1176.45e6, "7": 1207.14e6, "6": 1278.75e6, "8": 1191.795e6},
    "C": {"2": 1561.098e6, "7": 1207.14e6, "6": 1268.52e6, "1": 1575.42e6, "5": 1176.45e6},
    "J": {"1": 1575.42e6, "2": 1227.60e6, "5": 1176.45e6, "6": 1278.75e6},
}
GLO_G1, GLO_DG1 = 1602.0e6, 0.5625e6
GLO_G2, GLO_DG2 = 1246.0e6, 0.4375e6

# 消息 code 缺失或与频段不符时使用的属性
DEFAULT_ATTR = {
    "G1": "C", "G2": "L", "G5": "Q", "R1": "C", "R2": "C",
    "E1": "C", "E5": "Q", "E7": "Q", "E6": "C", "E8": "Q",
    "C2": "I", "C7": "I", "C6": "I", "C1": "P", "C5": "P",
    "J1": "C", "J2": "L", "J5": "Q", "J6": "S",
}

# Galileo 星历数据源：bit0 I/NAV E1-B、bit2 E5b、bit9 钟差对应 E5b/E1；C++ 读取器只接受 bit0 置位的记录
GAL_INAV_SOURCE = 517


def sat2sys_prn(sat):
    for sys_char, first, count in SAT_RANGES:
        if first <= sat < first + count:
            return sys_char, sat - first + 1
    return None, 0


def gps_seconds(week, tow):
    return week * WEEK_SECONDS + tow


def split_gps_seconds(total):
    """GPS 起点以来的秒 -> (整秒 datetime, 小数秒)，小数部分四舍五入到 1e-7 s。"""
    whole = math.floor(total)
    frac = round(total - whole, 7)
    if frac >= 1.0:
        whole, frac = whole + 1, 0.0
    return GPS_EPOCH + datetime.timedelta(seconds=whole), frac


def leap_seconds_at(total):
    t = GPS_EPOCH + datetime.timedelta(seconds=total)
    for start, leap in LEAP_SECONDS_TABLE:
        if t >= start:
            return leap
    raise ValueError("data older than 2006, pass --leap-seconds explicitly")


def lla2ecef(lat_deg, lon_deg, h):
    a, e2 = 6378137.0, 6.69437999014e-3
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    return ((n + h) * math.cos(lat) * math.cos(lon), (n + h) * math.cos(lat) * math.sin(lon), (n * (1 - e2) + h) * math.sin(lat))


def hdr(content, label):
    return content.ljust(60)[:60] + label + "\n"


def d19(v):
    """RINEX 导航 D19.12。"""
    if v != 0 and abs(v) < 1e-99:
        v = 0.0
    return f"{v: .12E}".replace("E", "D").rjust(19)


def d12(v):
    """RINEX 导航头 D12.4。"""
    return f"{v: .4E}".replace("E", "D").rjust(12)


def rinex_band(sys_char, freq):
    """载波频率 -> RINEX 频段字符；GLONASS 同时返回频道号。"""
    if sys_char == "R":
        k1 = round((freq - GLO_G1) / GLO_DG1)
        if -7 <= k1 <= 6 and abs(freq - (GLO_G1 + k1 * GLO_DG1)) < 1e3:
            return "1", k1
        k2 = round((freq - GLO_G2) / GLO_DG2)
        if -7 <= k2 <= 6 and abs(freq - (GLO_G2 + k2 * GLO_DG2)) < 1e3:
            return "2", k2
        return None, None
    for band, f in BAND_FREQS.get(sys_char, {}).items():
        if abs(freq - f) < 1e3:
            return band, None
    return None, None


def signal_name(sys_char, band, code):
    """频段 + 消息 code 的属性；code 与频段不符（如北斗 B1I 的 CODE_L1I）时只取属性字符。"""
    code_str = CODE_STRS[code] if 0 <= code < len(CODE_STRS) else ""
    attr = code_str[1] if len(code_str) == 2 else DEFAULT_ATTR.get(sys_char + band, "C")
    return band + attr


class BagReader:
    def __init__(self, bag_path):
        import rosbag2_py

        meta_path = os.path.join(bag_path, "metadata.yaml")
        if not os.path.isfile(meta_path):
            sys.exit(f"metadata.yaml not found in {bag_path} (pass the bag directory)")
        with open(meta_path) as f:
            storage_id = yaml.safe_load(f)["rosbag2_bagfile_information"]["storage_identifier"]
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id), rosbag2_py.ConverterOptions("cdr", "cdr"))
        self.types = {t.name: t.type for t in self.reader.get_all_topics_and_types()}

    def messages(self, topics):
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        import rosbag2_py

        present = [t for t in topics if t in self.types]
        msg_types = {t: get_message(self.types[t]) for t in present}
        self.reader.set_filter(rosbag2_py.StorageFilter(topics=present))
        while self.reader.has_next():
            topic, data, _ = self.reader.read_next()
            yield topic, deserialize_message(data, msg_types[topic])


def collect(bag_path, topics):
    epochs = []  # [(gps_seconds, [(sys_char, prn, {signal: (psr, cp, dopp, cn0, lli)})])]
    ephems = {}  # (sat, iode, toe_seconds) -> msg
    glo_ephems = {}  # (sat, toe_seconds) -> msg
    iono = None
    approx_lla = None
    glo_channels = {}  # GLONASS slot -> 频道号

    reader = BagReader(bag_path)
    for name in topics.values():
        if name not in reader.types:
            print(f"[rosbag2rinex] warning: topic {name} not in bag", file=sys.stderr)

    for topic, msg in reader.messages(list(topics.values())):
        if topic == topics["meas"]:
            if not msg.meas:
                continue
            t0 = gps_seconds(msg.meas[0].time.week, msg.meas[0].time.tow)
            sats = []
            for obs in msg.meas:
                sys_char, prn = sat2sys_prn(obs.sat)
                if sys_char is None or abs(gps_seconds(obs.time.week, obs.time.tow) - t0) > 1e-3:
                    continue
                signals = {}
                for i, freq in enumerate(obs.freqs):
                    band, channel = rinex_band(sys_char, freq)
                    if band is None:
                        continue
                    if channel is not None:
                        glo_channels.setdefault(prn, channel)
                    status = obs.status[i] if i < len(obs.status) else 0
                    psr = obs.psr[i] if i < len(obs.psr) and status & 0x01 and obs.psr[i] > 0 else 0.0
                    cp = obs.cp[i] if i < len(obs.cp) and (status & 0x06) == 0x06 else 0.0
                    dopp = obs.dopp[i] if i < len(obs.dopp) else 0.0
                    cn0 = obs.cn0[i] if i < len(obs.cn0) else 0.0
                    lli = obs.lli[i] & 0x03 if i < len(obs.lli) else 0
                    if psr == 0 and cp == 0:
                        continue
                    code = obs.code[i] if i < len(obs.code) else 0
                    signals.setdefault(signal_name(sys_char, band, code), (psr, cp, dopp, cn0, lli))
                if signals:
                    sats.append((sys_char, prn, signals))
            if sats:
                epochs.append((t0, sats))
        elif topic == topics["ephem"]:
            key = (msg.sat, msg.iode, gps_seconds(msg.toe.week, msg.toe.tow))
            ephems.setdefault(key, msg)
        elif topic == topics["glo_ephem"]:
            glo_ephems.setdefault((msg.sat, gps_seconds(msg.toe.week, msg.toe.tow)), msg)
            _, prn = sat2sys_prn(msg.sat)
            glo_channels[prn] = msg.freqo  # 星历中的频道号优先
        elif topic == topics["iono"]:
            if iono is None and len(msg.data) == 8 and any(v != 0 for v in msg.data):
                iono = list(msg.data)
        elif topic == topics["lla"]:
            if approx_lla is None and msg.status.status >= 0 and abs(msg.latitude) < 90 and abs(msg.longitude) <= 180:
                approx_lla = (msg.latitude, msg.longitude, msg.altitude)

    epochs.sort(key=lambda e: e[0])
    return epochs, ephems, glo_ephems, iono, approx_lla, glo_channels


def program_line():
    now = datetime.datetime.utcnow().strftime("%Y%m%d %H%M%S UTC")
    return hdr(f"{'rosbag2rinex.py':<20}{getpass.getuser()[:20]:<20}{now:<20}", "PGM / RUN BY / DATE")


def write_obs(path, epochs, approx_lla, glo_channels, leap, marker):
    # 各星座出现过的信号 -> 观测类型 C/L/D/S
    sys_signals = {}
    for _, sats in epochs:
        for sys_char, _, signals in sats:
            sys_signals.setdefault(sys_char, set()).update(signals)
    sys_order = [s for s in "GRECJ" if s in sys_signals]
    sys_types = {s: [k + sig for sig in sorted(sys_signals[s]) for k in "CLDS"] for s in sys_order}

    lines = [
        hdr(f"{'3.04':>9}{'':11}{'OBSERVATION DATA':<20}{'M: Mixed':<20}", "RINEX VERSION / TYPE"),
        program_line(),
        hdr(marker, "MARKER NAME"),
        hdr(f"{'':<20}{'':<40}", "OBSERVER / AGENCY"),
        hdr(f"{'':<20}{'u-blox':<20}{'':<20}", "REC # / TYPE / VERS"),
        hdr(f"{'':<20}{'':<20}", "ANT # / TYPE"),
    ]
    xyz = lla2ecef(*approx_lla) if approx_lla else (0.0, 0.0, 0.0)
    lines.append(hdr("".join(f"{v:14.4f}" for v in xyz), "APPROX POSITION XYZ"))
    lines.append(hdr("".join(f"{v:14.4f}" for v in (0.0, 0.0, 0.0)), "ANTENNA: DELTA H/E/N"))
    for s in sys_order:
        types = sys_types[s]
        for i in range(0, len(types), 13):
            head = f"{s}  {len(types):3d}" if i == 0 else " " * 6
            lines.append(hdr(head + "".join(f" {t}" for t in types[i:i + 13]), "SYS / # / OBS TYPES"))
    lines.append(hdr("DBHZ", "SIGNAL STRENGTH UNIT"))
    if len(epochs) > 1:
        dts = sorted(round(b[0] - a[0], 3) for a, b in zip(epochs, epochs[1:]))
        lines.append(hdr(f"{dts[len(dts) // 2]:10.3f}", "INTERVAL"))
    first, frac = split_gps_seconds(epochs[0][0])
    lines.append(hdr(f"{first.year:6d}{first.month:6d}{first.day:6d}{first.hour:6d}{first.minute:6d}{first.second + frac:13.7f}     GPS", "TIME OF FIRST OBS"))
    for s in sys_order:
        for t in sys_types[s]:
            if t[0] == "L":
                lines.append(hdr(f"{s} {t}  0.00000", "SYS / PHASE SHIFT"))
    slots = sorted(glo_channels.items())
    for i in range(0, max(len(slots), 1), 8):
        head = f"{len(slots):3d} " if i == 0 else " " * 4
        lines.append(hdr(head + "".join(f"R{prn:02d} {k:2d} " for prn, k in slots[i:i + 8]), "GLONASS SLOT / FRQ #"))
    lines.append(hdr(" C1C    0.000 C1P    0.000 C2C    0.000 C2P    0.000", "GLONASS COD/PHS/BIS"))
    lines.append(hdr(f"{leap:6d}", "LEAP SECONDS"))
    lines.append(hdr("", "END OF HEADER"))

    def field(value, lli=0, cn0=0.0):
        if value == 0:
            return " " * 16
        ssi = str(min(9, max(1, int(cn0 / 6)))) if cn0 > 0 else " "
        return f"{value:14.3f}{str(lli) if lli else ' '}{ssi}"

    with open(path, "w") as f:
        f.writelines(lines)
        for t, sats in epochs:
            ep, frac = split_gps_seconds(t)
            sats = sorted(sats, key=lambda s: ("GRECJ".index(s[0]), s[1]))
            f.write(f"> {ep.year:4d} {ep.month:02d} {ep.day:02d} {ep.hour:02d} {ep.minute:02d}{ep.second + frac:11.7f}  0{len(sats):3d}\n")
            for sys_char, prn, signals in sats:
                row = f"{sys_char}{prn:02d}"
                for typ in sys_types[sys_char]:
                    psr, cp, dopp, cn0, lli = signals.get(typ[1:], (0.0, 0.0, 0.0, 0.0, 0))
                    if typ[0] == "C":
                        row += field(psr, 0, cn0)
                    elif typ[0] == "L":
                        row += field(cp, lli, cn0)
                    elif typ[0] == "D":
                        row += field(dopp if psr or cp else 0.0)
                    else:
                        row += field(cn0 if psr or cp else 0.0)
                f.write(row.rstrip() + "\n")


def nav_epoch(sys_char, prn, ep):
    return f"{sys_char}{prn:02d} {ep.year:4d} {ep.month:02d} {ep.day:02d} {ep.hour:02d} {ep.minute:02d} {ep.second:02d}"


def nav_line(values):
    return "    " + "".join(d19(v) for v in values) + "\n"


def round_epoch(total):
    ep, frac = split_gps_seconds(total)
    return ep + datetime.timedelta(seconds=round(frac))


def kepler_record(msg):
    sys_char, prn = sat2sys_prn(msg.sat)
    toc = gps_seconds(msg.toc.week, msg.toc.tow)
    toe = gps_seconds(msg.toe.week, msg.toe.tow)
    ttr = gps_seconds(msg.ttr.week, msg.ttr.tow) if msg.ttr.week else toe
    if sys_char == "C":
        # 北斗：历元、toe、ttr 都写 BDT，周数为 BDT 周
        bdt_toe = toe - BDT_GPST_OFFSET - BDT_EPOCH_GPS_WEEK * WEEK_SECONDS
        week = int(bdt_toe // WEEK_SECONDS)
        toe_sow = bdt_toe - week * WEEK_SECONDS
        ttr_sow = ttr - BDT_GPST_OFFSET - BDT_EPOCH_GPS_WEEK * WEEK_SECONDS - week * WEEK_SECONDS
        epoch = round_epoch(toc - BDT_GPST_OFFSET)
    else:
        week = msg.week
        toe_sow = msg.toe_tow
        ttr_sow = ttr - week * WEEK_SECONDS  # 跨周时可为负或超过一周，RINEX 允许
        epoch = round_epoch(toc)
    sqrt_a = math.sqrt(msg.a) if msg.a > 0 else 0.0
    out = nav_epoch(sys_char, prn, epoch) + "".join(d19(v) for v in (msg.af0, msg.af1, msg.af2)) + "\n"
    out += nav_line((msg.iode, msg.crs, msg.delta_n, msg.m0))
    out += nav_line((msg.cuc, msg.e, msg.cus, sqrt_a))
    out += nav_line((toe_sow, msg.cic, msg.omg0, msg.cis))
    out += nav_line((msg.i0, msg.crc, msg.omg, msg.omg_dot))
    if sys_char == "E":
        source = msg.code if msg.code & 0x01 else GAL_INAV_SOURCE
        out += nav_line((msg.i_dot, source, week, 0.0))
        out += nav_line((msg.ura, msg.health, msg.tgd0, msg.tgd1))
        out += nav_line((ttr_sow, 0.0))
    elif sys_char == "C":
        out += nav_line((msg.i_dot, 0.0, week, 0.0))
        out += nav_line((msg.ura, msg.health, msg.tgd0, msg.tgd1))
        out += nav_line((ttr_sow, msg.iodc))
    else:
        out += nav_line((msg.i_dot, 0.0, week, 0.0))
        out += nav_line((msg.ura, msg.health, msg.tgd0, msg.iodc))
        out += nav_line((ttr_sow, 0.0))
    return out


def glonass_record(msg, leap):
    _, prn = sat2sys_prn(msg.sat)
    # RINEX GLONASS 历元为 UTC；C++ 读取器再加 LEAP SECONDS 回到 GPST
    toe_utc = gps_seconds(msg.toe.week, msg.toe.tow) - leap
    ttr_utc = gps_seconds(msg.ttr.week, msg.ttr.tow) - leap if msg.ttr.week else toe_utc
    out = nav_epoch("R", prn, round_epoch(toe_utc)) + "".join(d19(v) for v in (-msg.tau_n, msg.gamma, ttr_utc % WEEK_SECONDS)) + "\n"
    out += nav_line((msg.pos_x * 1e-3, msg.vel_x * 1e-3, msg.acc_x * 1e-3, msg.health))
    out += nav_line((msg.pos_y * 1e-3, msg.vel_y * 1e-3, msg.acc_y * 1e-3, msg.freqo))
    out += nav_line((msg.pos_z * 1e-3, msg.vel_z * 1e-3, msg.acc_z * 1e-3, msg.age))
    return out


def write_nav(path, ephems, glo_ephems, iono, leap):
    lines = [
        hdr(f"{'3.04':>9}{'':11}{'N: GNSS NAV DATA':<20}{'M: MIXED':<20}", "RINEX VERSION / TYPE"),
        program_line(),
    ]
    if iono:
        lines.append(hdr("GPSA " + "".join(d12(v) for v in iono[:4]), "IONOSPHERIC CORR"))
        lines.append(hdr("GPSB " + "".join(d12(v) for v in iono[4:]), "IONOSPHERIC CORR"))
    lines.append(hdr(f"{leap:6d}", "LEAP SECONDS"))
    lines.append(hdr("", "END OF HEADER"))

    records = []
    for (sat, _, toe), msg in ephems.items():
        sys_char, _ = sat2sys_prn(sat)
        if sys_char in ("G", "E", "C", "J"):
            records.append((toe, sat, kepler_record(msg)))
    for (sat, toe), msg in glo_ephems.items():
        records.append((toe, sat, glonass_record(msg, leap)))
    records.sort(key=lambda r: (r[0], r[1]))
    with open(path, "w") as f:
        f.writelines(lines)
        f.writelines(r[2] for r in records)
    return len(records)


def main():
    parser = argparse.ArgumentParser(description="Convert rover GNSS data in a ROS 2 bag to RINEX 3.04 observation + navigation files.")
    parser.add_argument("bag", help="ROS 2 bag 目录（含 metadata.yaml）")
    parser.add_argument("-o", "--output-dir", default=".", help="输出目录，默认当前目录")
    parser.add_argument("--name", default=None, help="输出文件名前缀，默认取 bag 目录名")
    parser.add_argument("--marker", default="ROVER", help="观测文件 MARKER NAME")
    parser.add_argument("--leap-seconds", type=int, default=None, help="GPST−UTC 闰秒数，默认按数据日期查表")
    parser.add_argument("--meas-topic", default="/ublox_driver/range_meas")
    parser.add_argument("--ephem-topic", default="/ublox_driver/ephem")
    parser.add_argument("--glo-ephem-topic", default="/ublox_driver/glo_ephem")
    parser.add_argument("--iono-topic", default="/ublox_driver/iono_params")
    parser.add_argument("--lla-topic", default="/ublox_driver/receiver_lla")
    args = parser.parse_args()

    bag = os.path.abspath(os.path.expanduser(args.bag.rstrip("/")))
    topics = {"meas": args.meas_topic, "ephem": args.ephem_topic, "glo_ephem": args.glo_ephem_topic, "iono": args.iono_topic, "lla": args.lla_topic}
    epochs, ephems, glo_ephems, iono, approx_lla, glo_channels = collect(bag, topics)
    if not epochs:
        sys.exit("no observation epoch found in " + args.meas_topic)
    if iono is None:
        print("[rosbag2rinex] warning: no ionosphere parameters, nav header has no GPSA/GPSB (SPP will wait forever)", file=sys.stderr)
    leap = args.leap_seconds if args.leap_seconds is not None else leap_seconds_at(epochs[0][0])

    out_dir = os.path.abspath(os.path.expanduser(args.output_dir))
    os.makedirs(out_dir, exist_ok=True)
    name = args.name or os.path.basename(bag)
    obs_path = os.path.join(out_dir, name + ".obs")
    nav_path = os.path.join(out_dir, name + ".nav")
    write_obs(obs_path, epochs, approx_lla, glo_channels, leap, args.marker)
    num_records = write_nav(nav_path, ephems, glo_ephems, iono, leap)

    first, _ = split_gps_seconds(epochs[0][0])
    last, _ = split_gps_seconds(epochs[-1][0])
    print(f"[rosbag2rinex] {len(epochs)} epochs ({first} ~ {last} GPST) -> {obs_path}")
    print(f"[rosbag2rinex] {num_records} ephemerides ({len(ephems)} Keplerian, {len(glo_ephems)} GLONASS), leap seconds {leap} -> {nav_path}")


if __name__ == "__main__":
    main()
