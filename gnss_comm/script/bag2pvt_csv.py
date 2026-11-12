#!/usr/bin/env python3
"""把 ROS 2 bag 中接收机自身的定位解（/ublox_driver/receiver_pvt）导出为 CSV，作为 SPP / RTK 结果的对比真值。

GVINS 数据集中该话题是 ZED-F9P 内部 RTK 引擎的实时解（3 km 外基站 RTCM），carr_soln = 2 的历元可作真值。

列与 gnss_solver_node 的 spp_enu / rtk_enu CSV 对齐，可直接按 timestamp 匹配：
    timestamp      UTC Unix 时间 [s]，由 GPST 周/周内秒减闰秒得到（与解算 CSV 相同）
    x, y, z        ENU [m]，原点默认取首个有效 /ublox_driver/receiver_lla（与在线模式的 ENU 原点相同）
    lat, lon, alt  WGS84 经纬度 [deg]、椭球高 [m]
    ecef_x/y/z     ECEF [m]
    carr_soln      0 无载波解 / 1 RTK 浮点 / 2 RTK 固定
    fix_type, valid_fix, diff_soln, num_sv, h_acc, v_acc, p_dop, vel_n, vel_e, vel_d
    gps_week, gps_tow

用法（需先 source 工作空间以加载 gnss_msg）：
    source /opt/ros/humble/setup.bash && source ~/happy_room/vins_ws/install/setup.bash
    python3 bag2pvt_csv.py ~/happy_room/vins_ws/bags/complex_environment_ros2 -o ~/happy_room/vins_ws/gnss_comm_output/gt_pvt.csv
"""

import argparse
import csv
import math
import os
import sys

import yaml

GPS_UNIX_OFFSET = 315964800  # 1980-01-06 的 Unix 时间
LEAP_SECONDS_TABLE = [(1483228800, 18), (1435708800, 17), (1341100800, 16), (1230768000, 15), (1136073600, 14)]  # (UTC Unix 生效时刻, 闰秒)


def leap_seconds(gps_unix):
    for start, leap in LEAP_SECONDS_TABLE:
        if gps_unix - leap >= start:
            return leap
    raise ValueError("data older than 2006, pass --leap-seconds")


def lla2ecef(lat_deg, lon_deg, h):
    a, e2 = 6378137.0, 6.69437999014e-3
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    return ((n + h) * math.cos(lat) * math.cos(lon), (n + h) * math.cos(lat) * math.sin(lon), (n * (1 - e2) + h) * math.sin(lat))


def ecef2enu(origin_lla, xyz):
    ox, oy, oz = lla2ecef(*origin_lla)
    dx, dy, dz = xyz[0] - ox, xyz[1] - oy, xyz[2] - oz
    lat, lon = math.radians(origin_lla[0]), math.radians(origin_lla[1])
    e = -math.sin(lon) * dx + math.cos(lon) * dy
    n = -math.sin(lat) * math.cos(lon) * dx - math.sin(lat) * math.sin(lon) * dy + math.cos(lat) * dz
    u = math.cos(lat) * math.cos(lon) * dx + math.cos(lat) * math.sin(lon) * dy + math.sin(lat) * dz
    return e, n, u


def read_bag(bag_path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    meta_path = os.path.join(bag_path, "metadata.yaml")
    if not os.path.isfile(meta_path):
        sys.exit(f"metadata.yaml not found in {bag_path} (pass the bag directory)")
    with open(meta_path) as f:
        storage_id = yaml.safe_load(f)["rosbag2_bagfile_information"]["storage_identifier"]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    present = [t for t in topics if t in types]
    msg_types = {t: get_message(types[t]) for t in present}
    reader.set_filter(rosbag2_py.StorageFilter(topics=present))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        yield topic, deserialize_message(data, msg_types[topic])


def main():
    parser = argparse.ArgumentParser(description="Export /ublox_driver/receiver_pvt of a ROS 2 bag to CSV as ground truth.")
    parser.add_argument("bag", help="ROS 2 bag 目录（含 metadata.yaml）")
    parser.add_argument("-o", "--output", default=None, help="输出 CSV 路径，默认 <bag 目录名>_pvt.csv")
    parser.add_argument("--pvt-topic", default="/ublox_driver/receiver_pvt")
    parser.add_argument("--lla-topic", default="/ublox_driver/receiver_lla")
    parser.add_argument("--origin", type=float, nargs=3, metavar=("LAT", "LON", "ALT"), help="ENU 原点，默认取首个有效 receiver_lla")
    parser.add_argument("--fixed-only", action="store_true", help="只导出 carr_soln = 2（RTK 固定）的历元")
    parser.add_argument("--leap-seconds", type=int, default=None, help="GPST−UTC 闰秒，默认按日期查表")
    args = parser.parse_args()

    bag = os.path.abspath(os.path.expanduser(args.bag.rstrip("/")))
    origin = tuple(args.origin) if args.origin else None
    rows, seen = [], set()
    for topic, msg in read_bag(bag, [args.pvt_topic, args.lla_topic]):
        if topic == args.lla_topic:
            if origin is None and msg.status.status >= 0 and abs(msg.latitude) < 90 and abs(msg.longitude) <= 180:
                origin = (msg.latitude, msg.longitude, msg.altitude)
            continue
        if not msg.valid_fix or (args.fixed_only and msg.carr_soln != 2):
            continue
        gps_unix = GPS_UNIX_OFFSET + msg.time.week * 604800 + msg.time.tow
        leap = args.leap_seconds if args.leap_seconds is not None else leap_seconds(gps_unix)
        timestamp = round(gps_unix - leap, 3)
        if timestamp in seen:
            continue
        seen.add(timestamp)
        rows.append((timestamp, msg))
    if not rows:
        sys.exit(f"no valid message on {args.pvt_topic}")
    if origin is None:
        origin = (rows[0][1].latitude, rows[0][1].longitude, rows[0][1].altitude)
        print(f"[bag2pvt_csv] no receiver_lla, ENU origin = first PVT", file=sys.stderr)
    rows.sort(key=lambda r: r[0])

    output = os.path.abspath(os.path.expanduser(args.output or os.path.basename(bag) + "_pvt.csv"))
    os.makedirs(os.path.dirname(output), exist_ok=True)
    header = ["timestamp", "x", "y", "z", "lat", "lon", "alt", "ecef_x", "ecef_y", "ecef_z", "carr_soln", "fix_type", "valid_fix", "diff_soln", "num_sv",
              "h_acc", "v_acc", "p_dop", "vel_n", "vel_e", "vel_d", "gps_week", "gps_tow"]
    counts = {0: 0, 1: 0, 2: 0}
    with open(output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for timestamp, m in rows:
            xyz = lla2ecef(m.latitude, m.longitude, m.altitude)
            e, n, u = ecef2enu(origin, xyz)
            counts[m.carr_soln] = counts.get(m.carr_soln, 0) + 1
            writer.writerow([f"{timestamp:.3f}", f"{e:.4f}", f"{n:.4f}", f"{u:.4f}", f"{m.latitude:.9f}", f"{m.longitude:.9f}", f"{m.altitude:.4f}",
                             f"{xyz[0]:.4f}", f"{xyz[1]:.4f}", f"{xyz[2]:.4f}", m.carr_soln, m.fix_type, int(m.valid_fix), int(m.diff_soln), m.num_sv,
                             f"{m.h_acc:.4f}", f"{m.v_acc:.4f}", f"{m.p_dop:.2f}", f"{m.vel_n:.4f}", f"{m.vel_e:.4f}", f"{m.vel_d:.4f}", m.time.week, f"{m.time.tow:.3f}"])
    print(f"[bag2pvt_csv] {len(rows)} epochs (carr_soln 0/1/2: {counts.get(0, 0)}/{counts.get(1, 0)}/{counts.get(2, 0)}), "
          f"ENU origin {origin[0]:.9f} {origin[1]:.9f} {origin[2]:.3f} -> {output}")


if __name__ == "__main__":
    main()
