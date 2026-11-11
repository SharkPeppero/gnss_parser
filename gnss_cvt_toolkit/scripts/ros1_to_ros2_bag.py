#!/usr/bin/env python3
"""ROS1 bag → ROS2 bag（GVINS 数据集：GNSS + IMU + 双目）。

ROS1 的 gnss_comm/msg/* 与 ROS2 的 gnss_msg/msg/* 字段顺序与类型一致，仅部分字段名大小写不同
（CN0→cn0、OMG0→omg0 …），二进制布局相同；因此直接把 ROS1 原始字节转 CDR，再以 gnss_msg 类型名写出。
sensor_msgs/Image、Imu、NavSatFix 等标准类型按 Humble 定义转换（丢弃 ROS1 header.seq）。

用法：
  python3 ros1_to_ros2_bag.py input.bag output_dir [--storage sqlite3|mcap] [--exclude /topic ...]
依赖：pip install rosbags
"""
import argparse
from pathlib import Path

from rosbags.rosbag1 import Reader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

GNSS_MSG_DIR = Path(__file__).resolve().parents[2] / "gnss_msg" / "msg"
TYPE_RENAME = {"gnss_comm/msg/": "gnss_msg/msg/"}


def ros2_typestore():
    store = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    for path in sorted(GNSS_MSG_DIR.glob("*.msg")):
        types.update(get_types_from_msg(path.read_text(), f"gnss_msg/msg/{path.stem}"))
    store.register(types)
    return store


def rename(msgtype):
    for src, dst in TYPE_RENAME.items():
        if msgtype.startswith(src):
            return dst + msgtype[len(src):]
    return msgtype


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--storage", choices=["sqlite3", "mcap"], default="sqlite3")
    # gvins/msg/VISensorExternalTrigger 在 ROS2 中没有对应消息包，默认跳过。
    parser.add_argument("--exclude", nargs="*", default=["/external_trigger"])
    args = parser.parse_args()

    store = ros2_typestore()
    storage = StoragePlugin.MCAP if args.storage == "mcap" else StoragePlugin.SQLITE3

    with Reader(args.input) as reader, Writer(args.output, version=8, storage_plugin=storage) as writer:
        # ros1_to_cdr 需要 ROS2 类型定义（内部自行丢弃 Header.seq），所以 ROS1 自定义类型也注册进同一个 ROS2 store；
        # 内嵌 msgdef 里的 std_msgs/Header 是 ROS1 版本（带 seq），跳过以保留 ROS2 定义。
        for conn in reader.connections:
            if conn.msgtype not in store.types:
                types = get_types_from_msg(conn.msgdef.data, conn.msgtype)
                store.register({k: v for k, v in types.items() if k not in store.types})

        connections = [c for c in reader.connections if c.topic not in args.exclude]
        out = {}
        for conn in connections:
            msgtype = rename(conn.msgtype)
            if msgtype not in store.types:
                raise SystemExit(f"{conn.topic}: ROS2 中找不到类型 {msgtype}，可用 --exclude 跳过")
            out[conn.id] = writer.add_connection(conn.topic, msgtype, typestore=store)
            print(f"{conn.topic:36s} {conn.msgtype} -> {msgtype} ({conn.msgcount})")

        total = sum(c.msgcount for c in connections)
        for count, (conn, timestamp, raw) in enumerate(reader.messages(connections=connections), 1):
            writer.write(out[conn.id], timestamp, store.ros1_to_cdr(raw, conn.msgtype))
            if count % 50000 == 0:
                print(f"{count}/{total}", flush=True)
    print(f"done: {total} messages -> {args.output}")


if __name__ == "__main__":
    main()
