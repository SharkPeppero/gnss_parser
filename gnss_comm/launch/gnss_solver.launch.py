"""GNSS 解算（SPP / RTK）启动文件。

mode:=online（默认）：交互式输入 Bag 路径，启动 gnss_solver_node 和 RViz 后回放 Bag，后续只切换 Bag；
    RTK 滤波器跨历元保留状态，切换的 Bag 与上一个时间不连续时会因间隔超过 max_gap_s 自动重置。
mode:=offline：gnss_solver_node 直接读取 alg_cfg 中 offline.* 的流动站 RINEX 文件，处理完后节点保持运行，Ctrl-C 退出。
两种模式的差分数据都从 alg_cfg 的 correction.* 离线文件加载；SPP / RTK 开关见 alg_cfg 的 spp.enable / rtk.enable。
"""

import os
import subprocess
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def read_bag_path(tty):
    """读取并验证 Bag 路径。"""
    while True:
        print(
            "\n[gnss_solver] 输入 Bag 路径（回车或 q 退出）：",
            flush=True,
        )
        line = tty.readline()
        if not line:
            return None

        path = os.path.expanduser(line.strip().strip("'\""))

        if path == "" or path.lower() == "q":
            return None

        if not os.path.exists(path):
            print(f"[gnss_solver] 路径不存在：{path}", flush=True)
            continue

        return os.path.abspath(path)


def prompt_loop(rate, first_bag):
    """依次播放 Bag，算法节点保持运行。"""
    with open("/dev/tty", "r") as tty:
        path = first_bag

        while path is not None:
            print(
                f"[gnss_solver] 开始回放：{path} (rate={rate})",
                flush=True,
            )

            try:
                result = subprocess.run([
                    "ros2", "bag", "play", path,
                    "--rate", rate,
                    # 等节点完成订阅再回放，避免丢掉开头的星历
                    "--delay", "3.0",
                ])
                if result.returncode != 0:
                    print(
                        f"[gnss_solver] 回放退出码：{result.returncode}",
                        flush=True,
                    )
            except KeyboardInterrupt:
                return

            print("[gnss_solver] 回放结束。", flush=True)
            path = read_bag_path(tty)


def solver_and_rviz(mode):
    parameters = [
        LaunchConfiguration("ros_cfg"),
        LaunchConfiguration("alg_cfg"),
        {"mode": mode},
    ]

    solver_node = Node(
        package="gnss_comm",
        executable="gnss_solver_node",
        name="gnss_solver_node",
        output="screen",
        parameters=parameters,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_cfg")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return [solver_node, rviz_node]


def start(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    if mode == "offline":
        return solver_and_rviz(mode)
    if mode != "online":
        print(f"[gnss_solver] mode 只能是 online 或 offline：{mode}", flush=True)
        return [EmitEvent(event=Shutdown())]

    # online：首次读取有效 Bag 路径后，创建算法、RViz 和回放进程
    try:
        with open("/dev/tty", "r") as tty:
            first_bag = read_bag_path(tty)
    except OSError as exc:
        print(f"[gnss_solver] 无法读取终端：{exc}", flush=True)
        return [EmitEvent(event=Shutdown())]

    if first_bag is None:
        return [EmitEvent(event=Shutdown())]

    rate = LaunchConfiguration("rate").perform(context)
    bag_prompt = ExecuteProcess(
        cmd=[
            sys.executable,
            "-u",
            os.path.abspath(__file__),
            rate,
            first_bag,
        ],
        name="bag_prompt",
        output="screen",
    )

    return solver_and_rviz(mode) + [
        bag_prompt,
        RegisterEventHandler(
            OnProcessExit(
                target_action=bag_prompt,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="online",
            description="online：回放 Bag；offline：读取 alg_cfg 中 offline.* 的 RINEX 文件。",
        ),
        DeclareLaunchArgument(
            "rate",
            default_value="1.0",
            description="online 模式的 Bag 回放倍速；offline 节流见 alg_cfg 的 offline.playback_rate。",
        ),
        DeclareLaunchArgument(
            "ros_cfg",
            default_value=PathJoinSubstitution([
                FindPackageShare("gnss_comm"),
                "config",
                "ros_cfg.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "alg_cfg",
            default_value=PathJoinSubstitution([
                FindPackageShare("gnss_comm"),
                "config",
                "alg_cfg.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "rviz_cfg",
            default_value=PathJoinSubstitution([
                FindPackageShare("gnss_comm"),
                "rviz_cfg",
                "gnss.rviz",
            ]),
        ),
        OpaqueFunction(function=start),
    ])


if __name__ == "__main__":
    try:
        prompt_loop(
            sys.argv[1] if len(sys.argv) > 1 else "1.0",
            sys.argv[2] if len(sys.argv) > 2 else None,
        )
    except KeyboardInterrupt:
        pass
