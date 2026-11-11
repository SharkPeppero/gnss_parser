
"""交互式离线 SPP：首次输入 Bag 后启动算法和 RViz，后续只切换 Bag。"""

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
            "\n[spp_offline] 输入 Bag 路径（回车或 q 退出）：",
            flush=True,
        )
        line = tty.readline()
        if not line:
            return None

        path = os.path.expanduser(line.strip().strip("'\""))

        if path == "" or path.lower() == "q":
            return None

        if not os.path.exists(path):
            print(f"[spp_offline] 路径不存在：{path}", flush=True)
            continue

        return os.path.abspath(path)


def prompt_loop(rate, first_bag):
    """依次播放 Bag，算法节点保持运行。"""
    with open("/dev/tty", "r") as tty:
        path = first_bag

        while path is not None:
            print(
                f"[spp_offline] 开始回放：{path} (rate={rate})",
                flush=True,
            )

            try:
                result = subprocess.run([
                    "ros2", "bag", "play", path,
                    "--rate", rate,
                    "--delay", "2.0",
                ])
                if result.returncode != 0:
                    print(
                        f"[spp_offline] 回放退出码：{result.returncode}",
                        flush=True,
                    )
            except KeyboardInterrupt:
                return

            print("[spp_offline] 回放结束。", flush=True)
            path = read_bag_path(tty)


def start_offline(context, *args, **kwargs):
    """首次读取有效 Bag 路径后，创建算法、RViz 和回放进程。"""
    try:
        with open("/dev/tty", "r") as tty:
            first_bag = read_bag_path(tty)
    except OSError as exc:
        print(f"[spp_offline] 无法读取终端：{exc}", flush=True)
        return [EmitEvent(event=Shutdown())]

    if first_bag is None:
        return [EmitEvent(event=Shutdown())]

    rate = LaunchConfiguration("rate").perform(context)

    spp_node = Node(
        package="gnss_comm",
        executable="spp_solver_node",
        name="spp_solver_node",
        output="screen",
        parameters=[
            LaunchConfiguration("ros_cfg"),
            LaunchConfiguration("alg_cfg"),
            {"output_path": LaunchConfiguration("output_path")},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_cfg")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

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

    return [
        spp_node,
        rviz_node,
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
            "rate",
            default_value="1.0",
            description="Bag 回放倍速。",
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
            "output_path",
            default_value=os.path.join(
                os.path.expanduser("~"),
                "gnss_comm_output",
                "spp",
            ),
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
        OpaqueFunction(function=start_offline),
    ])


if __name__ == "__main__":
    try:
        prompt_loop(
            sys.argv[1] if len(sys.argv) > 1 else "1.0",
            sys.argv[2] if len(sys.argv) > 2 else None,
        )
    except KeyboardInterrupt:
        pass
