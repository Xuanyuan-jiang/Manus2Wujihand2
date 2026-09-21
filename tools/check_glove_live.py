#!/usr/bin/env python3
"""把手摊平，用它检查 MANUS 手套的弯曲传感器标定是否正确。

读 `/manus_glove_0` 里 MANUS **自己算的** ergonomics 角度 —— 那是 SDK 的输出，
在 manus_input_py 和 retarget 之前，所以能把"标定问题"和"我们代码的问题"分开。

手掌完全摊平时，所有 Stretch 与 Spread 都应接近 0（约 ±15 度内）。
若 MCP 接近零但 PIP 读到上百度，就是**弯曲传感器**没标定好 —— 注意这类问题
换 .mcal 文件是修不好的：.mcal 只含手型几何（cmcPosition / fingerLength /
wristPosition），不含传感器的原始值到角度的映射。那部分在 MANUS Core 的用户
配置里，只能重跑 MANUS Core 的标定流程。

用法:
    source env_ros.sh
    python3 tools/check_glove_live.py                 # 摊平手，看有没有 FAIL
    python3 tools/check_glove_live.py --topic /manus_glove_1
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

# 摊平手时各类角度的容许范围（度）
FLAT_TOLERANCE = 15.0
# 人体关节活动度上限，超过即为不可能值
ANATOMICAL_MAX = {"MCPStretch": 100.0, "PIPStretch": 115.0,
                  "DIPStretch": 90.0, "Spread": 30.0}


def classify(name: str) -> str:
    for key in ANATOMICAL_MAX:
        if name.endswith(key):
            return key
    return ""


def main() -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

    try:
        from manus_ros2_msgs.msg import ManusGlove
    except ImportError:
        print("ERROR: 导入不到 manus_ros2_msgs，先 source env_ros.sh", file=sys.stderr)
        return 1

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/manus_glove_0")
    parser.add_argument("--seconds", type=float, default=2.0,
                        help="采样时长，取中位数以抑制抖动")
    args = parser.parse_args()

    rclpy.init()
    frames = []
    try:
        node = Node("check_glove_live")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        node.create_subscription(ManusGlove, args.topic, frames.append, qos)
        print(f"请把手完全摊平，采样 {args.seconds:.0f} 秒 …")
        t0 = time.time()
        while time.time() - t0 < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    if not frames:
        print(f"没有收到 {args.topic}；manus_data_publisher 在跑吗？", file=sys.stderr)
        return 1

    names = [e.type for e in frames[0].ergonomics]
    values = np.median(np.array([[e.value for e in f.ergonomics] for f in frames]), axis=0)

    print(f"\n收到 {len(frames)} 帧，取中位数。摊平手时各项都应接近 0。\n")
    print(f"  {'ergonomics':32s}{'中位值':>10s}   判定")
    impossible, off_zero = [], []
    for name, value in zip(names, values):
        kind = classify(name)
        limit = ANATOMICAL_MAX.get(kind)
        if limit is not None and abs(value) > limit:
            verdict = f"FAIL 超出人体极限 ±{limit:.0f}"
            impossible.append(name)
        elif abs(value) > FLAT_TOLERANCE:
            verdict = f"偏离零位 (>{FLAT_TOLERANCE:.0f})"
            off_zero.append(name)
        else:
            verdict = "OK"
        print(f"  {name:32s}{value:10.1f}   {verdict}")

    print()
    if impossible:
        print(f"{len(impossible)} 项超出人体关节活动度 —— 弯曲传感器标定是错的：")
        print(f"  {', '.join(impossible)}")
        print("\n这类问题换 .mcal 文件修不好（.mcal 只有几何，没有传感器映射）。")
        print("需要在 MANUS Core 里重跑手套标定流程。")
        return 1
    if off_zero:
        print(f"{len(off_zero)} 项偏离零位较多，若手确实摊平则标定精度不足：")
        print(f"  {', '.join(off_zero)}")
        return 1
    print("全部接近零位 —— 弯曲传感器标定正常。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
