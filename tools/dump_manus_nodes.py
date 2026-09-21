#!/usr/bin/env python3
"""打印 /manus_glove_0 的骨骼节点拓扑，用于裁定 /hand_input 的取点是否错位。

这是对「骨架错位一节」最直接的检查：它绕过 manus_input_py 的转换，
直接看 MANUS SDK 报上来的节点树。

读这份输出时务必注意
--------------------
`joint_type` 字段**本身就是错的**，不能直接采信。manus_ros2 的
`JointTypeToString`（src/ManusDataPublisher.cpp）把 SDK 按**骨头**命名的枚举
翻译成按**关节**命名的字符串时整体错了一位：

    SDK 枚举 (骨头)              节点实际位置      代码标成    应该是
    FingerJointType_Metacarpal   掌骨根 ≈ 腕       "MCP"      腕/掌骨根
    FingerJointType_Proximal     近节指骨根        "PIP"      MCP
    FingerJointType_Intermediate 中节指骨根        "IP"       PIP
    FingerJointType_Distal       远节指骨根        "DIP"      DIP  (对)
    FingerJointType_Tip          指尖              "TIP"      TIP  (对)

骨骼绑定中节点位于骨头**根部**，所以「近节指骨的根」就是 MCP 关节。

因此判定要靠**几何**而非标签：看每个节点到父节点的距离。
掌骨约 6-8 cm，近节指骨约 3.5-5 cm，中节约 2-3.5 cm，远节约 1.5-2.5 cm。
第一段若有 6-8 cm，说明该链的第一个节点是掌骨根，不是 MCP。

用法
----
    source env_ros.sh
    python3 tools/dump_manus_nodes.py                    # 系统 python3 即可
    python3 tools/dump_manus_nodes.py --topic /manus_glove_1
"""

from __future__ import annotations

import argparse
import math
import sys

# 供参考的骨长区间（米）
BONE_REF = (
    ("掌骨", 0.055, 0.085),
    ("近节指骨", 0.032, 0.058),
    ("中节指骨", 0.016, 0.038),
    ("远节指骨", 0.012, 0.030),
)


def classify(length: float) -> str:
    hits = [name for name, lo, hi in BONE_REF if lo <= length <= hi]
    if not hits:
        return "?"
    return "/".join(hits)


def main() -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

    try:
        from manus_ros2_msgs.msg import ManusGlove
    except ImportError:
        print(
            "ERROR: 导入不到 manus_ros2_msgs。先 source env_ros.sh（它会 source\n"
            "       ros2_ws/install/setup.bash）再跑本脚本。",
            file=sys.stderr,
        )
        return 1

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/manus_glove_0")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    received = []
    try:
        node = Node("dump_manus_nodes")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        node.create_subscription(ManusGlove, args.topic, received.append, qos)
        deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
        while not received and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    if not received:
        print(
            f"{args.timeout:.0f}s 内没有收到 {args.topic}；manus_data_publisher 在跑吗？",
            file=sys.stderr,
        )
        return 1

    msg = received[-1]
    print(f"topic={args.topic}  glove_id={msg.glove_id}  side={msg.side}")
    print(f"raw_node_count={msg.raw_node_count}  实际节点数={len(msg.raw_nodes)}")
    print()

    pos = {n.node_id: (n.pose.position.x, n.pose.position.y, n.pose.position.z) for n in msg.raw_nodes}

    header = f"{'id':>4} {'parent':>7} {'chain':<8} {'标签':<8} {'到父节点':>10}   骨长量级判定"
    print(header)
    print("-" * len(header))

    for n in sorted(msg.raw_nodes, key=lambda x: x.node_id):
        parent = n.parent_node_id
        if parent in pos and parent != n.node_id:
            a, b = pos[n.node_id], pos[parent]
            length = math.dist(a, b)
            length_text = f"{length * 100:8.2f}cm"
            verdict = classify(length)
        else:
            length_text = f"{'—':>10}"
            verdict = "(根节点)"
        print(
            f"{n.node_id:>4} {parent:>7} {n.chain_type:<8} {n.joint_type:<8} "
            f"{length_text}   {verdict}"
        )

    print()
    print("=" * 70)
    print("怎么读：对每条手指链，从根往下第一段若是 6-8 cm，那一段就是掌骨，")
    print("说明该链的第一个节点在腕侧，**不是 MCP**。此时 MediaPipe 的 21 点应")
    print("从第二个节点（近节指骨根 = 真 MCP）开始取，并一直取到 Tip。")
    print()
    print("对照 manus_input_py 的 MEDIAPIPE_TO_MANUS 表，检查它每指取的 4 个 id")
    print("是否正好是 [近节根, 中节根, 远节根, 指尖]。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
