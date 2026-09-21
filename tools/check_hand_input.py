#!/usr/bin/env python3
"""检查 /hand_input 的 21 点骨架是否存在"整体错位一节"的问题。

背景
----
MANUS 的骨骼节点位于每根骨头的**根部**，因此：

    FingerJointType_Metacarpal   掌骨根  ≈ 腕部，不是 MCP 关节
    FingerJointType_Proximal     近节指骨根 = 真正的 MCP
    FingerJointType_Intermediate 中节指骨根 = 真正的 PIP
    FingerJointType_Distal       远节指骨根 = 真正的 DIP
    FingerJointType_Tip          指尖       = TIP

若 manus_input_py 取的是 [Metacarpal, Proximal, Intermediate, Distal] 的根，
发布出来的每根手指就是 [掌骨根, MCP, PIP, DIP]，而 MediaPipe 要的是
[MCP, PIP, DIP, TIP] —— 整条链向近端错一节，指尖从未发布。

用法
----
    # 离线：检查一帧已抓好的数据
    python3 tools/check_hand_input.py tests/data/hand_input_straight.yaml

    # 在线：直接订阅 /hand_input 抓一帧再检查（需先 source env_ros.sh）
    python3 tools/check_hand_input.py --live

    # 同时跑一遍 retarget，看四指 abd（需要 conda wuji2 的 wuji_sdk）
    python3 tools/check_hand_input.py tests/data/hand_input_straight.yaml --retarget
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

FINGERS = ("index", "middle", "ring", "pinky")
# MediaPipe: 0=WRIST, 1-4=THUMB, 5-8=INDEX, 9-12=MIDDLE, 13-16=RING, 17-20=PINKY
MCP_IDX = {"index": 5, "middle": 9, "ring": 13, "pinky": 17}

# 成年人手部尺寸参考区间（米）。取得宽松，只用于抓"量级不对"，不做精细判别。
REF = {
    "wrist_to_mcp": (0.070, 0.115),
    "mcp_spread": (0.060, 0.095),
    "proximal": (0.032, 0.058),
    "intermediate": (0.018, 0.038),
    "distal": (0.014, 0.030),
}


def load_yaml_frame(path: str) -> np.ndarray:
    import yaml

    with open(path, encoding="utf-8") as handle:
        msg = next(x for x in yaml.safe_load_all(handle) if x)
    values = np.asarray(msg["data"], dtype=np.float64)
    if values.size != 63:
        raise ValueError(f"{path}: 期望 63 个 float，实际 {values.size}")
    return values.reshape(21, 3)


def capture_live_frame(topic: str, timeout_s: float) -> np.ndarray:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import Float32MultiArray

    rclpy.init()
    try:
        node = Node("check_hand_input")
        received: list[np.ndarray] = []
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        node.create_subscription(
            Float32MultiArray,
            topic,
            lambda m: received.append(np.asarray(m.data, dtype=np.float64)),
            qos,
        )
        deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while not received and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    if not received:
        raise TimeoutError(f"{timeout_s:.0f}s 内没有收到 {topic}；manus_input 在跑吗？")
    values = received[-1]
    if values.size != 63:
        raise ValueError(f"期望 63 个 float，实际 {values.size}")
    return values.reshape(21, 3)


def _fmt(value: float, lo: float, hi: float) -> tuple[str, bool]:
    ok = lo <= value <= hi
    return f"{value * 100:6.1f} cm  (参考 {lo * 100:.0f}-{hi * 100:.0f})  {'OK' if ok else 'FAIL'}", ok


def check(points: np.ndarray, *, retarget: bool) -> int:
    dist = lambda a, b: float(np.linalg.norm(points[b] - points[a]))
    failures = 0

    print("=" * 66)
    print("判据 1 — 四个「MCP」是否落在同一点/同一平面")
    print("-" * 66)
    mcp = points[[MCP_IDX[f] for f in FINGERS]]
    wrist_dists = [dist(0, MCP_IDX[f]) for f in FINGERS]
    for name, value in zip(FINGERS, wrist_dists):
        print(f"  腕 -> {name:7s} MCP   {value * 100:6.2f} cm")

    # 真手的 MCP 排呈弓形，任一轴上都该有毫米级的高低差。若某一轴的极差塌缩到
    # float32 量化噪声（~1e-8 m）而整体又确实散开，说明这四个点在源数据里共用
    # 同一个坐标值 —— 掌骨在腕侧的会聚端正是这样。
    spans = mcp.max(axis=0) - mcp.min(axis=0)
    overall = float(np.max(np.linalg.norm(mcp - mcp.mean(axis=0), axis=1)))
    degenerate = [axis for axis in range(3) if spans[axis] < 1e-6]
    print(f"  各轴极差  x={spans[0]:.2e}  y={spans[1]:.2e}  z={spans[2]:.2e} m")
    if degenerate and overall > 1e-3:
        names = ", ".join("xyz"[a] for a in degenerate)
        print(f"  !! {names} 轴极差已塌缩到量化噪声量级，而四点整体散开 {overall * 100:.2f} cm")
        print("     真手 MCP 呈弓形，任一轴都该有毫米级高低差 —— 这是")
        print("     「取到了掌骨根而非 MCP」的强信号")
        failures += 1
    else:
        print("  四个 MCP 分布正常   OK")

    print()
    print("判据 2 — 腕到 MCP 的距离量级")
    print("-" * 66)
    lo, hi = REF["wrist_to_mcp"]
    text, ok = _fmt(dist(0, MCP_IDX["middle"]), lo, hi)
    print(f"  腕 -> 中指 MCP        {text}")
    if not ok:
        print("     偏小说明这个点其实贴着腕部，是掌骨根")
        failures += 1

    print()
    print("判据 3 — MCP 排横向展宽（食指 -> 小指）")
    print("-" * 66)
    lo, hi = REF["mcp_spread"]
    text, ok = _fmt(dist(MCP_IDX["index"], MCP_IDX["pinky"]), lo, hi)
    print(f"  食指MCP -> 小指MCP    {text}")
    if not ok:
        print("     偏窄说明取到的是掌骨在腕侧的会聚端")
        failures += 1

    print()
    print("判据 4 — 指节长度与递减关系")
    print("-" * 66)
    print(f"  {'':8s}{'近节':>10s}{'中节':>10s}{'远节':>10s}   判定")
    for finger in FINGERS:
        base = MCP_IDX[finger]
        seg = [dist(base, base + 1), dist(base + 1, base + 2), dist(base + 2, base + 3)]
        notes = []
        if not REF["proximal"][0] <= seg[0] <= REF["proximal"][1]:
            notes.append("近节超界")
        if not seg[0] > seg[1] > seg[2]:
            notes.append("未递减")
        verdict = "OK" if not notes else "FAIL: " + "/".join(notes)
        if notes:
            failures += 1
        print(
            f"  {finger:8s}{seg[0] * 100:9.2f}{seg[1] * 100:10.2f}{seg[2] * 100:10.2f}   {verdict}"
        )
    print("  注：近节明显偏长（>6 cm）通常说明它其实是掌骨长度")

    if retarget:
        print()
        print("判据 5 — retarget 输出的四指侧摆")
        print("-" * 66)
        try:
            from wuji_sdk import HandModel, Handedness, RetargetSession

            session = RetargetSession.for_hand(
                HandModel.WujiHand2, side=Handedness.Right
            )
            for _ in range(8):
                q = np.asarray(session.step(points.astype(np.float32)), dtype=np.float64)
            abd = np.degrees(q[[5, 9, 13, 17]])
            for name, value in zip(FINGERS, abd):
                print(f"  {name:8s} abd = {value:+7.2f} deg")
            if np.all(abd > 5.0) or np.all(abd < -5.0):
                print("  !! 四指同号且量级可观 —— 与「同向外摆」症状一致")
                failures += 1
            else:
                print("  四指侧摆无明显同向偏置   OK")
        except ImportError:
            print("  跳过：当前解释器没有 wuji_sdk（用 conda wuji2 的 python 跑）")

    print()
    print("=" * 66)
    if failures:
        print(f"结论：{failures} 项判据未通过 —— 骨架很可能整体错位一节。")
        print("下一步：用 tools/dump_manus_nodes.py 直接看 /manus_glove_0 的节点拓扑。")
    else:
        print("结论：所有判据通过，骨架尺寸正常。")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frame", nargs="?", help="已抓好的一帧 yaml；省略则配合 --live")
    parser.add_argument("--live", action="store_true", help="直接订阅 /hand_input 抓一帧")
    parser.add_argument("--topic", default="/hand_input")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--retarget", action="store_true", help="附带跑一遍 retarget")
    args = parser.parse_args()

    if args.live:
        points = capture_live_frame(args.topic, args.timeout)
        print(f"已从 {args.topic} 抓到一帧\n")
    elif args.frame:
        points = load_yaml_frame(args.frame)
        print(f"已读入 {args.frame}\n")
    else:
        parser.error("需要给一个 yaml 文件，或加 --live")

    return check(points, retarget=args.retarget)


if __name__ == "__main__":
    raise SystemExit(main())
