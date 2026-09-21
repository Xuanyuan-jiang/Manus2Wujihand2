#!/usr/bin/env python3
"""验证 manus_input_py 的取点逻辑：按骨头语义选，且对新旧 joint_type 标签都正确。

用合成骨架离线测，不需要手套。

    source env_ros.sh
    python3 tests/test_skeleton_mapping.py
"""

from __future__ import annotations

import sys
import types

import numpy as np

# 合成一只解剖学比例正常的右手（米）。放在 y=0 平面上，
# 这样 _node_position 的 y 取反不影响断言。
#   x 向小指侧，z 向指尖
WRIST = (0.0, 0.0, 0.0)

# 每根骨头的根节点位置。手指链：掌骨根 / 近节根(=MCP) / 中节根(=PIP) / 远节根(=DIP) / 指尖
# 掌骨在腕侧会聚、向 MCP 排发散,这是真实解剖结构,也正是错位一节时
# 展宽会明显偏窄的原因。
FINGER_GEOMETRY = {
    # chain:   掌骨根            MCP               PIP               DIP               TIP
    "Index": ((-0.020, 0.0, 0.020), (-0.037, 0.0, 0.093), (-0.039, 0.0, 0.138), (-0.040, 0.0, 0.163), (-0.041, 0.0, 0.182)),
    "Middle": ((-0.006, 0.0, 0.020), (-0.011, 0.0, 0.098), (-0.012, 0.0, 0.148), (-0.013, 0.0, 0.176), (-0.013, 0.0, 0.197)),
    "Ring": ((0.008, 0.0, 0.020), (0.015, 0.0, 0.094), (0.016, 0.0, 0.140), (0.017, 0.0, 0.166), (0.017, 0.0, 0.186)),
    "Pinky": ((0.020, 0.0, 0.020), (0.040, 0.0, 0.084), (0.042, 0.0, 0.120), (0.043, 0.0, 0.140), (0.044, 0.0, 0.158)),
}
# 拇指只有近节、远节两根指骨，没有中节(Intermediate)。实机数据证实拇指链为
# [Metacarpal, Proximal, Distal, Tip]，其唯一的 IP 关节在远节指骨根部。
# 掌骨根(=CMC) / 近节根(=MCP) / 远节根(=IP) / 指尖
THUMB_GEOMETRY = ((-0.024, 0.0, 0.018), (-0.045, 0.0, 0.052), (-0.062, 0.0, 0.080), (-0.074, 0.0, 0.101))

FINGER_BONES = ("Metacarpal", "Proximal", "Intermediate", "Distal", "Tip")
THUMB_BONES = ("Metacarpal", "Proximal", "Distal", "Tip")

# manus_ros2 当前（未修）的 JointTypeToString 输出
LEGACY_LABEL = {
    "Metacarpal": "MCP",
    "Proximal": "PIP",
    "Intermediate": "IP",
    "Distal": "DIP",
    "Tip": "TIP",
}



# ---------------------------------------------------------------------------
# 手性基准：Hand 2 的 right.urdf 在 45 度屈曲下 FK 出来的 21 点。
#
# 这是**唯一可靠的手性真值**。历史上 _node_position 的 y 取反被反复改动，就是因为
# 缺这个基准：早期判据取 cross(index-wrist, pinky-wrist)·(thumb-wrist)，在摊平的手
# 上退化（拇指离掌面不足 1 mm），符号是噪声，据此判断会得出相反结论。
#
# 手指屈曲后判据条件数很好，这组点的手性值为 -9.17e-02，可信度 9.18e-02。
# ---------------------------------------------------------------------------
RIGHT_HAND_FLEXED_45 = (
    (+0.000000, +0.000000, +0.000000),
    (+0.016660, -0.008130, +0.017017),
    (+0.047607, -0.026083, +0.043384),
    (+0.042302, -0.033917, +0.077081),
    (+0.038059, -0.040184, +0.104039),
    (+0.020353, +0.003360, +0.070679),
    (+0.020940, -0.031446, +0.109462),
    (+0.018108, -0.060789, +0.110547),
    (+0.015842, -0.084264, +0.111415),
    (-0.001473, +0.008340, +0.073903),
    (-0.003154, -0.026333, +0.112771),
    (-0.003177, -0.055816, +0.113786),
    (-0.003196, -0.079402, +0.114598),
    (-0.021879, +0.005626, +0.064714),
    (-0.026099, -0.030000, +0.102514),
    (-0.024172, -0.059432, +0.103079),
    (-0.022631, -0.082977, +0.103530),
    (-0.039145, +0.000761, +0.054041),
    (-0.044580, -0.032213, +0.088657),
    (-0.041082, -0.059474, +0.089583),
    (-0.038283, -0.081282, +0.090325),
)


# 左手基准：同样来自 left.urdf 的 45 度屈曲 FK。左右手限位完全相同，
# 手性符号相反（右负左正），大小对称。
LEFT_HAND_FLEXED_45 = (
    (+0.000000, +0.000000, +0.000000),
    (-0.016660, -0.008130, +0.017017),
    (-0.047782, -0.025911, +0.044011),
    (-0.042175, -0.035086, +0.078368),
    (-0.037690, -0.042426, +0.105854),
    (-0.020353, +0.003360, +0.070679),
    (-0.020940, -0.031446, +0.109461),
    (-0.018107, -0.060789, +0.110546),
    (-0.015841, -0.084264, +0.111414),
    (+0.001473, +0.008340, +0.073903),
    (+0.003155, -0.026333, +0.112772),
    (+0.003179, -0.055816, +0.113787),
    (+0.003198, -0.079402, +0.114598),
    (+0.021879, +0.005626, +0.064714),
    (+0.026098, -0.030000, +0.102514),
    (+0.024170, -0.059432, +0.103079),
    (+0.022627, -0.082977, +0.103530),
    (+0.039145, +0.000761, +0.054041),
    (+0.044492, -0.032926, +0.087977),
    (+0.040839, -0.060180, +0.088371),
    (+0.037916, -0.081982, +0.088687),
)


def check_chirality(node) -> list[str]:
    """右手屈曲姿态的手性必须为负；镜像后必须为正。"""
    failures = []
    right = np.asarray(RIGHT_HAND_FLEXED_45, dtype=np.float32)
    value, confidence = node._chirality(right)
    print(f"  右手 URDF 屈曲 45 度: 手性={value:+.4e} 可信度={confidence:.4e}", end="")
    if confidence < 0.015:
        failures.append(f"基准姿态的可信度只有 {confidence:.4e}，判据已退化")
    if value >= 0:
        failures.append(f"右手手性应为负，实际 {value:+.4e}")

    mirrored = right.copy()
    mirrored[:, 1] *= -1
    mvalue, _ = node._chirality(mirrored)
    if mvalue <= 0:
        failures.append(f"镜像后手性应为正，实际 {mvalue:+.4e}")

    # 左手 URDF 必须给出正号，且与右手大小对称
    left = np.asarray(LEFT_HAND_FLEXED_45, dtype=np.float32)
    lvalue, lconf = node._chirality(left)
    print(f"\n  左手 URDF 屈曲 45 度: 手性={lvalue:+.4e} 可信度={lconf:.4e}", end="")
    if lvalue <= 0:
        failures.append(f"左手手性应为正，实际 {lvalue:+.4e}")
    if abs(abs(lvalue) - abs(value)) > 1e-3:
        failures.append(f"左右手性大小应对称：右 {value:+.4e} 左 {lvalue:+.4e}")

    # 伸直的手必须被判为「退化」，不能给出貌似确定的答案
    flat = right.copy()
    flat[:, 1] = 0.0
    _, flat_conf = node._chirality(flat)
    if flat_conf >= 0.015:
        failures.append(f"摊平手的可信度应低于阈值，实际 {flat_conf:.4e}")

    print(f"  -> {'FAIL' if failures else 'PASS'}")
    return failures


def make_node(node_id: int, chain: str, bone: str, xyz, legacy: bool):
    position = types.SimpleNamespace(x=xyz[0], y=-xyz[1], z=xyz[2])
    return types.SimpleNamespace(
        node_id=node_id,
        parent_node_id=node_id - 1,
        chain_type=chain,
        joint_type=LEGACY_LABEL[bone] if legacy else bone,
        pose=types.SimpleNamespace(position=position),
    )


def make_glove(legacy: bool):
    nodes = [make_node(1, "Hand", "Metacarpal", WRIST, legacy)]
    next_id = 2
    for bone, xyz in zip(THUMB_BONES, THUMB_GEOMETRY):
        nodes.append(make_node(next_id, "Thumb", bone, xyz, legacy))
        next_id += 1
    for chain in ("Index", "Middle", "Ring", "Pinky"):
        for bone, xyz in zip(FINGER_BONES, FINGER_GEOMETRY[chain]):
            nodes.append(make_node(next_id, chain, bone, xyz, legacy))
            next_id += 1
    return types.SimpleNamespace(raw_nodes=nodes, side="right", glove_id=0)


def run_case(node, legacy: bool) -> list[str]:
    label = "旧标签(未修的 manus_ros2)" if legacy else "新标签(已修的 manus_ros2)"
    pose = node._convert_to_mediapipe(make_glove(legacy))
    failures = []

    def near(got, want, tol, what):
        if abs(got - want) > tol:
            failures.append(f"[{label}] {what}: 期望 {want:.4f}, 实际 {got:.4f}")

    dist = lambda a, b: float(np.linalg.norm(pose[b] - pose[a]))

    # 腕
    near(float(np.linalg.norm(pose[0])), 0.0, 1e-6, "腕应在原点")
    # 四指 MCP 必须是近节指骨的根，而不是掌骨根
    near(pose[5][2], 0.093, 1e-4, "食指 MCP 应取近节根 (z=0.093)")
    near(pose[9][2], 0.098, 1e-4, "中指 MCP 应取近节根 (z=0.098)")
    # 指尖必须被取到
    near(pose[8][2], 0.182, 1e-4, "食指 TIP 应取指尖 (z=0.182)")
    near(pose[20][2], 0.158, 1e-4, "小指 TIP 应取指尖 (z=0.158)")
    # 拇指从掌骨根起
    near(pose[1][2], 0.018, 1e-4, "拇指 CMC 应取掌骨根 (z=0.018)")
    near(pose[4][2], 0.101, 1e-4, "拇指 TIP 应取指尖 (z=0.101)")
    # 整体尺寸
    near(dist(0, 9), 0.0986, 2e-3, "腕->中指MCP")
    near(dist(5, 17), 0.0775, 2e-3, "MCP 横向展宽")
    # 指节递减
    for name, base in (("index", 5), ("middle", 9), ("ring", 13), ("pinky", 17)):
        seg = [dist(base, base + 1), dist(base + 1, base + 2), dist(base + 2, base + 3)]
        if not seg[0] > seg[1] > seg[2]:
            failures.append(f"[{label}] {name} 指节未递减: {[round(s, 4) for s in seg]}")
        if seg[0] > 0.060:
            failures.append(f"[{label}] {name} 近节 {seg[0]:.4f} 过长，疑似仍取到掌骨")

    print(f"  {label}: 腕->中指MCP={dist(0, 9) * 100:.1f}cm "
          f"展宽={dist(5, 17) * 100:.1f}cm 食指近节={dist(5, 6) * 100:.1f}cm "
          f"-> {'FAIL' if failures else 'PASS'}")
    return failures


def main() -> int:
    import rclpy
    from manus_input_py.manus_input_node import ManusInputConfig, ManusInputNode

    rclpy.init()
    try:
        node = ManusInputNode(ManusInputConfig())
        print("按骨头语义选点：")
        failures = run_case(node, legacy=True) + run_case(node, legacy=False)
        print("\n手性基准（防止 y 取反被误改）：")
        failures += check_chirality(node)
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    print()
    if failures:
        for line in failures:
            print("  FAIL", line)
        print(f"\n{len(failures)} 项断言失败")
        return 1
    print("全部断言通过：新旧 joint_type 标签下都取到 [MCP, PIP, DIP, TIP]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
