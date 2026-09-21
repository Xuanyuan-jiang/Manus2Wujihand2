#!/usr/bin/env python3
"""校验 MANUS 手套标定文件（.mcal）是否自洽。

标定坏掉时 SDK **不会报错** —— `side` 字段只是个标签，`CoreSdk_SetGloveCalibration`
不会去核对文件里的几何是不是真的属于那一侧。于是一份"标着 right、内容是左手"的
文件会被静默接受，下游表现为关节角离谱、手指朝腕部折回，很难追到源头。

本脚本做三项检查：
  1. side 标签 与 几何手性 是否一致
  2. wristRotationOffset 的旋转角是否在合理范围（正常标定 < 30 度）
  3. fingerLength 是否落在成年人区间

用法:
    python3 tools/check_mcal.py ros2_ws/src/manus_ros2/calibration/*.mcal
"""

from __future__ import annotations

import json
import math
import sys

import numpy as np

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
# 成年人手指总长（掌骨根到指尖）合理区间，单位 m
LENGTH_REF = {
    "thumb": (0.090, 0.140),
    "index": (0.075, 0.115),
    "middle": (0.080, 0.125),
    "ring": (0.075, 0.120),
    "pinky": (0.055, 0.095),
}
MAX_WRIST_ROTATION_DEG = 30.0
# 拇指离掌平面至少要有这么远，手性判据才有意义（单位 m）
MIN_THUMB_OUT_OF_PLANE = 0.003


def load(path: str) -> dict:
    raw = open(path, encoding="utf-8", errors="replace").read()
    start = raw.index("{")
    depth = 0
    for i, ch in enumerate(raw[start:], start):
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return json.loads(raw[start : i + 1])
    raise ValueError(f"{path}: 找不到完整的 JSON 块")


def chirality(cal: dict) -> tuple[float, float]:
    """以五指 cmcPosition 求手性不变量，返回 ``(值, 拇指离掌面的距离)``。

    ⚠️ 这个判据**条件数很差**：它靠拇指偏离掌平面的程度来定手性，而 .mcal 里记的
    是掌骨根位置，五个点往往近乎共面。实测官方默认标定的拇指离掌面约 9 mm，
    而某些标定只有 0.03 mm —— 那时符号纯粹是浮点噪声，据此判手性会得出错误结论。

    所以这里把拇指离掌面的距离一并返回，由调用方判断结果是否可信；低于阈值时
    应当报「无法判定」，而不是给一个看似确定的答案。

    真正可靠的手性判据需要**屈曲姿态**下的手指弯曲方向（见
    manus_input_py 的 ``_chirality``），而 .mcal 是静态手型，没有这个信息。
    """
    p = {
        f: np.array([cal[f]["cmcPosition"][a] for a in "xyz"])
        for f in FINGERS
    }
    normal = np.cross(p["index"] - p["middle"], p["pinky"] - p["middle"])
    norm = float(np.linalg.norm(normal))
    if norm < 1e-12:
        return 0.0, 0.0
    value = float(np.dot(p["thumb"] - p["middle"], normal))
    return value, abs(value) / norm


def check(path: str) -> list[str]:
    cal = load(path)
    problems: list[str] = []

    side = str(cal.get("side", "")).lower()
    chi, out_of_plane = chirality(cal)
    print(f"  side 标签        : {side!r}")
    print(f"  拇指离掌面       : {out_of_plane * 1000:.2f} mm  (判据可信度)")
    if out_of_plane < MIN_THUMB_OUT_OF_PLANE:
        print(f"  几何手性         : 无法判定 —— 五指掌骨根近乎共面，符号是数值噪声")
        print(f"                     （需 > {MIN_THUMB_OUT_OF_PLANE * 1000:.0f} mm 才可信）")
    else:
        geometric_side = "left" if chi > 0 else "right"
        print(f"  几何手性不变量   : {chi:+.3e}  ->  实际是 {geometric_side} 手")
        if side in ("left", "right") and side != geometric_side:
            problems.append(
                f"side 标着 {side!r}，但几何是 {geometric_side} 手 —— "
                f"标定时很可能戴错了手或选错了手套"
            )

    rotation = cal.get("wristRotationOffset")
    if rotation:
        angle = math.degrees(2 * math.acos(min(1.0, abs(float(rotation["w"])))))
        print(f"  腕部旋转偏移     : {angle:.1f}°")
        if angle > MAX_WRIST_ROTATION_DEG:
            problems.append(
                f"腕部旋转偏移 {angle:.1f}° 远超正常范围（< {MAX_WRIST_ROTATION_DEG:.0f}°）"
                f" —— 整只手的骨架会被整体转歪"
            )

    print("  手指总长 (m)     :")
    for finger in FINGERS:
        length = cal.get(finger, {}).get("fingerLength")
        if length is None:
            problems.append(f"{finger} 缺少 fingerLength")
            continue
        lo, hi = LENGTH_REF[finger]
        ok = lo <= length <= hi
        print(f"      {finger:8s} {length:.4f}  (参考 {lo:.3f}-{hi:.3f})  {'OK' if ok else 'FAIL'}")
        if not ok:
            problems.append(f"{finger} 总长 {length:.4f} m 超出成年人区间 {lo:.3f}-{hi:.3f}")

    return problems


def main(argv: list[str]) -> int:
    if not argv:
        print(__doc__)
        return 2

    failed = 0
    for path in argv:
        print("=" * 70)
        print(path)
        print("-" * 70)
        try:
            problems = check(path)
        except Exception as exc:
            print(f"  解析失败: {type(exc).__name__}: {exc}")
            failed += 1
            continue
        if problems:
            failed += 1
            print("\n  问题:")
            for line in problems:
                print(f"    - {line}")
        else:
            print("\n  自洽性检查全部通过")
        print()

    if failed:
        print(f"{failed}/{len(argv)} 个文件有问题 —— 不要用它们做遥操作，重新标定。")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
