#!/usr/bin/env python3
"""把 adaptive_finger_qp.py 的触觉 QP 插件接到 Wuji Hand 2 上。

插件本身不碰硬件，只做运动学层面的关节位置修正。本脚本负责三件适配：

  1. 读实体手的 20 轴当前位置，整理成 {URDF关节名: 位置}
  2. 取指尖三轴力，整理成 {指尖名: [Fx, Fy, Fz]}
  3. 把 result.joint_positions 发回实体手（默认不发，只打印）

    # 离线自检，不需要手
    ~/miniconda3/envs/wuji2/bin/python scripts/07_tactile_qp_wuji2.py --selftest

    # 接实体手只读跑一圈（默认；读位置、算 QP、打印，不下发）
    ~/miniconda3/envs/wuji2/bin/python scripts/07_tactile_qp_wuji2.py --side right \
        --force middle=0,0,9 --duration 3

    # 真下发（要求 20 轴无故障；会 enable，finally 里失能）
    ~/miniconda3/examples/... --control

## 关节顺序

Hand 2 的 flat-20 命令序与 hand2 URDF 的 independent_movable_joints **逐位一致**，
20 条限位也完全相同（见 --selftest 的核对）。所以不需要映射表，zip 即可：

    idx 0..3   thumb  cmc_flex / cmc_abd / mcp / ip
    idx 4..19  index/middle/ring/pinky 各 mcp_flex / mcp_abd / pip / dip

## 力从哪来

`--force-source device` 读设备的 fingertip/<finger>/data。**本机两只手的指尖触觉
模块都没装**（`tactile_status()` 五指全部 NodeOfflineError，`tactile_online_mask=0`），
所以该路径目前无法在本机验证，见 DeviceForceSource 里的说明。

`--force-source sim`（默认）用 `--force` 注入合成力，用于在没有触觉硬件时验证
整条链路：读位置 -> QP -> 限位钳制 -> 下发。合成力驱动实体手时脚本会显式告警。
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

# 插件是个单文件，放在仓库根下某个目录里（历史上叫过 haichao，现在叫 finger_qp）。
# 按文件名找而不是写死目录名，改名不会打断这里。
def _find_plugin_dir() -> Path:
    root = Path(__file__).resolve().parent.parent
    for candidate in sorted(root.glob("*/adaptive_finger_qp.py")):
        return candidate.parent
    raise SystemExit(
        f"ERROR: 在 {root} 下找不到 adaptive_finger_qp.py，无法加载触觉 QP 插件"
    )


sys.path.insert(0, str(_find_plugin_dir()))
from adaptive_finger_qp import AdaptiveFingerQP, QPConfig  # noqa: E402

from wuji_sdk import JointCommand, SdkManager, WujiHand2  # noqa: E402


JOINT_COUNT = 20
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
DEFAULT_ADDRESSES = {
    "right": "192.168.1.111:7447",  # WH2KA01260818006
    "left": "192.168.1.110:7447",   # WH2JA01260813009
}
_URDF_ROOT = Path(
    "/home/mzsun/Projects/retarget_repo/wuji-retargeting/wuji_retargeting"
    "/wuji-description/hand2/body/urdf"
)
DEFAULT_URDFS = {"right": _URDF_ROOT / "right.urdf", "left": _URDF_ROOT / "left.urdf"}
# 指尖 link 名。两侧只差前缀，运行时会核对是否真的存在于 URDF。
TIP_LINK_SUFFIX = {
    "thumb": "thumb_tip",
    "index": "index_finger_tip",
    "middle": "middle_finger_tip",
    "ring": "ring_finger_tip",
    "pinky": "pinky_tip",
}
# 官方 Hand 2 Beta 2 限位，与 04_manus_wuji2.py 同一份。这里只用来**核对** URDF，
# 实际钳制用 URDF 解析出来的值 —— 两者一致是 --selftest 的断言之一。
_THUMB_LIMITS_DEG = ((-68, 74), (-85, 40), (-60, 90), (-60, 90))
_FINGER_LIMITS_DEG = ((-60, 90), (-40, 40), (-60, 120), (-60, 90))
OFFICIAL_LIMITS_RAD = np.deg2rad(
    np.asarray(_THUMB_LIMITS_DEG + _FINGER_LIMITS_DEG * 4, dtype=np.float64)
)
NON_BLOCKING_SEVERITY = "Warning"
_DIAG_WARMUP_FRAMES = 2


def joint_labels() -> list[str]:
    return [f"{f}_S{i + 1}" for f in FINGERS for i in range(4)]


def nid_to_index(nid: int) -> Optional[int]:
    if nid <= 0:
        return None
    bus, node_index = divmod(nid - 1, 5)
    if bus >= 5 or node_index >= 4:
        return None
    return bus * 4 + node_index


def tip_links(side: str) -> dict:
    prefix = "r_" if side == "right" else "l_"
    return {f: prefix + s for f, s in TIP_LINK_SUFFIX.items()}


def build_controller(side: str, urdf: Path, config: QPConfig) -> AdaptiveFingerQP:
    controller = AdaptiveFingerQP(str(urdf), tip_links=tip_links(side), config=config)
    names = list(controller.controlled_joint_names)
    if len(names) != JOINT_COUNT:
        raise RuntimeError(f"URDF 解析出 {len(names)} 个可动关节，期望 {JOINT_COUNT}")
    return controller


def check_joint_order(controller: AdaptiveFingerQP) -> np.ndarray:
    """核对 URDF 关节顺序/限位与官方 flat-20 表一致，返回 URDF 限位 (20,2)。"""
    names = list(controller.controlled_joint_names)
    lo, hi = controller.model.joint_limits(names)
    limits = np.stack([lo, hi], axis=1)
    bad = []
    for i, name in enumerate(names):
        if not np.allclose(limits[i], OFFICIAL_LIMITS_RAD[i], atol=math.radians(1.0)):
            bad.append(
                f"  idx {i} {joint_labels()[i]} ({name}): URDF "
                f"{math.degrees(limits[i][0]):.1f}..{math.degrees(limits[i][1]):.1f} "
                f"vs 官方 {math.degrees(OFFICIAL_LIMITS_RAD[i][0]):.1f}.."
                f"{math.degrees(OFFICIAL_LIMITS_RAD[i][1]):.1f}"
            )
    if bad:
        raise RuntimeError(
            "URDF 关节顺序或限位与 Hand 2 的 flat-20 命令序不符，不能直接 zip：\n"
            + "\n".join(bad)
        )
    return limits


# --------------------------------------------------------------------------- 力

class SimForceSource:
    """合成力。用于没有触觉硬件时验证整条链路。"""

    def __init__(self, forces: dict):
        self.forces = forces

    def read(self) -> dict:
        return dict(self.forces)

    def describe(self) -> str:
        if not self.forces:
            return "sim（无力，QP 应输出全零 delta）"
        return "sim " + " ".join(
            f"{k}=[{v[0]:.1f},{v[1]:.1f},{v[2]:.1f}]" for k, v in self.forces.items()
        )


class DeviceForceSource:
    """设备指尖触觉。

    **本机未验证**：两只 Hand 2 的指尖触觉模块都没装（``tactile_status()`` 五指
    全部抛 ``NodeOfflineError``，``tactile_online_mask = 0``，订阅
    ``fingertip/*/data`` 收不到帧）。SDK 把这一路定义为「纯数值载荷，按
    ``FingertipSensorInfo.format`` 解释」，而 format 只有接上模块才读得到 ——
    没有硬件就无法知道三轴力在载荷里的位置和量纲。

    所以这里不猜格式：先 ``tactile_status`` 逐指确认在线，再把 info.format 打出来。
    真装上模块后，按打印出来的 format 在 ``_decode`` 里补完解码即可。
    """

    def __init__(self, hand):
        self.hand = hand
        self.subs: dict = {}
        offline = []
        for finger in FINGERS:
            try:
                self.hand.tactile_status(finger)
            except Exception as exc:
                offline.append(f"{finger}: {type(exc).__name__}: {exc}")
        if offline:
            raise RuntimeError(
                "指尖触觉模块不在线，--force-source device 无法使用：\n  "
                + "\n  ".join(offline)
                + "\n装上模块后重试；只想验证链路请用 --force-source sim。"
            )
        for finger in FINGERS:
            self.subs[finger] = self.hand.device.subscribe(f"fingertip/{finger}/data")

    def _decode(self, finger: str, payload) -> list:
        raise NotImplementedError(
            f"fingertip/{finger}/data 的载荷布局未知。请先打印 FingertipSensorInfo."
            "format，按它补完本方法。"
        )

    def read(self) -> dict:
        out = {}
        for finger, sub in self.subs.items():
            frame = sub.recv()
            if frame is not None:
                out[finger] = self._decode(finger, frame)
        return out

    def describe(self) -> str:
        return "device（fingertip/*/data）"

    def close(self) -> None:
        for sub in self.subs.values():
            try:
                sub.close()
            except Exception:
                pass


# ------------------------------------------------------------------------- 硬件

def read_positions(hand) -> np.ndarray:
    sub = hand.joint_states().subscribe()
    try:
        frame = None
        deadline = time.time() + 2.0
        while frame is None and time.time() < deadline:
            frame = sub.recv()
        if frame is None:
            raise RuntimeError("2 秒内没收到 joint_states")
    finally:
        sub.close()
    q = np.full(JOINT_COUNT, np.nan)
    for j in frame.joints:
        idx = nid_to_index(int(j.nid))
        if idx is not None:
            q[idx] = float(j.position)
    if not np.all(np.isfinite(q)):
        raise RuntimeError(f"joint_states 缺少轴: {np.flatnonzero(~np.isfinite(q)).tolist()}")
    return q


def check_diagnostics(hand) -> list:
    """返回停机级故障列表（Warning 级不算）。"""
    sub = hand.joint_diagnostics().subscribe()
    try:
        got, frame = 0, None
        while got < _DIAG_WARMUP_FRAMES:
            frame = sub.recv()
            if frame is not None:
                got += 1
    finally:
        sub.close()
    blocking = []
    for e in frame.joints:
        code = int(e.error_code_current)
        if not code:
            continue
        info = WujiHand2.describe_error(code)
        severity = info["severity"] if info else "Unknown"
        if severity != NON_BLOCKING_SEVERITY:
            idx = nid_to_index(int(e.nid))
            label = joint_labels()[idx] if idx is not None else f"nid{e.nid}"
            name = info["name"] if info else "Unknown"
            blocking.append(f"{label} 0x{code:04X}({name},{severity})")
    return blocking


def rate_limit(previous: np.ndarray, target: np.ndarray, max_speed: float, dt: float):
    step = max(max_speed * dt, 0.0)
    return np.clip(target, previous - step, previous + step)


# --------------------------------------------------------------------------- 主

def run_once(controller, names, q, forces, limits) -> tuple:
    q_dict = {n: float(v) for n, v in zip(names, q)}
    result = controller.adjust(q_dict, forces)
    target = np.array([result.joint_positions[n] for n in names], dtype=np.float64)
    delta = np.array([result.delta[n] for n in names], dtype=np.float64)
    clamped = np.clip(target, limits[:, 0], limits[:, 1])
    n_clamped = int(np.count_nonzero(np.abs(clamped - target) > 1e-9))
    return result, clamped, delta, n_clamped


def print_row(i: int, result, q, delta, n_clamped: int) -> None:
    moved = [
        f"{joint_labels()[k]}{math.degrees(delta[k]):+.3f}°"
        for k in np.flatnonzero(np.abs(delta) > 1e-9)
    ]
    print(
        f"  #{i:<3} active={list(result.active_fingertips) or '-'} "
        f"iters={result.solver_iterations} obj={result.objective:.3e} "
        f"clamped={n_clamped} |delta|max={math.degrees(np.abs(delta).max()):.3f}°"
    )
    if moved:
        print(f"       {' '.join(moved[:8])}{' …' if len(moved) > 8 else ''}")


def selftest(side: str, urdf: Path) -> int:
    print(f"===== 离线自检（{side}, {urdf.name}）=====")
    config = QPConfig()
    controller = build_controller(side, urdf, config)
    names = list(controller.controlled_joint_names)
    print(f"[1] URDF 解析出 {len(names)} 个独立可动关节  OK")

    limits = check_joint_order(controller)
    print("[2] 20 条关节顺序与限位和官方 flat-20 表逐位一致  OK")

    model_links = set(controller.model.joints_by_child) | {
        j.parent for j in controller.model.joints.values()
    }
    missing = [v for v in tip_links(side).values() if v not in model_links]
    if missing:
        print(f"[3] 指尖 link 缺失: {missing}  FAIL")
        return 1
    print(f"[3] 5 个指尖 link 都在 URDF 里  OK")

    q = np.zeros(JOINT_COUNT)
    q[[4, 6, 8, 10]] = math.radians(30.0)  # 食指/中指半屈，避免奇异位形
    _r, _c, delta0, _n = run_once(controller, names, q, {}, limits)
    if np.any(np.abs(delta0) > 1e-12):
        print(f"[4] 无力时 delta 非零（max {np.abs(delta0).max():.2e}）  FAIL")
        return 1
    print("[4] 无力输入 -> delta 全零  OK")

    result, clamped, delta, n_clamped = run_once(
        controller, names, q, {"middle": [0.0, 0.0, 12.0]}, limits
    )
    if not np.any(np.abs(delta) > 1e-9):
        print("[5] 12 N 受力没有产生任何调整  FAIL")
        return 1
    max_step = config.max_joint_step
    if np.abs(delta).max() > max_step + 1e-12:
        print(f"[5] 单步超过 max_joint_step（{np.abs(delta).max():.5f} > {max_step}）  FAIL")
        return 1
    print(
        f"[5] 12 N 受力 -> 非零调整，|delta|max="
        f"{math.degrees(np.abs(delta).max()):.3f}° ≤ max_joint_step "
        f"{math.degrees(max_step):.3f}°  OK"
    )
    print(f"    生效指尖 {list(result.active_fingertips)}，迭代 {result.solver_iterations}")

    target = np.array([result.joint_positions[n] for n in names])
    if np.any(target < limits[:, 0] - 1e-9) or np.any(target > limits[:, 1] + 1e-9):
        print("[6] 输出越过 URDF 限位  FAIL")
        return 1
    print(f"[6] 输出全部落在 URDF 限位内（本次钳制 {n_clamped} 轴）  OK")

    print("\n自检通过。运动学与 QP 这一侧可以接 Hand 2。")
    return 0


def live(args, controller, names, limits) -> int:
    address = args.address or DEFAULT_ADDRESSES[args.side]
    print(f"===== {args.side} hand @ {address} =====")
    hand = SdkManager.instance().connect(
        address=address, device_name=f"wuji_hand_2_{args.side}_tqp"
    )
    online = int(hand.online_joints_count().get())
    if online != JOINT_COUNT:
        print(f"ERROR: 只有 {online}/{JOINT_COUNT} 个关节在线", file=sys.stderr)
        return 2

    source = (
        DeviceForceSource(hand) if args.force_source == "device"
        else SimForceSource(args.forces)
    )
    print(f"力来源：{source.describe()}")

    blocking = check_diagnostics(hand)
    if blocking:
        print(f"停机级关节故障：{', '.join(blocking)}")
        if args.control:
            print("不能在带故障的情况下 --control。先用 scripts/05_clear_faults.py 看现状。")
            return 2
        print("（只读模式继续；不下发任何指令）")

    if args.control and args.force_source == "sim":
        print(
            "\n!! 警告：正在用**合成力**驱动实体手。这只适合链路验证，"
            "不是真实触觉闭环。\n"
        )

    publisher = None
    enabled = False
    try:
        if args.control:
            hand.enable()
            enabled = True
            print("已使能。")
            publisher = hand.joint_command().publish()

        q = read_positions(hand)
        command = q.copy()
        last = time.time()
        deadline = time.time() + args.duration
        period = 1.0 / args.rate
        i = 0
        while time.time() < deadline:
            q = read_positions(hand)
            forces = source.read()
            result, target, delta, n_clamped = run_once(
                controller, names, q, forces, limits
            )
            i += 1
            if i % max(1, int(args.rate / 4)) == 1:
                print_row(i, result, q, delta, n_clamped)
            if args.control:
                now = time.time()
                command = rate_limit(
                    command, target, args.max_speed, min(max(now - last, 0.0), 0.1)
                )
                last = now
                publisher.send(
                    [JointCommand(position=float(p), velocity=0.0, effort=0.0)
                     for p in command]
                )
                if check_diagnostics(hand):
                    print("运行中出现停机级故障，停止下发。", file=sys.stderr)
                    return 1
            time.sleep(period)
        print(f"\n跑了 {i} 次迭代。{'已下发' if args.control else '全程未下发任何指令'}。")
        return 0
    finally:
        if publisher is not None:
            try:
                publisher.close()
            except Exception:
                pass
        if enabled:
            try:
                hand.disable()
                print("已失能。")
            except Exception as exc:
                print(f"!! 失能失败：{exc}", file=sys.stderr)
        if isinstance(source, DeviceForceSource):
            source.close()


def parse_force(text: str) -> tuple:
    if "=" not in text:
        raise argparse.ArgumentTypeError("格式应为 finger=Fz 或 finger=Fx,Fy,Fz")
    name, _, value = text.partition("=")
    name = name.strip()
    if name not in FINGERS:
        raise argparse.ArgumentTypeError(f"未知指尖 {name!r}，应为 {'/'.join(FINGERS)}")
    parts = [p for p in value.split(",") if p.strip()]
    try:
        nums = [float(p) for p in parts]
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r} 的数值解析失败")
    if len(nums) == 1:
        nums = [0.0, 0.0, nums[0]]
    if len(nums) != 3:
        raise argparse.ArgumentTypeError(f"{text!r} 需要 1 个或 3 个数")
    return name, nums


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="把触觉 QP 插件接到 Wuji Hand 2；默认只读不下发。"
    )
    p.add_argument("--side", choices=("right", "left"), default="right")
    p.add_argument("--address", default=None, metavar="HOST:PORT")
    p.add_argument("--urdf", default=None, metavar="PATH")
    p.add_argument("--selftest", action="store_true", help="离线自检，不连接硬件")
    p.add_argument("--force-source", choices=("sim", "device"), default="sim")
    p.add_argument(
        "--force", action="append", default=[], type=parse_force, metavar="F=Fx,Fy,Fz",
        help="合成力，可重复，如 --force middle=0,0,9 或 --force index=8",
    )
    p.add_argument("--duration", type=float, default=3.0, metavar="SECONDS")
    p.add_argument("--rate", type=float, default=20.0, metavar="HZ")
    p.add_argument("--max-speed", type=float, default=1.0, metavar="RAD_PER_SEC")
    p.add_argument("--control", action="store_true", help="真的下发（默认只打印）")
    p.add_argument("--enable-balance", action="store_true", help="启用多指 Balance")
    args = p.parse_args(argv)
    args.forces = dict(args.force)
    args.urdf = Path(args.urdf) if args.urdf else DEFAULT_URDFS[args.side]
    if not args.urdf.is_file():
        p.error(f"URDF 不存在: {args.urdf}")
    return args


def main() -> int:
    args = parse_args(sys.argv[1:])
    if args.selftest:
        return selftest(args.side, args.urdf)
    config = QPConfig(enable_balance=args.enable_balance)
    controller = build_controller(args.side, args.urdf, config)
    names = list(controller.controlled_joint_names)
    limits = check_joint_order(controller)
    try:
        return live(args, controller, names, limits)
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n已中断。", file=sys.stderr)
        raise SystemExit(130)
