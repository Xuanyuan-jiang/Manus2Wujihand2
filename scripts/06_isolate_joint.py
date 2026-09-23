#!/usr/bin/env python3
"""隔离实验：单独使能一个关节，判断它的故障是负载/机械引起还是节点本身的问题。

针对的场景：某个关节一使能就跳 ImmediateStop（实测 2026-09-22 右手 middle_S4
的 0x2102 Overcurrent 就是这样），需要在报修前拿到「零负载下照样跳」的证据。

    # 断电重启之后再跑。手要悬空，目标手指不碰任何东西。
    ~/miniconda3/envs/wuji2/bin/python scripts/06_isolate_joint.py --side right --joint middle_S4

两个阶段，中间自动失能：
    A  只使能目标关节，保持 --hold 秒，全程采样
    B  只使能其余 19 个关节（目标位掩码置 0），保持 --hold 秒

**本脚本只 enable/disable，从不下发任何位置或力矩指令。**使能后关节会保持当前
位置，可能有轻微抱死感，这是正常的。任何异常按 Ctrl+C，finally 里会失能。

前置条件：20 个关节必须全部 Ready 且无故障码。带着故障码跑这个实验没有意义 ——
clear_fault 对已跳闸的 ImmediateStop 无效（见 05_clear_faults.py），只能断电重启。

退出码：0 = 目标关节零负载下使能正常；1 = 目标关节跳闸；2 = 前置条件不满足或连接失败。
"""

from __future__ import annotations

import argparse
import datetime as _dt
import sys
import time
from typing import Optional

from wuji_sdk import SdkManager, WujiHand2


JOINT_COUNT = 20
DEFAULT_ADDRESSES = {
    "right": "192.168.1.111:7447",  # WH2KA01260817029
    "left": "192.168.1.110:7447",   # WH2JA01260813009
}
# 订阅后的第一帧 comm_response_rate_pct 还没算出来，丢掉它。同 05_clear_faults.py。
_DIAG_WARMUP_FRAMES = 2
# enable 之后等关节进入 Enabled 的上限。04_manus_wuji2.py 用的也是 5s。
_ENABLE_TIMEOUT_S = 5.0
# 通电前的反悔窗口。
_ARM_COUNTDOWN_S = 3

# 这几个小工具和 05_clear_faults.py 里的是同一份。scripts/ 下的脚本按项目惯例
# 各自独立可读可跑，不共享模块，所以这里照抄而不是 import。
def nid_to_index(nid: int) -> Optional[int]:
    if nid <= 0:
        return None
    bus, node_index = divmod(nid - 1, 5)
    if bus >= 5 or node_index >= 4:
        return None
    return bus * 4 + node_index


def label_to_path(label: str) -> str:
    return label.replace("_", "/", 1)


def describe(code: int) -> dict:
    info = WujiHand2.describe_error(code)
    if info is None:
        return {"name": "Unknown", "severity": "Unknown"}
    return info


def read_error_log(hand, label: str) -> Optional[list]:
    try:
        log = hand.device.get(f"{label_to_path(label)}/error_log")
    except Exception:
        return None
    return [(e.error_code, e.timestamp_us, e.uptime_us) for e in log.entries]


def format_log_entry(entry: tuple) -> str:
    code, ts, uptime = entry
    when = _dt.datetime.fromtimestamp(ts / 1e6)
    return f"0x{code:04X} @ {when:%Y-%m-%d %H:%M:%S} (该关节上电后 {uptime / 1e6:.1f}s)"


def read_diagnostics(hand) -> list:
    sub = hand.joint_diagnostics().subscribe()
    try:
        got = 0
        frame = None
        while got < _DIAG_WARMUP_FRAMES:
            frame = sub.recv()
            if frame is not None:
                got += 1
        return sorted(frame.joints, key=lambda j: j.nid)
    finally:
        sub.close()


def preflight(hand, labels: dict) -> list:
    """要求 20 轴全部 Ready 且无故障码，否则返回问题列表。"""
    problems = []
    online = int(hand.online_joints_count().get())
    if online != JOINT_COUNT:
        problems.append(f"只有 {online}/{JOINT_COUNT} 个关节在线")
    for e in read_diagnostics(hand):
        idx = nid_to_index(e.nid)
        label = labels.get(idx, f"nid{e.nid}?")
        code = int(e.error_code_current)
        state = e.status_word.ext_state_name
        if code:
            info = describe(code)
            problems.append(
                f"{label} 带故障码 0x{code:04X}({info['name']},{info['severity']})"
            )
        elif state != "Ready":
            problems.append(f"{label} 状态是 {state}，不是 Ready")
    return problems


def watch(hand, labels: dict, seconds: float, expect_enabled: set) -> dict:
    """采样 ``seconds`` 秒，返回每个关节的状态集合、电流峰值和首个故障。"""
    sub = hand.joint_diagnostics().subscribe()
    stats: dict = {}
    frames = 0
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            frame = sub.recv()
            if frame is None:
                continue
            frames += 1
            now = time.time() - t0
            for e in frame.joints:
                idx = nid_to_index(e.nid)
                s = stats.setdefault(
                    idx, {"states": set(), "imax": 0.0, "fault": None, "flags": set()}
                )
                s["states"].add(e.status_word.ext_state_name)
                s["imax"] = max(s["imax"], abs(e.current))
                for name, on in (
                    ("pos", e.status_word.position_limit_active),
                    ("vel", e.status_word.velocity_limit_active),
                    ("cur", e.status_word.current_limit_active),
                ):
                    if on:
                        s["flags"].add(name)
                code = int(e.error_code_current)
                if code and s["fault"] is None:
                    s["fault"] = (code, now)
    finally:
        sub.close()
    return {"frames": frames, "seconds": time.time() - t0, "joints": stats}


def report(title: str, result: dict, labels: dict, focus: set) -> None:
    print(f"\n--- {title}：{result['frames']} 帧 / {result['seconds']:.1f}s")
    print(f"{'joint':<10} {'states seen':<22} {'|I|max':>7}  {'limits':<12} fault")
    for idx in sorted(result["joints"]):
        s = result["joints"][idx]
        fault = ""
        if s["fault"]:
            code, when = s["fault"]
            info = describe(code)
            fault = f"0x{code:04X}({info['name']}) @ +{when:.2f}s"
        mark = " *" if idx in focus else "  "
        print(
            f"{labels.get(idx, idx):<10}{mark}{','.join(sorted(s['states'])):<20} "
            f"{s['imax']:7.3f}  {','.join(sorted(s['flags'])) or '-':<12} {fault}"
        )


def enable_and_watch(hand, labels, mask: list, hold: float, title: str) -> dict:
    """使能 mask 指定的关节，采样 hold 秒，然后无论如何都失能。"""
    focus = {i for i, on in enumerate(mask) if on}
    names = ", ".join(labels[i] for i in sorted(focus))
    print(f"\n===== {title} =====")
    print(f"将要使能：{names if len(focus) <= 4 else f'{len(focus)} 个关节'}")
    for remaining in range(_ARM_COUNTDOWN_S, 0, -1):
        print(f"  {remaining}… (Ctrl+C 取消)", end="\r", flush=True)
        time.sleep(1)
    print("  使能中…        ")
    try:
        hand.enable(joints=mask)
        # 等目标关节进入 Enabled；跳闸的话状态会停在 Stopped，等到超时即可，
        # 采样阶段会把故障码和发生时刻记下来。
        deadline = time.time() + _ENABLE_TIMEOUT_S
        while time.time() < deadline:
            entries = read_diagnostics(hand)
            states = {
                nid_to_index(e.nid): e.status_word.ext_state_name for e in entries
            }
            if all(states.get(i) == "Enabled" for i in focus):
                print(f"  {len(focus)} 个关节已进入 Enabled")
                break
            if any(states.get(i) == "Stopped" for i in focus):
                print("  有关节进入 Stopped —— 跳闸了")
                break
        else:
            print(f"  {_ENABLE_TIMEOUT_S}s 内未全部进入 Enabled，继续采样")
        return watch(hand, labels, hold, focus)
    finally:
        try:
            hand.disable()
            print("  已失能（整手）")
        except Exception as exc:
            print(f"  !! 失能失败：{exc}", file=sys.stderr)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="单独使能一个关节，判断故障是负载/机械引起还是节点本身的问题。"
    )
    parser.add_argument("--side", choices=("right", "left"), default="right")
    parser.add_argument("--address", default=None, metavar="HOST:PORT")
    parser.add_argument(
        "--joint",
        default="middle_S4",
        metavar="LABEL|INDEX",
        help="要隔离的关节，形如 middle_S4 或 11（默认 middle_S4）",
    )
    parser.add_argument(
        "--hold", type=float, default=10.0, metavar="SECONDS", help="每阶段保持秒数"
    )
    parser.add_argument(
        "--skip-others",
        action="store_true",
        help="只做阶段 A，不做「使能其余 19 轴」的对照",
    )
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args(sys.argv[1:])
    address = args.address or DEFAULT_ADDRESSES[args.side]

    print(f"===== {args.side} hand @ {address} =====")
    print("确认：手悬空放置，目标手指不接触任何物体。本脚本只使能，不下发运动指令。")
    try:
        hand = SdkManager.instance().connect(
            address=address, device_name=f"wuji_hand_2_{args.side}_isolate"
        )
    except Exception as exc:
        print(f"ERROR: 连接失败：{exc}", file=sys.stderr)
        return 2
    labels = {j.index: j.label for j in hand.joints()}

    by_label = {v: k for k, v in labels.items()}
    if args.joint.isdigit():
        target = int(args.joint)
        if target not in labels:
            print(f"ERROR: --joint {args.joint} 不是 0..19 的关节序号", file=sys.stderr)
            return 2
    elif args.joint in by_label:
        target = by_label[args.joint]
    else:
        print(f"ERROR: --joint {args.joint} 不是合法关节名", file=sys.stderr)
        return 2
    target_label = labels[target]

    problems = preflight(hand, labels)
    if problems:
        print("\n前置条件不满足，不做实验：")
        for p in problems:
            print(f"  - {p}")
        print(
            "\n带着故障码跑这个实验没有意义。clear_fault 对已跳闸的 ImmediateStop 无效，"
            "\n请先硬件断电重启（拔电源，不是软 reboot），上电后立刻重跑。"
        )
        return 2
    print("前置检查通过：20 个关节全部 Ready，无故障码。")

    log_before = read_error_log(hand, target_label)

    mask_a = [1 if i == target else 0 for i in range(JOINT_COUNT)]
    result_a = enable_and_watch(
        hand, labels, mask_a, args.hold, f"阶段 A：只使能 {target_label}"
    )
    report(f"阶段 A（{target_label} 单独使能）", result_a, labels, {target})

    stat = result_a["joints"].get(target, {})
    tripped = stat.get("fault") is not None
    log_after = read_error_log(hand, target_label)
    if log_before is not None and log_after is not None and log_after != log_before:
        new = [e for e in log_after if e not in log_before]
        for entry in new:
            print(f"\n  新增 error_log：{format_log_entry(entry)}")

    if not args.skip_others:
        mask_b = [0 if i == target else 1 for i in range(JOINT_COUNT)]
        result_b = enable_and_watch(
            hand, labels, mask_b, args.hold, f"阶段 B：使能除 {target_label} 外的 19 轴"
        )
        report("阶段 B（其余 19 轴）", result_b, labels, set())
        others_bad = [
            labels[i] for i, s in result_b["joints"].items()
            if i != target and s["fault"] is not None
        ]
        print(
            "\n阶段 B：其余 19 轴全部正常。" if not others_bad
            else f"\n阶段 B：这些关节也报了故障 —— {', '.join(others_bad)}"
        )

    print("\n===== 结论 =====")
    if tripped:
        code, when = stat["fault"]
        info = describe(code)
        print(
            f"{target_label} 在**零负载、单独使能**的条件下，使能后 {when:.2f}s 就跳了 "
            f"0x{code:04X}({info['name']},{info['severity']})。"
        )
        print("负载和机械已被排除 —— 问题在这个节点自身（电机/驱动/编码器）。")
        print("下一步：wuji logs export 打一个 support bundle，连同本输出报无际技术支持。")
        return 1
    print(f"{target_label} 单独使能正常，保持 {args.hold:.0f}s 未跳闸。")
    print("说明零负载下它是好的，问题出在带负载或与其他关节同时工作时 —— 还有的查。")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n已中断。", file=sys.stderr)
        raise SystemExit(130)
