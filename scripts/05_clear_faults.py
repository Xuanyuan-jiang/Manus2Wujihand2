#!/usr/bin/env python3
"""读取 Wuji Hand 2 的关节故障，并按需清错（clear_fault）。

默认**只读**：连上手、打印 20 个关节的状态与故障明细，什么也不改。
真正要复位故障锁存才加 `--clear`。

    # 只读体检
    ~/miniconda3/envs/wuji2/bin/python scripts/05_clear_faults.py --side right
    # 清掉右手所有故障
    ~/miniconda3/envs/wuji2/bin/python scripts/05_clear_faults.py --side right --clear
    # 只清指定关节
    ~/miniconda3/envs/wuji2/bin/python scripts/05_clear_faults.py --side right --clear --joint middle_S4
    # 两只手一起看
    ~/miniconda3/envs/wuji2/bin/python scripts/05_clear_faults.py --side both

`clear_fault` 只复位故障锁存、让关节从 Stopped 回到 Ready。它**不使能、不下发
任何运动指令**，本脚本也从不调用 enable()。

清错前请先用手确认对应手指能自由屈伸（没卡住、没缠线）。固件给出的 resolution
是「检查负载/接线，清错后重试」—— 如果清完一使能又立刻报同一个码，那就不是
软件把它顶到限位，是机械侧的问题，别反复清。

退出码：0 = 全部 Ready 且无故障码；1 = 仍有故障；2 = 参数或连接错误。
"""

from __future__ import annotations

import argparse
import datetime as _dt
import sys
import time
from typing import Optional

from wuji_sdk import SdkManager, WujiHand2


JOINT_COUNT = 20
# 本机的两只 Hand 2，与 04_manus_wuji2.py 保持一致。
DEFAULT_ADDRESSES = {
    "right": "192.168.1.111:7447",  # WH2KA01260818006
    "left": "192.168.1.110:7447",   # WH2JA01260813009
}
# 固件的 severity 分级里只有 Warning 不停机，见 04_manus_wuji2.py 的 split_joint_errors。
NON_BLOCKING_SEVERITY = "Warning"
# clear_fault 下发后等多久再读回故障码。
_CLEAR_SETTLE_S = 0.5


def nid_to_index(nid: int) -> Optional[int]:
    """SDK 的 5 槽总线 NID 布局 -> 连续的 20 轴序号。与 04_manus_wuji2.py 同一张映射。"""
    if nid <= 0:
        return None
    bus, node_index = divmod(nid - 1, 5)
    if bus >= 5 or node_index >= 4:
        return None
    return bus * 4 + node_index


def label_to_path(label: str) -> str:
    """JointHandle.label 'middle_S4' -> 资源路径 'middle/S4'。"""
    return label.replace("_", "/", 1)


def describe(code: int) -> dict:
    """describe_error 的安全包装：SDK 比固件旧时会返回 None。"""
    info = WujiHand2.describe_error(code)
    if info is None:
        return {
            "name": "Unknown",
            "severity": "Unknown",
            "clear_policy": "Unknown",
            "desc": "",
            "cause": "",
            "resolution": "",
        }
    return info


# 订阅后的第一帧里 comm_response_rate_pct 还没算出来，全是 0（实测第 2 帧起才是
# 100）。丢掉它，否则每次体检都会显示一整列假的 0% 通信率。
_DIAG_WARMUP_FRAMES = 2


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


def print_table(labels: dict, entries: list) -> list:
    """打印 20 轴总览，返回有故障码的 (index, label, code) 列表。"""
    print(
        f"{'nid':>3} {'joint':<10} {'state':<8} {'err':<7} "
        f"{'I(A)':>7} {'mcu°C':>6} {'vbus':>6} {'comm%':>6} {'t/o':>4}  limits"
    )
    faulted = []
    for e in entries:
        idx = nid_to_index(e.nid)
        label = labels.get(idx, f"nid{e.nid}?")
        code = int(e.error_code_current)
        sw = e.status_word
        # position/velocity/current_limit_active 是过流类故障最有用的旁证。
        flags = [
            n
            for n, on in (
                ("pos", sw.position_limit_active),
                ("vel", sw.velocity_limit_active),
                ("cur", sw.current_limit_active),
            )
            if on
        ]
        mark = "  <<<" if (code or sw.ext_state_name != "Ready") else ""
        print(
            f"{e.nid:>3} {label:<10} {sw.ext_state_name:<8} 0x{code:04X} "
            f"{e.current:7.3f} {e.mcu_temp_c_fb:6.1f} {e.vbus_v_fb:6.2f} "
            f"{e.comm_response_rate_pct:6.1f} {e.comm_timeout_total:4d}  "
            f"{','.join(flags) if flags else '-'}{mark}"
        )
        if code:
            faulted.append((idx, label, code))
    return faulted


def print_fault_detail(hand, label: str, code: int) -> None:
    info = describe(code)
    print(f"\n  {label}  0x{code:04X} {info['name']}")
    print(f"    severity     : {info['severity']}")
    print(f"    clear_policy : {info['clear_policy']}")
    for key, title in (("desc", "描述"), ("cause", "原因"), ("resolution", "处理")):
        if info.get(key):
            print(f"    {title}         : {info[key]}")
    for line in format_error_log(hand, label):
        print(f"    error_log    : {line}")


def read_error_log(hand, label: str) -> Optional[list]:
    """读一个关节的 error_log。SDK 没有 JointHandle.error_log()，只能走通用资源路径。

    返回 ``(error_code, timestamp_us, uptime_us)`` 的列表；读失败返回 None（读不到
    日志不该让整个体检失败，但也不能假装它是空的 —— 那会让下面的前后对照误判）。
    """
    try:
        log = hand.device.get(f"{label_to_path(label)}/error_log")
    except Exception:
        return None
    return [(e.error_code, e.timestamp_us, e.uptime_us) for e in log.entries]


def format_error_log(hand, label: str) -> list[str]:
    entries = read_error_log(hand, label)
    if entries is None:
        return ["读取失败"]
    out = []
    for code, ts, uptime in entries:
        when = _dt.datetime.fromtimestamp(ts / 1e6)
        out.append(
            f"0x{code:04X} @ {when:%Y-%m-%d %H:%M:%S} (该关节上电后 {uptime / 1e6:.1f}s)"
        )
    return out or ["（空）"]


def resolve_targets(faulted: list, wanted: list[str], labels: dict) -> list:
    """把 --joint 的 label 或 0-19 序号解析成要清的关节，未指定则清全部有故障的。"""
    if not wanted:
        return faulted
    by_label = {label: (idx, label, code) for idx, label, code in faulted}
    by_index = {idx: (idx, label, code) for idx, label, code in faulted}
    known = set(labels.values())
    targets = []
    for token in wanted:
        if token.isdigit():
            idx = int(token)
            if idx not in labels:
                raise SystemExit(f"ERROR: --joint {token} 不是 0..19 的关节序号")
            if idx not in by_index:
                print(f"[skip] {labels[idx]} 当前没有故障码")
                continue
            targets.append(by_index[idx])
        else:
            if token not in known:
                raise SystemExit(
                    f"ERROR: --joint {token} 不是合法关节名；合法值形如 middle_S4"
                )
            if token not in by_label:
                print(f"[skip] {token} 当前没有故障码")
                continue
            targets.append(by_label[token])
    return targets


def handle_one(address: str, side: str, do_clear: bool, wanted: list[str]) -> int:
    print(f"\n===== {side} hand @ {address} =====")
    hand = SdkManager.instance().connect(
        address=address, device_name=f"wuji_hand_2_{side}_faults"
    )
    labels = {j.index: j.label for j in hand.joints()}

    online = int(hand.online_joints_count().get())
    if online != JOINT_COUNT:
        print(f"WARNING: 只有 {online}/{JOINT_COUNT} 个关节在线")

    faulted = print_table(labels, read_diagnostics(hand))
    for _idx, label, code in faulted:
        print_fault_detail(hand, label, code)

    if not faulted:
        print("\n无故障码。")
        return 0

    blocking = [f for f in faulted if describe(f[2])["severity"] != NON_BLOCKING_SEVERITY]
    print(
        f"\n{len(faulted)} 个关节有故障码"
        f"（其中 {len(blocking)} 个是停机级，Warning 级不影响控制）。"
    )

    if not do_clear:
        print("只读模式：没有改动任何状态。要复位故障锁存请加 --clear。")
        return 1

    targets = resolve_targets(faulted, wanted, labels)
    if not targets:
        print("没有要清的关节。")
        return 1

    # 清错前先存一份 error_log。复查时故障码还在，有两种完全不同的可能，
    # 必须靠 error_log 有没有长出新条目来区分（实测 2026-09-22 右手 middle_S4
    # 就是后一种，当时脚本报的是前一种，结论下反了）：
    #   多了新条目 -> 清掉了又立刻重新触发，故障条件仍然存在（机械/负载侧）
    #   没有新条目 -> 设备根本没执行这次清错（写入被接受但静默忽略）
    before = {label: read_error_log(hand, label) for _idx, label, _code in targets}

    print("\nclear_fault（只复位故障锁存，不使能、不下发运动）：")
    for idx, label, code in targets:
        print(f"  {label}: 0x{code:04X} -> ", end="", flush=True)
        try:
            hand.joint(idx).clear_fault()
        except Exception as exc:
            print(f"失败 ({exc})")
            continue
        # clear_fault 是异步下发（SDK 的 set 只保证发出去了）。给固件一点时间，
        # 免得把「还没处理完」读成「没清掉」。实测这个延时不是成败关键，但读数要稳。
        time.sleep(_CLEAR_SETTLE_S)
        print(f"0x{int(hand.joint(idx).error_code().get()):04X}")

    print("\n复查：")
    still = print_table(labels, read_diagnostics(hand))
    if still:
        print(f"\n仍有 {len(still)} 个关节带故障码。")
        for _idx, label, _code in targets:
            if label not in {lbl for _i, lbl, _c in still}:
                continue
            after = read_error_log(hand, label)
            old, new = before.get(label), after
            for line in format_error_log(hand, label):
                print(f"  {label} error_log: {line}")
            if old is None or new is None:
                print(f"  {label}: error_log 读不到，无法判断是没清动还是清完重现。")
            elif new != old:
                print(
                    f"  {label}: 清掉后立刻重新触发（error_log 多了新条目）—— "
                    "故障条件仍然存在。别反复清，按 resolution 检查负载/接线。"
                )
            else:
                print(
                    f"  {label}: 设备没有执行这次清错（故障码未变，error_log 也没有"
                    "新条目）。不是清完重现，是根本没清动。"
                )
                print(
                    "    下一步：硬件断电重启（拔电源，不是软 reboot），"
                    "上电后立刻再跑一次本脚本的只读模式；仍然是同一个码就报修。"
                )
        return 1
    print("\n全部关节已回到无故障状态。注意：clear_fault 不使能，控制仍需重新启动控制节点。")
    return 0


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="读取 Wuji Hand 2 关节故障；默认只读，--clear 才复位故障锁存。"
    )
    parser.add_argument("--side", choices=("right", "left", "both"), default="right")
    parser.add_argument(
        "--address",
        default=None,
        metavar="HOST:PORT",
        help="覆盖地址；默认按 --side 取（right=%s, left=%s）"
        % (DEFAULT_ADDRESSES["right"], DEFAULT_ADDRESSES["left"]),
    )
    parser.add_argument(
        "--clear",
        action="store_true",
        help="执行 clear_fault；不加则只打印，不改任何状态",
    )
    parser.add_argument(
        "--joint",
        action="append",
        default=[],
        metavar="LABEL|INDEX",
        help="只清这个关节，可重复；形如 middle_S4 或 11。默认清全部有故障的",
    )
    args = parser.parse_args(argv)

    if args.side == "both":
        if args.address:
            parser.error("--side both 不能带 --address；它对两只手含义不同")
        if args.joint:
            parser.error("--side both 不能带 --joint；请分别指定 --side right/left")
    if args.joint and not args.clear:
        parser.error("--joint 只在 --clear 时有意义")
    return args


def main() -> int:
    args = parse_args(sys.argv[1:])
    sides = ("right", "left") if args.side == "both" else (args.side,)
    failed = 0
    for side in sides:
        address = args.address or DEFAULT_ADDRESSES[side]
        try:
            failed |= handle_one(address, side, args.clear, args.joint)
        except SystemExit:
            raise
        except Exception as exc:
            print(f"ERROR: {side} hand @ {address} 处理失败：{exc}", file=sys.stderr)
            return 2
    return failed


if __name__ == "__main__":
    raise SystemExit(main())
