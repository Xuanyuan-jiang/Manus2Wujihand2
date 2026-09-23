#!/usr/bin/env python3
"""只读: 连接右手 Wuji Hand 2, 打印 20 个关节角, 干净退出。绝不发任何运动指令。

本脚本刻意不调用: enable() / joint_command().publish() / effort_limit().set()
/ mit_params().set() / set_origin() / reboot()。
"""
import os, sys, time

EXPECT_ENV = os.environ.get("WUJI2_ENV", "wuji2")
if os.path.basename(sys.prefix) != EXPECT_ENV:
    sys.exit(f"ERROR: 跑在了错误的解释器 {sys.executable} (prefix={sys.prefix})。\n"
             f"       base env 也装了 wuji_sdk, 不会报错但环境是错的。\n"
             f"       请先 `source <仓库根>/env.sh`，或直接用 conda 环境 {EXPECT_ENV} 的 python。")

from wuji_sdk import SdkManager, DeviceType, Handedness, set_log_level

FINGERS = ["thumb", "index", "middle", "ring", "pinky"]
LABELS = [f"{f}_S{s}" for f in FINGERS for s in (1, 2, 3, 4)]

def flat_index(nid: int) -> int:
    """joint_states 的 nid (1-based, 每 bus 跳过第 5 个 tactile 节点) -> 扁平 0..19。
    实测 nid 集合 = {1,2,3,4, 6,7,8,9, 11,12,13,14, 16,17,18,19, 21,22,23,24}。
    注意: 不要改用 WujiHand2.joint_id_from_bus_node(), 它是 0-based (bus*5+node-1),
    与 joint_states 的 1-based nid 差 1。
    """
    return ((nid - 1) // 5) * 4 + ((nid - 1) % 5)

def main() -> int:
    set_log_level("warn")
    m = SdkManager.instance()
    hands = [d for d in m.scan() if d.device_type == DeviceType.WujiHand2]
    if not hands:
        print("ERROR: 没扫到 Wuji Hand 2 (本机 Hand2 走以太网 192.168.1.111:7447, "
              "先 ping 通再说; USB 上那两个 0483:2000 是 WujiHand v1, 不是 Hand2)",
              file=sys.stderr)
        return 1
    for d in hands:
        print(f"found  sn={d.sn}  addr={d.address}  transport={d.transport_type}")

    h = m.connect(handedness=Handedness.Right, device_name="ro_probe")
    try:
        fw = getattr(h.info, "firmware_version", "<unknown>")
        print(f"connected  sn={h.serial_number}  side={h.handedness().get()}  "
              f"fw={fw}  online_joints={h.online_joints_count().get()}")

        sub = h.joint_states().subscribe()
        try:
            latest, deadline = None, time.monotonic() + 5.0
            while time.monotonic() < deadline:
                while True:                       # recv() 非阻塞, 排空积压只留最新一帧
                    f = sub.recv()
                    if f is None:
                        break
                    latest = f
                if latest is not None and latest.num_joints >= 20:
                    break
                time.sleep(0.02)
        finally:
            sub.close()

        if latest is None:
            print("ERROR: 5 秒内没收到 joint_states", file=sys.stderr)
            return 1

        pos = [None] * 20
        for j in latest.joints:
            k = flat_index(j.nid)
            if 0 <= k < 20:
                pos[k] = j.position

        print(f"\nseq={latest.header.seq}  num_joints={latest.num_joints}  (单位: 弧度 rad)")
        for k, (lab, p) in enumerate(zip(LABELS, pos)):
            print(f"  [{k:2d}] {lab:10s} {'  OFFLINE' if p is None else f'{p:+8.4f}'}")

        miss = [LABELS[k] for k, p in enumerate(pos) if p is None]
        print(f"\nOK: 读到 {20 - len(miss)}/20 个关节。" + (f" 缺失={miss}" if miss else ""))
        return 0
    finally:
        m.disconnect_all()

if __name__ == "__main__":
    raise SystemExit(main())
