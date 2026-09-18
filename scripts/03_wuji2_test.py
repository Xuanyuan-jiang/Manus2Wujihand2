import time
import math

from wuji_sdk import SdkManager, JointCommand


HZ = 200
MOVE_TIME = 1.2

# MIT 参数
KP = 3.0
KD = 0.05
EFFORT_LIMIT = 1.5


# ============================================================
# 手势定义
# 单位：rad
#
# joint:
#  0  1  2  3   -> thumb
#  4  5  6  7   -> index
#  8  9 10 11   -> middle
# 12 13 14 15   -> ring
# 16 17 18 19   -> pinky
# ============================================================

OPEN = [0.0] * 20


def finger_pose(finger_start, mcp=0.55, pip=0.75, dip=0.55):
    """生成单根四指弯曲姿态"""
    q = [0.0] * 20

    q[finger_start + 0] = mcp
    q[finger_start + 1] = 0.0
    q[finger_start + 2] = pip
    q[finger_start + 3] = dip

    return q


INDEX = finger_pose(4)
MIDDLE = finger_pose(8)
RING = finger_pose(12)
PINKY = finger_pose(16)


FOUR_FINGER_FIST = [
    # thumb
    0.0, 0.0, 0.0, 0.0,

    # index
    0.55, 0.0, 0.75, 0.55,

    # middle
    0.55, 0.0, 0.75, 0.55,

    # ring
    0.55, 0.0, 0.75, 0.55,

    # pinky
    0.55, 0.0, 0.75, 0.55,
]


THUMB_CURL = [
    # thumb
    0.0, 0.0, 0.55, 0.55,

    # other fingers
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
]


FULL_FIST = [
    # thumb
    0.0, 0.0, 0.45, 0.55,

    # index
    0.55, 0.0, 0.75, 0.55,

    # middle
    0.55, 0.0, 0.75, 0.55,

    # ring
    0.55, 0.0, 0.75, 0.55,

    # pinky
    0.55, 0.0, 0.75, 0.55,
]


GESTURES = {
    "0": ("OPEN / ZERO", OPEN),
    "1": ("INDEX CURL", INDEX),
    "2": ("MIDDLE CURL", MIDDLE),
    "3": ("RING CURL", RING),
    "4": ("PINKY CURL", PINKY),
    "5": ("FOUR-FINGER FIST", FOUR_FINGER_FIST),
    "6": ("THUMB CURL", THUMB_CURL),
    "7": ("FULL FIST", FULL_FIST),
}


def read_joint_positions(hand):
    sub = hand.joint_states().subscribe()

    frame = None

    while frame is None:
        frame = sub.recv()
        time.sleep(0.001)

    positions = [0.0] * 20

    for j in frame.joints:

        bus, node_index = divmod(j.nid - 1, 5)

        if bus < 5 and node_index < 4:
            index = bus * 4 + node_index
            positions[index] = float(j.position)

    sub.close()

    return positions


def wait_enabled(hand, timeout=5.0):

    sub = hand.joint_diagnostics().subscribe()

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:

        frame = sub.recv()

        if (
            frame is not None
            and frame.joints
            and all(j.status_word.ext_state == 2 for j in frame.joints)
        ):
            sub.close()
            return

        time.sleep(0.01)

    sub.close()

    raise RuntimeError("Enable timeout")


def send_position(pub, q):

    commands = [
        JointCommand(
            position=float(p),
            velocity=0.0,
            effort=0.0,
        )
        for p in q
    ]

    pub.send(commands)


def move_smooth(pub, q_start, q_target):

    steps = int(MOVE_TIME * HZ)

    for i in range(steps + 1):

        t = i / steps

        # cosine interpolation
        alpha = 0.5 - 0.5 * math.cos(math.pi * t)

        q = [
            a + alpha * (b - a)
            for a, b in zip(q_start, q_target)
        ]

        send_position(pub, q)

        time.sleep(1.0 / HZ)


def print_position(q):

    print("\nCurrent joint positions:")

    names = [
        "TH1", "TH2", "TH3", "TH4",
        "FF1", "FF2", "FF3", "FF4",
        "MF1", "MF2", "MF3", "MF4",
        "RF1", "RF2", "RF3", "RF4",
        "LF1", "LF2", "LF3", "LF4",
    ]

    for i, value in enumerate(q):
        print(
            f"{i:02d} {names[i]:3s}: "
            f"{value:+.3f}"
        )


def print_menu():

    print(
        """
========================================
          WUJI HAND 2 CONTROL
========================================

0 : Open / Zero

1 : Curl INDEX
2 : Curl MIDDLE
3 : Curl RING
4 : Curl PINKY

5 : Four-finger fist

6 : Curl THUMB

7 : Full fist

p : Print current joint positions

q : Quit

========================================
"""
    )


def main():

    manager = SdkManager.instance()

    hand = None
    pub = None

    try:

        print("Searching for Wuji Hand 2...")

        hand = manager.auto_connect(
            device_name="wuji_hand_2"
        )

        print()
        print("Connected:", hand.serial_number)

        count = hand.online_joints_count().get()

        print(f"Online joints: {count}/20")

        if count != 20:
            raise RuntimeError(
                f"Only {count}/20 joints online"
            )

        # --------------------------------------------------
        # MIT controller
        # --------------------------------------------------

        hand.effort_limit().set(EFFORT_LIMIT)
        hand.mit_params().set((KP, KD))

        print(
            f"MIT: kp={KP}, "
            f"kd={KD}, "
            f"effort_limit={EFFORT_LIMIT}A"
        )

        # --------------------------------------------------
        # Enable
        # --------------------------------------------------

        print("Enabling hand...")

        hand.enable()

        wait_enabled(hand)

        print("Hand enabled.")

        pub = hand.joint_command().publish()

        # --------------------------------------------------
        # 读取当前姿态
        # --------------------------------------------------

        current = read_joint_positions(hand)

        print_position(current)

        print_menu()

        # --------------------------------------------------
        # Keyboard loop
        # --------------------------------------------------

        while True:

            key = input("Command > ").strip().lower()

            if key == "q":
                break

            if key == "p":

                current = read_joint_positions(hand)

                print_position(current)

                continue

            if key not in GESTURES:

                print("Unknown command.")

                continue

            name, target = GESTURES[key]

            print()
            print(f">>> {name}")

            # 从真实当前位置开始
            current = read_joint_positions(hand)

            move_smooth(
                pub,
                current,
                target,
            )

            actual = read_joint_positions(hand)

            print(
                f"Finished: {name}"
            )

            print(
                f"max actual = "
                f"{max(abs(x) for x in actual):.3f} rad"
            )

    except KeyboardInterrupt:

        print("\nCtrl-C")

    except Exception as e:

        print("\nERROR:", e)

        raise

    finally:

        if pub is not None:
            pub.close()

        if hand is not None:

            try:
                hand.disable()
                print("Hand disabled.")
            except Exception:
                pass

        try:
            manager.disconnect_all()
        except Exception:
            pass


if __name__ == "__main__":
    main()