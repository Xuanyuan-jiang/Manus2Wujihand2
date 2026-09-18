import time

from wuji_sdk import SdkManager, JointCommand, WujiHand2


HZ = 200
DURATION = 2.0


def get_positions(hand):
    sub = hand.joint_states().subscribe()

    frame = None
    while frame is None:
        frame = sub.recv()

    q = [0.0] * 20

    for j in frame.joints:
        bus, node_index = divmod(j.nid - 1, 5)

        if bus < 5 and node_index < 4:
            q[bus * 4 + node_index] = float(j.position)

    sub.close()
    return q


def check_faults(hand):
    sub = hand.joint_diagnostics().subscribe()

    frame = None
    while frame is None:
        frame = sub.recv()

    has_fault = False

    for j in frame.joints:
        if j.error_code_current:
            has_fault = True
            print(
                f"[FAULT] nid={j.nid}, "
                f"code={j.error_code_current}, "
                f"{WujiHand2.describe_error(j.error_code_current)}"
            )

    sub.close()

    return has_fault


def wait_enabled(hand):
    sub = hand.joint_diagnostics().subscribe()

    deadline = time.monotonic() + 5.0

    while time.monotonic() < deadline:
        frame = sub.recv()

        if (
            frame is not None
            and frame.joints
            and all(j.status_word.ext_state == 2 for j in frame.joints)
        ):
            sub.close()
            return

    sub.close()
    raise RuntimeError("Enable timeout")


def main():

    manager = SdkManager.instance()
    hand = None
    pub = None

    try:
        print("Connecting...")

        hand = manager.auto_connect(
            device_name="wuji_hand_2"
        )

        print("SN:", hand.serial_number)

        count = hand.online_joints_count().get()
        print("Online joints:", count)

        # ---------- 检查错误 ----------
        print("\nChecking faults...")

        if check_faults(hand):
            print("Fault detected, clearing...")
            hand.clear_fault()
            time.sleep(0.5)

            if check_faults(hand):
                raise RuntimeError("Fault still active")

        # ---------- 当前姿态 ----------
        q_start = get_positions(hand)

        print("\nCurrent position:")
        for i, q in enumerate(q_start):
            print(f"J{i:02d}: {q:+.4f}")

        # ---------- MIT 参数 ----------
        print("\nSetting MIT parameters...")

        hand.effort_limit().set(1.5)
        hand.mit_params().set((3.0, 0.05))

        # ---------- Enable ----------
        print("Enabling...")

        hand.enable()
        wait_enabled(hand)

        print("Enabled.")

        # ---------- Publisher ----------
        pub = hand.joint_command().publish()

        print("\n===== MOVING TO ZERO =====")

        steps = int(HZ * DURATION)

        for i in range(1, steps + 1):

            ratio = i / steps

            # 从当前姿态平滑移动到 0
            q_cmd = [
                q * (1.0 - ratio)
                for q in q_start
            ]

            commands = [
                JointCommand(
                    position=q,
                    velocity=0.0,
                    effort=0.0,
                )
                for q in q_cmd
            ]

            pub.send(commands)

            # 每 0.25 秒打印一次实际状态
            if i % 50 == 0:

                q_actual = get_positions(hand)

                print(
                    f"{i/HZ:.2f}s | "
                    f"J0 cmd={q_cmd[0]:+.4f}, "
                    f"actual={q_actual[0]:+.4f} | "
                    f"J19 cmd={q_cmd[19]:+.4f}, "
                    f"actual={q_actual[19]:+.4f}"
                )

            time.sleep(1.0 / HZ)

        time.sleep(0.5)

        q_final = get_positions(hand)

        print("\n===== FINAL =====")

        for i, (before, after) in enumerate(
            zip(q_start, q_final)
        ):
            print(
                f"J{i:02d}: "
                f"{before:+.4f} -> {after:+.4f}"
            )

    except KeyboardInterrupt:
        print("\nCtrl-C")

    finally:

        if pub is not None:
            pub.close()

        if hand is not None:
            try:
                hand.disable()
                print("\nHand disabled.")
            except Exception:
                pass

        manager.disconnect_all()


if __name__ == "__main__":
    main()