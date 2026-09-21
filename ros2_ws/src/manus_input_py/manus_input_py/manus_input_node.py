"""ROS2 node that streams Manus glove hand data and publishes ROS topics."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from ament_index_python.packages import get_package_share_directory

try:
    import yaml
except ImportError as exc:
    raise ImportError("PyYAML is required to load Manus input configuration files.") from exc

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from std_msgs.msg import Float32MultiArray
    from rclpy.utilities import remove_ros_args
except ImportError as exc:
    raise ImportError("This module requires ROS 2 Python packages (rclpy, std_msgs).") from exc

from manus_ros2_msgs.msg import ManusGlove


SEMANTIC_CHAIN_NAMES = ("Thumb", "Index", "Middle", "Ring", "Pinky")

# MANUS 的骨骼节点位于每根**骨头的根部**，所以「近节指骨的根」就是 MCP 关节。
# MediaPipe: 0=WRIST, 1-4=THUMB, 5-8=INDEX, 9-12=MIDDLE, 13-16=RING, 17-20=PINKY
#
# 拇指从掌骨起算（MediaPipe 的 THUMB_CMC 正是掌骨根），四指从近节指骨起算。
# 这个差别是解剖学事实，不是特例处理。
#
# 拇指只有近节、远节两根指骨，没有中节，所以链里**没有 Intermediate**，
# 而它唯一的 IP 关节就在远节指骨的根部 —— 实机数据证实拇指链为
# [Metacarpal, Proximal, Distal, Tip]。
# 注意 ManusSDKTypes.h 中 "//thumb doesn't have it" 这句注释标在了 Distal 行上，
# 是标错了行：拇指缺的是 Intermediate。
MEDIAPIPE_BONE_CHAIN = {
    "Thumb": ("Metacarpal", "Proximal", "Distal", "Tip"),         # CMC, MCP, IP,  TIP
    "Index": ("Proximal", "Intermediate", "Distal", "Tip"),       # MCP, PIP, DIP, TIP
    "Middle": ("Proximal", "Intermediate", "Distal", "Tip"),
    "Ring": ("Proximal", "Intermediate", "Distal", "Tip"),
    "Pinky": ("Proximal", "Intermediate", "Distal", "Tip"),
}

# manus_ros2 的 JointTypeToString 把 SDK 按骨头命名的枚举翻译成按关节命名的
# 字符串时整体错了一位（Proximal -> "PIP"、Intermediate -> "IP"）。这张表把那些
# 旧标签翻译回骨头名，因此本节点对修好前后的 manus_ros2 都能正常工作。
# 一旦 JointTypeToString 改为直接输出骨头名，这里的键不再命中，直接透传。
LEGACY_JOINT_TYPE_ALIASES = {
    "MCP": "Metacarpal",
    "PIP": "Proximal",
    "IP": "Intermediate",
    "DIP": "Distal",
    "TIP": "Tip",
}


@dataclass
class ManusInputConfig:
    """Configuration for the Manus input streaming node."""

    config_path: Optional[str] = None

    # Publishing behaviour
    publish_rate_hz: float = 50.0
    # 每只手发各自的 topic，{side} 会被替换成 right / left。
    # 两只手完全独立：一只掉线不影响另一只。
    publish_hand_topic: str = "/hand_input_{side}"
    include_right_hand: bool = True
    include_left_hand: bool = True

    # Glove IDs
    left_glove_id: int = 0
    right_glove_id: int = 1

    @classmethod
    def from_file(cls, path: str | Path) -> "ManusInputConfig":
        """Load configuration from a YAML file."""
        cfg_path = Path(path).expanduser().resolve()
        if not cfg_path.exists():
            raise FileNotFoundError(f"Config file not found: {cfg_path}")

        with cfg_path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}

        if not isinstance(raw, dict):
            raise ValueError("ManusInputConfig expects a mapping at the root of the YAML file.")

        valid_fields = {field.name for field in fields(cls)}
        data: Dict[str, Any] = {"config_path": str(cfg_path)}
        ignored_keys: list[str] = []

        for key, value in raw.items():
            if key in valid_fields:
                data[key] = value
            else:
                ignored_keys.append(key)

        if ignored_keys:
            print(f"[ManusInputConfig] Ignoring unknown keys: {', '.join(sorted(ignored_keys))}")

        return cls(**data)


class ManusInputNode(Node):
    """ROS2 node that reads Manus glove data and publishes ROS topics."""

    def __init__(self, config: ManusInputConfig):
        super().__init__("manus_input")
        self.config = config

        # 每只手各自的最新数据与「有新帧」标志。手套断开后不重复发陈旧数据，
        # 否则下游 retarget 会持续输出同一姿态导致实体手抖动。
        self._fingers: Dict[str, Optional[np.ndarray]] = {"right": None, "left": None}
        self._pending: Dict[str, bool] = {"right": False, "left": False}
        # 首帧骨架自检每只手各做一次
        self._skeleton_checked: Dict[str, bool] = {"right": False, "left": False}

        self._enabled_sides = tuple(
            side for side, on in (("right", config.include_right_hand),
                                  ("left", config.include_left_hand)) if on
        )
        if not self._enabled_sides:
            raise ValueError("include_right_hand 与 include_left_hand 不能同时为 false")

        # Configure QoS for real-time performance
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 每只手一个 publisher
        self._hand_publishers: Dict[str, Any] = {}
        for side in self._enabled_sides:
            topic = self.config.publish_hand_topic.format(side=side)
            self._hand_publishers[side] = self.create_publisher(
                Float32MultiArray, topic, qos_profile
            )

        # Subscribe to all manus_glove topics, use msg.side to determine left/right
        self.create_subscription(ManusGlove, "/manus_glove_0", self._glove_callback, qos_profile)
        self.create_subscription(ManusGlove, "/manus_glove_1", self._glove_callback, qos_profile)
        self.get_logger().info("[Manus Input] Subscribed to /manus_glove_0 and /manus_glove_1")

        # Timer for publishing
        self.timer = self.create_timer(
            1.0 / max(self.config.publish_rate_hz, 1.0), self._publish_latest_frame
        )
        topics = ", ".join(
            f"{side}->{self.config.publish_hand_topic.format(side=side)}"
            for side in self._enabled_sides
        )
        self.get_logger().info(
            f"[Manus Input] Publishing at {self.config.publish_rate_hz:.1f} Hz: {topics}"
        )

    def _convert_to_mediapipe(self, msg: ManusGlove) -> np.ndarray:
        """Convert Manus raw nodes to MediaPipe (21, 3) format.

        按 (chain_type, 骨头) 语义选点,不使用硬编码 node_id。原实现用一张写死的
        node_id 表,每根手指取到的是 [掌骨根, MCP, PIP, DIP] —— 比 MediaPipe 要的
        [MCP, PIP, DIP, TIP] 整体向近端错了一节,指尖从未发布。
        """
        by_chain: Dict[str, Dict[str, list]] = {}
        for node in msg.raw_nodes:
            bone = LEGACY_JOINT_TYPE_ALIASES.get(node.joint_type, node.joint_type)
            by_chain.setdefault(node.chain_type, {}).setdefault(bone, []).append(node)

        hand_bones = by_chain.get("Hand")
        if not hand_bones:
            self.get_logger().error("raw_nodes 中没有 chain_type='Hand' 的腕部节点")
            return np.zeros((21, 3), dtype=np.float32)
        wrist = min(
            (node for nodes in hand_bones.values() for node in nodes),
            key=lambda item: item.node_id,
        )

        mediapipe_pose = [self._node_position(wrist)]
        for chain_name in SEMANTIC_CHAIN_NAMES:
            bones = by_chain.get(chain_name, {})
            for bone_name in MEDIAPIPE_BONE_CHAIN[chain_name]:
                candidates = bones.get(bone_name)
                if not candidates:
                    self.get_logger().error(
                        f"{chain_name} 链缺少 {bone_name} 节点; "
                        f"该链现有: {sorted(bones)}"
                    )
                    return np.zeros((21, 3), dtype=np.float32)
                mediapipe_pose.append(
                    self._node_position(min(candidates, key=lambda item: item.node_id))
                )

        pose = np.asarray(mediapipe_pose, dtype=np.float32)
        self._log_skeleton_sanity_once(pose, msg.side)
        return pose

    @staticmethod
    def _chirality(pose: np.ndarray) -> tuple[float, float]:
        """手性判据。返回 ``(值, 可信度)``;屈曲姿态下**右手为负**。

            u    = 腕 -> 中指MCP        远端方向
            r    = 小指MCP -> 食指MCP    桡侧方向
            curl = 中指尖相对中指MCP、垂直于 u 的分量（屈曲时指向掌心）
            值   = dot(curl, normalize(cross(u, r)))

        基准取自 Hand 2 的 ``right.urdf``:屈曲 30-60 度时为 -5.5e-2 ~ -7.1e-2。

        可信度即 ``|curl|``。手指伸直时 curl 趋近 0,判据退化,符号不可信 ——
        这正是早期判据在摊平手上误判的原因,所以这里把它一并返回,由调用方决定
        是否采信。
        """
        u = pose[9] - pose[0]
        r = pose[5] - pose[17]
        norm_u = np.linalg.norm(u)
        n = np.cross(u, r)
        norm_n = np.linalg.norm(n)
        if norm_u < 1e-6 or norm_n < 1e-12:
            return 0.0, 0.0
        u_hat = u / norm_u
        v = pose[12] - pose[9]
        curl = v - np.dot(v, u_hat) * u_hat
        return float(np.dot(curl, n / norm_n)), float(np.linalg.norm(curl))

    def _log_skeleton_sanity_once(self, pose: np.ndarray, side: str) -> None:
        """首帧打印一次选点后的骨架尺寸与手性,便于当场发现取点或镜像问题。"""
        key = side.lower()
        if self._skeleton_checked.get(key, True):
            return
        self._skeleton_checked[key] = True

        wrist_to_mcp = float(np.linalg.norm(pose[9] - pose[0]))
        mcp_spread = float(np.linalg.norm(pose[17] - pose[5]))
        proximal = float(np.linalg.norm(pose[6] - pose[5]))
        chirality, confidence = self._chirality(pose)
        self.get_logger().info(
            f"[骨架自检 {side}] 腕->中指MCP={wrist_to_mcp * 100:.1f}cm "
            f"MCP展宽={mcp_spread * 100:.1f}cm 食指近节={proximal * 100:.1f}cm "
            f"手性={chirality:+.2e}(可信度{confidence * 100:.1f}cm)"
        )
        if wrist_to_mcp < 0.05 or proximal > 0.060:
            self.get_logger().error(
                f"[骨架自检 {side}] 尺寸不合常理 —— 取点可能错位一节。"
                "用 tools/dump_manus_nodes.py 核对节点拓扑,不要在此状态下 --control。"
            )

        # 手指伸直时判据退化,不足以判手性 —— 报告但不下结论。
        if confidence < 0.015:
            self.get_logger().warning(
                f"[骨架自检 {side}] 手性判据退化(可信度仅 {confidence * 100:.1f}cm,需 >1.5cm)。"
                "手指伸直时无法判定手性,请弯曲手指后重启本节点再看。"
            )
            return

        expected_negative = side.lower() == "right"
        mirrored = (chirality > 0) if expected_negative else (chirality < 0)
        if mirrored:
            self.get_logger().error(
                f"[骨架自检 {side}] 手性与 side='{side}' 相反 —— 数据被镜像了。"
                "retarget 会把屈曲解成伸展,实体手将朝手背方向持续运动。"
                "绝对不要在此状态下 --control。"
            )

    @staticmethod
    def _node_position(node) -> np.ndarray:
        """按原样取节点位置。

        上游在这里做 ``y = -pose.position.y``（注释称 "same as manus_data_viz"）。
        对单个轴取反是一次镜像;配合 MANUS Core 3.2.0 标定出来的数据,它会把右手
        变成左手。判据见 ``_chirality``:屈曲姿态下右手为负,而

            取反   -> +5.99e-02   左手 ✗
            不取反 -> -5.99e-02   右手 ✓（与 right.urdf 屈曲基准 -5.5e-2~-7.1e-2 同号）

        retarget 输出独立印证:取反时给出 MCP=-55.7 度而 PIP=-0.3 度,即 MCP 极度
        反折却手指笔直,解剖学上不可能;不取反给出 MCP=+24 度 PIP=+65 度,是正常的
        半握姿态。

        历史教训:这里曾被反复改动。早期用的手性判据取
        ``cross(index-wrist, pinky-wrist) . (thumb-wrist)``,在摊平的手上退化
        (拇指离掌面仅 0.86 mm,而 URDF 基准 9.3 mm),符号是噪声,据此判断会得到
        相反结论。改动此处前务必用屈曲姿态下的 ``_chirality`` 判据,并核对
        retarget 输出在解剖学上是否讲得通。
        """
        pose = node.pose
        return np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ], dtype=np.float32)

    # 原有的 _order_chain_nodes_by_joint_type / _convert_to_mediapipe_semantic 已删除:
    # 它们按 joint_type 字符串选 "MCP"/"PIP"/"DIP"/"TIP",而这些字符串正是被
    # manus_ros2 的 JointTypeToString 错误标注的那一组,因此那条"语义后备路径"
    # 与硬编码 id 路径犯的是同一个错。现在只保留 _convert_to_mediapipe 一条路径,
    # 按骨头名选点。

    def _glove_callback(self, msg: ManusGlove) -> None:
        """收到一只手套的数据：转换后暂存，由定时器发布。"""
        side = msg.side.lower()
        if side not in ("left", "right"):
            self.get_logger().warning(f"Unknown side: {msg.side}")
            return
        if side not in self._enabled_sides:
            return
        self._fingers[side] = self._convert_to_mediapipe(msg)
        self._pending[side] = True

    def _publish_latest_frame(self) -> None:
        """把各手最新的一帧发到各自的 topic。

        两只手**互不阻塞**：某只手没有新数据就跳过它，另一只照常发。
        原实现把两只手拼进同一条 topic，且任一只手缺数据就整体不发 ——
        左手掉线会把右手一起停掉，这个耦合在遥操作中不可接受。
        """
        for side in self._enabled_sides:
            if not self._pending[side]:
                continue
            fingers = self._fingers[side]
            if fingers is None:
                continue
            self._pending[side] = False
            message = Float32MultiArray()
            message.data = fingers.flatten().astype(np.float32).tolist()
            self._hand_publishers[side].publish(message)


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish Manus glove hand data to ROS2 topics.")
    parser.add_argument(
        "-c",
        "--config",
        default=None,
        help="Path to a Manus input YAML configuration file.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    program_name = sys.argv[0] if sys.argv else "manus_input"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)

    if args.config:
        config_path = Path(args.config)
    else:
        share_dir = Path(get_package_share_directory("manus_input_py"))
        config_path = share_dir / "config" / "manus_input.yaml"

    config = ManusInputConfig.from_file(str(config_path))

    rclpy.init(args=raw_argv)
    node = ManusInputNode(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
