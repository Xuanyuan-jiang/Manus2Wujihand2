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
MEDIAPIPE_BONE_CHAIN = {
    "Thumb": ("Metacarpal", "Proximal", "Intermediate", "Tip"),   # CMC, MCP, IP,  TIP
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
    publish_hand_topic: str = "/hand_input"
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

        # Storage for latest data from each glove
        self._left_fingers: Optional[np.ndarray] = None
        self._right_fingers: Optional[np.ndarray] = None
        # Freshness flag: only publish when new data arrives from C++ layer
        # Prevents infinite re-publishing of stale data causing dexterous hand jitter after glove disconnects
        self._new_data_received: bool = False
        # 首帧做一次骨架尺寸自检并打印,只做一次
        self._skeleton_checked: bool = False

        # Configure QoS for real-time performance
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publisher
        self.hand_publisher = self.create_publisher(
            Float32MultiArray, self.config.publish_hand_topic, qos_profile
        )

        # Subscribe to all manus_glove topics, use msg.side to determine left/right
        self.create_subscription(ManusGlove, "/manus_glove_0", self._glove_callback, qos_profile)
        self.create_subscription(ManusGlove, "/manus_glove_1", self._glove_callback, qos_profile)
        self.get_logger().info("[Manus Input] Subscribed to /manus_glove_0 and /manus_glove_1")

        # Timer for publishing
        self.timer = self.create_timer(
            1.0 / max(self.config.publish_rate_hz, 1.0), self._publish_latest_frame
        )
        self.get_logger().info(
            f"[Manus Input] Publishing hand data on '{self.config.publish_hand_topic}' "
            f"at {self.config.publish_rate_hz:.1f} Hz."
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
        self._log_skeleton_sanity_once(pose)
        return pose

    def _log_skeleton_sanity_once(self, pose: np.ndarray) -> None:
        """首帧打印一次选点后的骨架尺寸,便于当场看出取点是否仍然错位。"""
        if self._skeleton_checked:
            return
        self._skeleton_checked = True

        wrist_to_mcp = float(np.linalg.norm(pose[9] - pose[0]))
        mcp_spread = float(np.linalg.norm(pose[17] - pose[5]))
        proximal = float(np.linalg.norm(pose[6] - pose[5]))
        self.get_logger().info(
            f"[骨架自检] 腕->中指MCP={wrist_to_mcp * 100:.1f}cm "
            f"MCP展宽={mcp_spread * 100:.1f}cm 食指近节={proximal * 100:.1f}cm"
        )
        if wrist_to_mcp < 0.05 or proximal > 0.060:
            self.get_logger().error(
                "[骨架自检] 尺寸不合常理 —— 取点可能仍然错位一节。"
                "用 tools/dump_manus_nodes.py 核对节点拓扑,不要在此状态下 --control。"
            )

    @staticmethod
    def _node_position(node) -> np.ndarray:
        pose = node.pose
        return np.array([
            pose.position.x,
            -pose.position.y,
            pose.position.z,
        ], dtype=np.float32)

    # 原有的 _order_chain_nodes_by_joint_type / _convert_to_mediapipe_semantic 已删除:
    # 它们按 joint_type 字符串选 "MCP"/"PIP"/"DIP"/"TIP",而这些字符串正是被
    # manus_ros2 的 JointTypeToString 错误标注的那一组,因此那条"语义后备路径"
    # 与硬编码 id 路径犯的是同一个错。现在只保留 _convert_to_mediapipe 一条路径,
    # 按骨头名选点。

    def _glove_callback(self, msg: ManusGlove) -> None:
        """Callback for glove data, determines left/right based on msg.side."""
        mediapipe_data = self._convert_to_mediapipe(msg)
        if msg.side.lower() == "left":
            self._left_fingers = mediapipe_data
            self._new_data_received = True
        elif msg.side.lower() == "right":
            self._right_fingers = mediapipe_data
            self._new_data_received = True
        else:
            self.get_logger().warning(f"Unknown side: {msg.side}")

    def _publish_latest_frame(self) -> None:
        """Publish the latest hand data. Skip if no valid data received yet.

        Only publishes when new data arrives from C++ layer. Prevents infinite
        re-publishing of stale data causing dexterous hand jitter after glove disconnects.

        Data format: [right_hand (63), left_hand (63)] - MediaPipe 21-point format.
        """
        # No new data: skip publishing to avoid stale data causing continuous downstream retarget
        if not self._new_data_received:
            return

        # Check data completeness before clearing flag (prevent data loss)
        if self.config.include_right_hand and self._right_fingers is None:
            return  # Right hand data not yet arrived, keep flag for next retry
        if self.config.include_left_hand and self._left_fingers is None:
            return  # Left hand data not yet arrived, keep flag for next retry

        # Data complete, clear flag and publish
        self._new_data_received = False

        payloads = []
        if self.config.include_right_hand:
            payloads.append(self._right_fingers.flatten())
        if self.config.include_left_hand:
            payloads.append(self._left_fingers.flatten())

        if payloads:
            hand_msg = Float32MultiArray()
            hand_msg.data = np.concatenate(payloads).astype(np.float32).tolist()
            self.hand_publisher.publish(hand_msg)


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
