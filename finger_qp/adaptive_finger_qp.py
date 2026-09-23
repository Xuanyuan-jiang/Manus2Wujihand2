#!/usr/bin/env python3
"""Framework-independent tactile QP joint-position adapter.

This module contains no Isaac Sim, ROS, hardware-driver, or project-local
imports.  Its only third-party dependency is NumPy.  Angles are radians,
forces are newtons, lengths are metres, and torques are newton-metres.

The minimum runtime inputs are:

1. a hand URDF;
2. the current position of every controlled URDF joint; and
3. one force vector per fingertip.

Force dictionary keys may be URDF fingertip link names directly, or logical
names resolved through ``tip_links``.  A plain three-element force is assumed
to be expressed in the fingertip-link local frame.  A scalar is interpreted as
local +Z force.  Rich measurements may specify ``frame`` and a local contact
point, for example::

    forces = {
        "index": {
            "force": [0.2, -0.1, 8.0],
            "frame": "local",
            "contact_point_local": [0.0, 0.0, 0.006],
        }
    }

Python API::

    controller = AdaptiveFingerQP(
        "right_hand.urdf",
        tip_links={"index": "right_index_fingertip"},
    )
    result = controller.adjust(current_joint_positions, forces)
    send_to_robot(result.joint_positions)

For a control loop, keep one ``AdaptiveFingerQP`` instance per hand so contact
hysteresis remains continuous.  ``result.delta`` is the position increment for
this call.  The caller remains responsible for actuator-level safety, timing,
collision avoidance, and emergency stopping.

Command-line JSON interface::

    python adaptive_finger_qp.py --urdf hand.urdf \
        --joints joints.json --forces forces.json --tip-map tip_links.json

The command prints a JSON result to stdout.  Use ``--describe`` to list the
URDF's independent movable joints and leaf links.
"""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


Vec3 = tuple[float, float, float]
ForceRanges = tuple[tuple[float, float], tuple[float, float], tuple[float, float]]


@dataclass(frozen=True)
class QPConfig:
    """Numerical and control parameters for one QP update."""

    contact_threshold_n: float = 0.5
    contact_release_threshold_n: float = 0.3
    force_target_ranges_local_n: ForceRanges = (
        (-5.0, 5.0),
        (-5.0, 5.0),
        (0.0, 5.0),
    )
    force_gain_position_per_n_xyz: Vec3 = (
        math.radians(0.2),
        math.radians(0.2),
        math.radians(0.4),
    )
    force_relief_direction_xyz: Vec3 = (1.0, 1.0, 1.0)
    task_axis_weights_xyz: Vec3 = (1.0, 1.0, 1.0)
    max_joint_step: float = math.radians(0.5)
    regularization: float = 1.0e-3
    stability_eps: float = 1.0e-6
    force_weight: float = 1.0
    max_solver_iterations: int = 64
    solver_tolerance: float = 1.0e-7
    smoothing_k: float = 10.0

    # Optional soft multi-contact equilibrium objective.  It is disabled by
    # default because the base force-relief QP needs fewer modelling
    # assumptions.  It can be enabled using only URDF kinematics and tip forces.
    enable_balance: bool = False
    force_balance_weight: float = 1.0
    torque_balance_weight: float = 1.0
    force_residual_stiffness: float = 100.0
    torque_residual_stiffness: float = 100.0
    target_net_force_xyz: Vec3 = (0.0, 0.0, 0.0)
    target_net_force_tolerance_n: float = 5.0
    target_net_torque_xyz: Vec3 = (0.0, 0.0, 0.0)
    target_net_torque_tolerance_nm: float = 10.0
    min_active_contacts_for_balance: int = 2
    balance_force_clip_n: float = 40.0


@dataclass(frozen=True)
class QPResult:
    """Adjusted joint command and diagnostics from one controller update."""

    joint_positions: dict[str, float]
    delta: dict[str, float]
    active_fingertips: tuple[str, ...]
    force_errors_local_n: dict[str, tuple[float, float, float]]
    solver_iterations: int
    objective: float

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["active_fingertips"] = list(self.active_fingertips)
        value["force_errors_local_n"] = {
            name: list(force) for name, force in self.force_errors_local_n.items()
        }
        return value


@dataclass(frozen=True)
class _UrdfJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    mimic_joint: str | None
    mimic_multiplier: float
    mimic_offset: float


@dataclass(frozen=True)
class _Measurement:
    force: np.ndarray
    frame: str
    contact_point_local: np.ndarray | None
    target_ranges: ForceRanges | None


@dataclass(frozen=True)
class _KinematicSample:
    name: str
    force_local: np.ndarray
    force_base: np.ndarray
    position_base: np.ndarray
    rotation_local_to_base: np.ndarray
    jacobian_base: np.ndarray
    jacobian_local: np.ndarray
    weight: float


def _vec3(value: Any, *, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64).reshape(-1)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain exactly three finite values")
    return result


def _xml_vec(text: str | None, default: Iterable[float]) -> np.ndarray:
    if text is None or not text.strip():
        return np.asarray(tuple(default), dtype=np.float64)
    return _vec3([float(value) for value in text.split()], name="URDF vector")


def _normalise_ranges(value: Any) -> ForceRanges:
    if len(value) != 3:
        raise ValueError("force target ranges must contain x, y, and z ranges")
    result: list[tuple[float, float]] = []
    for pair in value:
        if len(pair) != 2:
            raise ValueError("each force target range must contain [low, high]")
        low, high = float(pair[0]), float(pair[1])
        if not math.isfinite(low) or not math.isfinite(high):
            raise ValueError("force target ranges must be finite")
        result.append((min(low, high), max(low, high)))
    return result[0], result[1], result[2]


def _rotation_from_rpy(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    ry = np.asarray(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
    rz = np.asarray(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
    return rz @ ry @ rx


def _origin_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = _rotation_from_rpy(rpy)
    transform[:3, 3] = xyz
    return transform


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if norm <= 1.0e-12:
        raise ValueError("URDF joint axis has zero length")
    x, y, z = (float(value) for value in axis / norm)
    c, s = math.cos(float(angle)), math.sin(float(angle))
    t = 1.0 - c
    return np.asarray(
        (
            (c + x * x * t, x * y * t - z * s, x * z * t + y * s),
            (y * x * t + z * s, c + y * y * t, y * z * t - x * s),
            (z * x * t - y * s, z * y * t + x * s, c + z * z * t),
        ),
        dtype=np.float64,
    )


def _motion_transform(joint_type: str, axis: np.ndarray, value: float) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    if joint_type in {"revolute", "continuous"}:
        transform[:3, :3] = _axis_rotation(axis, value)
    elif joint_type == "prismatic":
        norm = float(np.linalg.norm(axis))
        if norm <= 1.0e-12:
            raise ValueError("URDF prismatic joint axis has zero length")
        transform[:3, 3] = (axis / norm) * float(value)
    elif joint_type != "fixed":
        raise ValueError(f"unsupported URDF joint type: {joint_type!r}")
    return transform


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = (float(value) for value in vector)
    return np.asarray(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))


def _radial_deadband(vector: np.ndarray, tolerance: float) -> np.ndarray:
    tolerance = max(float(tolerance), 0.0)
    norm = float(np.linalg.norm(vector))
    if norm <= tolerance:
        return np.zeros(3, dtype=np.float64)
    if norm <= 1.0e-12:
        return vector.copy()
    return vector * ((norm - tolerance) / norm)


class URDFKinematicModel:
    """Small URDF tree parser with analytic point-position Jacobians."""

    _SUPPORTED_TYPES = {"fixed", "revolute", "continuous", "prismatic"}

    def __init__(self, urdf_path: str | Path) -> None:
        self.urdf_path = Path(urdf_path).expanduser().resolve()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"URDF not found: {self.urdf_path}")

        root = ET.parse(self.urdf_path).getroot()
        self.robot_name = str(root.attrib.get("name", self.urdf_path.stem))
        self.links = {str(link.attrib["name"]) for link in root.findall("link")}
        self.joints: dict[str, _UrdfJoint] = {}
        self.joints_by_child: dict[str, _UrdfJoint] = {}

        for element in root.findall("joint"):
            joint_type = str(element.attrib.get("type", "fixed")).lower()
            if joint_type not in self._SUPPORTED_TYPES:
                raise ValueError(
                    f"joint {element.attrib.get('name')!r} uses unsupported type "
                    f"{joint_type!r}; supported types are {sorted(self._SUPPORTED_TYPES)}"
                )
            parent_element = element.find("parent")
            child_element = element.find("child")
            if parent_element is None or child_element is None:
                raise ValueError(f"joint {element.attrib.get('name')!r} lacks parent or child")
            origin = element.find("origin")
            axis = element.find("axis")
            limit = element.find("limit")
            mimic = element.find("mimic")
            lower = -math.inf
            upper = math.inf
            if joint_type in {"revolute", "prismatic"} and limit is not None:
                lower = float(limit.attrib.get("lower", -math.inf))
                upper = float(limit.attrib.get("upper", math.inf))
            lower, upper = min(lower, upper), max(lower, upper)
            joint = _UrdfJoint(
                name=str(element.attrib["name"]),
                joint_type=joint_type,
                parent=str(parent_element.attrib["link"]),
                child=str(child_element.attrib["link"]),
                xyz=_xml_vec(origin.attrib.get("xyz") if origin is not None else None, (0, 0, 0)),
                rpy=_xml_vec(origin.attrib.get("rpy") if origin is not None else None, (0, 0, 0)),
                axis=_xml_vec(axis.attrib.get("xyz") if axis is not None else None, (1, 0, 0)),
                lower=lower,
                upper=upper,
                mimic_joint=str(mimic.attrib["joint"]) if mimic is not None else None,
                mimic_multiplier=float(mimic.attrib.get("multiplier", 1.0)) if mimic is not None else 1.0,
                mimic_offset=float(mimic.attrib.get("offset", 0.0)) if mimic is not None else 0.0,
            )
            if joint.name in self.joints:
                raise ValueError(f"duplicate URDF joint name: {joint.name}")
            if joint.child in self.joints_by_child:
                raise ValueError(f"URDF link {joint.child!r} has multiple parent joints")
            self.joints[joint.name] = joint
            self.joints_by_child[joint.child] = joint

        self.independent_movable_joints = tuple(
            joint.name
            for joint in self.joints.values()
            if joint.joint_type != "fixed" and joint.mimic_joint is None
        )
        parent_links = {joint.parent for joint in self.joints.values()}
        child_links = {joint.child for joint in self.joints.values()}
        self.root_links = tuple(sorted(self.links - child_links))
        self.leaf_links = tuple(sorted(self.links - parent_links))

    def joint_limits(self, names: Sequence[str]) -> tuple[np.ndarray, np.ndarray]:
        lower, upper = [], []
        for name in names:
            joint = self.joints.get(str(name))
            if joint is None or joint.joint_type == "fixed" or joint.mimic_joint is not None:
                raise ValueError(f"{name!r} is not an independent movable URDF joint")
            lower.append(joint.lower)
            upper.append(joint.upper)
        return np.asarray(lower, dtype=np.float64), np.asarray(upper, dtype=np.float64)

    def _path_to_link(self, target_link: str) -> list[_UrdfJoint]:
        if target_link not in self.links:
            raise KeyError(f"URDF has no link named {target_link!r}")
        path: list[_UrdfJoint] = []
        seen: set[str] = set()
        current = target_link
        while current in self.joints_by_child:
            if current in seen:
                raise ValueError(f"cycle found while resolving link {target_link!r}")
            seen.add(current)
            joint = self.joints_by_child[current]
            path.append(joint)
            current = joint.parent
        path.reverse()
        return path

    def _resolve_motion_source(self, joint: _UrdfJoint) -> tuple[str, float, float]:
        source = joint
        multiplier = 1.0
        offset = 0.0
        seen: set[str] = set()
        while source.mimic_joint is not None:
            if source.name in seen:
                raise ValueError(f"mimic cycle involving joint {source.name!r}")
            seen.add(source.name)
            previous_multiplier = multiplier
            multiplier = previous_multiplier * source.mimic_multiplier
            offset = previous_multiplier * source.mimic_offset + offset
            try:
                source = self.joints[source.mimic_joint]
            except KeyError as exc:
                raise ValueError(
                    f"mimic joint {joint.name!r} references missing joint "
                    f"{source.mimic_joint!r}"
                ) from exc
        return source.name, multiplier, offset

    def point_pose_jacobian(
        self,
        joint_positions: Mapping[str, float],
        *,
        target_link: str,
        controlled_joint_names: Sequence[str],
        target_point_local: Iterable[float] | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return target position, local-to-base rotation, and 3xN Jacobian."""

        controlled = tuple(str(name) for name in controlled_joint_names)
        column = {name: index for index, name in enumerate(controlled)}
        transform = np.eye(4, dtype=np.float64)
        active: list[tuple[str, float, str, np.ndarray, np.ndarray]] = []

        for joint in self._path_to_link(str(target_link)):
            joint_frame = transform @ _origin_transform(joint.xyz, joint.rpy)
            if joint.joint_type == "fixed":
                transform = joint_frame
                continue
            source_name, multiplier, offset = self._resolve_motion_source(joint)
            source_value = float(joint_positions.get(source_name, 0.0))
            value = multiplier * source_value + offset
            axis_base = joint_frame[:3, :3] @ joint.axis
            norm = float(np.linalg.norm(axis_base))
            if norm <= 1.0e-12:
                raise ValueError(f"joint {joint.name!r} has a zero axis")
            axis_base = axis_base / norm
            if source_name in column:
                active.append(
                    (source_name, multiplier, joint.joint_type, joint_frame[:3, 3].copy(), axis_base)
                )
            transform = joint_frame @ _motion_transform(joint.joint_type, joint.axis, value)

        rotation = transform[:3, :3].copy()
        position = transform[:3, 3].copy()
        if target_point_local is not None:
            position = position + rotation @ _vec3(target_point_local, name="contact_point_local")

        jacobian = np.zeros((3, len(controlled)), dtype=np.float64)
        for source_name, multiplier, joint_type, origin, axis in active:
            index = column[source_name]
            if joint_type in {"revolute", "continuous"}:
                jacobian[:, index] += multiplier * np.cross(axis, position - origin)
            elif joint_type == "prismatic":
                jacobian[:, index] += multiplier * axis
        return position, rotation, jacobian


def _measurement(value: Any) -> _Measurement:
    frame = "local"
    contact_point = None
    target_ranges = None
    raw_force = value
    if isinstance(value, Mapping):
        raw_force = value.get("force", value.get("force_xyz", value.get("force_local_xyz")))
        if raw_force is None and "force_world_xyz" in value:
            raw_force = value["force_world_xyz"]
            frame = "base"
        frame = str(value.get("frame", frame)).lower()
        if value.get("contact_point_local") is not None:
            contact_point = _vec3(value["contact_point_local"], name="contact_point_local")
        if value.get("target_ranges") is not None:
            target_ranges = _normalise_ranges(value["target_ranges"])
    if np.isscalar(raw_force):
        raw_force = (0.0, 0.0, float(raw_force))
    if frame not in {"local", "base", "world"}:
        raise ValueError("force frame must be 'local', 'base', or 'world'")
    return _Measurement(
        force=_vec3(raw_force, name="fingertip force"),
        frame="base" if frame == "world" else frame,
        contact_point_local=contact_point,
        target_ranges=target_ranges,
    )


def _force_error(force: np.ndarray, ranges: ForceRanges) -> np.ndarray:
    result = np.zeros(3, dtype=np.float64)
    for axis, value in enumerate(force):
        low, high = ranges[axis]
        if value < low:
            result[axis] = value - low
        elif value > high:
            result[axis] = value - high
    return result


def _solve_box_qp(
    hessian: np.ndarray,
    gradient: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    *,
    warm_start: np.ndarray | None,
    max_iterations: int,
    tolerance: float,
) -> tuple[np.ndarray, int]:
    """Solve min 0.5*x.T*H*x + g.T*x under element-wise bounds."""

    hessian = 0.5 * (hessian + hessian.T)
    if np.any(lower > upper):
        raise ValueError("infeasible QP bounds")
    if warm_start is not None and warm_start.shape == gradient.shape:
        x = np.clip(np.asarray(warm_start, dtype=np.float64), lower, upper)
    else:
        try:
            x = np.linalg.solve(hessian, -gradient)
        except np.linalg.LinAlgError:
            x = np.zeros_like(gradient)
        x = np.clip(x, lower, upper)

    try:
        lipschitz = float(np.linalg.eigvalsh(hessian)[-1])
    except np.linalg.LinAlgError:
        lipschitz = float(np.linalg.norm(hessian, ord=np.inf))
    step = 1.0 / max(lipschitz, 1.0e-12)
    iterations = 0
    for iterations in range(1, max(1, int(max_iterations)) + 1):
        next_x = np.clip(x - step * (hessian @ x + gradient), lower, upper)
        if float(np.max(np.abs(next_x - x), initial=0.0)) <= max(float(tolerance), 0.0):
            x = next_x
            break
        x = next_x
    if not np.all(np.isfinite(x)):
        raise FloatingPointError("QP solver produced non-finite joint adjustments")
    return x, iterations


class AdaptiveFingerQP:
    """Stateful, hardware-independent fingertip-force QP controller."""

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        tip_links: Mapping[str, str] | None = None,
        controlled_joint_names: Sequence[str] | None = None,
        config: QPConfig | None = None,
        joint_limit_overrides: Mapping[str, Sequence[float]] | None = None,
        max_joint_step_overrides: Mapping[str, float] | None = None,
    ) -> None:
        self.model = URDFKinematicModel(urdf_path)
        self.tip_links = {str(key): str(value) for key, value in (tip_links or {}).items()}
        self.config = config or QPConfig()
        self.controlled_joint_names = tuple(
            str(name)
            for name in (
                controlled_joint_names
                if controlled_joint_names is not None
                else self.model.independent_movable_joints
            )
        )
        if not self.controlled_joint_names:
            raise ValueError("URDF contains no independent movable joints")
        if len(set(self.controlled_joint_names)) != len(self.controlled_joint_names):
            raise ValueError("controlled_joint_names contains duplicates")

        lower, upper = self.model.joint_limits(self.controlled_joint_names)
        for name, values in (joint_limit_overrides or {}).items():
            if name not in self.controlled_joint_names or len(values) != 2:
                raise ValueError(f"invalid joint limit override for {name!r}")
            index = self.controlled_joint_names.index(name)
            low, high = float(values[0]), float(values[1])
            lower[index], upper[index] = min(low, high), max(low, high)
        self._lower = lower
        self._upper = upper

        default_step = max(float(self.config.max_joint_step), 0.0)
        self._max_step = np.full(len(self.controlled_joint_names), default_step, dtype=np.float64)
        for name, value in (max_joint_step_overrides or {}).items():
            if name not in self.controlled_joint_names:
                raise ValueError(f"unknown max-step override joint {name!r}")
            self._max_step[self.controlled_joint_names.index(name)] = max(float(value), 0.0)

        self._contact_active: dict[str, bool] = {}
        self._last_delta = np.zeros(len(self.controlled_joint_names), dtype=np.float64)

    def reset(self) -> None:
        """Clear hysteresis and QP warm-start state."""

        self._contact_active.clear()
        self._last_delta.fill(0.0)

    def _coerce_joint_positions(
        self, values: Mapping[str, float] | Sequence[float]
    ) -> tuple[np.ndarray, dict[str, float]]:
        if isinstance(values, Mapping):
            all_positions = {str(name): float(value) for name, value in values.items()}
            missing = [name for name in self.controlled_joint_names if name not in all_positions]
            if missing:
                raise ValueError(f"missing controlled joint positions: {missing}")
        else:
            array = np.asarray(values, dtype=np.float64).reshape(-1)
            if array.shape[0] != len(self.controlled_joint_names):
                raise ValueError(
                    f"expected {len(self.controlled_joint_names)} joint positions, got {array.shape[0]}"
                )
            all_positions = {
                name: float(array[index])
                for index, name in enumerate(self.controlled_joint_names)
            }
        q = np.asarray(
            [all_positions[name] for name in self.controlled_joint_names], dtype=np.float64
        )
        if not np.all(np.isfinite(q)):
            raise ValueError("joint positions must be finite")
        return q, all_positions

    def _contact_weight(self, name: str, force_norm: float) -> float:
        threshold = max(float(self.config.contact_threshold_n), 0.0)
        release = min(threshold, max(float(self.config.contact_release_threshold_n), 0.0))
        active = bool(self._contact_active.get(name, False))
        if not active and force_norm > threshold:
            active = True
        elif active and force_norm <= release:
            active = False
        self._contact_active[name] = active

        if force_norm <= release:
            return 0.0
        if threshold <= release:
            return 1.0 if active else 0.0
        midpoint = 0.5 * (threshold + release)
        exponent = -float(self.config.smoothing_k) * (force_norm - midpoint)
        exponent = min(max(exponent, -60.0), 60.0)
        return 1.0 / (1.0 + math.exp(exponent))

    def _bounds(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        lower = np.maximum(self._lower - q, -self._max_step)
        upper = np.minimum(self._upper - q, self._max_step)
        # If an input is already farther outside a URDF limit than one allowed
        # step, make the QP move it toward the valid interval instead of failing.
        for index in range(q.shape[0]):
            if lower[index] <= upper[index]:
                continue
            if q[index] < self._lower[index]:
                lower[index] = upper[index] = self._max_step[index]
            elif q[index] > self._upper[index]:
                lower[index] = upper[index] = -self._max_step[index]
            else:
                lower[index] = upper[index] = 0.0
        return lower, upper

    def _add_balance_cost(
        self,
        hessian: np.ndarray,
        gradient: np.ndarray,
        samples: Sequence[_KinematicSample],
    ) -> int:
        if not self.config.enable_balance:
            return 0
        if len(samples) < max(1, int(self.config.min_active_contacts_for_balance)):
            return 0

        positions = np.asarray([sample.position_base for sample in samples])
        centre = np.mean(positions, axis=0)
        force_residual = -_vec3(self.config.target_net_force_xyz, name="target net force")
        torque_residual = -_vec3(self.config.target_net_torque_xyz, name="target net torque")
        force_matrix = np.zeros_like(gradient)[None, :].repeat(3, axis=0)
        torque_matrix = np.zeros_like(force_matrix)
        force_clip = max(float(self.config.balance_force_clip_n), 0.0)

        for sample in samples:
            force = sample.force_base.copy()
            norm = float(np.linalg.norm(force))
            if force_clip > 0.0 and norm > force_clip:
                force *= force_clip / norm
            position = sample.position_base - centre
            force_jacobian = -sample.jacobian_base
            force_residual += sample.weight * force
            torque_residual += sample.weight * np.cross(position, force)
            force_matrix += sample.weight * force_jacobian
            torque_matrix += sample.weight * (_skew(position) @ force_jacobian)

        force_residual = _radial_deadband(
            force_residual, self.config.target_net_force_tolerance_n
        )
        torque_residual = _radial_deadband(
            torque_residual, self.config.target_net_torque_tolerance_nm
        )
        force_weight = max(float(self.config.force_balance_weight), 0.0)
        torque_weight = max(float(self.config.torque_balance_weight), 0.0)
        force_stiffness = max(float(self.config.force_residual_stiffness), 1.0e-12)
        torque_stiffness = max(float(self.config.torque_residual_stiffness), 1.0e-12)

        if force_weight > 0.0 and np.any(force_residual):
            weight = 2.0 * force_weight
            hessian += weight * (force_matrix.T @ force_matrix)
            gradient += weight * (force_matrix.T @ (force_residual / force_stiffness))
        if torque_weight > 0.0 and np.any(torque_residual):
            weight = 2.0 * torque_weight
            hessian += weight * (torque_matrix.T @ torque_matrix)
            gradient += weight * (torque_matrix.T @ (torque_residual / torque_stiffness))
        return len(samples)

    def adjust(
        self,
        joint_positions: Mapping[str, float] | Sequence[float],
        fingertip_forces: Mapping[str, Any],
    ) -> QPResult:
        """Compute one safe incremental joint-position adjustment."""

        q, all_positions = self._coerce_joint_positions(joint_positions)
        dimension = q.shape[0]
        regularization = max(float(self.config.regularization), 0.0)
        stability = max(float(self.config.stability_eps), 1.0e-12)
        hessian = (regularization + stability) * np.eye(dimension, dtype=np.float64)
        gradient = np.zeros(dimension, dtype=np.float64)
        samples: list[_KinematicSample] = []
        active: list[str] = []
        errors: dict[str, tuple[float, float, float]] = {}
        force_objective_count = 0

        seen = set()
        axis_weights = np.maximum(
            _vec3(self.config.task_axis_weights_xyz, name="task axis weights"), 0.0
        )
        gains = _vec3(self.config.force_gain_position_per_n_xyz, name="force gains")
        directions = _vec3(self.config.force_relief_direction_xyz, name="relief directions")
        default_ranges = _normalise_ranges(self.config.force_target_ranges_local_n)

        for logical_name, raw_measurement in fingertip_forces.items():
            name = str(logical_name)
            seen.add(name)
            target_link = self.tip_links.get(name, name)
            measurement = _measurement(raw_measurement)
            position, rotation, jacobian_base = self.model.point_pose_jacobian(
                all_positions,
                target_link=target_link,
                controlled_joint_names=self.controlled_joint_names,
                target_point_local=measurement.contact_point_local,
            )
            force_local = (
                measurement.force
                if measurement.frame == "local"
                else rotation.T @ measurement.force
            )
            force_base = rotation @ force_local
            weight = self._contact_weight(name, float(np.linalg.norm(force_local)))
            jacobian_local = rotation.T @ jacobian_base
            if weight > 1.0e-3 and np.any(jacobian_base):
                active.append(name)
                samples.append(
                    _KinematicSample(
                        name=name,
                        force_local=force_local,
                        force_base=force_base,
                        position_base=position,
                        rotation_local_to_base=rotation,
                        jacobian_base=jacobian_base,
                        jacobian_local=jacobian_local,
                        weight=weight,
                    )
                )

            error = _force_error(force_local, measurement.target_ranges or default_ranges)
            errors[name] = tuple(float(value) for value in error)
            if weight <= 1.0e-3 or not np.any(error) or not np.any(jacobian_local):
                continue
            target = directions * gains * error
            weighted_jacobian = axis_weights[:, None] * jacobian_local
            weighted_target = axis_weights * target
            combined_weight = max(float(self.config.force_weight), 0.0) * weight
            hessian += combined_weight * (weighted_jacobian.T @ weighted_jacobian)
            gradient += -combined_weight * (weighted_jacobian.T @ weighted_target)
            force_objective_count += 1

        for old_name in set(self._contact_active) - seen:
            self._contact_active[old_name] = False

        balance_count = self._add_balance_cost(hessian, gradient, samples)
        if force_objective_count == 0 and balance_count == 0:
            delta = np.zeros(dimension, dtype=np.float64)
            iterations = 0
        else:
            lower, upper = self._bounds(q)
            delta, iterations = _solve_box_qp(
                hessian,
                gradient,
                lower,
                upper,
                warm_start=self._last_delta,
                max_iterations=self.config.max_solver_iterations,
                tolerance=self.config.solver_tolerance,
            )
        self._last_delta = delta.copy()

        adjusted = q + delta
        finite_lower = np.isfinite(self._lower)
        finite_upper = np.isfinite(self._upper)
        adjusted[finite_lower] = np.maximum(adjusted[finite_lower], self._lower[finite_lower])
        adjusted[finite_upper] = np.minimum(adjusted[finite_upper], self._upper[finite_upper])
        output = dict(all_positions)
        delta_by_name: dict[str, float] = {}
        for index, name in enumerate(self.controlled_joint_names):
            output[name] = float(adjusted[index])
            delta_by_name[name] = float(adjusted[index] - q[index])
        objective = float(0.5 * delta @ hessian @ delta + gradient @ delta)
        return QPResult(
            joint_positions=output,
            delta=delta_by_name,
            active_fingertips=tuple(active),
            force_errors_local_n=errors,
            solver_iterations=iterations,
            objective=objective,
        )


def adjust_joint_positions(
    urdf_path: str | Path,
    joint_positions: Mapping[str, float] | Sequence[float],
    fingertip_forces: Mapping[str, Any],
    *,
    tip_links: Mapping[str, str] | None = None,
    controlled_joint_names: Sequence[str] | None = None,
    config: QPConfig | None = None,
) -> QPResult:
    """Stateless convenience wrapper for a single adjustment."""

    return AdaptiveFingerQP(
        urdf_path,
        tip_links=tip_links,
        controlled_joint_names=controlled_joint_names,
        config=config,
    ).adjust(joint_positions, fingertip_forces)


def _read_json(path: str | Path) -> Any:
    with Path(path).expanduser().open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _config_from_mapping(value: Mapping[str, Any] | None) -> QPConfig:
    if value is None:
        return QPConfig()
    known = {field.name for field in fields(QPConfig)}
    unknown = sorted(set(value) - known)
    if unknown:
        raise ValueError(f"unknown QP config fields: {unknown}")
    return QPConfig(**dict(value))


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, help="hand URDF path")
    parser.add_argument("--joints", help="JSON object containing current joint positions")
    parser.add_argument("--forces", help="JSON object containing fingertip forces")
    parser.add_argument("--tip-map", help="optional JSON mapping from force keys to URDF tip links")
    parser.add_argument("--config", help="optional JSON object overriding QPConfig fields")
    parser.add_argument("--describe", action="store_true", help="print URDF joints and leaf links")
    parser.add_argument("--compact", action="store_true", help="emit compact JSON")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_argument_parser().parse_args(argv)
    model = URDFKinematicModel(args.urdf)
    if args.describe:
        print(
            json.dumps(
                {
                    "robot": model.robot_name,
                    "urdf": str(model.urdf_path),
                    "independent_movable_joints": list(model.independent_movable_joints),
                    "root_links": list(model.root_links),
                    "leaf_links": list(model.leaf_links),
                },
                ensure_ascii=False,
                indent=None if args.compact else 2,
            )
        )
        return 0
    if not args.joints or not args.forces:
        raise SystemExit("--joints and --forces are required unless --describe is used")
    joints = _read_json(args.joints)
    forces = _read_json(args.forces)
    tip_map = _read_json(args.tip_map) if args.tip_map else None
    config = _config_from_mapping(_read_json(args.config) if args.config else None)
    result = AdaptiveFingerQP(args.urdf, tip_links=tip_map, config=config).adjust(joints, forces)
    print(
        json.dumps(
            result.to_dict(),
            ensure_ascii=False,
            indent=None if args.compact else 2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
