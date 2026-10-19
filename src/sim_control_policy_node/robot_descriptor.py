"""Machine-readable robot descriptor for simulation skills.

A ``robot.yaml`` next to the MJCF/URDF model is the single source of truth for
the skill: it drives the SKILL.md robot table (via the skill's generator
script), the ``sim.control``/``get_robot_info`` tool output, and the per-robot
simulator/motion configuration. Keep it intentionally declarative so that
adding a robot never requires code changes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

_KINDS = ("revolute", "prismatic")
_UNITS = {"revolute": "radians", "prismatic": "meters"}


@dataclass(frozen=True)
class JointDescriptor:
    """One controlled joint with its Forge-boundary unit and limits."""

    name: str
    kind: str
    limits: tuple[float, float]

    @property
    def unit(self) -> str:
        return _UNITS[self.kind]

    def to_json(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "kind": self.kind,
            "unit": self.unit,
            "limits": [self.limits[0], self.limits[1]],
        }


@dataclass(frozen=True)
class RobotDescriptor:
    """Validated contents of one ``robots/<name>/robot.yaml``."""

    name: str
    source: str  # mjcf | urdf
    model: str  # file name relative to the descriptor directory
    joints: tuple[JointDescriptor, ...]
    eef_site: str
    initial_qpos: dict[str, float]
    has_gripper: bool
    notes: str

    @property
    def dof(self) -> int:
        return len(self.joints)

    def to_info_json(self) -> dict[str, Any]:
        """Payload of the ``get_robot_info`` query operation."""
        return {
            "name": self.name,
            "source": self.source,
            "model": self.model,
            "dof": self.dof,
            "joints": [joint.to_json() for joint in self.joints],
            "eef_site": self.eef_site,
            "initial_qpos": dict(self.initial_qpos),
            "has_gripper": self.has_gripper,
            "notes": self.notes,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> RobotDescriptor:
        if not isinstance(data, dict):
            raise ValueError("robot descriptor must be a mapping")
        allowed = {"name", "source", "model", "joints", "eef_site",
                   "initial_qpos", "has_gripper", "notes"}
        unknown = sorted(set(data) - allowed)
        if unknown:
            raise ValueError(f"unknown robot descriptor keys: {unknown}")

        name = data.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError("robot descriptor 'name' must be a non-empty string")
        source = data.get("source")
        if source not in ("mjcf", "urdf"):
            raise ValueError("robot descriptor 'source' must be mjcf or urdf")
        model = data.get("model")
        if not isinstance(model, str) or not model.strip():
            raise ValueError("robot descriptor 'model' must be a file name")

        raw_joints = data.get("joints")
        if not isinstance(raw_joints, list) or not raw_joints:
            raise ValueError("robot descriptor 'joints' must be a non-empty list")
        joints: list[JointDescriptor] = []
        for entry in raw_joints:
            if not isinstance(entry, dict):
                raise ValueError("each joint must be a mapping")
            joint_name = entry.get("name")
            if not isinstance(joint_name, str) or not joint_name.strip():
                raise ValueError("joint 'name' must be a non-empty string")
            kind = entry.get("kind")
            if kind not in _KINDS:
                raise ValueError(f"joint {joint_name!r} kind must be one of {_KINDS}")
            key = "limits_rad" if kind == "revolute" else "limits_m"
            limits = entry.get(key)
            if (
                not isinstance(limits, list)
                or len(limits) != 2
                or any(isinstance(v, bool) or not isinstance(v, int | float) for v in limits)
                or not all(math.isfinite(float(v)) for v in limits)
                or float(limits[0]) >= float(limits[1])
            ):
                raise ValueError(
                    f"joint {joint_name!r} {key} must be two finite numbers [min, max]"
                )
            joints.append(JointDescriptor(joint_name, kind, (float(limits[0]), float(limits[1]))))
        if len({joint.name for joint in joints}) != len(joints):
            raise ValueError("joint names must be unique")

        eef_site = data.get("eef_site")
        if not isinstance(eef_site, str) or not eef_site.strip():
            raise ValueError("robot descriptor 'eef_site' must be a non-empty string")

        raw_initial = data.get("initial_qpos", {})
        if not isinstance(raw_initial, dict):
            raise ValueError("robot descriptor 'initial_qpos' must be a mapping")
        initial_qpos: dict[str, float] = {}
        joint_names = {joint.name for joint in joints}
        for joint_name, value in raw_initial.items():
            if joint_name not in joint_names:
                raise ValueError(
                    f"initial_qpos references unknown joint {joint_name!r}"
                )
            if (
                isinstance(value, bool)
                or not isinstance(value, int | float)
                or not math.isfinite(float(value))
            ):
                raise ValueError(f"initial_qpos[{joint_name!r}] must be a finite number")
            initial_qpos[joint_name] = float(value)

        has_gripper = data.get("has_gripper", any(j.kind == "prismatic" for j in joints))
        if not isinstance(has_gripper, bool):
            raise ValueError("robot descriptor 'has_gripper' must be a boolean")
        notes = data.get("notes", "")
        if not isinstance(notes, str):
            raise ValueError("robot descriptor 'notes' must be a string")

        return cls(
            name=name,
            source=source,
            model=model,
            joints=tuple(joints),
            eef_site=eef_site,
            initial_qpos=initial_qpos,
            has_gripper=has_gripper,
            notes=notes,
        )


def load_robot_descriptor(path: str | Path) -> RobotDescriptor:
    """Load and validate one robots/<name>/robot.yaml file."""
    file_path = Path(path).expanduser().resolve()
    if not file_path.is_file():
        raise FileNotFoundError(f"robot descriptor not found: {file_path}")
    data = yaml.safe_load(file_path.read_text(encoding="utf-8")) or {}
    return RobotDescriptor.from_dict(data)
