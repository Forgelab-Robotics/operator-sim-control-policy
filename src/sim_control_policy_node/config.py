"""Node configuration loading for the sim control policy node."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

_DEFAULT_REHOME_TIMEOUT_MS = 15_000
_DEFAULT_POLL_INTERVAL_MS = 1_000
_DEFAULT_SETTLE_MS = 500


@dataclass(frozen=True)
class InnerToolSpec:
    """Identity of a Tool the reset re-home dispatches through the gateway."""

    tool_id: str
    implementation_id: str
    endpoint_id: str
    operation: str
    arguments: dict[str, Any]


@dataclass(frozen=True)
class ResetConfig:
    """Optional re-home stage appended to ``reset_scene``.

    ``reset_scene`` alone leaves the arm wherever the trajectory controllers'
    hold positions drag it back to, so the reset also drives the robot to the
    descriptor's ``initial_qpos`` through the standard Tool path (public
    ``tool_request`` channel) and completes when those inner actions finish.
    """

    motion: InnerToolSpec | None
    gripper: InnerToolSpec | None
    timeout_ms: int
    poll_interval_ms: int
    settle_ms: int


@dataclass(frozen=True)
class SimControlPolicyConfig:
    """Everything the node needs: which robot.yaml describes the simulated robot."""

    robot_descriptor_path: Path
    reset: ResetConfig | None = None

    @classmethod
    def from_dict(cls, data: dict) -> SimControlPolicyConfig:
        if not isinstance(data, dict):
            raise ValueError("sim control policy config must be a mapping")
        allowed = {"robot_descriptor_path", "reset"}
        unknown = sorted(set(data) - allowed)
        if unknown:
            raise ValueError(f"unknown sim control policy config keys: {unknown}")
        raw = data.get("robot_descriptor_path")
        if not isinstance(raw, str) or not raw.strip():
            raise ValueError("robot_descriptor_path must be set to a robots/<name>/robot.yaml path")
        reset = None if data.get("reset") is None else _parse_reset(data["reset"])
        return cls(robot_descriptor_path=Path(raw).expanduser(), reset=reset)


def _positive_int(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"reset.{field} must be a positive integer")
    return value


def _non_negative_int(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"reset.{field} must be a non-negative integer")
    return value


def _parse_reset(data: Any) -> ResetConfig:
    if not isinstance(data, dict):
        raise ValueError("reset must be a mapping")
    allowed = {"motion", "gripper", "timeout_ms", "poll_interval_ms", "settle_ms"}
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise ValueError(f"unknown reset config keys: {unknown}")
    motion = None if data.get("motion") is None else _parse_inner(data["motion"], "motion")
    gripper = None if data.get("gripper") is None else _parse_inner(data["gripper"], "gripper")
    if motion is None and gripper is not None:
        raise ValueError("reset.gripper requires reset.motion (an arm re-home is mandatory)")
    return ResetConfig(
        motion=motion,
        gripper=gripper,
        timeout_ms=_positive_int(
            data.get("timeout_ms", _DEFAULT_REHOME_TIMEOUT_MS), "timeout_ms"
        ),
        poll_interval_ms=_positive_int(
            data.get("poll_interval_ms", _DEFAULT_POLL_INTERVAL_MS), "poll_interval_ms"
        ),
        settle_ms=_non_negative_int(
            data.get("settle_ms", _DEFAULT_SETTLE_MS), "settle_ms"
        ),
    )


def _parse_inner(data: Any, name: str) -> InnerToolSpec:
    if not isinstance(data, dict):
        raise ValueError(f"reset.{name} must be a mapping")
    allowed = {
        "tool_id",
        "implementation_id",
        "endpoint_id",
        "operation",
        "arguments",
    }
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise ValueError(f"unknown reset.{name} keys: {unknown}")
    for field in ("tool_id", "implementation_id", "endpoint_id", "operation"):
        value = data.get(field)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"reset.{name}.{field} must be a non-empty string")
    arguments = data.get("arguments", {})
    if not isinstance(arguments, dict):
        raise ValueError(f"reset.{name}.arguments must be a mapping")
    return InnerToolSpec(
        tool_id=data["tool_id"],
        implementation_id=data["implementation_id"],
        endpoint_id=data["endpoint_id"],
        operation=data["operation"],
        arguments=dict(arguments),
    )


def load_config(path: str | Path) -> SimControlPolicyConfig:
    """Resolve a --config YAML; relative robot paths stay relative to the file."""
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"sim control policy config not found: {config_path}")
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    config = SimControlPolicyConfig.from_dict(data)
    descriptor = config.robot_descriptor_path
    if not descriptor.is_absolute():
        descriptor = (config_path.parent / descriptor).resolve()
    return SimControlPolicyConfig(robot_descriptor_path=descriptor, reset=config.reset)
