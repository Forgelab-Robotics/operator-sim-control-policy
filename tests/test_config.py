from __future__ import annotations

from pathlib import Path

import pytest

from sim_control_policy_node.config import (
    InnerToolSpec,
    ResetConfig,
    SimControlPolicyConfig,
    load_config,
)


def _write(tmp_path: Path, name: str, text: str) -> Path:
    file_path = tmp_path / name
    file_path.write_text(text, encoding="utf-8")
    return file_path


def test_loads_absolute_robot_descriptor_path(tmp_path: Path) -> None:
    config = _write(tmp_path, "sim_control_policy.yaml", "robot_descriptor_path: /abs/robot.yaml\n")

    assert load_config(config) == SimControlPolicyConfig(
        robot_descriptor_path=Path("/abs/robot.yaml")
    )


def test_relative_robot_descriptor_resolves_against_config_directory(tmp_path: Path) -> None:
    nested = tmp_path / "profiles" / "mujoco-piper"
    nested.mkdir(parents=True)
    config = _write(
        nested,
        "sim_control_policy.yaml",
        "robot_descriptor_path: ../../robots/piper/robot.yaml\n",
    )

    loaded = load_config(config)

    assert loaded.robot_descriptor_path == (
        tmp_path / "robots" / "piper" / "robot.yaml"
    ).resolve()


def test_unknown_keys_are_rejected(tmp_path: Path) -> None:
    config = _write(
        tmp_path,
        "sim_control_policy.yaml",
        "robot_descriptor_path: /abs/robot.yaml\nsimulator_hz: 50\n",
    )
    with pytest.raises(ValueError):
        load_config(config)


@pytest.mark.parametrize(
    "text",
    [
        "",
        "robot_descriptor_path: ''\n",
        "robot_descriptor_path: null\n",
        "robot_descriptor_path: [a, b]\n",
    ],
)
def test_invalid_robot_descriptor_values_are_rejected(tmp_path: Path, text: str) -> None:
    config = _write(tmp_path, "sim_control_policy.yaml", text)
    with pytest.raises(ValueError):
        load_config(config)


def test_missing_config_file_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        load_config(tmp_path / "absent.yaml")


def test_from_dict_rejects_non_mapping() -> None:
    with pytest.raises(ValueError):
        SimControlPolicyConfig.from_dict(["not", "a", "mapping"])  # type: ignore[arg-type]


# -- reset re-home configuration ---------------------------------------------


def test_loads_reset_block_with_defaults(tmp_path: Path) -> None:
    config = _write(
        tmp_path,
        "sim_control_policy.yaml",
        (
            "robot_descriptor_path: /abs/robot.yaml\n"
            "reset:\n"
            "  motion:\n"
            "    tool_id: sim.set_joint_positions\n"
            "    implementation_id: motion.integration\n"
            "    endpoint_id: motion.server\n"
            "    operation: move_joints\n"
            "    arguments:\n"
            "      group_name: piper_arm\n"
            "      velocity_scale: 0.3\n"
        ),
    )

    loaded = load_config(config)

    assert loaded.reset == ResetConfig(
        motion=InnerToolSpec(
            tool_id="sim.set_joint_positions",
            implementation_id="motion.integration",
            endpoint_id="motion.server",
            operation="move_joints",
            arguments={"group_name": "piper_arm", "velocity_scale": 0.3},
        ),
        gripper=None,
        timeout_ms=15_000,
        poll_interval_ms=1_000,
        settle_ms=500,
    )


def test_loads_gripper_stage_and_overrides(tmp_path: Path) -> None:
    config = _write(
        tmp_path,
        "sim_control_policy.yaml",
        (
            "robot_descriptor_path: /abs/robot.yaml\n"
            "reset:\n"
            "  timeout_ms: 30000\n"
            "  poll_interval_ms: 250\n"
            "  motion:\n"
            "    tool_id: sim.set_joint_positions\n"
            "    implementation_id: motion.integration\n"
            "    endpoint_id: motion.server\n"
            "    operation: move_joints\n"
            "    arguments: {group_name: so101_arm}\n"
            "  gripper:\n"
            "    tool_id: sim.set_gripper_opening\n"
            "    implementation_id: piper.gripper\n"
            "    endpoint_id: gripper.controller\n"
            "    operation: set_opening\n"
        ),
    )

    loaded = load_config(config)

    assert loaded.reset is not None
    assert (loaded.reset.timeout_ms, loaded.reset.poll_interval_ms, loaded.reset.settle_ms) == (
        30_000,
        250,
        500,
    )
    assert loaded.reset.gripper is not None
    assert loaded.reset.gripper.operation == "set_opening"
    assert loaded.reset.gripper.arguments == {}


_MOTION_BLOCK = (
    "  motion:\n"
    "    tool_id: t\n"
    "    implementation_id: i\n"
    "    endpoint_id: e\n"
    "    operation: o\n"
)
_GRIPPER_BLOCK = (
    "  gripper:\n"
    "    tool_id: t\n"
    "    implementation_id: i\n"
    "    endpoint_id: e\n"
    "    operation: o\n"
)


@pytest.mark.parametrize(
    "reset_text",
    [
        # gripper without motion is meaningless: the arm re-home is mandatory
        _GRIPPER_BLOCK,
        # unknown keys at every level
        "  extra: true\n" + _MOTION_BLOCK,
        "  motion:\n    tool_id: t\n    unknown: x\n",
        "  motion: [not, a, mapping]\n",
        "  motion:\n    tool_id: ''\n"
        "    implementation_id: i\n    endpoint_id: e\n    operation: o\n",
        _MOTION_BLOCK + "    arguments: [1]\n",
        "  timeout_ms: 0\n",
        "  timeout_ms: -5\n",
        "  poll_interval_ms: fast\n",
        "  settle_ms: -1\n",
        "reset: [not, a, mapping]\n",
    ],
)
def test_invalid_reset_blocks_are_rejected(tmp_path: Path, reset_text: str) -> None:
    text = "robot_descriptor_path: /abs/robot.yaml\n"
    if not reset_text.startswith("reset"):
        text += "reset:\n"
    config = _write(tmp_path, "sim_control_policy.yaml", text + reset_text)
    with pytest.raises(ValueError):
        load_config(config)


def test_reset_null_keeps_instant_completion(tmp_path: Path) -> None:
    config = _write(
        tmp_path,
        "sim_control_policy.yaml",
        "robot_descriptor_path: /abs/robot.yaml\nreset:\n",
    )

    assert load_config(config).reset is None
