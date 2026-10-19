from __future__ import annotations

import textwrap
from pathlib import Path

import pytest

from sim_control_policy_node.robot_descriptor import (
    JointDescriptor,
    RobotDescriptor,
    load_robot_descriptor,
)


def _robot_yaml(**overrides: object) -> str:
    values: dict[str, object] = {
        "name": "piper",
        "source": "mjcf",
        "model": "piper_mujoco/scene.xml",
        "joints": [
            {"name": "joint1", "kind": "revolute", "limits_rad": [-1.0, 1.0]},
            {"name": "joint2", "kind": "revolute", "limits_rad": [-0.5, 0.5]},
            {"name": "finger_left", "kind": "prismatic", "limits_m": [0.0, 0.02]},
        ],
        "eef_site": "tcp",
        "initial_qpos": {"joint1": 0.0, "joint2": 0.1},
        "notes": "test arm",
    }
    values.update(overrides)
    lines = ["name: piper", "source: mjcf", "model: piper_mujoco/scene.xml", "joints:"]
    for joint in values["joints"]:  # type: ignore[index]
        lines.append(f"  - name: {joint['name']}")
        lines.append(f"    kind: {joint['kind']}")
        if "limits_rad" in joint:
            lines.append(f"    limits_rad: [{joint['limits_rad'][0]}, {joint['limits_rad'][1]}]")
        if "limits_m" in joint:
            lines.append(f"    limits_m: [{joint['limits_m'][0]}, {joint['limits_m'][1]}]")
    del values["joints"]
    for key, value in values.items():
        if key == "notes":
            lines.append(f"{key}: {value!r}")
        else:
            lines.append(f"{key}: {value}")
    return "\n".join(lines) + "\n"


def test_parse_full_descriptor(tmp_path: Path) -> None:
    file_path = tmp_path / "robot.yaml"
    file_path.write_text(_robot_yaml(), encoding="utf-8")

    descriptor = load_robot_descriptor(file_path)

    assert descriptor.name == "piper"
    assert descriptor.source == "mjcf"
    assert descriptor.model == "piper_mujoco/scene.xml"
    assert descriptor.dof == 3
    assert [joint.name for joint in descriptor.joints] == ["joint1", "joint2", "finger_left"]
    assert descriptor.joints[0].unit == "radians"
    assert descriptor.joints[2].unit == "meters"
    assert descriptor.eef_site == "tcp"
    assert descriptor.initial_qpos == {"joint1": 0.0, "joint2": 0.1}
    assert descriptor.has_gripper is True  # inferred from the prismatic finger
    assert descriptor.notes == "test arm"


def test_info_json_roundtrip(tmp_path: Path) -> None:
    file_path = tmp_path / "robot.yaml"
    file_path.write_text(_robot_yaml(has_gripper=False), encoding="utf-8")

    info = load_robot_descriptor(file_path).to_info_json()

    assert info == {
        "name": "piper",
        "source": "mjcf",
        "model": "piper_mujoco/scene.xml",
        "dof": 3,
        "joints": [
            {"name": "joint1", "kind": "revolute", "unit": "radians", "limits": [-1.0, 1.0]},
            {"name": "joint2", "kind": "revolute", "unit": "radians", "limits": [-0.5, 0.5]},
            {"name": "finger_left", "kind": "prismatic", "unit": "meters", "limits": [0.0, 0.02]},
        ],
        "eef_site": "tcp",
        "initial_qpos": {"joint1": 0.0, "joint2": 0.1},
        "has_gripper": False,
        "notes": "test arm",
    }


@pytest.mark.parametrize(
    "bad_yaml",
    [
        # unknown top-level key
        _robot_yaml() + "extra_key: 1\n",
        # bad source
        _robot_yaml().replace("source: mjcf", "source: onshape"),
        # empty joints
        textwrap.dedent(
            """
            name: piper
            source: mjcf
            model: scene.xml
            joints: []
            eef_site: tcp
            """
        ),
        # wrong limits key for kind
        textwrap.dedent(
            """
            name: piper
            source: mjcf
            model: scene.xml
            joints:
              - name: joint1
                kind: revolute
                limits_m: [0.0, 1.0]
            eef_site: tcp
            """
        ),
        # inverted limits
        textwrap.dedent(
            """
            name: piper
            source: mjcf
            model: scene.xml
            joints:
              - name: joint1
                kind: revolute
                limits_rad: [1.0, -1.0]
            eef_site: tcp
            """
        ),
        # non-finite limit
        textwrap.dedent(
            """
            name: piper
            source: mjcf
            model: scene.xml
            joints:
              - name: joint1
                kind: revolute
                limits_rad: [-.inf, 1.0]
            eef_site: tcp
            """
        ),
        # duplicate joint names
        textwrap.dedent(
            """
            name: piper
            source: mjcf
            model: scene.xml
            joints:
              - name: joint1
                kind: revolute
                limits_rad: [-1.0, 1.0]
              - name: joint1
                kind: revolute
                limits_rad: [-1.0, 1.0]
            eef_site: tcp
            """
        ),
        # initial_qpos references unknown joint
        textwrap.dedent(
            """
            name: piper
            source: mjcf
            model: scene.xml
            joints:
              - name: joint1
                kind: revolute
                limits_rad: [-1.0, 1.0]
            eef_site: tcp
            initial_qpos:
              ghost_joint: 0.5
            """
        ),
        # non-bool has_gripper
        textwrap.dedent(
            """
            name: piper
            source: mjcf
            model: scene.xml
            joints:
              - name: joint1
                kind: revolute
                limits_rad: [-1.0, 1.0]
            eef_site: tcp
            has_gripper: "yes"
            """
        ),
    ],
)
def test_invalid_descriptors_are_rejected(tmp_path: Path, bad_yaml: str) -> None:
    file_path = tmp_path / "robot.yaml"
    file_path.write_text(bad_yaml, encoding="utf-8")
    with pytest.raises(ValueError):
        load_robot_descriptor(file_path)


def test_missing_file_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        load_robot_descriptor(tmp_path / "absent.yaml")


def test_joint_descriptor_unit_mapping() -> None:
    assert JointDescriptor("j", "revolute", (0.0, 1.0)).unit == "radians"
    assert JointDescriptor("f", "prismatic", (0.0, 0.01)).unit == "meters"


def test_from_dict_rejects_non_mapping() -> None:
    with pytest.raises(ValueError):
        RobotDescriptor.from_dict(["not", "a", "mapping"])  # type: ignore[arg-type]
