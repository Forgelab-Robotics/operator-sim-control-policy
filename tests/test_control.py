from __future__ import annotations

import asyncio

import pytest

from forge_tool import (
    ToolContext,
    ToolEndpointError,
    ToolError,
    ToolEvent,
    ToolExecutionKey,
    ToolRequest,
    ToolResult,
)
from sim_control_policy_node.config import InnerToolSpec, ResetConfig
from sim_control_policy_node.control import InnerHandle, InnerInvoker, SimResetEndpoint
from sim_control_policy_node.info import RESET_OPERATION
from sim_control_policy_node.robot_descriptor import JointDescriptor, RobotDescriptor


def _context(
    key: ToolExecutionKey | None = None,
    *,
    deadline_ms: int | None = None,
) -> ToolContext:
    return ToolContext(
        execution_key=key or ToolExecutionKey("invoke-1", "attempt-1"),
        tool_id="sim.reset",
        implementation_id="sim-control-policy",
        endpoint_id="sim.control",
        operation=RESET_OPERATION,
        deadline_ms=deadline_ms,
    )


class _Events:
    def __init__(self) -> None:
        self.items: list[ToolEvent] = []

    async def emit(self, event: ToolEvent) -> None:
        self.items.append(event)


class _FakeInvoker(InnerInvoker):
    def __init__(self) -> None:
        self.invoked: list[InnerToolSpec] = []
        self.lookups: list[str] = []
        self.cancels: list[str] = []
        self._counter = 0

    def invoke(self, spec: InnerToolSpec, *, deadline_ms: int) -> InnerHandle:
        self._counter += 1
        self.invoked.append(spec)
        return InnerHandle(
            invocation_id=f"inner-{self._counter}",
            attempt_id="1",
            request_id=f"req-{self._counter}",
        )

    def lookup_result(self, handle: InnerHandle) -> None:
        self.lookups.append(handle.invocation_id)

    def cancel(self, handle: InnerHandle) -> None:
        self.cancels.append(handle.invocation_id)


_DESCRIPTOR = RobotDescriptor(
    name="testbot",
    source="mjcf",
    model="scene.xml",
    joints=(
        JointDescriptor("joint1", "revolute", (-1.0, 1.0)),
        JointDescriptor("joint2", "revolute", (-1.0, 1.0)),
        JointDescriptor("gripper", "prismatic", (0.0, 0.1)),
    ),
    eef_site="tcp",
    initial_qpos={"joint1": 0.25, "gripper": 0.05},
    has_gripper=True,
    notes="",
)

_REHOME_CONFIG = ResetConfig(
    motion=InnerToolSpec(
        tool_id="sim.set_joint_positions",
        implementation_id="motion.integration",
        endpoint_id="motion.server",
        operation="move_joints",
        arguments={
            "group_name": "test_arm",
            "velocity_scale": 0.3,
            "acceleration_scale": 0.3,
        },
    ),
    gripper=InnerToolSpec(
        tool_id="sim.set_gripper_opening",
        implementation_id="piper.gripper",
        endpoint_id="gripper.controller",
        operation="set_opening",
        arguments={},
    ),
    timeout_ms=10_000,
    poll_interval_ms=1_000,
    settle_ms=0,
)


def _rehome_endpoint(
    sent: list[str], *, epoch_ms: int = 1_000
) -> tuple[SimResetEndpoint, _FakeInvoker, list[int]]:
    now = [epoch_ms]
    invoker = _FakeInvoker()
    endpoint = SimResetEndpoint(
        sent.append,
        invoker=invoker,
        reset_config=_REHOME_CONFIG,
        descriptor=_DESCRIPTOR,
        epoch_ms=lambda: now[0],
    )
    return endpoint, invoker, now


def _start_rehome(endpoint: SimResetEndpoint, events: _Events | None = None) -> None:
    asyncio.run(endpoint.start(ToolRequest({}), _context(), events or _Events()))


def _admit_all(invoker: _FakeInvoker, endpoint: SimResetEndpoint) -> None:
    for index in range(len(invoker.invoked)):
        endpoint.on_invoke_response(f"req-{index + 1}", "accepted", None)


def test_start_sends_reset_and_completes_immediately() -> None:
    sent: list[str] = []
    events = _Events()
    endpoint = SimResetEndpoint(sent.append)

    accepted = asyncio.run(endpoint.start(ToolRequest({}), _context(), events))

    assert sent == ["reset_scene"]
    assert accepted.details == {"command": "reset_scene"}
    assert [(event.type, dict(event.data)) for event in events.items] == [
        ("executor_completed", {"command": "reset_scene"})
    ]
    assert endpoint.recorded_executions() == 1
    assert endpoint.last_output() == {"command": "reset_scene", "episode_reset": True}

    key = ToolExecutionKey("invoke-1", "attempt-1")
    assert asyncio.run(endpoint.status(key)).phase == "completed"
    response = asyncio.run(endpoint.result(key))
    assert response.status == "available"
    assert response.result is not None
    assert response.result.status == "succeeded"
    assert response.result.outputs == {"command": "reset_scene", "episode_reset": True}


def test_start_is_idempotent_for_same_key() -> None:
    sent: list[str] = []
    endpoint = SimResetEndpoint(sent.append)
    key = ToolExecutionKey("invoke-1", "attempt-1")

    first = asyncio.run(endpoint.start(ToolRequest({}), _context(key), _Events()))
    second = asyncio.run(endpoint.start(ToolRequest({}), _context(key), _Events()))

    assert first.details == {"command": "reset_scene"}
    assert second.details["idempotent"] is True
    assert second.details["phase"] == "terminal"
    assert sent == ["reset_scene"]  # reset forwarded exactly once


def test_start_rejects_arguments_and_wrong_operation() -> None:
    endpoint = SimResetEndpoint(lambda command: None)
    with pytest.raises(ToolEndpointError):
        asyncio.run(endpoint.start(ToolRequest({"force": True}), _context(), _Events()))

    bad_operation = ToolContext(
        execution_key=ToolExecutionKey("invoke-2", "attempt-1"),
        tool_id="sim.reset",
        implementation_id="sim-control-policy",
        endpoint_id="sim.control",
        operation="get_robot_info",
    )
    with pytest.raises(ToolEndpointError):
        asyncio.run(endpoint.start(ToolRequest({}), bad_operation, _Events()))


def test_start_rejects_expired_deadline_and_shutdown() -> None:
    endpoint = SimResetEndpoint(lambda command: None, epoch_ms=lambda: 5_000)
    with pytest.raises(ToolEndpointError):
        asyncio.run(endpoint.start(ToolRequest({}), _context(deadline_ms=4_000), _Events()))

    endpoint.begin_shutdown()
    with pytest.raises(ToolEndpointError):
        asyncio.run(endpoint.start(ToolRequest({}), _context(), _Events()))


def test_cancel_and_unknown_key_handling() -> None:
    endpoint = SimResetEndpoint(lambda command: None)
    asyncio.run(endpoint.start(ToolRequest({}), _context(), _Events()))

    response = asyncio.run(
        endpoint.cancel(ToolExecutionKey("invoke-1", "attempt-1"))
    )
    assert (response.command, response.status) == ("cancel", "terminal")

    rejected = asyncio.run(
        endpoint.cancel(ToolExecutionKey("ghost", "attempt-1"))
    )
    assert rejected.status == "rejected"

    assert asyncio.run(endpoint.status(ToolExecutionKey("ghost", "a"))).phase == "unknown"
    assert asyncio.run(endpoint.result(ToolExecutionKey("ghost", "a"))).status == "not_found"


def test_result_cache_eviction_keeps_recent_executions() -> None:
    endpoint = SimResetEndpoint(lambda command: None, result_cache_size=2)
    for index in range(3):
        asyncio.run(
            endpoint.start(
                ToolRequest({}),
                _context(ToolExecutionKey(f"invoke-{index}", "attempt-1")),
                _Events(),
            )
        )

    assert endpoint.recorded_executions() == 2
    assert (
        asyncio.run(endpoint.result(ToolExecutionKey("invoke-0", "attempt-1"))).status
        == "not_found"
    )
    assert (
        asyncio.run(endpoint.result(ToolExecutionKey("invoke-2", "attempt-1"))).status
        == "available"
    )


def test_result_cache_size_must_be_positive() -> None:
    with pytest.raises(ValueError):
        SimResetEndpoint(lambda command: None, result_cache_size=0)


# -- re-home (reset_scene + inner Tool calls) --------------------------------


def test_rehome_dispatches_inner_calls_with_initial_qpos() -> None:
    sent: list[str] = []
    endpoint, invoker, _now = _rehome_endpoint(sent)
    events = _Events()

    accepted = asyncio.run(endpoint.start(ToolRequest({}), _context(), events))

    assert sent == ["reset_scene"]
    assert accepted.details == {"command": "reset_scene", "rehome": "running"}
    assert events.items == []  # no terminal event until the re-home finishes

    motion, gripper = invoker.invoked
    assert motion.operation == "move_joints"
    assert motion.arguments["joint_names"] == ["joint1", "joint2"]  # gripper excluded
    assert motion.arguments["positions"] == [0.25, 0.0]  # joint2 defaults to 0.0
    assert motion.arguments["group_name"] == "test_arm"
    assert gripper.operation == "set_opening"
    assert gripper.arguments["opening_m"] == 0.05

    key = ToolExecutionKey("invoke-1", "attempt-1")
    assert asyncio.run(endpoint.status(key)).phase == "running"
    assert asyncio.run(endpoint.result(key)).status == "pending"


def test_rehome_polls_only_admitted_calls() -> None:
    endpoint, invoker, now = _rehome_endpoint([])
    _start_rehome(endpoint)

    now[0] += 2_000  # past poll_interval, but nothing admitted yet
    endpoint.on_tick()
    assert invoker.lookups == []

    _admit_all(invoker, endpoint)
    endpoint.on_tick()
    assert sorted(invoker.lookups) == ["inner-1", "inner-2"]


def test_rehome_completes_when_all_inner_calls_succeed() -> None:
    endpoint, invoker, _now = _rehome_endpoint([])
    _start_rehome(endpoint)
    _admit_all(invoker, endpoint)

    endpoint.on_result_response("inner-1", ToolResult(status="succeeded"), None)
    # gripper still outstanding, so the reset keeps running
    assert (
        asyncio.run(endpoint.status(ToolExecutionKey("invoke-1", "attempt-1"))).phase
        == "running"
    )

    endpoint.on_result_response("inner-2", ToolResult(status="succeeded"), None)
    key = ToolExecutionKey("invoke-1", "attempt-1")
    status = asyncio.run(endpoint.status(key))
    assert status.phase == "completed"
    response = asyncio.run(endpoint.result(key))
    assert response.status == "available"
    assert response.result is not None
    assert response.result.status == "succeeded"
    assert response.result.outputs == {
        "command": "reset_scene",
        "episode_reset": True,
        "rehomed": True,
        "gripper_status": "succeeded",
        "motion_status": "succeeded",
    }


def test_rehome_failed_inner_call_fails_reset_with_retryable_error() -> None:
    endpoint, invoker, _now = _rehome_endpoint([])
    _start_rehome(endpoint)
    _admit_all(invoker, endpoint)

    endpoint.on_result_response(
        "inner-1",
        None,
        ToolError(code="FINAL_POSE_TOLERANCE_VIOLATION", message="off by 20mm"),
    )
    endpoint.on_result_response("inner-2", ToolResult(status="succeeded"), None)

    key = ToolExecutionKey("invoke-1", "attempt-1")
    status = asyncio.run(endpoint.status(key))
    assert status.phase == "failed"
    assert status.error is not None
    response = asyncio.run(endpoint.result(key))
    assert response.result is not None
    assert response.result.status == "failed"
    assert response.result.error is not None
    assert response.result.error.retryable is True
    assert "re-home failed" in response.result.error.message
    assert response.result.outputs["rehomed"] is False
    assert response.result.outputs["motion_status"] == "failed"


def test_rehome_rejected_invoke_fails_only_that_stage() -> None:
    endpoint, invoker, _now = _rehome_endpoint([])
    _start_rehome(endpoint)

    endpoint.on_invoke_response(
        "req-1", "rejected", ToolError(code="FORGE_CAPACITY", message="busy")
    )
    endpoint.on_invoke_response("req-2", "accepted", None)
    endpoint.on_result_response("inner-2", ToolResult(status="succeeded"), None)

    response = asyncio.run(endpoint.result(ToolExecutionKey("invoke-1", "attempt-1")))
    assert response.result is not None
    assert response.result.status == "failed"
    assert response.result.outputs["motion_status"] == "failed"
    assert response.result.outputs["gripper_status"] == "succeeded"


def test_rehome_times_out() -> None:
    endpoint, _invoker, now = _rehome_endpoint([])
    _start_rehome(endpoint)

    now[0] += 10_001  # past timeout_ms
    endpoint.on_tick()

    key = ToolExecutionKey("invoke-1", "attempt-1")
    status = asyncio.run(endpoint.status(key))
    assert status.phase == "failed"
    response = asyncio.run(endpoint.result(key))
    assert response.result is not None
    assert response.result.error is not None
    assert response.result.error.code == "SIM_RESET_REHOME_TIMEOUT"
    assert response.result.error.retryable is True
    assert response.result.outputs["rehomed"] is False
    assert response.result.outputs["motion_status"] == "timed_out"


def test_rehome_cancel_cancels_inner_calls() -> None:
    endpoint, invoker, _now = _rehome_endpoint([])
    _start_rehome(endpoint)
    _admit_all(invoker, endpoint)

    response = asyncio.run(
        endpoint.cancel(ToolExecutionKey("invoke-1", "attempt-1"), reason="user stop")
    )
    assert (response.command, response.status) == ("cancel", "terminal")
    assert sorted(invoker.cancels) == ["inner-1", "inner-2"]

    key = ToolExecutionKey("invoke-1", "attempt-1")
    status = asyncio.run(endpoint.status(key))
    assert status.phase == "cancelled"
    result_response = asyncio.run(endpoint.result(key))
    assert result_response.result is not None
    assert result_response.result.status == "cancelled"
    assert result_response.result.error is None  # cancelled carries no error
    assert result_response.result.outputs["cancel_reason"] == "user stop"


def test_rehome_completion_event_triggers_lookup() -> None:
    endpoint, invoker, _now = _rehome_endpoint([])
    _start_rehome(endpoint)
    _admit_all(invoker, endpoint)

    endpoint.on_inner_event("inner-1", "executor_completed")
    assert invoker.lookups == ["inner-1"]

    endpoint.on_inner_event("inner-1", "progress")  # non-terminal events ignored
    assert invoker.lookups == ["inner-1"]


def test_rehome_idempotent_while_running() -> None:
    sent: list[str] = []
    endpoint, invoker, _now = _rehome_endpoint(sent)
    _start_rehome(endpoint)

    second = asyncio.run(endpoint.start(ToolRequest({}), _context(), _Events()))

    assert second.details["idempotent"] is True
    assert second.details["phase"] == "running"
    assert sent == ["reset_scene"]
    assert len(invoker.invoked) == 2  # inner calls dispatched exactly once


def test_rehome_settle_ms_defers_dispatch_until_first_tick() -> None:
    sent: list[str] = []
    invoker = _FakeInvoker()
    now = [1_000]
    config = ResetConfig(
        motion=_REHOME_CONFIG.motion,
        gripper=None,
        timeout_ms=10_000,
        poll_interval_ms=1_000,
        settle_ms=500,
    )
    endpoint = SimResetEndpoint(
        sent.append,
        invoker=invoker,
        reset_config=config,
        descriptor=_DESCRIPTOR,
        epoch_ms=lambda: now[0],
    )

    _start_rehome(endpoint)
    assert sent == ["reset_scene"]
    assert invoker.invoked == []  # held back while the reset settles

    now[0] += 499
    endpoint.on_tick()
    assert invoker.invoked == []

    now[0] += 1
    endpoint.on_tick()
    assert len(invoker.invoked) == 1  # only motion: gripper stage is None


def test_rehome_without_arm_joints_is_rejected() -> None:
    finger_only = RobotDescriptor(
        name="claw",
        source="mjcf",
        model="scene.xml",
        joints=(JointDescriptor("gripper", "prismatic", (0.0, 0.1)),),
        eef_site="tcp",
        initial_qpos={},
        has_gripper=True,
        notes="",
    )
    endpoint = SimResetEndpoint(
        lambda command: None,
        invoker=_FakeInvoker(),
        reset_config=_REHOME_CONFIG,
        descriptor=finger_only,
    )
    with pytest.raises(ToolEndpointError):
        asyncio.run(endpoint.start(ToolRequest({}), _context(), _Events()))
