from __future__ import annotations

import json
from pathlib import Path

from forge_tool import (
    EndpointRegistryResponse,
    ToolAccepted,
    ToolContext,
    ToolError,
    ToolEvent,
    ToolExecutionKey,
    ToolRequest,
    ToolResult,
    ToolResultResponse,
    invoke_response_from_payload,
    make_endpoint_registry_response_envelope,
    make_error_response_envelope,
    make_event_envelope,
    make_invoke_request_envelope,
    make_invoke_response_envelope,
    make_result_response_envelope,
    make_status_request_envelope,
    status_response_from_payload,
    validate_registration_envelope,
)
from forge_tool._tool_message import ToolMessage
from forge_tool.dora import (
    tool_envelope_to_message,
    tool_message_to_envelope,
)
from sim_control_policy_node.config import InnerToolSpec, ResetConfig, SimControlPolicyConfig
from sim_control_policy_node.info import INFO_OPERATION, RESET_OPERATION, SIM_CONTROL_DESCRIPTOR
from sim_control_policy_node.robot_descriptor import JointDescriptor, RobotDescriptor
from sim_control_policy_node.runner import SimControlPolicyRunner


def result_of(envelope):
    outcome = invoke_response_from_payload(envelope.payload)
    assert outcome.status == "succeeded", outcome
    return outcome


def _descriptor() -> RobotDescriptor:
    return RobotDescriptor(
        name="piper",
        source="mjcf",
        model="piper_mujoco/scene.xml",
        joints=(
            JointDescriptor("joint1", "revolute", (-1.0, 1.0)),
            JointDescriptor("gripper", "prismatic", (0.0, 0.02)),
        ),
        eef_site="tcp",
        initial_qpos={"joint1": 0.0},
        has_gripper=True,
        notes="test arm",
    )


def _context(operation: str) -> ToolContext:
    return ToolContext(
        execution_key=ToolExecutionKey("invoke-1", "attempt-1"),
        tool_id="sim.get_robot_info",
        implementation_id="sim-control-policy",
        endpoint_id="sim.control",
        operation=operation,
    )


def _arrow(envelope):
    return tool_envelope_to_message(envelope).to_arrow()


def _envelope(batch):
    return tool_message_to_envelope(ToolMessage.from_arrow(batch))


class _Node:
    def __init__(self) -> None:
        self.outputs: list[tuple[str, object]] = []

    def send_output(self, output_id, value, **kwargs) -> None:
        self.outputs.append((output_id, value))

    def tool_out_envelopes(self):
        return [_envelope(value) for output_id, value in self.outputs if output_id == "tool_out"]

    def tool_request_envelopes(self):
        return [
            _envelope(value)
            for output_id, value in self.outputs
            if output_id == "tool_request"
        ]

    def sim_commands(self) -> list[dict]:
        return [
            json.loads(bytes(value).decode("utf-8"))
            for output_id, value in self.outputs
            if output_id == "sim_command"
        ]


def _runner(node: _Node) -> SimControlPolicyRunner:
    return SimControlPolicyRunner(
        node,
        SimControlPolicyConfig(robot_descriptor_path=Path("/absent.yaml")),
        descriptor=_descriptor(),
        endpoint_instance_id="instance-1",
        clock=lambda: 10.0,
    )


def test_runner_registers_and_unregisters() -> None:
    node = _Node()
    runner = _runner(node)

    runner._drive_lease()
    registration = node.tool_out_envelopes()[-1]
    assert registration.message_type == "endpoint.register"
    assert validate_registration_envelope(registration) == SIM_CONTROL_DESCRIPTOR

    response = make_endpoint_registry_response_envelope(
        EndpointRegistryResponse(
            operation="register",
            status="accepted",
            registry_revision=1,
            lease_ttl_ms=3000,
        ),
        registration,
    )
    runner._handle_tool(_arrow(response))
    assert runner.process_event({"type": "STOP"}) == 0
    assert node.tool_out_envelopes()[-1].message_type == "endpoint.unregister"


def _invoke_response(node: _Node):
    envelopes = [
        envelope
        for envelope in node.tool_out_envelopes()
        if envelope.message_type == "tool.invoke.response"
    ]
    assert envelopes, "no tool.invoke.response was sent"
    return envelopes[-1]


def test_info_query_flows_through_binding() -> None:
    node = _Node()
    runner = _runner(node)
    invoke = make_invoke_request_envelope(
        ToolRequest({}),
        _context(INFO_OPERATION),
        request_id="invoke-1",
        endpoint_instance_id="instance-1",
    )

    runner._handle_tool(_arrow(invoke))

    response = _invoke_response(node)
    assert result_of(response).status == "succeeded"
    assert result_of(response).outputs["name"] == "piper"
    assert result_of(response).outputs["dof"] == 2


def test_reset_action_sends_sim_command_and_completes() -> None:
    node = _Node()
    runner = _runner(node)
    invoke = make_invoke_request_envelope(
        ToolRequest({}),
        _context(RESET_OPERATION),
        request_id="invoke-1",
        endpoint_instance_id="instance-1",
    )

    runner._handle_tool(_arrow(invoke))

    assert node.sim_commands() == [{"command": "reset_scene"}]
    response = _invoke_response(node)
    result = result_of(response)
    assert result.outputs == {"command": "reset_scene", "episode_reset": True}
    # The terminal event emitted inside start arrives as an early event envelope.
    assert any(
        envelope.message_type == "tool.event"
        for envelope in node.tool_out_envelopes()
    )


def test_tick_and_unknown_events_drive_lease_without_side_effects() -> None:
    node = _Node()
    runner = _runner(node)

    assert runner.process_event({"type": "INPUT", "id": "tick", "value": None}) is None
    assert runner.process_event({"type": "INPUT", "id": "unknown", "value": None}) is None
    assert runner.process_event({"type": "OTHER"}) is None
    assert node.sim_commands() == []


def test_error_event_returns_nonzero() -> None:
    node = _Node()
    runner = _runner(node)

    assert runner.process_event({"type": "ERROR", "error": "boom"}) == 1


# -- gateway caller channel (reset re-home) ---------------------------------


def _rehome_runner(node: _Node) -> SimControlPolicyRunner:
    config = SimControlPolicyConfig(
        robot_descriptor_path=Path("/absent.yaml"),
        reset=ResetConfig(
            motion=InnerToolSpec(
                tool_id="sim.set_joint_positions",
                implementation_id="motion.integration",
                endpoint_id="motion.server",
                operation="move_joints",
                arguments={"group_name": "piper_arm", "velocity_scale": 0.3},
            ),
            gripper=None,
            timeout_ms=10_000,
            poll_interval_ms=1_000,
            settle_ms=0,
        ),
    )
    return SimControlPolicyRunner(
        node,
        config,
        descriptor=_descriptor(),
        endpoint_instance_id="instance-1",
        clock=lambda: 10.0,
    )


def _outer_reset_invoke():
    return make_invoke_request_envelope(
        ToolRequest({}),
        ToolContext(
            execution_key=ToolExecutionKey("invoke-1", "attempt-1"),
            tool_id="sim.reset",
            implementation_id="sim-control-policy",
            endpoint_id="sim.control",
            operation=RESET_OPERATION,
        ),
        request_id="invoke-1",
        endpoint_instance_id="instance-1",
    )


def _outer_status(
    node: _Node, runner: SimControlPolicyRunner, request_id: str = "status-1"
) -> str:
    """Phase of the outer reset as the gateway would observe it."""
    runner._handle_tool(
        _arrow(
            make_status_request_envelope(
                ToolContext(
                    execution_key=ToolExecutionKey("invoke-1", "attempt-1"),
                    tool_id="sim.reset",
                    implementation_id="sim-control-policy",
                    endpoint_id="sim.control",
                    operation=RESET_OPERATION,
                ),
                request_id=request_id,
                endpoint_instance_id="instance-1",
            )
        )
    )
    responses = [
        envelope
        for envelope in node.tool_out_envelopes()
        if envelope.message_type == "tool.status.response"
    ]
    assert responses, "no tool.status.response was sent"
    return status_response_from_payload(responses[-1].payload).phase


def test_reset_rehome_flows_through_gateway_caller_channel() -> None:
    node = _Node()
    runner = _rehome_runner(node)

    runner._handle_tool(_arrow(_outer_reset_invoke()))

    # The outer invoke is only accepted; reset_scene went to the simulator.
    assert node.sim_commands() == [{"command": "reset_scene"}]
    assert _invoke_response(node).payload["outcome"] == "accepted"

    # The inner move_joints invoke left on the gateway caller channel.
    inner_requests = node.tool_request_envelopes()
    assert [envelope.message_type for envelope in inner_requests] == ["tool.invoke.request"]
    inner = inner_requests[0]
    assert (inner.endpoint_id, inner.operation) == ("motion.server", "move_joints")
    arguments = inner.payload["arguments"]
    assert arguments["joint_names"] == ["joint1"]  # gripper joint excluded
    assert arguments["positions"] == [0.0]
    assert arguments["group_name"] == "piper_arm"

    assert _outer_status(node, runner) == "running"

    # Gateway accepts the inner invoke, then streams its completion event.
    runner._handle_tool_response(
        _arrow(make_invoke_response_envelope(ToolAccepted(details={}), inner))
    )
    runner._handle_tool_response(
        _arrow(
            make_event_envelope(
                ToolEvent(type="executor_completed", data={}),
                ToolContext(
                    execution_key=ToolExecutionKey(
                        inner.invocation_id, inner.attempt_id
                    ),
                    tool_id="sim.set_joint_positions",
                    implementation_id="motion.integration",
                    endpoint_id="motion.server",
                    operation="move_joints",
                ),
                endpoint_instance_id="motion-instance",
                sequence=1,
            )
        )
    )

    # The event triggered a result lookup on the caller channel.
    lookups = [
        envelope
        for envelope in node.tool_request_envelopes()
        if envelope.message_type == "tool.result.request"
    ]
    assert len(lookups) == 1
    runner._handle_tool_response(
        _arrow(
            make_result_response_envelope(
                ToolResultResponse(
                    status="available",
                    result=ToolResult(status="succeeded"),
                ),
                lookups[0],
            )
        )
    )

    assert _outer_status(node, runner, request_id="status-2") == "completed"


def test_pending_inner_result_keeps_reset_running() -> None:
    node = _Node()
    runner = _rehome_runner(node)
    runner._handle_tool(_arrow(_outer_reset_invoke()))

    inner = node.tool_request_envelopes()[0]
    runner._handle_tool_response(
        _arrow(make_invoke_response_envelope(ToolAccepted(details={}), inner))
    )
    # Advance past poll_interval_ms so the tick-driven poll fires.
    start_ms = runner.reset_endpoint._epoch_ms()
    runner.reset_endpoint._epoch_ms = lambda: start_ms + 2_000
    runner.process_event({"type": "INPUT", "id": "tick", "value": None})

    lookups = [
        envelope
        for envelope in node.tool_request_envelopes()
        if envelope.message_type == "tool.result.request"
    ]
    assert len(lookups) == 1

    # A pending lookup response must not fail the reset.
    runner._handle_tool_response(
        _arrow(
            make_result_response_envelope(
                ToolResultResponse(status="pending"), lookups[0]
            )
        )
    )
    assert _outer_status(node, runner) == "running"


def test_inner_invoke_error_fails_the_reset() -> None:
    node = _Node()
    runner = _rehome_runner(node)
    runner._handle_tool(_arrow(_outer_reset_invoke()))

    inner = node.tool_request_envelopes()[0]
    runner._handle_tool_response(
        _arrow(
            make_error_response_envelope(
                ToolError(code="FORGE_TOOL_UNKNOWN", message="no such tool"),
                inner,
            )
        )
    )
    assert _outer_status(node, runner) == "failed"
