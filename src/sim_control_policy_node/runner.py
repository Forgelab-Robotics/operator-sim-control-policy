"""Dora runner for the sim.control Tool endpoint (info + reset operations).

Besides serving the endpoint's own tool traffic on ``tool_in``/``tool_out``,
the runner doubles as a gateway caller while a reset re-home is in flight:
inner Tool invokes, cancel requests, and result lookups leave on
``tool_request`` and the gateway's replies and streamed events arrive on
``tool_response`` (the gateway's public caller channel, routed by each
envelope's own endpoint/operation identity).
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
import uuid
from collections.abc import Callable, Iterable
from typing import Any, cast

from forge_tool import (
    ToolAccepted,
    ToolContext,
    ToolEndpointHandler,
    ToolEnvelope,
    ToolError,
    ToolExecutionKey,
    ToolRequest,
    error_from_payload,
    event_from_payload,
    invoke_response_from_payload,
    make_control_request_envelope,
    make_invoke_request_envelope,
    make_result_request_envelope,
    result_response_from_payload,
)
from forge_tool._tool_message import ToolMessage
from forge_tool.dora import (
    DoraToolEndpointBinding,
    tool_envelope_to_message,
    tool_message_to_envelope,
)

from .config import InnerToolSpec, SimControlPolicyConfig
from .control import InnerHandle, InnerInvoker, SimResetEndpoint
from .endpoint_lease import EndpointLease
from .info import INFO_OPERATION, RESET_OPERATION, SIM_CONTROL_DESCRIPTOR, SimInfoEndpoint
from .robot_descriptor import RobotDescriptor

logger = logging.getLogger(__name__)

_CALLER_ID = "sim_control_policy"


class GatewayToolInvoker(InnerInvoker):
    """Inner-call transport over the gateway's public ``tool_request`` channel."""

    def __init__(self, node: Any, *, caller_id: str = _CALLER_ID) -> None:
        self._node = node
        self._caller_id = caller_id
        self._contexts: dict[str, ToolContext] = {}
        self._lookup_requests: dict[str, str] = {}

    @staticmethod
    def _new_id(prefix: str) -> str:
        return f"{prefix}-{uuid.uuid4().hex[:12]}"

    def _send(self, envelope: ToolEnvelope) -> None:
        self._node.send_output("tool_request", tool_envelope_to_message(envelope).to_arrow())

    def invoke(self, spec: InnerToolSpec, *, deadline_ms: int) -> InnerHandle:
        invocation_id = self._new_id("sim-reset")
        attempt_id = self._new_id("attempt")
        request_id = self._new_id("req")
        context = ToolContext(
            execution_key=ToolExecutionKey(
                invocation_id=invocation_id,
                attempt_id=attempt_id,
            ),
            tool_id=spec.tool_id,
            implementation_id=spec.implementation_id,
            endpoint_id=spec.endpoint_id,
            operation=spec.operation,
            caller_id=self._caller_id,
            deadline_ms=deadline_ms,
        )
        self._contexts[invocation_id] = context
        self._send(
            make_invoke_request_envelope(
                ToolRequest(arguments=spec.arguments),
                context,
                request_id=request_id,
                endpoint_instance_id=None,
            )
        )
        return InnerHandle(
            invocation_id=invocation_id,
            attempt_id=attempt_id,
            request_id=request_id,
        )

    def lookup_result(self, handle: InnerHandle) -> None:
        context = self._contexts.get(handle.invocation_id)
        if context is None:
            return
        request_id = self._new_id("lookup")
        self._lookup_requests[request_id] = handle.invocation_id
        self._send(
            make_result_request_envelope(
                context,
                request_id=request_id,
                endpoint_instance_id=None,
            )
        )

    def cancel(self, handle: InnerHandle) -> None:
        context = self._contexts.pop(handle.invocation_id, None)
        if context is None:
            return
        self._send(
            make_control_request_envelope(
                "cancel",
                context,
                request_id=self._new_id("cancel"),
                endpoint_instance_id=None,
                reason="sim reset cancelled",
            )
        )

    def resolve_lookup(self, request_id: str | None) -> str | None:
        """Map a result-lookup response back to its inner invocation id."""
        if request_id is None:
            return None
        return self._lookup_requests.pop(request_id, None)

    def forget(self, invocation_id: str) -> None:
        """Drop a finished inner call's context so lookups stop for it."""
        self._contexts.pop(invocation_id, None)


class SimControlPolicyRunner:
    def __init__(
        self,
        node: Any,
        config: SimControlPolicyConfig,
        descriptor: RobotDescriptor | None = None,
        *,
        endpoint_instance_id: str | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.node = node
        if descriptor is None:
            from .robot_descriptor import load_robot_descriptor

            descriptor = load_robot_descriptor(config.robot_descriptor_path)
        self.descriptor = descriptor
        self.endpoint_instance_id = endpoint_instance_id or str(uuid.uuid4())
        self.clock = clock
        self._async = asyncio.Runner()
        self._invoker = GatewayToolInvoker(node) if config.reset is not None else None
        self.reset_endpoint = SimResetEndpoint(
            self._send_reset,
            invoker=self._invoker,
            reset_config=config.reset,
            descriptor=descriptor,
        )
        self.handler = ToolEndpointHandler(
            SIM_CONTROL_DESCRIPTOR,
            endpoint_instance_id=self.endpoint_instance_id,
            operations={
                INFO_OPERATION: SimInfoEndpoint(descriptor),
                RESET_OPERATION: self.reset_endpoint,
            },
        )
        self.binding = DoraToolEndpointBinding(self.handler, event_sink=self._send_batch)
        self.lease = EndpointLease(SIM_CONTROL_DESCRIPTOR, self.endpoint_instance_id)

    def _send_reset(self, command: str) -> None:
        # mujoco_sim parses sim_command as raw UTF-8 JSON bytes
        # ({"command": "reset_scene"} resets to the configured initial_state).
        payload = json.dumps({"command": command}).encode("utf-8")
        self.node.send_output("sim_command", payload)

    async def _send_batch(self, batch: object) -> None:
        self.node.send_output("tool_out", batch)

    def _send_envelope(self, envelope: ToolEnvelope) -> None:
        self.node.send_output("tool_out", tool_envelope_to_message(envelope).to_arrow())

    def _drive_lease(self) -> None:
        request = self.lease.poll(self.clock())
        if request is not None:
            self._send_envelope(request)

    def _handle_tool(self, value: object) -> None:
        envelope = tool_message_to_envelope(ToolMessage.from_arrow(value))
        if envelope.message_type == "endpoint.registry.response":
            self.lease.acknowledge(envelope, self.clock())
            return
        for batch in self._async.run(self.binding.dispatch_input(value)):
            self.node.send_output("tool_out", batch)

    def _handle_tool_response(self, value: object) -> None:
        """Decode one gateway caller-channel reply and feed the reset endpoint."""
        envelope = tool_message_to_envelope(ToolMessage.from_arrow(value))
        message_type = envelope.message_type
        if message_type == "tool.invoke.response":
            response = invoke_response_from_payload(envelope.payload)
            if isinstance(response, ToolAccepted):
                self.reset_endpoint.on_invoke_response(
                    cast(str, envelope.request_id), "accepted", None
                )
            elif isinstance(response, ToolError):
                self.reset_endpoint.on_invoke_response(
                    cast(str, envelope.request_id), "rejected", response
                )
            else:  # an inner call completing inline; treat as its own result
                self.reset_endpoint.on_result_response(
                    cast(str, envelope.invocation_id), response, None
                )
                if self._invoker is not None:
                    self._invoker.forget(cast(str, envelope.invocation_id))
        elif message_type == "tool.event":
            event = event_from_payload(envelope.payload)
            self.reset_endpoint.on_inner_event(
                cast(str, envelope.invocation_id), event.type
            )
        elif message_type == "tool.result.response":
            response = result_response_from_payload(envelope.payload)
            invocation_id = (
                self._invoker.resolve_lookup(envelope.request_id)
                if self._invoker is not None
                else None
            )
            if invocation_id is None:
                return
            if response.status == "available":
                self.reset_endpoint.on_result_response(invocation_id, response.result, None)
                self._invoker.forget(invocation_id)
            # "pending" just means the inner action is still running; the
            # poll loop will look it up again.
        elif message_type == "tool.error":
            error = error_from_payload(envelope.payload)
            request_id = envelope.request_id
            invocation_id = (
                self._invoker.resolve_lookup(request_id) if self._invoker is not None else None
            )
            if invocation_id is not None:
                self.reset_endpoint.on_result_response(invocation_id, None, error)
                self._invoker.forget(invocation_id)
            else:
                # No lookup outstanding with this id: a rejected inner invoke.
                self.reset_endpoint.on_invoke_response(
                    cast(str, request_id), "rejected", error
                )
        else:
            logger.debug("ignoring gateway caller message %r", message_type)

    def run(self, events: Iterable[dict[str, Any]]) -> int:
        self._drive_lease()
        for event in events:
            result = self.process_event(event)
            if result is not None:
                return result
        return 0

    def process_event(self, event: dict[str, Any]) -> int | None:
        event_type = event.get("type")
        if event_type == "STOP":
            self.reset_endpoint.begin_shutdown()
            unregister = self.lease.stop()
            if unregister is not None:
                self._send_envelope(unregister)
            self._async.close()
            return 0
        if event_type == "ERROR":
            logger.error("Dora node error: %s", event.get("error", "unknown"))
            self._async.close()
            return 1
        if event_type != "INPUT":
            return None

        self._drive_lease()
        input_id = event.get("id")
        try:
            if input_id == "tool_in":
                self._handle_tool(event["value"])
            elif input_id == "tool_response":
                self._handle_tool_response(event["value"])
            elif input_id == "tick":
                self.reset_endpoint.on_tick()
        except (TypeError, ValueError) as exc:
            logger.warning("ignored invalid sim control input: %s", exc)
        self._drive_lease()
        return None


def run_sim_control_policy(config: SimControlPolicyConfig) -> int:
    from dora import Node

    node = Node()
    return SimControlPolicyRunner(node, config).run(node)


__all__ = ["GatewayToolInvoker", "SimControlPolicyRunner", "run_sim_control_policy"]
