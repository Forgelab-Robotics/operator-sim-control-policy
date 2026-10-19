"""Forge Action endpoint: reset the simulated scene (with optional re-home).

``reset`` forwards mujoco_sim's existing ``reset_scene`` command over the
``sim_command`` topic. ``reset_scene`` alone is not enough: the joint
trajectory controllers keep streaming their hold positions, so the arm is
dragged straight back to its pre-reset pose. When the profile configures a
re-home, the endpoint additionally dispatches the descriptor's
``initial_qpos`` through the standard Tool path (``sim.set_joint_positions``
plus, for parallel grippers, ``sim.set_gripper_opening``) and completes the
reset only when those inner actions reach their terminal results. Without a
re-home config the action still completes inside ``start`` exactly as before.
"""

from __future__ import annotations

import json
import time
import uuid
from collections import OrderedDict
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from forge_tool import (
    ToolAccepted,
    ToolContext,
    ToolControlResponse,
    ToolEndpointError,
    ToolError,
    ToolEvent,
    ToolEventEmitter,
    ToolExecutionKey,
    ToolExecutionStatus,
    ToolRequest,
    ToolResult,
    ToolResultResponse,
)

from .config import InnerToolSpec, ResetConfig
from .info import RESET_OPERATION
from .robot_descriptor import RobotDescriptor

ResetSender = Callable[[str], None]

_RESET_COMMAND = "reset_scene"
_GRIPPER_JOINT = "gripper"


def _error(code: str, message: str, *, retryable: bool = False) -> ToolEndpointError:
    return ToolEndpointError(ToolError(code=code, message=message, retryable=retryable))


@dataclass(frozen=True)
class InnerHandle:
    """Gateway-side identity of one dispatched inner Tool invocation."""

    invocation_id: str
    attempt_id: str
    request_id: str


class InnerInvoker:
    """Runner-facing transport for the inner re-home Tool calls."""

    def invoke(self, spec: InnerToolSpec, *, deadline_ms: int) -> InnerHandle:
        raise NotImplementedError

    def lookup_result(self, handle: InnerHandle) -> None:
        raise NotImplementedError

    def cancel(self, handle: InnerHandle) -> None:
        raise NotImplementedError


@dataclass
class _InnerCall:
    handle: InnerHandle
    admitted: bool = False
    terminal: bool = False
    status: str | None = None
    error: ToolError | None = None
    last_poll_ms: int = 0


@dataclass
class _PendingReset:
    dispatch_at_ms: int = 0
    dispatched: bool = False
    calls: dict[str, _InnerCall] = field(default_factory=dict)
    deadline_ms: int = 0


class _Execution:
    __slots__ = ("arguments_json", "pending", "result")

    def __init__(
        self,
        arguments_json: str,
        pending: _PendingReset | None,
        result: ToolResult | None,
    ) -> None:
        self.arguments_json = arguments_json
        self.pending = pending
        self.result = result


def _rehome_specs(
    config: ResetConfig, descriptor: RobotDescriptor
) -> list[tuple[str, InnerToolSpec]]:
    if config.motion is None:
        return []
    arm_joints = [j for j in descriptor.joints if j.name != _GRIPPER_JOINT]
    if not arm_joints:
        raise _error(
            "SIM_RESET_REHOME_MISCONFIGURED",
            "robot descriptor declares no arm joints to re-home",
        )
    joint_names = [j.name for j in arm_joints]
    positions = [float(descriptor.initial_qpos.get(name, 0.0)) for name in joint_names]
    motion_arguments = {
        **config.motion.arguments,
        "joint_names": joint_names,
        "positions": positions,
    }
    specs: list[tuple[str, InnerToolSpec]] = [
        (
            "motion",
            InnerToolSpec(
                tool_id=config.motion.tool_id,
                implementation_id=config.motion.implementation_id,
                endpoint_id=config.motion.endpoint_id,
                operation=config.motion.operation,
                arguments=motion_arguments,
            ),
        )
    ]
    if config.gripper is not None:
        opening = float(descriptor.initial_qpos.get(_GRIPPER_JOINT, 0.0))
        specs.append(
            (
                "gripper",
                InnerToolSpec(
                    tool_id=config.gripper.tool_id,
                    implementation_id=config.gripper.implementation_id,
                    endpoint_id=config.gripper.endpoint_id,
                    operation=config.gripper.operation,
                    arguments={**config.gripper.arguments, "opening_m": opening},
                ),
            )
        )
    return specs


class SimResetEndpoint:
    """Tracked Action SPI adapter: ``reset_scene`` plus optional re-home."""

    def __init__(
        self,
        send_reset: ResetSender,
        *,
        invoker: InnerInvoker | None = None,
        reset_config: ResetConfig | None = None,
        descriptor: RobotDescriptor | None = None,
        result_cache_size: int = 32,
        epoch_ms: Callable[[], int] = lambda: time.time_ns() // 1_000_000,
        id_factory: Callable[[], str] | None = None,
    ) -> None:
        if result_cache_size < 1:
            raise ValueError("result_cache_size must be positive")
        self._send_reset = send_reset
        self._invoker = invoker
        self._reset_config = reset_config
        self._descriptor = descriptor
        self._result_cache_size = result_cache_size
        self._epoch_ms = epoch_ms
        self._id_factory = id_factory or (lambda: uuid.uuid4().hex)
        self._executions: OrderedDict[ToolExecutionKey, _Execution] = OrderedDict()
        self._by_request_id: dict[str, tuple[ToolExecutionKey, str]] = {}
        self._by_invocation_id: dict[str, tuple[ToolExecutionKey, str]] = {}
        self._accepting = True

    # -- SPI -----------------------------------------------------------------

    async def start(  # noqa: RUF029 - SPI signature is async by protocol.
        self,
        request: ToolRequest,
        context: ToolContext,
        events: ToolEventEmitter,
    ) -> ToolAccepted:
        if context.operation != RESET_OPERATION:
            raise _error(
                "FORGE_PROTOCOL_UNKNOWN_OPERATION",
                f"unknown operation {context.operation!r}",
            )
        if not self._accepting:
            raise _error(
                "FORGE_ENDPOINT_STOPPING",
                "sim control endpoint is shutting down and no longer accepts resets",
                retryable=True,
            )
        if context.deadline_ms is not None and self._epoch_ms() >= context.deadline_ms:
            raise _error("FORGE_DEADLINE_EXCEEDED", "the Tool invocation deadline has expired")
        if request.arguments:
            raise _error("SIM_INVALID_ARGUMENT", "reset takes no arguments")

        key = context.execution_key
        arguments_json = json.dumps(request.arguments, sort_keys=True, separators=(",", ":"))
        existing = self._executions.get(key)
        if existing is not None:
            if existing.arguments_json != arguments_json:
                raise _error(
                    "FORGE_EXECUTION_CONFLICT",
                    "execution key was already used with different action arguments",
                )
            phase = "running" if existing.pending is not None else "terminal"
            return ToolAccepted(
                details={"command": _RESET_COMMAND, "idempotent": True, "phase": phase}
            )

        self._send_reset(_RESET_COMMAND)

        pending = self._start_rehome(key)
        if pending is None:
            result = ToolResult(
                status="succeeded",
                outputs={"command": _RESET_COMMAND, "episode_reset": True},
            )
            self._record(key, _Execution(arguments_json, None, result))
            # Emitting the terminal event inside start lets the handler answer
            # the invoke with the completed result in a single round trip.
            await events.emit(
                ToolEvent(type="executor_completed", data={"command": _RESET_COMMAND})
            )
            return ToolAccepted(details={"command": _RESET_COMMAND})
        self._record(key, _Execution(arguments_json, pending, None))
        return ToolAccepted(details={"command": _RESET_COMMAND, "rehome": "running"})

    async def cancel(  # noqa: RUF029 - SPI signature is async by protocol.
        self,
        key: ToolExecutionKey,
        reason: str | None = None,
    ) -> ToolControlResponse:
        execution = self._executions.get(key)
        if execution is None:
            return ToolControlResponse(
                command="cancel",
                status="rejected",
                error=ToolError(
                    code="FORGE_EXECUTION_NOT_FOUND",
                    message="execution key is unknown",
                ),
            )
        pending = execution.pending
        if pending is not None:
            if self._invoker is not None:
                for call in pending.calls.values():
                    if not call.terminal:
                        self._invoker.cancel(call.handle)
            # ToolResult only carries an error for failed/unknown, so the
            # cancellation reason rides in outputs instead.
            self._complete(
                key,
                execution,
                ToolResult(
                    status="cancelled",
                    outputs={
                        "command": _RESET_COMMAND,
                        "episode_reset": True,
                        "cancel_reason": reason or "no reason",
                    },
                ),
            )
        return ToolControlResponse(command="cancel", status="terminal")

    async def status(  # noqa: RUF029 - SPI signature is async by protocol.
        self, key: ToolExecutionKey
    ) -> ToolExecutionStatus:
        execution = self._executions.get(key)
        if execution is None:
            return ToolExecutionStatus(
                phase="unknown",
                error=ToolError(
                    code="FORGE_EXECUTION_NOT_FOUND",
                    message="execution key is unknown or its result expired",
                ),
            )
        if execution.pending is not None:
            return ToolExecutionStatus(
                phase="running",
                details={"command": _RESET_COMMAND, "rehome": "running"},
            )
        result = execution.result
        assert result is not None
        phase = {"succeeded": "completed", "cancelled": "cancelled", "stopped": "stopped"}.get(
            result.status, "failed"
        )
        return ToolExecutionStatus(
            phase=phase,
            error=result.error if phase == "failed" else None,
            details=dict(result.outputs),
        )

    async def result(  # noqa: RUF029 - SPI signature is async by protocol.
        self, key: ToolExecutionKey
    ) -> ToolResultResponse:
        execution = self._executions.get(key)
        if execution is None:
            return ToolResultResponse(status="not_found")
        if execution.pending is not None:
            return ToolResultResponse(status="pending")
        return ToolResultResponse(status="available", result=execution.result)

    # -- re-home driving -----------------------------------------------------

    def _start_rehome(self, key: ToolExecutionKey) -> _PendingReset | None:
        if self._reset_config is None or self._invoker is None or self._descriptor is None:
            return None
        now_ms = self._epoch_ms()
        pending = _PendingReset(
            dispatch_at_ms=now_ms + self._reset_config.settle_ms,
            deadline_ms=now_ms + self._reset_config.timeout_ms,
        )
        if self._reset_config.settle_ms == 0:
            self._dispatch_rehome(key, pending, now_ms)
        return pending

    def _dispatch_rehome(self, key: ToolExecutionKey, pending: _PendingReset, now_ms: int) -> None:
        """Send the inner re-home invokes.

        They are held back by ``settle_ms`` after ``reset_scene``: the physics
        reset and the controllers' hold positions need a moment to converge, or
        motion_server plans from the pre-reset state and the trajectory dies
        with PATH_TOLERANCE_VIOLATION.
        """
        assert self._reset_config is not None and self._descriptor is not None
        for role, spec in _rehome_specs(self._reset_config, self._descriptor):
            handle = self._invoker.invoke(spec, deadline_ms=pending.deadline_ms)
            call = _InnerCall(handle=handle, last_poll_ms=now_ms)
            pending.calls[role] = call
            self._by_request_id[handle.request_id] = (key, role)
            self._by_invocation_id[handle.invocation_id] = (key, role)
        pending.dispatched = True

    def on_tick(self) -> None:
        """Poll admitted inner calls and expire re-homes past their deadline."""
        now_ms = self._epoch_ms()
        for key, execution in list(self._executions.items()):
            pending = execution.pending
            if pending is None:
                continue
            if now_ms >= pending.deadline_ms:
                timeout_ms = self._reset_config.timeout_ms if self._reset_config else 0
                self._complete(
                    key,
                    execution,
                    ToolResult(
                        status="failed",
                        outputs=self._rehome_outputs(pending, rehomed=False, timed_out=True),
                        error=ToolError(
                            code="SIM_RESET_REHOME_TIMEOUT",
                            message=(
                                "scene reset was sent but the re-home did not "
                                f"finish within {timeout_ms} ms"
                            ),
                            retryable=True,
                        ),
                    ),
                )
                continue
            if self._invoker is None or self._reset_config is None:
                continue
            if not pending.dispatched:
                if now_ms >= pending.dispatch_at_ms:
                    self._dispatch_rehome(key, pending, now_ms)
                continue
            for call in pending.calls.values():
                if call.terminal or not call.admitted:
                    continue
                if now_ms - call.last_poll_ms >= self._reset_config.poll_interval_ms:
                    call.last_poll_ms = now_ms
                    self._invoker.lookup_result(call.handle)

    def on_invoke_response(self, request_id: str, outcome: str, error: ToolError | None) -> None:
        """Admission response for one dispatched inner invoke."""
        entry = self._by_request_id.get(request_id)
        if entry is None:
            return
        key, role = entry
        execution = self._executions.get(key)
        if execution is None or execution.pending is None:
            return
        call = execution.pending.calls.get(role)
        if call is None or call.terminal:
            return
        if outcome == "accepted":
            call.admitted = True
            return
        call.terminal = True
        call.status = "failed"
        call.error = error or ToolError(
            code="SIM_RESET_REHOME_REJECTED",
            message=f"the {role} re-home invoke was rejected by the gateway",
            retryable=True,
        )
        self._maybe_complete(key, execution)

    def on_inner_event(self, invocation_id: str, event_type: str) -> None:
        """Gateway-streamed event for one inner invocation; terminal types trigger a lookup."""
        entry = self._by_invocation_id.get(invocation_id)
        if entry is None or event_type != "executor_completed":
            return
        key, role = entry
        execution = self._executions.get(key)
        if execution is None or execution.pending is None:
            return
        call = execution.pending.calls.get(role)
        if call is None or call.terminal or not call.admitted:
            return
        if self._invoker is not None:
            self._invoker.lookup_result(call.handle)

    def on_result_response(
        self, invocation_id: str, result: ToolResult | None, error: ToolError | None
    ) -> None:
        """Authoritative result of one inner action, addressed by its invocation id.

        Lookup responses correlate to the lookup's own request id, so the runner
        resolves that to the inner invocation id before calling this.
        """
        entry = self._by_invocation_id.get(invocation_id)
        if entry is None:
            return
        key, role = entry
        execution = self._executions.get(key)
        if execution is None or execution.pending is None:
            return
        call = execution.pending.calls.get(role)
        if call is None or call.terminal:
            return
        call.terminal = True
        if result is not None and result.status == "succeeded":
            call.status = "succeeded"
        else:
            call.status = "failed"
            call.error = (result.error if result is not None else None) or error or ToolError(
                code="SIM_RESET_REHOME_FAILED",
                message=f"the {role} re-home action failed",
                retryable=True,
            )
        self._maybe_complete(key, execution)

    def _maybe_complete(self, key: ToolExecutionKey, execution: _Execution) -> None:
        pending = execution.pending
        if pending is None or any(not call.terminal for call in pending.calls.values()):
            return
        failed = {role: call for role, call in pending.calls.items() if call.status != "succeeded"}
        if failed:
            role, call = next(iter(failed.items()))
            error = call.error or ToolError(
                code="SIM_RESET_REHOME_FAILED",
                message=f"the {role} re-home action failed",
                retryable=True,
            )
            result = ToolResult(
                status="failed",
                outputs=self._rehome_outputs(pending, rehomed=False),
                error=ToolError(
                    code=error.code,
                    message=f"scene was reset but the {role} re-home failed: {error.message}",
                    retryable=True,
                ),
            )
        else:
            result = ToolResult(
                status="succeeded",
                outputs=self._rehome_outputs(pending, rehomed=True),
            )
        self._complete(key, execution, result)

    def _rehome_outputs(
        self, pending: _PendingReset, *, rehomed: bool, timed_out: bool = False
    ) -> dict[str, Any]:
        outputs: dict[str, Any] = {
            "command": _RESET_COMMAND,
            "episode_reset": True,
            "rehomed": rehomed,
        }
        for role, call in sorted(pending.calls.items()):
            outputs[f"{role}_status"] = call.status or (
                "timed_out" if timed_out else "unknown"
            )
        return outputs

    def _complete(self, key: ToolExecutionKey, execution: _Execution, result: ToolResult) -> None:
        execution.pending = None
        execution.result = result
        for request_id, owner in list(self._by_request_id.items()):
            if owner[0] == key:
                self._by_request_id.pop(request_id, None)
        for invocation_id, owner in list(self._by_invocation_id.items()):
            if owner[0] == key:
                self._by_invocation_id.pop(invocation_id, None)

    def _record(self, key: ToolExecutionKey, execution: _Execution) -> None:
        self._executions[key] = execution
        while len(self._executions) > self._result_cache_size:
            evicted_key, evicted = self._executions.popitem(last=False)
            if evicted.pending is not None:
                for call in evicted.pending.calls.values():
                    self._by_request_id.pop(call.handle.request_id, None)
                    self._by_invocation_id.pop(call.handle.invocation_id, None)

    # -- lifecycle -----------------------------------------------------------

    def begin_shutdown(self) -> None:
        """Stop admission; outstanding executions keep their cached results."""
        self._accepting = False

    def reset_for_tests(self) -> None:
        self._executions.clear()
        self._by_request_id.clear()
        self._by_invocation_id.clear()
        self._accepting = True

    def recorded_executions(self) -> int:
        return len(self._executions)

    def last_output(self) -> dict[str, Any] | None:
        if not self._executions:
            return None
        execution = next(reversed(self._executions.values()))
        if execution.result is None:
            return None
        return dict(execution.result.outputs)
