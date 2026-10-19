# Tool Contracts

Endpoint `sim.control` exposes two operations, each with concurrency 1.

| tool_id | implementation_id | endpoint_id | operation | semantics |
| --- | --- | --- | --- | --- |
| `sim.get_robot_info` | `sim-control-policy` | `sim.control` | `get_robot_info` | query |
| `sim.reset` | `sim-control-policy` | `sim.control` | `reset` | action |

The wire contract is implemented by `SimInfoEndpoint` and `SimResetEndpoint`
(`control.py`) on top of the protocol-independent `RobotDescriptor`
(`robot_descriptor.py`). The descriptor never depends on `forge_tool`.

## `sim.control/get_robot_info` (query)

Requests take no arguments. Successful outputs are the validated robot
descriptor: `name`, `source`, `model`, `dof`, `joints` (ordered `name`, `kind`,
`limits`), `eef_site`, `initial_qpos`, `has_gripper`, and `notes`. Agents use it
to discover any bundled or user-supplied robot at runtime instead of parsing
skill documents. The descriptor is loaded once at startup from the profile's
`robot_descriptor_path` (strict schema; unknown keys are rejected) and is
read-only afterwards.

## `sim.control/reset` (action)

Requests take no arguments. The action has two stages:

1. **Scene reset (immediate).** Sends `{"command": "reset_scene"}` to the
   simulator over the `sim_command` topic, resetting the scene to the simulator
   config's `initial_state`.
2. **Re-home (configured; deferred completion).** When the profile configures a
   `reset:` block, the invoke is only *accepted* at start; it completes when the
   re-home finishes. After a `settle_ms` window (default 500, letting the
   simulator state propagate before motion planning), the node dispatches
   inner Tool invokes through the gateway's public caller channel
   (`tool_request`/`tool_response`):
   - `move_joints` on the configured motion endpoint, driving the arm joints —
     descriptor joints minus the joint named `gripper` — to
     `initial_qpos` values (missing entries default to `0.0`), with the
     profile-provided `group_name` and velocity/acceleration scales;
   - optional `set_opening` on the configured gripper endpoint, to
     `initial_qpos["gripper"]`.

   Completion is aggregated from gateway invoke responses, completion events,
   and `poll_interval_ms` result lookups, bounded by `timeout_ms` (default
   15000). Without a `reset:` block there is no re-home and the invoke completes
   in one round trip.

Outputs: `command`, `episode_reset`, and — when a re-home is configured —
`rehomed` plus per-stage `motion_status`/`gripper_status`
(`succeeded`/`failed`/`timed_out`/`unknown`). Cancellation outputs
`cancel_reason` and carries no error.

## Timing, idempotency, and errors

- Invokes are idempotent per `ToolExecutionKey`: replays return
  `{"idempotent": true, "phase": <running|terminal>}` without re-sending either
  stage. Completed outcomes are retained for a bounded number of executions.
- An expired Tool context or an in-progress shutdown rejects the invoke before
  the scene reset is sent.
- Re-home failures fail the outer action with retryable
  `SIM_RESET_REHOME_FAILED` (a stage failed; the scene was already reset) or
  `SIM_RESET_REHOME_REJECTED` (the gateway rejected an inner invoke);
  `SIM_RESET_REHOME_TIMEOUT` marks timed-out stages. Cancel/status for unknown
  executions use `FORGE_EXECUTION_NOT_FOUND`.
- The reset is not atomic with subsequent actions: orchestration must treat the
  terminal result as the barrier.

See the PAOS-owned `robot-control-sim` skill for the current MuJoCo dataflow,
gateway tool registration, and per-robot re-home profiles.
