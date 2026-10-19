# Security policy

## Reporting a vulnerability

Do not open a public issue for a suspected vulnerability. Use GitHub's private
vulnerability reporting for this repository. Include affected versions,
reproduction steps, impact, and any suggested mitigation.

## Deployment boundary

The Sim Control Policy Node is a command-line Dora node. It performs no
authentication and operates against a simulated scene.

- Run the node only on trusted hosts within an isolated robot network.
- Treat the robot descriptor and profile YAML as safety-relevant input; validate
  joint limits and the reset re-home targets before connecting the node to any
  real robot. The node is designed for MuJoCo simulation and must not be wired
  to physical hardware.
- `get_robot_info` is a Query: it only reports the validated descriptor. The
  `reset` Action commands the simulator and dispatches re-home motion through
  the gateway to downstream motion/gripper nodes; those nodes' safety controls
  must be validated independently.
- Do not expose policy processes or their configuration to untrusted networks.

Security fixes are supported on the latest released version.
