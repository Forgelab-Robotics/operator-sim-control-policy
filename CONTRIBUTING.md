# Contributing

Thank you for contributing to the Sim Control Policy Node.

## Development

Use Python 3.12 and `uv`:

```bash
uv sync --frozen --all-groups
uv run pytest -q
uv run forge-sim-control-policy --help
```

The vendored `forge-tool` protocol under `src/forge_tool` is resolved
automatically by the pytest configuration and the package init; it needs no
environment setup. Keep it byte-identical to the copy in the other policy
nodes.

Keep changes focused and add tests for behavior changes. Keep the Tool wire
contract (`control.py`, `runner.py`) separate from the robot descriptor model
(`robot_descriptor.py`): descriptor loading never depends on `forge_tool`.

## Security and generated data

- Report vulnerabilities privately as described in `SECURITY.md`.
- Never commit credentials, private repository URLs, recordings, runtime state,
  machine-specific paths, or personal data.
- Do not run tests that command physical hardware without independent safety
  controls and an operator-accessible emergency stop.

By submitting a contribution, you agree that it is licensed under Apache
License 2.0.

## README capability summary

Changes to Node entrypoints, Endpoint operations or Tool APIs must update the root
README capability summary in the same change. Follow the [Node README contract](https://github.com/PhyAgentOS/forge/blob/master/CONTRIBUTING.md#node-readme-contract);
count Nodes, Endpoints, operations and deployment Tools separately.
