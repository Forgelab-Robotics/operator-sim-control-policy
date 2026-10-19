# Open-source release audit

- Date: 2026-10-19
- Repository: `Forgelab-Robotics/operator-sim-control-policy` (public GitHub)
- Version audited: `v0.1.2`

## Decision

The Sim Control Policy Node can be published as open source (Apache-2.0) with a
deterministic Ubuntu 20.04 PyInstaller binary release, following the
established `operator-*` publication model (reference:
`operator-relative-pose-policy`).

## Completed checks

- `uv sync --frozen --all-groups` resolves entirely from public indexes; no
  local path dependencies in `pyproject.toml`.
- `uv run pytest -q` — all unit tests pass locally (68 tests).
- `uv run ruff check .` — clean.
- `uv run forge-sim-control-policy --help` — CLI verifies.
- `uv export --frozen --format requirements-txt --no-dev --no-emit-project`
  + `uvx pip-audit --requirement` — no known advisories in the locked runtime
  set.
- `detect-secrets scan --all-files` — 0 findings on the tracked tree
  (untracked `.venv/`, caches ignored).
- History blob scan — trivially clean: the public repository starts from a
  fresh initial commit; no prior internal history is carried.
- `git grep -E 'gitlab\.ex-ai\.cn|meta-emt|/home/|/Users/'` — no matches on the
  publishable tree (CI enforces this on every push).
- Largest tracked file is well under the 100 MiB GitHub limit; `dist/`,
  `build/`, and virtual environments are gitignored and never published.

## License findings

- Project code: Apache-2.0 (`LICENSE`, `NOTICE`).
- Runtime dependencies are public PyPI packages (`dora-rs` MIT, `forge-msgs`
  Apache-2.0, `pyarrow` Apache-2.0, `PyYAML` MIT); none are vendored.
- `forge-tool` (Apache-2.0) and the `ToolMessage` Arrow carrier subset of
  `forge-msgs` are vendored under `src/` with provenance recorded in
  `THIRD_PARTY_NOTICES.md`; the vendored tree is byte-identical to the copy in
  the sibling policy-node repositories.
- PyInstaller is build-time only (GPL-2.0-or-later with a bootloader exception);
  binary redistribution obligations are noted in `THIRD_PARTY_NOTICES.md`.

## Known limitations

- The `reset` re-home depends on the gateway's public caller channel; without a
  running gateway (and configured motion/gripper endpoints) the action fails
  with a retryable `SIM_RESET_*` error after the scene reset itself has already
  been applied.
- `robot_descriptor_path` is resolved relative to the profile config file; the
  packaged node expects the skill bundle's asset layout.

## Publication model

Fresh public history on GitHub (`Forgelab-Robotics/operator-sim-control-policy`);
the v0.1.2 tag triggers the manual `build-ubuntu20.yml` workflow, whose
artifact becomes the GitHub Release asset
`sim-control-policy-v0.1.2-ubuntu20.04-x86_64.tar.gz` (+ `.sha256`). The
artifact is registered in the PAOS resource manager (`direct` mode) and pinned
by sha256 in the `robot-control-sim` skill manifest.
