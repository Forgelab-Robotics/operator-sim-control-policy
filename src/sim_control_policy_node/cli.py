from __future__ import annotations

import argparse

from .config import load_config
from .runner import run_sim_control_policy


def main() -> int:
    parser = argparse.ArgumentParser(description="Sim Control Tool policy node (info + reset)")
    parser.add_argument("--config", required=True, help="Path to sim control policy YAML")
    args = parser.parse_args()
    return run_sim_control_policy(load_config(args.config))


if __name__ == "__main__":
    raise SystemExit(main())
