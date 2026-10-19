"""Command-line validation and profile-plan rendering."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from .contracts import BringupContract, ContractError


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate and resolve the greenhouse Jazzy bringup contract."
    )
    parser.add_argument("--config-dir", required=True)
    parser.add_argument("--profiles", required=True)
    parser.add_argument(
        "--host-profile",
        required=True,
        choices=("simulation", "dev-ci", "nuc-amd64"),
    )
    parser.add_argument(
        "--require-runtime-ready",
        action="store_true",
        help="Reject profile selections whose runtime implementation is not complete.",
    )
    parser.add_argument("--output", help="Write the resolved JSON plan to this file.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        contract = BringupContract.load(args.config_dir)
        resolved = contract.resolve(
            args.profiles,
            args.host_profile,
            require_runtime_ready=args.require_runtime_ready,
        )
    except ContractError as exc:
        print(f"bringup contract rejected: {exc}", file=sys.stderr)
        return 2

    payload = resolved.to_json() + "\n"
    if args.output:
        output = Path(args.output).expanduser()
        output.write_text(payload, encoding="utf-8")
    else:
        print(payload, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
