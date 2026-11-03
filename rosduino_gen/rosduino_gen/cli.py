# SPDX-License-Identifier: Apache-2.0
"""CLI entry point for rosduino-gen (requires a sourced ROS 2 workspace)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Sequence

from rosduino_gen.generate import (
    discover_packages,
    generate_packages,
    list_interfaces,
    load_types_file,
    ros_available,
    types_package_names,
)

_PACKAGE_NAME = "rosduino_gen"
_TYPES_FILE = "arduino_types.txt"


def package_root() -> Path:
    """Return the `rosduino_gen` package directory (source tree or share)."""
    for parent in Path(__file__).resolve().parents:
        manifest = parent / "package.xml"
        if not manifest.is_file():
            continue
        try:
            text = manifest.read_text(encoding="utf-8")
        except OSError:
            continue
        if f"<name>{_PACKAGE_NAME}</name>" in text:
            return parent
    if ros_available():
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(_PACKAGE_NAME))
    raise RuntimeError(f"cannot locate {_PACKAGE_NAME} package directory")


def default_out_dir() -> Path:
    return package_root() / "generated"


def default_types_path() -> Path:
    return package_root() / _TYPES_FILE


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rosduino-gen",
        description="Generate RosDuino C++ headers from ROS 2 .msg/.srv files.",
    )
    parser.add_argument(
        "--package",
        action="append",
        dest="packages",
        metavar="PKG",
        help="ROS 2 interface package (repeatable)",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help=f"Generate types listed in {_TYPES_FILE} (nested dependencies included)",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="Print available interfaces (pkg/msg|srv/Name) and exit; omit --package to list all",
    )
    parser.add_argument(
        "--out",
        help="Output directory (default: generated/ in this package)",
    )
    parser.add_argument(
        "--parser",
        choices=("auto", "simple", "rosidl"),
        default="auto",
        help="Parser backend (default: ROS parser if installed, else simple)",
    )
    parser.add_argument(
        "--default-seq-cap",
        type=int,
        default=8,
        help="Default Sequence capacity for unbounded T[] (default: 8)",
    )
    return parser


def _written_type_names(written: Sequence[Path]) -> set:
    return {f"{p.parent.parent.name}/{p.parent.name}/{p.stem}" for p in written}


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.all and args.packages:
        print("rosduino-gen: error: use either --all or --package, not both", file=sys.stderr)
        return 2
    if args.list:
        try:
            names = list_interfaces(None if not args.packages else args.packages)
        except Exception as exc:  # noqa: BLE001
            print(f"rosduino-gen: error: {exc}", file=sys.stderr)
            return 1
        for name in names:
            print(name)
        return 0
    if not args.all and not args.packages:
        print("rosduino-gen: error: pass --package, --all, or --list", file=sys.stderr)
        return 2

    out = Path(args.out) if args.out else default_out_dir()
    whole_packages: Optional[Sequence[str]] = None
    type_names: Optional[Sequence[str]] = None
    package_names: Optional[Sequence[str]] = None if args.all else args.packages
    try:
        if args.all:
            types_path = default_types_path()
            whole_packages, type_names = load_types_file(types_path)
            package_names = types_package_names(whole_packages, type_names)
        packages = discover_packages(package_names)
        written, warnings = generate_packages(
            packages,
            out,
            parser_name=args.parser,
            default_seq_cap=args.default_seq_cap,
            whole_packages=whole_packages,
            type_names=type_names,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"rosduino-gen: error: {exc}", file=sys.stderr)
        return 1
    for warning in warnings:
        print(f"rosduino-gen: warning: {warning}", file=sys.stderr)
    missing = [w for w in warnings if w.startswith("type not found:")]
    if missing:
        return 1
    if args.all and type_names:
        written_names = _written_type_names(written)
        absent = [name for name in type_names if name not in written_names]
        if absent:
            print(
                "rosduino-gen: error: type(s) not written: " + ", ".join(absent),
                file=sys.stderr,
            )
            return 1
    for path in written:
        print(path)
    if not written:
        print("rosduino-gen: warning: no .msg/.srv files found", file=sys.stderr)
        return 1 if args.all else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
