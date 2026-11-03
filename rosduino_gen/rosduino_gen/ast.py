# SPDX-License-Identifier: Apache-2.0
"""Shared interface AST for the simple parser and rosidl_parser backend."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

def normalize_type_expr(type_expr: str) -> str:
    """Canonical ROS 2 type string used to compare parsers.

    Nested types without an explicit ``msg``/``srv`` middle segment become
    ``pkg/msg/Name`` so both backends agree.
    """
    expr = type_expr.strip()
    bracket = ""
    bound = ""
    if "[" in expr:
        base, bracket = expr.split("[", 1)
        expr = base
        bracket = "[" + bracket
    if expr.startswith("string<=") or expr.startswith("wstring<="):
        return expr + bracket
    if "/" not in expr:
        return expr + bracket
    parts = expr.split("/")
    if len(parts) == 2 and parts[0] not in ("msg", "srv") and parts[1] not in ("msg", "srv"):
        expr = f"{parts[0]}/msg/{parts[1]}"
    elif len(parts) >= 3:
        expr = "/".join(parts)
    return expr + bound + bracket


def snake_name(name: str) -> str:
    out = []
    for i, ch in enumerate(name):
        if ch.isupper() and i > 0 and (
            name[i - 1].islower() or (i + 1 < len(name) and name[i + 1].islower())
        ):
            out.append("_")
        out.append(ch.lower())
    return "".join(out)


def include_guard(package: str, kind: str, name: str) -> str:
    return f"{package}_{kind}_{name}_H_".upper().replace("-", "_")


def ros_type_name(package: str, kind: str, name: str) -> str:
    return f"{package}/{kind}/{name}"


@dataclass(frozen=True)
class FieldSpec:
    type_expr: str
    name: str


@dataclass(frozen=True)
class ConstantSpec:
    type_expr: str
    name: str
    value: str


@dataclass(frozen=True)
class StructureSpec:
    fields: Tuple[FieldSpec, ...]
    constants: Tuple[ConstantSpec, ...]


@dataclass(frozen=True)
class InterfaceSpec:
    package: str
    name: str
    kind: str
    request: StructureSpec
    response: Optional[StructureSpec] = None
    source_text: Optional[str] = None

    @property
    def type_name(self) -> str:
        return ros_type_name(self.package, self.kind, self.name)
