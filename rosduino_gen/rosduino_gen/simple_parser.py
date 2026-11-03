# SPDX-License-Identifier: Apache-2.0
"""Standalone .msg/.srv parser used when rosidl_parser is not available."""

from __future__ import annotations

from pathlib import Path
from typing import List

from rosduino_gen.ast import (
    ConstantSpec,
    FieldSpec,
    InterfaceSpec,
    StructureSpec,
    normalize_type_expr,
)


def _strip_comment(line: str) -> str:
    in_quote = ""
    for i, ch in enumerate(line):
        if ch in ("'", '"'):
            if not in_quote:
                in_quote = ch
            elif in_quote == ch:
                in_quote = ""
        elif ch == "#" and not in_quote:
            return line[:i].rstrip()
    return line.rstrip()


def _split_assignment(line: str) -> tuple[str, str] | None:
    depth = 0
    for i, ch in enumerate(line):
        if ch == "[":
            depth += 1
        elif ch == "]":
            depth -= 1
        elif ch == "=" and depth == 0 and (i == 0 or line[i - 1] != "<"):
            return line[:i], line[i + 1 :]
    return None


def _parse_structure(text: str) -> StructureSpec:
    fields: List[FieldSpec] = []
    constants: List[ConstantSpec] = []
    for raw in text.splitlines():
        line = _strip_comment(raw).strip()
        if not line:
            continue
        assignment = _split_assignment(line)
        if assignment is not None:
            left, value = assignment
            parts = left.split()
            if len(parts) != 2:
                raise ValueError(f"invalid constant line: {raw!r}")
            constants.append(
                ConstantSpec(
                    type_expr=normalize_type_expr(parts[0]),
                    name=parts[1],
                    value=value.strip(),
                )
            )
            continue
        parts = line.split()
        if len(parts) != 2:
            raise ValueError(f"invalid field line: {raw!r}")
        fields.append(
            FieldSpec(type_expr=normalize_type_expr(parts[0]), name=parts[1])
        )
    return StructureSpec(fields=tuple(fields), constants=tuple(constants))


def parse_message_text(package: str, name: str, text: str) -> InterfaceSpec:
    return InterfaceSpec(
        package=package,
        name=name,
        kind="msg",
        request=_parse_structure(text),
        response=None,
        source_text=text,
    )


def parse_service_text(package: str, name: str, text: str) -> InterfaceSpec:
    parts = text.split("\n---\n")
    if len(parts) != 2:
        # Allow --- on its own line without requiring surrounding newlines only.
        chunks = []
        buf: List[str] = []
        for raw in text.splitlines():
            if raw.strip() == "---":
                chunks.append("\n".join(buf))
                buf = []
            else:
                buf.append(raw)
        chunks.append("\n".join(buf))
        if len(chunks) != 2:
            raise ValueError(f"service {name} must have exactly one --- separator")
        parts = chunks
    return InterfaceSpec(
        package=package,
        name=name,
        kind="srv",
        request=_parse_structure(parts[0]),
        response=_parse_structure(parts[1]),
        source_text=text,
    )


def parse_file(path: Path, package: str) -> InterfaceSpec:
    text = path.read_text(encoding="utf-8")
    suffix = path.suffix.lower()
    if suffix == ".msg":
        return parse_message_text(package, path.stem, text)
    if suffix == ".srv":
        return parse_service_text(package, path.stem, text)
    raise ValueError(f"unsupported interface file: {path}")
