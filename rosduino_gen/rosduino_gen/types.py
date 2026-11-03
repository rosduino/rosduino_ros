# SPDX-License-Identifier: Apache-2.0
"""ROS 2 type expressions → C++11 field types and CDR sizes for MCU headers."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple

from rosduino_gen.ast import InterfaceSpec, StructureSpec, normalize_type_expr

# Lite profile: ROSDUINO_RX_SIZE 96 − 8 byte frame header.
LITE_MAX_CDR = 88
DEFAULT_SEQ_CAP = 8
# Unbounded `string` has no IDL max; count length prefix + empty NUL so Lite
# types (Range, SetBool) still satisfy maxSerializedSize() <= 88. Longer values
# can fail at runtime with TooLarge.
DEFAULT_STRING_CAP = 0

_PRIMITIVES: Dict[str, Tuple[str, str, str, int]] = {
    # ros: (cpp, writeFn, readFn, wire_size)
    "bool": ("bool", "writeBool", "readBool", 1),
    "byte": ("uint8_t", "writeUInt8", "readUInt8", 1),
    "char": ("uint8_t", "writeUInt8", "readUInt8", 1),
    "int8": ("int8_t", "writeInt8", "readInt8", 1),
    "uint8": ("uint8_t", "writeUInt8", "readUInt8", 1),
    "int16": ("int16_t", "writeInt16", "readInt16", 2),
    "uint16": ("uint16_t", "writeUInt16", "readUInt16", 2),
    "int32": ("int32_t", "writeInt32", "readInt32", 4),
    "uint32": ("uint32_t", "writeUInt32", "readUInt32", 4),
    "int64": ("int64_t", "writeInt64", "readInt64", 8),
    "uint64": ("uint64_t", "writeUInt64", "readUInt64", 8),
    "float32": ("float", "writeFloat32", "readFloat32", 4),
    "float64": ("double", "writeFloat64", "readFloat64", 8),
}

_ARRAY = re.compile(r"^(.*)\[(\d+)\]$")
_BOUNDED_SEQ = re.compile(r"^(.*)\[<=(\d+)\]$")
_UNBOUNDED_SEQ = re.compile(r"^(.*)\[\]$")
_BOUNDED_STR = re.compile(r"^(w?string)<=(\d+)$")
_NESTED = re.compile(r"^([A-Za-z_]\w*)(/[A-Za-z_]\w*)+$")


@dataclass(frozen=True)
class ParsedType:
    kind: str  # primitive, string, nested, array, sequence
    ros: str
    cpp: str
    write_fn: str
    read_fn: str
    alignment: int
    wire_size: int  # primitive wire size; 0 otherwise
    bound: Optional[int]  # array length, seq cap, string max chars
    elem: Optional["ParsedType"]
    nested: Optional[Tuple[str, str, str]]  # pkg, kind, name
    avr_float64: bool = False


class TypeError_(ValueError):
    """Unsupported ROS type expression."""


def align(offset: int, alignment: int) -> int:
    if alignment <= 1:
        return offset
    rem = offset % alignment
    return offset if rem == 0 else offset + (alignment - rem)


def parse_type(
    type_expr: str,
    package: str,
    default_seq_cap: int = DEFAULT_SEQ_CAP,
) -> ParsedType:
    expr = normalize_type_expr(type_expr)
    match = _BOUNDED_SEQ.match(expr)
    if match:
        elem = parse_type(match.group(1), package, default_seq_cap)
        cap = int(match.group(2))
        return ParsedType(
            "sequence", expr, f"rosduino::Sequence<{elem.cpp}, {cap}U>", "", "", 4, 0, cap, elem, None
        )
    match = _UNBOUNDED_SEQ.match(expr)
    if match:
        elem = parse_type(match.group(1), package, default_seq_cap)
        cap = default_seq_cap
        return ParsedType(
            "sequence",
            expr,
            f"rosduino::Sequence<{elem.cpp}, {cap}U>",
            "",
            "",
            4,
            0,
            cap,
            elem,
            None,
        )
    match = _ARRAY.match(expr)
    if match:
        elem = parse_type(match.group(1), package, default_seq_cap)
        size = int(match.group(2))
        return ParsedType("array", expr, elem.cpp, "", "", elem.alignment, 0, size, elem, None)
    match = _BOUNDED_STR.match(expr)
    if match:
        if match.group(1) == "wstring":
            raise TypeError_(f"unsupported type: {expr}")
        bound = int(match.group(2))
        return ParsedType("string", expr, "const char*", "writeString", "readString", 4, 0, bound, None, None)
    if expr == "string":
        return ParsedType("string", expr, "const char*", "writeString", "readString", 4, 0, None, None, None)
    if expr == "wstring" or expr.startswith("wstring"):
        raise TypeError_(f"unsupported type: {expr}")
    if expr in _PRIMITIVES:
        cpp, write_fn, read_fn, size = _PRIMITIVES[expr]
        return ParsedType(
            "primitive",
            expr,
            cpp,
            write_fn,
            read_fn,
            size,
            size,
            None,
            None,
            None,
            avr_float64=False,
        )
    if "/" in expr:
        parts = expr.split("/")
        if len(parts) == 3:
            pkg, kind, name = parts
        elif len(parts) == 2:
            pkg, name = parts
            kind = "msg"
        else:
            raise TypeError_(f"unsupported type: {expr}")
        cpp = f"{pkg}::{kind}::{name}"
        return ParsedType("nested", expr, cpp, "", "", 1, 0, None, None, (pkg, kind, name))
    if re.match(r"^[A-Z]", expr):
        cpp = f"{package}::msg::{expr}"
        return ParsedType(
            "nested", expr, cpp, "", "", 1, 0, None, None, (package, "msg", expr)
        )
    raise TypeError_(f"unsupported type: {expr}")


def cpp_field(parsed: ParsedType, name: str, indent: str = "  ") -> str:
    if parsed.kind == "array":
        return f"{indent}{parsed.cpp} {name}[{parsed.bound}U];"
    return f"{indent}{parsed.cpp} {name};"


def cpp_constant(type_expr: str, name: str, value: str) -> str:
    parsed = parse_type(type_expr, "")
    cpp = parsed.cpp
    literal = _const_literal(parsed, value)
    return f"static const {cpp} {name} = {literal};"


def _const_literal(parsed: ParsedType, value: str) -> str:
    raw = value.strip()
    if parsed.ros == "bool":
        lower = raw.lower()
        if lower in ("1", "true"):
            return "true"
        if lower in ("0", "false"):
            return "false"
        return raw
    if parsed.ros in ("float32", "float64"):
        if parsed.avr_float64:
            return raw if "." in raw or "e" in raw.lower() else f"{raw}.0"
        if "f" not in raw.lower() and "e" not in raw.lower():
            return f"{raw}f" if "." in raw else f"{raw}.0f"
        return raw
    if parsed.ros.startswith("uint") or parsed.ros in ("byte", "char"):
        if raw.upper().endswith("U"):
            return raw
        return f"{raw}U"
    return raw


def nested_include(parsed: ParsedType) -> Optional[str]:
    if parsed.kind in ("array", "sequence") and parsed.elem is not None:
        return nested_include(parsed.elem)
    if parsed.kind != "nested" or parsed.nested is None:
        return None
    pkg, kind, name = parsed.nested
    return f"{pkg}/{kind}/{name}.h"


Catalog = Dict[str, InterfaceSpec]
ResolveFn = Callable[[str], Optional[InterfaceSpec]]


def field_end_offset(
    parsed: ParsedType,
    offset: int,
    resolve: ResolveFn,
    string_cap: int = DEFAULT_STRING_CAP,
) -> int:
    if parsed.kind == "primitive":
        return align(offset, parsed.alignment) + parsed.wire_size
    if parsed.kind == "string":
        chars = parsed.bound if parsed.bound is not None else string_cap
        return align(offset, 4) + 4 + chars + 1
    if parsed.kind == "array" and parsed.elem is not None and parsed.bound is not None:
        pos = offset
        for _ in range(parsed.bound):
            pos = field_end_offset(parsed.elem, pos, resolve, string_cap)
        return pos
    if parsed.kind == "sequence" and parsed.elem is not None and parsed.bound is not None:
        pos = align(offset, 4) + 4
        for _ in range(parsed.bound):
            pos = field_end_offset(parsed.elem, pos, resolve, string_cap)
        return pos
    if parsed.kind == "nested" and parsed.nested is not None:
        spec = resolve(parsed.ros)
        if spec is None:
            spec = resolve("/".join(parsed.nested))
        if spec is None:
            return offset
        return structure_end_offset(spec.request, offset, resolve, string_cap, spec.package)
    return offset


def structure_end_offset(
    structure: StructureSpec,
    offset: int,
    resolve: ResolveFn,
    string_cap: int,
    package: str,
    default_seq_cap: int = DEFAULT_SEQ_CAP,
) -> int:
    fields = structure.fields
    if not fields:
        return align(offset, 1) + 1  # structure_needs_at_least_one_member
    pos = offset
    for field in fields:
        parsed = parse_type(field.type_expr, package, default_seq_cap)
        pos = field_end_offset(parsed, pos, resolve, string_cap)
    return pos


def max_serialized_size(
    spec: InterfaceSpec,
    resolve: ResolveFn,
    default_seq_cap: int = DEFAULT_SEQ_CAP,
    string_cap: int = DEFAULT_STRING_CAP,
    which: str = "message",
) -> int:
    if spec.kind == "srv":
        req = structure_end_offset(spec.request, 0, resolve, string_cap, spec.package, default_seq_cap)
        resp = structure_end_offset(
            spec.response or StructureSpec((), ()),
            0,
            resolve,
            string_cap,
            spec.package,
            default_seq_cap,
        )
        if which == "request":
            return req
        if which == "response":
            return resp
        return max(req, resp)
    return structure_end_offset(spec.request, 0, resolve, string_cap, spec.package, default_seq_cap)


def fits_lite(size: int) -> bool:
    return size <= LITE_MAX_CDR
