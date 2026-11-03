# SPDX-License-Identifier: Apache-2.0
"""rosidl_parser backend: .msg/.srv -> IDL (rosidl_adapter) -> AST."""

from __future__ import annotations

import contextlib
import io
import tempfile
from pathlib import Path
from typing import Any, List

from rosduino_gen.ast import (
    ConstantSpec,
    FieldSpec,
    InterfaceSpec,
    StructureSpec,
    normalize_type_expr,
)

_IDL_BASIC = {
    "boolean": "bool",
    "float": "float32",
    "double": "float64",
    "octet": "byte",
}


def available() -> bool:
    try:
        import rosidl_adapter.msg  # noqa: F401
        import rosidl_parser.parser  # noqa: F401
    except ImportError:
        return False
    return True


def _type_expr(type_obj: Any) -> str:
    from rosidl_parser.definition import (
        Array,
        BasicType,
        BoundedSequence,
        BoundedString,
        NamedType,
        NamespacedType,
        UnboundedSequence,
        UnboundedString,
    )

    if isinstance(type_obj, BasicType):
        return _IDL_BASIC.get(type_obj.typename, type_obj.typename)
    if isinstance(type_obj, UnboundedString):
        return "string"
    if isinstance(type_obj, BoundedString):
        return f"string<={type_obj.maximum_size}"
    if isinstance(type_obj, NamespacedType):
        return normalize_type_expr("/".join(list(type_obj.namespaces) + [type_obj.name]))
    if isinstance(type_obj, NamedType):
        return normalize_type_expr(type_obj.name)
    if isinstance(type_obj, Array):
        return f"{_type_expr(type_obj.value_type)}[{type_obj.size}]"
    if isinstance(type_obj, UnboundedSequence):
        return f"{_type_expr(type_obj.value_type)}[]"
    if isinstance(type_obj, BoundedSequence):
        return f"{_type_expr(type_obj.value_type)}[<={type_obj.maximum_size}]"
    raise TypeError(f"unsupported rosidl type: {type(type_obj)!r}")


def _const_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _structure(message: Any) -> StructureSpec:
    fields: List[FieldSpec] = []
    constants: List[ConstantSpec] = []
    for member in message.structure.members:
        fields.append(FieldSpec(type_expr=_type_expr(member.type), name=member.name))
    for const in message.constants:
        type_expr = const.type if isinstance(const.type, str) else _type_expr(const.type)
        constants.append(
            ConstantSpec(
                type_expr=normalize_type_expr(type_expr),
                name=const.name,
                value=_const_value(const.value),
            )
        )
    return StructureSpec(fields=tuple(fields), constants=tuple(constants))


def _convert_to_idl(path: Path, package: str, kind: str) -> str:
    from rosidl_adapter.msg import convert_msg_to_idl
    from rosidl_adapter.srv import convert_srv_to_idl

    with tempfile.TemporaryDirectory() as tmp:
        pkg_dir = Path(tmp) / package
        rel = Path(kind) / path.name
        dest = pkg_dir / rel
        dest.parent.mkdir(parents=True)
        dest.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
        idl_dir = Path(tmp) / "idl"
        idl_dir.mkdir()
        sink = io.StringIO()
        with contextlib.redirect_stdout(sink), contextlib.redirect_stderr(sink):
            if kind == "msg":
                idl_path = convert_msg_to_idl(pkg_dir, package, rel, idl_dir)
            else:
                idl_path = convert_srv_to_idl(pkg_dir, package, rel, idl_dir)
        return idl_path.read_text(encoding="utf-8")


def parse_file(path: Path, package: str) -> InterfaceSpec:
    from rosidl_parser.definition import Message, Service
    from rosidl_parser.parser import parse_idl_string

    suffix = path.suffix.lower()
    if suffix == ".msg":
        kind = "msg"
    elif suffix == ".srv":
        kind = "srv"
    else:
        raise ValueError(f"unsupported interface file: {path}")

    source_text = path.read_text(encoding="utf-8")
    content = parse_idl_string(_convert_to_idl(path, package, kind))
    if kind == "msg":
        messages = [el for el in content.elements if isinstance(el, Message)]
        if not messages:
            raise ValueError(f"rosidl_parser produced no Message for {path}")
        return InterfaceSpec(
            package=package,
            name=path.stem,
            kind="msg",
            request=_structure(messages[0]),
            response=None,
            source_text=source_text,
        )
    services = [el for el in content.elements if isinstance(el, Service)]
    if not services:
        raise ValueError(f"rosidl_parser produced no Service for {path}")
    srv = services[0]
    return InterfaceSpec(
        package=package,
        name=path.stem,
        kind="srv",
        request=_structure(srv.request_message),
        response=_structure(srv.response_message),
        source_text=source_text,
    )
