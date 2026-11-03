# SPDX-License-Identifier: Apache-2.0
"""Shared parser tests: simple backend always, rosidl when installed."""

from __future__ import annotations

import unittest
from pathlib import Path

from rosduino_gen import rosidl_backend, simple_parser
from rosduino_gen.ast import ConstantSpec, FieldSpec

FIXTURES = Path(__file__).resolve().parent / "fixtures"


def _parse(backend: str, path: Path, package: str):
    if backend == "simple":
        return simple_parser.parse_file(path, package)
    return rosidl_backend.parse_file(path, package)


class ParserTests(unittest.TestCase):
    def _backends(self):
        names = ["simple"]
        if rosidl_backend.available():
            names.append("rosidl")
        return names

    def test_bool_field(self):
        path = FIXTURES / "std_msgs" / "msg" / "Bool.msg"
        for backend in self._backends():
            spec = _parse(backend, path, "std_msgs")
            self.assertEqual(spec.type_name, "std_msgs/msg/Bool")
            self.assertEqual(spec.kind, "msg")
            self.assertEqual(spec.request.fields, (FieldSpec("bool", "data"),))
            self.assertEqual(spec.request.constants, ())

    def test_int32_field(self):
        path = FIXTURES / "std_msgs" / "msg" / "Int32.msg"
        for backend in self._backends():
            spec = _parse(backend, path, "std_msgs")
            self.assertEqual(spec.request.fields, (FieldSpec("int32", "data"),))

    def test_range_nested_and_constants(self):
        path = FIXTURES / "sensor_msgs" / "msg" / "Range.msg"
        for backend in self._backends():
            spec = _parse(backend, path, "sensor_msgs")
            self.assertEqual(
                spec.request.constants,
                (
                    ConstantSpec("uint8", "ULTRASOUND", "0"),
                    ConstantSpec("uint8", "INFRARED", "1"),
                ),
            )
            self.assertEqual(spec.request.fields[0], FieldSpec("std_msgs/msg/Header", "header"))
            self.assertEqual(spec.request.fields[-1], FieldSpec("float32", "variance"))

    def test_set_bool_service(self):
        path = FIXTURES / "std_srvs" / "srv" / "SetBool.srv"
        for backend in self._backends():
            spec = _parse(backend, path, "std_srvs")
            self.assertEqual(spec.kind, "srv")
            self.assertEqual(spec.type_name, "std_srvs/srv/SetBool")
            self.assertEqual(spec.request.fields, (FieldSpec("bool", "data"),))
            self.assertIsNotNone(spec.response)
            self.assertEqual(
                spec.response.fields,
                (FieldSpec("bool", "success"), FieldSpec("string", "message")),
            )

    def test_arrays_and_bounded_types(self):
        text = "\n".join(
            [
                "int32[] values",
                "int32[4] fixed",
                "int32[<=8] bounded",
                "string<=20 label",
            ]
        )
        spec = simple_parser.parse_message_text("demo", "Arrays", text)
        self.assertEqual(
            [f.type_expr for f in spec.request.fields],
            ["int32[]", "int32[4]", "int32[<=8]", "string<=20"],
        )

    def test_simple_and_rosidl_agree(self):
        if not rosidl_backend.available():
            self.skipTest("rosidl_parser is not installed")
        samples = [
            (FIXTURES / "std_msgs" / "msg" / "Bool.msg", "std_msgs"),
            (FIXTURES / "std_msgs" / "msg" / "Int32.msg", "std_msgs"),
            (FIXTURES / "sensor_msgs" / "msg" / "Range.msg", "sensor_msgs"),
            (FIXTURES / "std_srvs" / "srv" / "SetBool.srv", "std_srvs"),
        ]
        for path, package in samples:
            simple = simple_parser.parse_file(path, package)
            rosidl = rosidl_backend.parse_file(path, package)
            self.assertEqual(simple, rosidl, msg=str(path))


if __name__ == "__main__":
    unittest.main()
