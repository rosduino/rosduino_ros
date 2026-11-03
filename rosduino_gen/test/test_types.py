# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import unittest

from rosduino_gen.simple_parser import parse_message_text, parse_service_text
from rosduino_gen.types import (
    LITE_MAX_CDR,
    cpp_field,
    fits_lite,
    max_serialized_size,
    parse_type,
)


class TypeParseTests(unittest.TestCase):
    def test_primitives(self):
        parsed = parse_type("int32", "std_msgs")
        self.assertEqual(parsed.cpp, "int32_t")
        self.assertEqual(parsed.write_fn, "writeInt32")
        self.assertEqual(cpp_field(parsed, "data").strip(), "int32_t data;")

    def test_float64_is_double(self):
        parsed = parse_type("float64", "std_msgs")
        self.assertEqual(parsed.cpp, "double")
        self.assertEqual(parsed.write_fn, "writeFloat64")
        self.assertIn("double data;", cpp_field(parsed, "data"))

    def test_string_and_bounded_string(self):
        self.assertEqual(parse_type("string", "std_msgs").cpp, "const char*")
        self.assertEqual(parse_type("string<=20", "std_msgs").bound, 20)

    def test_arrays_and_sequences(self):
        fixed = parse_type("int32[4]", "demo")
        self.assertEqual(fixed.kind, "array")
        self.assertIn("fixed[4U]", cpp_field(fixed, "fixed"))
        bounded = parse_type("int32[<=8]", "demo")
        self.assertEqual(bounded.cpp, "rosduino::Sequence<int32_t, 8U>")
        unbounded = parse_type("int32[]", "demo", default_seq_cap=4)
        self.assertEqual(unbounded.cpp, "rosduino::Sequence<int32_t, 4U>")

    def test_nested(self):
        parsed = parse_type("std_msgs/Header", "sensor_msgs")
        self.assertEqual(parsed.cpp, "std_msgs::msg::Header")
        self.assertEqual(parsed.nested, ("std_msgs", "msg", "Header"))

    def test_wstring_rejected(self):
        with self.assertRaises(ValueError):
            parse_type("wstring", "std_msgs")


class SizeTests(unittest.TestCase):
    def test_int32_is_four_bytes(self):
        spec = parse_message_text("std_msgs", "Int32", "int32 data\n")
        self.assertEqual(max_serialized_size(spec, lambda _: None), 4)
        self.assertTrue(fits_lite(4))
        self.assertFalse(fits_lite(LITE_MAX_CDR + 1))

    def test_bool_and_empty(self):
        self.assertEqual(
            max_serialized_size(parse_message_text("std_msgs", "Bool", "bool data\n"), lambda _: None),
            1,
        )
        self.assertEqual(
            max_serialized_size(parse_message_text("std_msgs", "Empty", ""), lambda _: None),
            1,
        )

    def test_sequence_and_array(self):
        spec = parse_message_text(
            "demo",
            "Arrays",
            "int32[] values\nint32[4] fixed\nstring<=20 label\n",
        )
        size = max_serialized_size(spec, lambda _: None, default_seq_cap=8)
        # values: 4 + 8*4 = 36; fixed: 16; label: 4+21 = 25; all 4-aligned from 0
        self.assertEqual(size, 36 + 16 + 25)

    def test_set_bool_fits_lite_with_empty_unbounded_string(self):
        spec = parse_service_text(
            "std_srvs",
            "SetBool",
            "bool data\n---\nbool success\nstring message\n",
        )
        req = max_serialized_size(spec, lambda _: None, which="request")
        resp = max_serialized_size(spec, lambda _: None, which="response")
        self.assertEqual(req, 1)
        # success 1, pad to 4, string 4+1 (empty) = 9
        self.assertEqual(resp, 9)
        self.assertTrue(fits_lite(resp))

    def test_bounded_string_uses_declared_cap(self):
        spec = parse_message_text("demo", "Label", "string<=20 label\n")
        self.assertEqual(max_serialized_size(spec, lambda _: None), 25)


if __name__ == "__main__":
    unittest.main()
