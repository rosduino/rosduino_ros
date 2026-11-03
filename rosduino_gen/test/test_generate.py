# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from rosduino_gen.ast import FieldSpec, InterfaceSpec, StructureSpec
from rosduino_gen.generate import (
    discover_packages,
    generate_package,
    generate_packages,
    list_interfaces,
    load_types_file,
    parse_types_text,
)
from rosduino_gen.render import GENERATED_BANNER, render_header
from rosduino_gen.simple_parser import parse_message_text, parse_service_text

FIXTURES = Path(__file__).resolve().parent / "fixtures"


class GenerateTests(unittest.TestCase):
    def test_int32_header_is_deterministic_and_hashed(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            first, warnings = generate_package(
                "std_msgs",
                out,
                FIXTURES / "std_msgs",
                parser_name="simple",
                share_dirs=[FIXTURES / "std_msgs"],
            )
            self.assertFalse(any("Int32" in w for w in warnings))
            self.assertGreaterEqual(len(first), 2)
            header = out / "std_msgs" / "msg" / "Int32.h"
            text = header.read_text(encoding="utf-8")
            self.assertIn(GENERATED_BANNER, text)
            self.assertIn("Do not edit.", text)
            self.assertIn("std_msgs/msg/Int32", text)
            self.assertIn("0xb6U", text)
            self.assertIn("0x26U", text)
            self.assertIn("using msg::Int32;", text)
            self.assertIn("int32_t data;", text)
            self.assertIn("writer.writeInt32(data)", text)
            self.assertIn("reader.readInt32(&data)", text)
            self.assertIn("maxSerializedSize() { return 4U; }", text)
            self.assertIn("ROSDUINO_FITS_LITE = true", text)
            self.assertIn("// std_msgs/msg/Int32", text)
            self.assertIn("// int32 data", text)
            again, _ = generate_package(
                "std_msgs",
                out,
                FIXTURES / "std_msgs",
                parser_name="simple",
                share_dirs=[FIXTURES / "std_msgs"],
            )
            self.assertEqual(header.read_text(encoding="utf-8"), text)
            self.assertTrue(all(p.exists() for p in again))

    def test_missing_hash_warns_and_writes_zeros(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            _, warnings = generate_package(
                "std_srvs",
                out,
                FIXTURES / "std_srvs",
                parser_name="simple",
            )
            self.assertTrue(any("std_srvs/srv/SetBool" in w for w in warnings))
            text = (out / "std_srvs" / "srv" / "SetBool.h").read_text(encoding="utf-8")
            self.assertIn("0x00U", text)
            self.assertIn("struct Request", text)
            self.assertIn("struct Response", text)
            self.assertIn("bool data;", text)
            self.assertIn("bool success;", text)
            self.assertIn("const char* message;", text)
            self.assertIn("using srv::SetBool;", text)
            self.assertIn("// std_srvs/srv/SetBool", text)
            self.assertIn("// bool data # e.g. for hardware enabling / disabling", text)
            self.assertIn("// ---", text)
            self.assertIn("// bool success   # indicate successful run of triggered service", text)
            self.assertIn("// string message # informational, e.g. for error messages", text)

    def test_list_interfaces_from_share_map(self):
        share_map = (
            ("std_msgs", FIXTURES / "std_msgs"),
            ("std_srvs", FIXTURES / "std_srvs"),
            ("sensor_msgs", FIXTURES / "sensor_msgs"),
        )
        names = list_interfaces(share_map=share_map)
        self.assertIn("std_msgs/msg/Bool", names)
        self.assertIn("std_msgs/msg/Int32", names)
        self.assertIn("std_srvs/srv/SetBool", names)
        self.assertIn("sensor_msgs/msg/Range", names)

    def test_generate_packages_from_share_map(self):
        with tempfile.TemporaryDirectory() as tmp:
            packages = discover_packages(
                share_map=(("std_srvs", FIXTURES / "std_srvs"),)
            )
            written, warnings = generate_packages(packages, Path(tmp), parser_name="simple")
            self.assertTrue(any("SetBool" in w for w in warnings))
            self.assertTrue(any(p.name == "SetBool.h" for p in written))

    def test_discover_packages_share_map_skips_empty(self):
        with tempfile.TemporaryDirectory() as tmp:
            empty = Path(tmp) / "empty"
            empty.mkdir()
            found = discover_packages(
                share_map=(
                    ("empty", empty),
                    ("std_msgs", FIXTURES / "std_msgs"),
                )
            )
            self.assertEqual([name for name, _ in found], ["std_msgs"])

    def test_list_interfaces_filters_by_name(self):
        share_map = (
            ("std_msgs", FIXTURES / "std_msgs"),
            ("std_srvs", FIXTURES / "std_srvs"),
        )
        names = list_interfaces(["std_srvs"], share_map=share_map)
        self.assertEqual(names, ["std_srvs/srv/SetBool"])

    def test_range_maps_nested_fields_and_constants(self):
        with tempfile.TemporaryDirectory() as tmp:
            written, _ = generate_package(
                "sensor_msgs",
                Path(tmp),
                FIXTURES / "sensor_msgs",
                parser_name="simple",
            )
            text = (Path(tmp) / "sensor_msgs" / "msg" / "Range.h").read_text(encoding="utf-8")
            self.assertTrue(any(p.name == "Range.h" for p in written))
            self.assertIn("#include <std_msgs/msg/Header.h>", text)
            self.assertIn("static const uint8_t ULTRASOUND = 0U;", text)
            self.assertIn("static const uint8_t INFRARED = 1U;", text)
            self.assertIn("std_msgs::msg::Header header;", text)
            self.assertIn("header.serialize(writer)", text)
            self.assertIn("header.deserialize(reader)", text)
            self.assertIn("writer.writeFloat32(range)", text)
            self.assertIn("ROSDUINO_FITS_LITE", text)
            self.assertIn("// # Radiation type constants.", text)
            self.assertIn("// uint8 ULTRASOUND=0", text)

    def test_arrays_header_uses_sequence(self):
        spec = parse_message_text(
            "demo",
            "Arrays",
            "int32[] values\nint32[4] fixed\nint32[<=8] bounded\nstring<=20 label\n",
        )
        text = render_header(spec, bytes(8))
        self.assertIn("rosduino::Sequence<int32_t, 8U> values;", text)
        self.assertIn("int32_t fixed[4U];", text)
        self.assertIn("rosduino::Sequence<int32_t, 8U> bounded;", text)
        self.assertIn("const char* label;", text)
        self.assertIn("writer.writeArray(fixed, 4U,", text)
        self.assertIn("writer.writeSequence(values,", text)

    def test_field_names_do_not_shadow_cdr_params(self):
        quat = parse_message_text("demo", "Quat", "float64 x\nfloat64 y\nfloat64 z\nfloat64 w\n")
        quat_text = render_header(quat, bytes(8))
        self.assertIn("writer.writeFloat64(w)", quat_text)
        self.assertNotIn("CdrWriter& w)", quat_text)
        color = parse_message_text(
            "demo", "Color", "float32 r\nfloat32 g\nfloat32 b\nfloat32 a\n"
        )
        color_text = render_header(color, bytes(8))
        self.assertIn("reader.readFloat32(&r)", color_text)
        self.assertNotIn("CdrReader& r)", color_text)

    def test_header_embeds_msg_source_as_comments(self):
        spec = parse_message_text("std_msgs", "Int32", "int32 data  # payload \n")
        text = render_header(spec, bytes(8))
        banner_at = text.find(f"// {GENERATED_BANNER}")
        guard_at = text.find("#ifndef")
        self.assertGreaterEqual(banner_at, 0)
        self.assertGreater(guard_at, banner_at)
        self.assertIn("// std_msgs/msg/Int32", text)
        self.assertIn("// int32 data  # payload", text)
        self.assertNotIn("/*", text[banner_at:guard_at])

    def test_header_embeds_srv_source_and_separator(self):
        spec = parse_service_text(
            "std_srvs",
            "SetBool",
            "bool data # e.g. for hardware\n---\nbool success\nstring message # error\n",
        )
        text = render_header(spec, bytes(8))
        self.assertIn("// std_srvs/srv/SetBool", text)
        self.assertIn("// bool data # e.g. for hardware", text)
        self.assertIn("// ---", text)
        self.assertIn("// string message # error", text)

    def test_header_reconstructs_source_when_ast_has_no_file(self):
        spec = InterfaceSpec(
            package="demo",
            name="Pair",
            kind="msg",
            request=StructureSpec(
                fields=(FieldSpec("int32", "a"), FieldSpec("int32", "b")),
                constants=(),
            ),
        )
        text = render_header(spec, bytes(8))
        self.assertIn("// demo/msg/Pair", text)
        self.assertIn("// int32 a", text)
        self.assertIn("// int32 b", text)
        self.assertNotIn("#", text.split("// demo/msg/Pair", 1)[1].split("#ifndef", 1)[0])

    def test_empty_message_has_dummy_member(self):
        spec = parse_message_text("std_msgs", "Empty", "")
        text = render_header(spec, bytes(8))
        self.assertIn("structure_needs_at_least_one_member", text)
        self.assertIn("return 1U;", text)

    def test_nested_catalog_counts_header_in_range_size(self):
        from rosduino_gen.simple_parser import parse_file

        catalog = {}
        for pkg, rel in (
            ("builtin_interfaces", "builtin_interfaces/msg/Time.msg"),
            ("std_msgs", "std_msgs/msg/Header.msg"),
            ("sensor_msgs", "sensor_msgs/msg/Range.msg"),
        ):
            path = FIXTURES / rel
            spec = parse_file(path, pkg)
            catalog[spec.type_name] = spec
        range_spec = catalog["sensor_msgs/msg/Range"]
        text = render_header(range_spec, bytes(8), resolve=catalog.get)
        self.assertIn("maxSerializedSize() { return 36U; }", text)
        self.assertIn("ROSDUINO_FITS_LITE = true", text)

    def test_arduino_types_file_lists_spec_types(self):
        path = Path(__file__).resolve().parents[1] / "arduino_types.txt"
        whole, types = load_types_file(path)
        self.assertIn("std_msgs", whole)
        self.assertIn("example_interfaces", whole)
        self.assertIn("geometry_msgs/msg/PoseStamped", types)
        self.assertIn("nav_msgs/msg/Odometry", types)

    def test_parse_types_text_splits_packages_and_types(self):
        whole, types = parse_types_text(
            "# comment\n"
            "std_msgs\n"
            "sensor_msgs/msg/Range\n"
            "std_srvs/srv/SetBool.srv\n"
            "\n"
        )
        self.assertEqual(whole, ["std_msgs"])
        self.assertEqual(types, ["sensor_msgs/msg/Range", "std_srvs/srv/SetBool"])

    def test_selection_emits_nested_deps_not_whole_sensor_package(self):
        with tempfile.TemporaryDirectory() as tmp:
            packages = discover_packages(
                share_map=(
                    ("builtin_interfaces", FIXTURES / "builtin_interfaces"),
                    ("std_msgs", FIXTURES / "std_msgs"),
                    ("sensor_msgs", FIXTURES / "sensor_msgs"),
                )
            )
            written, warnings = generate_packages(
                packages,
                Path(tmp),
                parser_name="simple",
                whole_packages=("builtin_interfaces", "std_msgs"),
                type_names=("sensor_msgs/msg/Range",),
            )
            names = {p.name for p in written}
            self.assertIn("Range.h", names)
            self.assertIn("Header.h", names)
            self.assertIn("Time.h", names)
            self.assertIn("Int32.h", names)
            self.assertFalse(any("type not found" in w for w in warnings))
            range_text = (Path(tmp) / "sensor_msgs" / "msg" / "Range.h").read_text(
                encoding="utf-8"
            )
            self.assertIn("Generated by rosduino-gen. Do not edit.", range_text)
            self.assertNotIn("TEMPORARY", range_text)


if __name__ == "__main__":
    unittest.main()
