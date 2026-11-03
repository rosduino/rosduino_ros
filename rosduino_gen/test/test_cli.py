# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import io
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from rosduino_gen.cli import default_out_dir, main
from rosduino_gen.generate import ros_available


class CliTests(unittest.TestCase):
    def test_requires_package_or_all_or_list(self):
        with patch("sys.stderr", io.StringIO()):
            self.assertEqual(main([]), 2)

    def test_default_out_is_package_generated(self):
        out = default_out_dir()
        self.assertEqual(out.name, "generated")
        self.assertTrue((out.parent / "package.xml").is_file())

    def test_package_without_out_uses_default_dir(self):
        with patch("rosduino_gen.cli.discover_packages", return_value=[]), patch(
            "rosduino_gen.cli.generate_packages", return_value=([], [])
        ) as generate, patch("sys.stdout", io.StringIO()), patch("sys.stderr", io.StringIO()):
            rc = main(["--package", "std_msgs"])
        self.assertEqual(rc, 0)
        self.assertEqual(generate.call_args[0][1], default_out_dir())

    def test_all_and_package_are_exclusive(self):
        with patch("sys.stderr", io.StringIO()):
            self.assertEqual(main(["--all", "--package", "std_msgs", "--out", "/tmp"]), 2)

    def test_list_without_ros_errors(self):
        if ros_available():
            self.skipTest("ROS 2 is sourced")
        buf = io.StringIO()
        with patch("sys.stderr", buf):
            self.assertEqual(main(["--list"]), 1)
        self.assertIn("ROS 2 is required", buf.getvalue())

    def test_list_std_msgs_when_ros_present(self):
        if not ros_available():
            self.skipTest("ROS 2 is not sourced")
        buf = io.StringIO()
        with patch("sys.stdout", buf):
            rc = main(["--list", "--package", "std_msgs"])
        self.assertEqual(rc, 0)
        names = buf.getvalue().splitlines()
        self.assertIn("std_msgs/msg/Bool", names)
        self.assertIn("std_msgs/msg/Int32", names)
        self.assertTrue(all(name.startswith("std_msgs/") for name in names))

    def test_list_all_includes_std_msgs_when_ros_present(self):
        if not ros_available():
            self.skipTest("ROS 2 is not sourced")
        buf = io.StringIO()
        with patch("sys.stdout", buf):
            rc = main(["--list"])
        self.assertEqual(rc, 0)
        names = buf.getvalue().splitlines()
        self.assertIn("std_msgs/msg/Bool", names)
        self.assertIn("std_srvs/srv/SetBool", names)

    def test_generate_std_msgs_when_ros_present(self):
        if not ros_available():
            self.skipTest("ROS 2 is not sourced")
        with tempfile.TemporaryDirectory() as tmp:
            with patch("sys.stdout", io.StringIO()), patch("sys.stderr", io.StringIO()):
                rc = main(["--package", "std_msgs", "--out", tmp, "--parser", "simple"])
            self.assertEqual(rc, 0)
            self.assertTrue((Path(tmp) / "std_msgs" / "msg" / "Bool.h").is_file())


if __name__ == "__main__":
    unittest.main()
