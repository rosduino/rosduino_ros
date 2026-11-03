# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import unittest
from pathlib import Path

from rosduino_gen.hashutil import ZERO_HASH, parse_rihs_hash, resolve_hash

FIXTURES = Path(__file__).resolve().parent / "fixtures"


class HashTests(unittest.TestCase):
    def test_parse_int32_rihs(self):
        digest = parse_rihs_hash(
            "RIHS01_b6578ded3c58c626000000000000000000000000000000000000000000000000"
        )
        self.assertEqual(
            digest,
            bytes([0xB6, 0x57, 0x8D, 0xED, 0x3C, 0x58, 0xC6, 0x26]),
        )

    def test_resolve_from_fixture_json(self):
        digest, warnings = resolve_hash(
            "std_msgs", "msg", "Int32", [FIXTURES / "std_msgs"]
        )
        self.assertEqual(warnings, [])
        self.assertEqual(digest[0], 0xB6)
        self.assertEqual(digest[7], 0x26)

    def test_missing_json_is_zero_with_warning(self):
        digest, warnings = resolve_hash("demo", "msg", "Missing", [FIXTURES])
        self.assertEqual(digest, ZERO_HASH)
        self.assertEqual(len(warnings), 1)
        self.assertIn("demo/msg/Missing", warnings[0])


if __name__ == "__main__":
    unittest.main()
