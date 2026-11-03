# SPDX-License-Identifier: Apache-2.0
"""Load the first 8 RIHS01 bytes from share/<pkg>/msg|srv/<Type>.json."""

from __future__ import annotations

import json
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

ZERO_HASH = bytes(8)


def parse_rihs_hash(hash_string: str) -> bytes:
    if not hash_string.startswith("RIHS01_"):
        raise ValueError(f"unsupported type hash string: {hash_string!r}")
    hexpart = hash_string.split("_", 1)[1]
    raw = bytes.fromhex(hexpart)
    if len(raw) < 8:
        raise ValueError(f"RIHS hash shorter than 8 bytes: {hash_string!r}")
    return raw[:8]


def load_hash_from_json(path: Path) -> bytes:
    data = json.loads(path.read_text(encoding="utf-8"))
    entries = data.get("type_hashes") or []
    if not entries:
        raise ValueError(f"{path}: missing type_hashes")
    hash_string = entries[0].get("hash_string")
    if not hash_string:
        raise ValueError(f"{path}: missing hash_string")
    return parse_rihs_hash(hash_string)


def find_hash_json(
    package: str, kind: str, name: str, search_dirs: Sequence[Path]
) -> Optional[Path]:
    rel = Path(package) / kind / f"{name}.json"
    candidates = [
        Path(root) / kind / f"{name}.json"
        for root in search_dirs
    ]
    candidates.extend(Path(root) / rel for root in search_dirs)
    for path in candidates:
        if path.is_file():
            return path
    return None


def resolve_hash(
    package: str, kind: str, name: str, search_dirs: Sequence[Path]
) -> Tuple[bytes, List[str]]:
    warnings: List[str] = []
    path = find_hash_json(package, kind, name, search_dirs)
    type_name = f"{package}/{kind}/{name}"
    if path is None:
        warnings.append(f"no RIHS hash for {type_name}, using 0")
        return ZERO_HASH, warnings
    try:
        return load_hash_from_json(path), warnings
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        warnings.append(f"invalid RIHS hash for {type_name} ({exc}), using 0")
        return ZERO_HASH, warnings


def format_c_bytes(data: bytes) -> str:
    items = [f"0x{b:02x}U" for b in data]
    return ", ".join(items[:4]) + ",\n       " + ", ".join(items[4:])
