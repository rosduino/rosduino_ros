# SPDX-License-Identifier: Apache-2.0
"""Discover interfaces, parse, render, and write headers."""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Set, Tuple

from rosduino_gen.ast import InterfaceSpec, StructureSpec
from rosduino_gen.hashutil import resolve_hash
from rosduino_gen.render import render_header
from rosduino_gen.types import DEFAULT_SEQ_CAP, parse_type
from rosduino_gen import rosidl_backend, simple_parser

ParseFn = Callable[[Path, str], InterfaceSpec]


def choose_parser(name: str) -> ParseFn:
    if name == "simple":
        return simple_parser.parse_file
    if name == "rosidl":
        if not rosidl_backend.available():
            raise RuntimeError("rosidl_parser is not installed")
        return rosidl_backend.parse_file
    if name == "auto":
        if rosidl_backend.available():
            return rosidl_backend.parse_file
        return simple_parser.parse_file
    raise ValueError(f"unknown parser {name!r}")


def discover_interfaces(msg_dir: Path) -> List[Path]:
    roots = []
    msg_sub = msg_dir / "msg"
    srv_sub = msg_dir / "srv"
    if msg_sub.is_dir():
        roots.append(msg_sub)
    if srv_sub.is_dir():
        roots.append(srv_sub)
    if not roots:
        roots.append(msg_dir)
    files: List[Path] = []
    for root in roots:
        files.extend(sorted(root.glob("*.msg")))
        files.extend(sorted(root.glob("*.srv")))
    files.sort(key=lambda p: (p.suffix, p.name))
    return files


def ros_available() -> bool:
    try:
        import ament_index_python.packages  # noqa: F401
        import ament_index_python.resources  # noqa: F401
    except ImportError:
        return False
    return True


def _share_has_interfaces(share: Path) -> bool:
    return (share / "msg").is_dir() or (share / "srv").is_dir()


def _interfaces_from_resource(package: str) -> List[str]:
    from ament_index_python.resources import get_resource

    content, _path = get_resource("rosidl_interfaces", package)
    listed: List[str] = []
    seen = set()
    for raw in content.splitlines():
        line = raw.strip()
        if not line or "/" not in line:
            continue
        kind, rest = line.split("/", 1)
        if kind not in ("msg", "srv"):
            continue
        stem = rest.rsplit("/", 1)[-1]
        if "." in stem:
            stem, ext = stem.rsplit(".", 1)
            if ext not in ("msg", "srv", "idl"):
                continue
        key = f"{package}/{kind}/{stem}"
        if key not in seen:
            seen.add(key)
            listed.append(key)
    return listed


def _interfaces_from_share(package: str, share: Path) -> List[str]:
    listed: List[str] = []
    for path in discover_interfaces(share):
        kind = path.parent.name if path.parent.name in ("msg", "srv") else path.suffix[1:]
        listed.append(f"{package}/{kind}/{path.stem}")
    return listed


def _require_ros() -> None:
    if not ros_available():
        raise RuntimeError(
            "ROS 2 is required; source /opt/ros/$ROS_DISTRO/setup.bash"
        )


def discover_packages(
    names: Sequence[str] | None = None,
    share_map: Sequence[Tuple[str, Path]] | None = None,
) -> List[Tuple[str, Path]]:
    """Return `(package, share_dir)` for packages that ship `msg/` or `srv/`."""
    if share_map is not None:
        items = [(name, Path(share)) for name, share in share_map]
        if names:
            wanted = set(names)
            items = [(name, share) for name, share in items if name in wanted]
    else:
        _require_ros()
        from ament_index_python.packages import get_package_share_directory
        from ament_index_python.resources import get_resources

        indexed = set(get_resources("rosidl_interfaces"))
        if names:
            wanted = list(dict.fromkeys(names))
            missing: List[str] = []
            selected: List[str] = []
            for name in wanted:
                if name in indexed:
                    selected.append(name)
                    continue
                try:
                    share = Path(get_package_share_directory(name))
                except Exception:
                    missing.append(name)
                    continue
                if _share_has_interfaces(share):
                    selected.append(name)
                else:
                    missing.append(name)
            if missing:
                raise RuntimeError("unknown package(s): " + ", ".join(sorted(set(missing))))
            items = []
            for name in selected:
                try:
                    items.append((name, Path(get_package_share_directory(name))))
                except Exception as exc:  # noqa: BLE001
                    raise RuntimeError(f"share directory for {name!r} not found") from exc
        else:
            items = []
            for name in sorted(indexed):
                try:
                    items.append((name, Path(get_package_share_directory(name))))
                except Exception:
                    continue
    result: List[Tuple[str, Path]] = []
    for name, share in sorted(items, key=lambda item: item[0]):
        if _share_has_interfaces(share):
            result.append((name, share))
    if names:
        found = {name for name, _ in result}
        empty = [name for name in names if name not in found]
        if empty:
            raise RuntimeError(
                "no msg/ or srv/ interfaces in: " + ", ".join(sorted(set(empty)))
            )
    return result


def parse_types_text(text: str) -> Tuple[List[str], List[str]]:
    """Return `(whole_packages, type_names)` from `arduino_types.txt`."""
    whole: List[str] = []
    types: List[str] = []
    seen_whole: Set[str] = set()
    seen_types: Set[str] = set()
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if "/" in line:
            parts = line.split("/")
            if len(parts) < 3:
                raise ValueError(f"type must be pkg/msg|srv/Name: {line}")
            pkg, kind, name = parts[0], parts[1], parts[2]
            if kind not in ("msg", "srv"):
                raise ValueError(f"type must be pkg/msg|srv/Name: {line}")
            if "." in name:
                name = name.rsplit(".", 1)[0]
            key = f"{pkg}/{kind}/{name}"
            if key not in seen_types:
                seen_types.add(key)
                types.append(key)
        else:
            if line not in seen_whole:
                seen_whole.add(line)
                whole.append(line)
    return whole, types


def load_types_file(path: Path) -> Tuple[List[str], List[str]]:
    return parse_types_text(path.read_text(encoding="utf-8"))


def types_package_names(whole_packages: Sequence[str], type_names: Sequence[str]) -> List[str]:
    names: List[str] = []
    seen: Set[str] = set()
    for name in [*whole_packages, *[item.split("/", 1)[0] for item in type_names]]:
        if name not in seen:
            seen.add(name)
            names.append(name)
    return names


def nested_type_keys(spec: InterfaceSpec, default_seq_cap: int) -> List[str]:
    keys: List[str] = []
    seen: Set[str] = set()

    def consider(type_expr: str) -> None:
        parsed = parse_type(type_expr, spec.package, default_seq_cap)
        while parsed.kind in ("array", "sequence") and parsed.elem is not None:
            parsed = parsed.elem
        if parsed.nested is None:
            return
        pkg, kind, name = parsed.nested
        key = f"{pkg}/{kind}/{name}"
        if key not in seen:
            seen.add(key)
            keys.append(key)

    def walk(structure: Optional[StructureSpec]) -> None:
        if structure is None:
            return
        for field in structure.fields:
            consider(field.type_expr)

    walk(spec.request)
    walk(spec.response)
    return keys


def expand_type_selection(
    catalog: Dict[str, InterfaceSpec],
    whole_packages: Sequence[str],
    type_names: Sequence[str],
    default_seq_cap: int = DEFAULT_SEQ_CAP,
) -> Tuple[Set[str], List[str]]:
    selected: Set[str] = set()
    whole = set(whole_packages)
    for spec in catalog.values():
        if spec.package in whole:
            selected.add(spec.type_name)
    missing = [name for name in type_names if name not in catalog]
    selected.update(name for name in type_names if name in catalog)
    pending = list(selected)
    while pending:
        key = pending.pop()
        spec = catalog.get(key)
        if spec is None:
            continue
        for dep in nested_type_keys(spec, default_seq_cap):
            if dep not in selected:
                selected.add(dep)
                pending.append(dep)
    return selected, missing


def list_interfaces(
    names: Sequence[str] | None = None,
    share_map: Sequence[Tuple[str, Path]] | None = None,
) -> List[str]:
    listed: List[str] = []
    use_index = share_map is None
    for package, share in discover_packages(names, share_map):
        if use_index:
            try:
                listed.extend(_interfaces_from_resource(package))
                continue
            except Exception:
                pass
        listed.extend(_interfaces_from_share(package, share))
    listed = sorted(set(listed))
    return listed


def _load_specs(
    package: str,
    msg_dir: Path,
    parse: ParseFn,
    warnings: List[str],
) -> List[InterfaceSpec]:
    specs: List[InterfaceSpec] = []
    for path in discover_interfaces(msg_dir):
        try:
            specs.append(parse(path, package))
        except Exception as exc:  # noqa: BLE001
            warnings.append(f"skip {package}/{path.name}: {exc}")
    return specs


def _emit_specs(
    package: str,
    out_dir: Path,
    search: Sequence[Path],
    specs: Sequence[InterfaceSpec],
    catalog: Dict[str, InterfaceSpec],
    default_seq_cap: int,
) -> Tuple[List[Path], List[str]]:
    warnings: List[str] = []
    msg_names = {spec.name for spec in specs if spec.kind == "msg"}
    srv_names = {spec.name for spec in specs if spec.kind == "srv"}
    conflicts = msg_names & srv_names
    for name in sorted(conflicts):
        warnings.append(f"skip short alias {package}::{name}: msg and srv share the name")
    written: List[Path] = []
    for spec in specs:
        digest, hash_warnings = resolve_hash(package, spec.kind, spec.name, search)
        warnings.extend(hash_warnings)
        try:
            text = render_header(
                spec,
                digest,
                default_seq_cap=default_seq_cap,
                emit_alias=spec.name not in conflicts,
                resolve=catalog.get,
            )
        except Exception as exc:  # noqa: BLE001
            warnings.append(f"skip {spec.type_name}: {exc}")
            continue
        dest = out_dir / package / spec.kind / f"{spec.name}.h"
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(text, encoding="utf-8", newline="\n")
        written.append(dest)
    return written, warnings


def generate_package(
    package: str,
    out_dir: Path,
    msg_dir: Path,
    parser_name: str = "auto",
    share_dirs: Sequence[Path] = (),
    default_seq_cap: int = DEFAULT_SEQ_CAP,
    catalog: Optional[Dict[str, InterfaceSpec]] = None,
) -> Tuple[List[Path], List[str]]:
    parse = choose_parser(parser_name)
    search = [msg_dir, *share_dirs]
    warnings: List[str] = []
    specs = _load_specs(package, msg_dir, parse, warnings)
    combined: Dict[str, InterfaceSpec] = dict(catalog or {})
    combined.update({spec.type_name: spec for spec in specs})
    written, more = _emit_specs(
        package, out_dir, search, specs, combined, default_seq_cap
    )
    warnings.extend(more)
    return written, warnings


def generate_packages(
    packages: Sequence[Tuple[str, Path]],
    out_dir: Path,
    parser_name: str = "auto",
    default_seq_cap: int = DEFAULT_SEQ_CAP,
    whole_packages: Optional[Sequence[str]] = None,
    type_names: Optional[Sequence[str]] = None,
) -> Tuple[List[Path], List[str]]:
    parse = choose_parser(parser_name)
    warnings: List[str] = []
    catalog: Dict[str, InterfaceSpec] = {}
    jobs: List[Tuple[str, Path, List[InterfaceSpec]]] = []
    for package, share in packages:
        specs = _load_specs(package, share, parse, warnings)
        for spec in specs:
            catalog[spec.type_name] = spec
        jobs.append((package, share, specs))
    selected: Optional[Set[str]] = None
    if whole_packages is not None or type_names is not None:
        selected, missing = expand_type_selection(
            catalog,
            whole_packages or (),
            type_names or (),
            default_seq_cap,
        )
        for name in missing:
            warnings.append(f"type not found: {name}")
    written: List[Path] = []
    for package, share, specs in jobs:
        emit = specs if selected is None else [spec for spec in specs if spec.type_name in selected]
        if not emit:
            continue
        files, more = _emit_specs(
            package, out_dir, [share], emit, catalog, default_seq_cap
        )
        written.extend(files)
        warnings.extend(more)
    return written, warnings
