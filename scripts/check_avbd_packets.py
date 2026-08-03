#!/usr/bin/env python3
"""Validate committed AVBD evidence packets against the shared schema.

Enforces PLAN-091 WP-091.1: AVBD packets at the current schema version
must machine-record the resolved solver configuration and the rigid-contact
selection source that actually ran (``resolved_solver_identity``). Packets
committed before the identity contract stay readable through a legacy
allowlist, but new packet files must be written at the current schema version.
The field contract lives in ``scripts/avbd_packet_schema.py``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
from collections.abc import Mapping
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from avbd_packet_schema import (  # noqa: E402
    AVBD_PACKET_SCHEMA_VERSION,
    PAPER_PACKET_SOURCE_PATHS,
    packet_schema_version_errors,
    resolved_solver_identity_errors,
)

__all__ = [
    "AVBD_PACKET_SCHEMA_VERSION",
    "packet_errors",
]
from capture_source_provenance import (  # noqa: E402
    CAPTURE_SOURCE_PROVENANCE_ALGORITHM,
    CaptureSourceProvenanceError,
    compute_capture_source_provenance,
)

REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_PACKET_DIR = REPO_ROOT / "docs" / "plans" / "104-vertex-block-descent-solver"
PACKET_GLOB = "avbd-*-packet.json"
SOURCE_PROVENANCE_ALGORITHM = "sha256-length-prefixed-path-and-content-v1"
LINKED_PACKET_KEYS = ("linked_avbd_evidence", "linked_avbd_vbd_evidence")
BENCHMARK_SOURCE_PATH = Path("tests/benchmark/simulation/bm_avbd_rigid_fixed_joint.cpp")
PAPER_CAPTURE_ROLES = {
    "avbd-paper-breakable-wall-packet.json": ("impact", "outcome"),
    "avbd-paper-vbd-comparison-packet.json": ("bend", "retention"),
    "avbd-paper-sequential-impulse-comparison-packet.json": (
        "fracture",
        "collapse",
    ),
}
PAPER_PACKET_SELF_METHOD_NAMES = {
    "avbd-paper-breakable-wall-packet.json": "avbd",
    "avbd-paper-vbd-comparison-packet.json": "vbd",
    "avbd-paper-sequential-impulse-comparison-packet.json": ("sequential_impulse"),
}
PAPER_REQUIRED_MEDIAN_RATIO_KEYS = {
    "avbd-paper-vbd-comparison-packet.json": frozenset(
        {"vbd_to_avbd_median_cpu_cost_ratio"}
    ),
    "avbd-paper-sequential-impulse-comparison-packet.json": frozenset(
        {
            "sequential_impulse_to_avbd_median_cpu_cost_ratio",
            "sequential_impulse_to_vbd_median_cpu_cost_ratio",
        }
    ),
}

# Packets committed before the resolved-solver-identity contract
# (WP-091.1). They remain readable at schema_version 1; their
# sequential-impulse contact rows are relabeled in prose instead of
# being rewritten. Do not add new packets here: new packet files must use the
# current AVBD_PACKET_SCHEMA_VERSION with a recorded identity.
LEGACY_IDENTITY_EXEMPT_PACKETS = frozenset(
    {
        "avbd-articulated-breakable-joint-packet.json",
        "avbd-articulated-breakable-motor-packet.json",
        "avbd-articulated-fixed-pair-breakable-joint-packet.json",
        "avbd-articulated-high-ratio-chain-packet.json",
        "avbd-articulated-prismatic-motor-packet.json",
        "avbd-articulated-prismatic-pair-breakable-motor-packet.json",
        "avbd-articulated-revolute-motor-packet.json",
        "avbd-articulated-spherical-breakable-joint-packet.json",
        "avbd-articulated-spherical-pair-breakable-joint-packet.json",
        "avbd-articulated-world-prismatic-breakable-motor-packet.json",
        "avbd-articulated-world-revolute-breakable-motor-packet.json",
        "avbd-demo2d-cards-packet.json",
        "avbd-demo2d-dynamic-friction-packet.json",
        "avbd-demo2d-fracture-packet.json",
        "avbd-demo2d-ground-packet.json",
        "avbd-demo2d-hanging-rope-packet.json",
        "avbd-demo2d-heavy-rope-packet.json",
        "avbd-demo2d-joint-grid-packet.json",
        "avbd-demo2d-motor-packet.json",
        "avbd-demo2d-net-packet.json",
        "avbd-demo2d-pyramid-packet.json",
        "avbd-demo2d-rod-packet.json",
        "avbd-demo2d-rope-packet.json",
        "avbd-demo2d-soft-body-packet.json",
        "avbd-demo2d-spring-packet.json",
        "avbd-demo2d-spring-ratio-packet.json",
        "avbd-demo2d-stack-packet.json",
        "avbd-demo2d-stack-ratio-packet.json",
        "avbd-demo2d-static-friction-packet.json",
        "avbd-demo3d-breakable-packet.json",
        "avbd-demo3d-bridge-packet.json",
        "avbd-demo3d-dynamic-friction-packet.json",
        "avbd-demo3d-ground-packet.json",
        "avbd-demo3d-heavy-rope-packet.json",
        "avbd-demo3d-pyramid-packet.json",
        "avbd-demo3d-rope-packet.json",
        "avbd-demo3d-soft-body-packet.json",
        "avbd-demo3d-spring-packet.json",
        "avbd-demo3d-spring-ratio-packet.json",
        "avbd-demo3d-stack-packet.json",
        "avbd-demo3d-stack-ratio-packet.json",
        "avbd-demo3d-static-friction-packet.json",
        "avbd-empty-baseline-packet.json",
        "avbd-paper-scale-high-ratio-chain-packet.json",
        "avbd-rigid-breakable-joint-packet.json",
        "avbd-rigid-prismatic-motor-packet.json",
        "avbd-rigid-revolute-motor-packet.json",
        "avbd-rigid-spherical-breakable-joint-packet.json",
    }
)

# These schema-version 3 packets were committed before schema version 4 moved
# public hard pair rows from the private AVBD compatibility projection into the
# Sequential Impulse family. Their writers now emit the current schema when
# regenerated. Keep only these exact historical names readable; a new filename
# must use AVBD_PACKET_SCHEMA_VERSION so it cannot claim the retired v3 solver
# identity contract.
LEGACY_PRE_SI_PAIR_ROW_PACKETS = frozenset(
    {
        "avbd-articulated-compliant-fracture-packet.json",
        "avbd-articulated-compliant-joints-packet.json",
        "avbd-articulated-compliant-motors-packet.json",
        "avbd-breakable-joint-scale-packet.json",
        "avbd-breakable-motor-scale-packet.json",
        "avbd-friction-coefficient-sweep-packet.json",
        "avbd-paper-scale-high-ratio-iteration-sweep-packet.json",
    }
)

# Pin each historical filename to the one legacy version it was committed
# with. A legacy filename may move directly to the current schema when its
# packet is regenerated, but it may not claim any other retired contract.
LEGACY_PACKET_SCHEMA_VERSIONS = {
    **{name: 1 for name in LEGACY_IDENTITY_EXEMPT_PACKETS},
    **{name: 3 for name in LEGACY_PRE_SI_PAIR_ROW_PACKETS},
}
LEGACY_SCHEMA_EXEMPT_PACKETS = frozenset(LEGACY_PACKET_SCHEMA_VERSIONS)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--packet",
        action="append",
        type=Path,
        default=None,
        help="Explicit packet file to validate (repeatable); defaults to "
        "every avbd-*-packet.json under --packet-dir.",
    )
    parser.add_argument(
        "--packet-dir",
        type=Path,
        default=DEFAULT_PACKET_DIR,
        help="Directory scanned for avbd-*-packet.json files.",
    )
    return parser.parse_args(argv)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _safe_relative_path(value: object) -> Path | None:
    if not isinstance(value, str) or not value:
        return None
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or path.as_posix() != value:
        return None
    return path


def _source_provenance_errors(
    packet: Mapping[str, object], packet_name: str
) -> list[str]:
    provenance = packet.get("source_provenance")
    if provenance is None:
        if packet_name in PAPER_CAPTURE_ROLES:
            return [
                f"{packet_name}: source_provenance must be an object for a "
                "current paper packet"
            ]
        return []
    if not isinstance(provenance, Mapping):
        return [f"{packet_name}: source_provenance must be an object"]

    errors: list[str] = []
    if provenance.get("algorithm") != SOURCE_PROVENANCE_ALGORITHM:
        errors.append(
            f"{packet_name}: source_provenance.algorithm must be "
            f"{SOURCE_PROVENANCE_ALGORITHM!r}"
        )
    files = provenance.get("files")
    if not isinstance(files, list) or not files:
        return errors + [
            f"{packet_name}: source_provenance.files must be a non-empty list"
        ]

    required_paths = PAPER_PACKET_SOURCE_PATHS.get(packet_name)
    actual_paths = [
        entry.get("path") if isinstance(entry, Mapping) else None for entry in files
    ]
    if required_paths is not None and actual_paths != list(required_paths):
        errors.append(
            f"{packet_name}: source_provenance.files paths must exactly match "
            "the canonical ordered paper-packet source contract"
        )

    combined = hashlib.sha256()
    seen: set[str] = set()
    for index, entry in enumerate(files):
        label = f"{packet_name}: source_provenance.files[{index}]"
        if not isinstance(entry, Mapping):
            errors.append(f"{label} must be an object")
            continue
        relative = _safe_relative_path(entry.get("path"))
        if relative is None:
            errors.append(f"{label}.path must be a safe repository-relative path")
            continue
        relative_text = relative.as_posix()
        if relative_text in seen:
            errors.append(f"{label}.path duplicates {relative_text!r}")
            continue
        seen.add(relative_text)
        source_path = REPO_ROOT / relative
        try:
            payload = source_path.read_bytes()
        except FileNotFoundError:
            errors.append(f"{label}.path does not exist: {relative_text}")
            continue

        current_hash = hashlib.sha256(payload).hexdigest()
        if entry.get("sha256") != current_hash:
            errors.append(
                f"{label}.sha256 drifted for {relative_text}: expected "
                f"{current_hash}"
            )
        encoded_path = relative_text.encode("utf-8")
        combined.update(struct.pack("<Q", len(encoded_path)))
        combined.update(encoded_path)
        combined.update(struct.pack("<Q", len(payload)))
        combined.update(payload)

    if provenance.get("digest") != combined.hexdigest():
        errors.append(
            f"{packet_name}: source_provenance.digest does not match current "
            "listed source contents"
        )
    return errors


def _paper_capture_source_binding_errors(
    packet: Mapping[str, object],
    packet_name: str,
) -> list[str]:
    roles = PAPER_CAPTURE_ROLES.get(packet_name)
    if roles is None:
        return []

    try:
        current = compute_capture_source_provenance(REPO_ROOT)
    except CaptureSourceProvenanceError as exc:
        return [
            f"{packet_name}: cannot resolve current capture source provenance: "
            f"{exc}"
        ]

    errors: list[str] = []
    visual = packet.get("visual_evidence")
    if not isinstance(visual, Mapping):
        return [f"{packet_name}: visual_evidence must be an object"]
    for role in roles:
        capture = visual.get(role)
        label = f"{packet_name}: visual_evidence.{role}.source_provenance"
        if not isinstance(capture, Mapping):
            errors.append(f"{packet_name}: visual_evidence.{role} must be an object")
            continue
        provenance = capture.get("source_provenance")
        if not isinstance(provenance, Mapping):
            errors.append(f"{label} must be an object")
            continue
        if provenance.get("algorithm") != CAPTURE_SOURCE_PROVENANCE_ALGORITHM:
            errors.append(
                f"{label}.algorithm must be " f"{CAPTURE_SOURCE_PROVENANCE_ALGORITHM!r}"
            )
        for key in ("digest", "file_count", "roots"):
            if provenance.get(key) != current[key]:
                errors.append(f"{label}.{key} does not match current source state")
        git_head = provenance.get("git_head")
        if (
            not isinstance(git_head, str)
            or len(git_head) != 40
            or any(character not in "0123456789abcdef" for character in git_head)
        ):
            errors.append(f"{label}.git_head must be a lowercase Git object ID")

    benchmark = packet.get("benchmark")
    if not isinstance(benchmark, Mapping):
        return errors + [f"{packet_name}: benchmark must be an object"]
    benchmark_provenance = benchmark.get("source_provenance")
    if not isinstance(benchmark_provenance, Mapping):
        errors.append(f"{packet_name}: benchmark.source_provenance must be an object")
        return errors
    if (
        benchmark_provenance.get("capture_source_provenance_digest")
        != current["digest"]
    ):
        errors.append(
            f"{packet_name}: benchmark source capture digest does not match "
            "current source state"
        )
    try:
        benchmark_source_hash = _sha256(REPO_ROOT / BENCHMARK_SOURCE_PATH)
    except FileNotFoundError:
        errors.append(
            f"{packet_name}: benchmark source file not found: "
            f"{BENCHMARK_SOURCE_PATH.as_posix()}"
        )
        return errors
    if benchmark_provenance.get("benchmark_source_sha256") != benchmark_source_hash:
        errors.append(
            f"{packet_name}: benchmark source hash does not match current "
            "benchmark translation unit"
        )

    context = benchmark.get("context")
    if not isinstance(context, Mapping):
        errors.append(f"{packet_name}: benchmark.context must be an object")
        return errors
    for key in (
        "capture_source_provenance_digest",
        "benchmark_source_sha256",
    ):
        if context.get(key) != benchmark_provenance.get(key):
            errors.append(
                f"{packet_name}: benchmark.context.{key} does not match "
                "benchmark.source_provenance"
            )
    return errors


def _linked_packet_errors(
    packet: Mapping[str, object],
    path: Path,
    visited: set[Path],
) -> list[str]:
    errors: list[str] = []
    for key in LINKED_PACKET_KEYS:
        link = packet.get(key)
        if link is None:
            continue
        if not isinstance(link, Mapping):
            errors.append(f"{path.name}: {key} must be an object")
            continue
        relative = _safe_relative_path(link.get("file"))
        if relative is None or len(relative.parts) != 1:
            errors.append(f"{path.name}: {key}.file must name one sibling packet file")
            continue
        linked_path = (path.parent / relative).resolve()
        try:
            linked_hash = _sha256(linked_path)
        except FileNotFoundError:
            errors.append(f"{path.name}: {key}.file not found: {relative.as_posix()}")
            continue
        if link.get("sha256") != linked_hash:
            errors.append(
                f"{path.name}: {key}.sha256 drifted for {relative.as_posix()}"
            )
        try:
            linked_packet = json.loads(linked_path.read_text())
        except json.JSONDecodeError as exc:
            errors.append(f"{path.name}: {key}.file has invalid JSON ({exc})")
            continue
        if not isinstance(linked_packet, dict):
            errors.append(f"{path.name}: {key}.file must contain a JSON object")
            continue
        expected_provenance_digest = link.get("source_provenance_digest")
        if expected_provenance_digest is not None:
            linked_provenance = linked_packet.get("source_provenance")
            actual_provenance_digest = (
                linked_provenance.get("digest")
                if isinstance(linked_provenance, Mapping)
                else None
            )
            if expected_provenance_digest != actual_provenance_digest:
                errors.append(
                    f"{path.name}: {key}.source_provenance_digest does not "
                    f"match {relative.as_posix()}"
                )
        errors.extend(_packet_errors(linked_path, visited))
    return errors


def _packet_errors(path: Path, visited: set[Path]) -> list[str]:
    path = path.resolve()
    if path in visited:
        return []
    visited.add(path)
    name = path.name
    if not path.is_file():
        return [f"{name}: packet file not found at {path}"]
    try:
        packet = json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        return [f"{name}: invalid JSON ({exc})"]
    if not isinstance(packet, dict):
        return [f"{name}: packet must be a JSON object"]

    errors = packet_schema_version_errors(packet, name)
    if errors:
        return errors

    version = packet["schema_version"]
    if version < AVBD_PACKET_SCHEMA_VERSION:
        expected_legacy_version = LEGACY_PACKET_SCHEMA_VERSIONS.get(name)
        if expected_legacy_version is None:
            errors.append(
                f"{name}: new AVBD packets must be written at schema_version "
                f"{AVBD_PACKET_SCHEMA_VERSION} with a "
                "recorded resolved_solver_identity (legacy allowlist covers "
                "only packets committed before the current schema contract)"
            )
        elif version != expected_legacy_version:
            errors.append(
                f"{name}: legacy allowlist requires schema_version "
                f"{expected_legacy_version}, got {version}; otherwise regenerate "
                f"the packet at current schema_version {AVBD_PACKET_SCHEMA_VERSION}"
            )
    errors.extend(resolved_solver_identity_errors(packet, name))
    errors.extend(_source_provenance_errors(packet, name))
    errors.extend(_paper_capture_source_binding_errors(packet, name))
    errors.extend(_paper_benchmark_timing_errors(packet, name))
    errors.extend(_linked_packet_errors(packet, path, visited))
    return errors


_MEDIAN_CPU_TIME_KEY = "median_cpu_time_per_step_ns"
_MEDIAN_RATIO_SUFFIX = "_median_cpu_cost_ratio"
_MEDIAN_RATIO_RELATIVE_TOLERANCE = 1e-9


def _finite_positive_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
        and value > 0.0
    )


def _paper_benchmark_timing_errors(
    packet: Mapping[str, object],
    packet_name: str,
) -> list[str]:
    """Fail closed when a paper packet's recorded medians and ratios disagree.

    The raw benchmark JSON referenced by ``benchmark.json_sha256`` is a
    temporary artifact, so the packet's own embedded medians are the only
    durable denominators. Every recorded median must be a positive finite
    number and every ``*_to_*_median_cpu_cost_ratio`` must equal the ratio of
    the embedded medians it names.
    """
    if packet_name not in PAPER_CAPTURE_ROLES:
        return []

    benchmark = packet.get("benchmark")
    if not isinstance(benchmark, Mapping):
        return []

    errors: list[str] = []
    medians: dict[str, float] = {}

    def record_median(label: str, key: str, timing: object) -> None:
        if not isinstance(timing, Mapping):
            errors.append(f"{packet_name}: {label} must be an object")
            return
        median = timing.get(_MEDIAN_CPU_TIME_KEY)
        if not _finite_positive_number(median):
            errors.append(
                f"{packet_name}: {label}.{_MEDIAN_CPU_TIME_KEY} must be a "
                "positive finite number"
            )
            return
        medians[key] = float(median)

    self_method_name = PAPER_PACKET_SELF_METHOD_NAMES[packet_name]
    timing = benchmark.get("timing")
    if timing is not None:
        record_median("benchmark.timing", self_method_name, timing)
    method = benchmark.get("method")
    if isinstance(method, Mapping) and method.get("timing") is not None:
        record_median("benchmark.method.timing", self_method_name, method.get("timing"))
    methods = benchmark.get("methods")
    if isinstance(methods, Mapping):
        for method_name, entry in methods.items():
            if isinstance(entry, Mapping):
                record_median(
                    f"benchmark.methods.{method_name}.timing",
                    str(method_name),
                    entry.get("timing"),
                )
    linked = packet.get("linked_avbd_vbd_evidence")
    if isinstance(linked, Mapping):
        linked_timings = linked.get("benchmark_method_timings")
        if isinstance(linked_timings, Mapping):
            for method_name, entry in linked_timings.items():
                record_median(
                    f"linked_avbd_vbd_evidence.benchmark_method_timings."
                    f"{method_name}",
                    str(method_name),
                    entry,
                )

    if self_method_name not in medians:
        errors.append(
            f"{packet_name}: benchmark must embed a positive finite "
            f"{self_method_name} {_MEDIAN_CPU_TIME_KEY}"
        )

    comparison = benchmark.get("comparison")
    required_ratio_keys = PAPER_REQUIRED_MEDIAN_RATIO_KEYS.get(packet_name, frozenset())
    if not isinstance(comparison, Mapping):
        if required_ratio_keys:
            errors.append(
                f"{packet_name}: benchmark.comparison must contain exactly "
                f"the required median ratio keys {sorted(required_ratio_keys)!r}"
            )
        return errors
    actual_ratio_keys = {
        key
        for key in comparison
        if isinstance(key, str) and key.endswith(_MEDIAN_RATIO_SUFFIX)
    }
    if actual_ratio_keys != required_ratio_keys:
        errors.append(
            f"{packet_name}: benchmark.comparison median ratio keys must be "
            f"exactly {sorted(required_ratio_keys)!r}, got "
            f"{sorted(actual_ratio_keys)!r}"
        )
    for key, value in comparison.items():
        if not isinstance(key, str) or not key.endswith(_MEDIAN_RATIO_SUFFIX):
            continue
        if not _finite_positive_number(value):
            errors.append(
                f"{packet_name}: benchmark.comparison.{key} must be a "
                "positive finite number"
            )
            continue
        stem = key[: -len(_MEDIAN_RATIO_SUFFIX)]
        if "_to_" not in stem:
            errors.append(
                f"{packet_name}: benchmark.comparison.{key} does not name a "
                "'<numerator>_to_<denominator>' median pair"
            )
            continue
        numerator_name, denominator_name = stem.rsplit("_to_", 1)
        numerator = medians.get(numerator_name)
        denominator = medians.get(denominator_name)
        if numerator is None or denominator is None:
            errors.append(
                f"{packet_name}: benchmark.comparison.{key} references a "
                "median the packet does not embed"
            )
            continue
        expected = numerator / denominator
        if not math.isclose(
            float(value),
            expected,
            rel_tol=_MEDIAN_RATIO_RELATIVE_TOLERANCE,
            abs_tol=0.0,
        ):
            errors.append(
                f"{packet_name}: benchmark.comparison.{key} is {value!r} but "
                f"the embedded medians record {expected!r}"
            )
    return errors


def packet_errors(path: Path) -> list[str]:
    return _packet_errors(path, set())


def collect_packets(args: argparse.Namespace) -> list[Path]:
    if args.packet:
        return list(args.packet)
    return sorted(args.packet_dir.glob(PACKET_GLOB))


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    packets = collect_packets(args)
    if not packets:
        print(f"No {PACKET_GLOB} packets found under {args.packet_dir}")
        return 1

    all_errors: list[str] = []
    for path in packets:
        all_errors.extend(packet_errors(path))

    if all_errors:
        for error in all_errors:
            print(f"ERROR: {error}")
        return 1

    print(f"Validated {len(packets)} AVBD packet(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
