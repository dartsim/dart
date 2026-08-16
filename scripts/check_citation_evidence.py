#!/usr/bin/env python3
"""Fail-closed validator for the PLAN-123 citation claim/evidence contract.

Validates, without running any simulation:

- `claims-manifest.json` against the `dart.citation_claim_manifest/v1` schema,
  including exact claim-ID agreement with the human corpus table and the capped
  first-wave family list;
- every packet under `evidence/` (recursively, excluding negative controls)
  and every packet a manifest lane references, wherever it sits, against the
  `dart.citation_claim_evidence/v1` schema: missing target commit, scene
  digest, requested/resolved method, command, ensemble, disposition, claim
  boundary, or review record fails, and a lane may not point at prose, a
  non-JSON file, a path outside `evidence/`, or a negative control;
- metric groups that must be measured-with-method or explicitly typed
  `unsupported` with a reason -- never silently absent, null, NaN, a spelled
  placeholder such as "n/a", or an unacknowledged exact zero (an unmeasurable
  quantity is a typed-unsupported marker; a real zero is declared in
  `measured_zero_fields`);
- `raw_paths` that resolve to files that exist;
- every packet in `evidence/negative-controls/`, which must FAIL validation
  (a permanent proof that the validator rejects incomplete evidence);
- manifest lane/evidence cross-links: referenced packets exist, agree on claim
  ID and branch, and `closed` lanes carry a disposition plus at least two
  review passes.

`--freshness` additionally requires every non-negative-control packet to
record the current `HEAD` commit; it is a packet-writing aid, not a CI gate,
because squash merges legitimately retire topic-branch commits.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import posixpath
import re
import subprocess
import sys
from pathlib import Path, PurePosixPath

REPO_ROOT = Path(__file__).resolve().parents[1]
PLAN_DIR = REPO_ROOT / "docs" / "plans" / "123-citation-driven-simulation-trust"
MANIFEST_PATH = PLAN_DIR / "claims-manifest.json"
CORPUS_PATH = PLAN_DIR / "citation-claim-corpus.md"
EVIDENCE_DIR = PLAN_DIR / "evidence"
NEGATIVE_DIR = EVIDENCE_DIR / "negative-controls"

MANIFEST_SCHEMA = "dart.citation_claim_manifest/v1"
PACKET_SCHEMA = "dart.citation_claim_evidence/v1"

CLAIM_ID_RE = re.compile(r"^CT-\d{3}$")
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
SCENE_DIGEST_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
CORPUS_ROW_RE = re.compile(r"^\|\s*(CT-\d{3})\s*\|", re.MULTILINE)

DISPOSITIONS = (
    "missing",
    "reproduced",
    "fixed",
    "version-specific",
    "not-applicable",
    "invalid-original-setup",
    "unresolved",
)
LANE_STATUSES = ("audit-required", "in-progress", "closed", "not-applicable")
UNSUPPORTED_SENTINELS = frozenset(
    {"", "-", "--", "n/a", "na", "none", "null", "tbd", "unknown", "unsupported"}
)

COMMAND_RE = re.compile(
    r"^(?:[A-Za-z_][A-Za-z0-9_]*=[^\s;|&`$]+[ \t]+)*" r"pixi run[ \t]+[^;|&`$\n\r]+\Z"
)
HASH_VALUE_RE = re.compile(r"^(sha256:)?[0-9a-f]{32,}$")
RAW_DATA_SUFFIXES = frozenset(
    {".csv", ".tsv", ".json", ".jsonl", ".ndjson", ".npz", ".npy", ".parquet"}
)
NUMERIC_BOOKKEEPING_KEYS = frozenset(
    {"seed", "seeds", "index", "idx", "id", "ids", "repeat", "repeats", "run"}
)


def _has_measurement_leaf(value: object) -> bool:
    """True when the value contains a numeric/boolean leaf beyond
    bookkeeping keys (the raw-rows measurement rule, applied to parsed
    artifacts)."""
    return any(
        (_is_finite_number(leaf) or isinstance(leaf, bool))
        and re.sub(r"(\[\d+\])+$", "", path.rsplit(".", 1)[-1])
        not in NUMERIC_BOOKKEEPING_KEYS
        for path, leaf in _metric_leaves(value)
    )


def _has_hash_list(value: object, length: int) -> bool:
    """True when a hash-named key holds a list of >= `length` digest values."""
    if isinstance(value, dict):
        for key, item in value.items():
            key_l = str(key).lower()
            if (
                ("sha256" in key_l or "hash" in key_l)
                and isinstance(item, list)
                and len(item) >= length
                and all(
                    _is_nonempty_str(entry) and HASH_VALUE_RE.match(entry.strip())
                    for entry in item
                )
            ):
                return True
            if _has_hash_list(item, length):
                return True
    elif isinstance(value, list):
        return any(_has_hash_list(item, length) for item in value)
    return False


def _has_hash_leaf(value: object) -> bool:
    """True when any nested key names a hash/sha256 with a non-empty value."""
    if isinstance(value, dict):
        for key, item in value.items():
            key_l = str(key).lower()
            if ("sha256" in key_l or "hash" in key_l) and (
                (_is_nonempty_str(item) and HASH_VALUE_RE.match(item.strip()))
                or (isinstance(item, (list, dict)) and _has_hash_leaf(item))
            ):
                return True
            if _has_hash_leaf(item):
                return True
    elif isinstance(value, list):
        return any(_has_hash_leaf(item) for item in value)
    return False


IDENTITY_PLACEHOLDER_VALUES = frozenset(
    UNSUPPORTED_SENTINELS
    | {"not measured", "pending", "todo", "missing", "unspecified"}
)


def _is_identity_value(value: object) -> bool:
    return (
        _is_nonempty_str(value)
        and value.strip().lower() not in IDENTITY_PLACEHOLDER_VALUES
    )


LANE_KEYS = ("dart7", "dart6")
BRANCH_BY_LANE = {"dart7": "main", "dart6": "release-6.20"}
FIRST_WAVE_FAMILY_CAP = 6

# String metric leaves are allowed only under keys that clearly name semantic
# metadata; everywhere else a string is prose masquerading as a measurement.
METRIC_STRING_KEY_EXACT = frozenset(
    {
        "method",
        "note",
        "regime",
        "unit",
        "units",
        "kind",
        "signature_test",
        "attribution",
        "contact_solver_method",
        "detector",
        "criteria_exceeded",
        "families_with_shrinking_drift",
    }
)
METRIC_STRING_KEY_SUFFIXES = (
    "_note",
    "_notes",
    "_semantics",
    "_method",
    "_methods",
    "_reasons",
    "_criteria",
    "_test",
    "_families",
    "_groups",
    "_detectors",
    "_basis",
)

# Visual evidence must be actual media; an ordinary text file satisfying a
# path-existence check is not a capture.
VISUAL_MEDIA_SUFFIXES = (
    ".png",
    ".jpg",
    ".jpeg",
    ".gif",
    ".webp",
    ".svg",
    ".apng",
    ".mp4",
    ".webm",
)

# configuration.requested/resolved must name a recognizable identity, not an
# arbitrary placeholder object. Keys that merely mention an identity word in
# a metadata role (method_note, backend_reason, ...) do not count.
FETCH_HINT_RE = re.compile(
    r"^git fetch origin pull/3445/head && git checkout ([0-9a-f]{40})$"
)
IDENTITY_KEY_TOKENS = frozenset(
    {
        "solver",
        "solvers",
        "method",
        "methods",
        "detector",
        "detectors",
        "integrator",
        "integration",
        "backend",
        "backends",
        "family",
        "families",
    }
)
IDENTITY_METADATA_SUFFIXES = (
    "_note",
    "_notes",
    "_semantics",
    "_reason",
    "_reasons",
    "_criteria",
    "_test",
    "_basis",
    "_provenance",
    "_policy",
)


def _is_identity_key(key: str) -> bool:
    """Whole-token identity match: `contact_solver_method` counts,
    `methodology` (substring only) and `method_note` (metadata role) do
    not."""
    if key.endswith(IDENTITY_METADATA_SUFFIXES):
        return False
    tokens = re.split(r"[^a-zA-Z0-9]+", key.lower())
    return any(token in IDENTITY_KEY_TOKENS for token in tokens)


def _has_identity_key(value: object) -> bool:
    """True when any key at any depth names a solver/method/... identity
    with a non-empty string value; {"placeholder": null} and
    {"method_note": "..."} have none."""
    if isinstance(value, dict):
        for key, item in value.items():
            if _is_identity_key(str(key)) and _is_identity_value(item):
                return True
            if _has_identity_key(item):
                return True
    elif isinstance(value, list):
        return any(_has_identity_key(item) for item in value)
    return False


PACKET_TOP_LEVEL_KEYS = {
    "schema",
    "claim_id",
    "title",
    "source",
    "target",
    "scene",
    "configuration",
    "ensemble",
    "metrics",
    "evidence",
    "result",
    "review",
    "host",
}
REQUIRED_PACKET_KEYS = PACKET_TOP_LEVEL_KEYS - {"host"}
METRIC_GROUPS = ("physical", "numerical", "performance", "allocation")

GIT_QUERY_ERRORS = (OSError, subprocess.CalledProcessError)


def _is_nonempty_str(value: object) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _is_finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _evidence_path_issue(raw_path: str, base_dir: Path | None) -> str | None:
    """Why a packet-referenced artifact path is unacceptable, or None.

    Paths must stay relative and resolve to an existing file INSIDE an
    approved root (the plan directory or the repository); an absolute or
    escaping path can satisfy a naive existence check with host-local data
    that is neither tracked nor portable.
    """
    pure = PurePosixPath(raw_path)
    if (
        pure.is_absolute()
        or raw_path[1:2] == ":"
        or "\\" in raw_path
        or ".." in pure.parts
    ):
        return (
            f"{raw_path!r} must be a relative path inside the repository "
            "evidence tree (no absolute paths, drive letters, or '..')"
        )
    if base_dir is None:
        return None
    if _resolve_evidence_path(raw_path, base_dir) is None:
        return (
            f"{raw_path!r} does not resolve to an existing file inside an "
            "approved evidence root; a dangling or escaping path is prose"
        )
    return None


def _resolve_evidence_path(raw_path: str, base_dir: "Path | None") -> "Path | None":
    """The resolved in-root file a relative evidence path names, or None."""
    if base_dir is None:
        return None
    for root in (base_dir, REPO_ROOT):
        candidate = (root / raw_path).resolve()
        if candidate.is_file() and candidate.is_relative_to(root.resolve()):
            return candidate
    return None


def _raw_data_content_issue(path: "Path") -> "str | None":
    """Why a raw-data artifact's bytes do not match its claimed format.

    Mirrors the visual check: a prose file renamed to `rows.csv` must not
    satisfy the raw-evidence requirement. JSON variants must parse; CSV/TSV
    need delimited tabular lines; NumPy/parquet containers must carry their
    magic bytes. Deep semantic validation of tabular contents is a recorded
    boundary.
    """
    suffix = path.suffix.lower()
    try:
        with path.open("rb") as stream:
            head = stream.read(1 * 1024 * 1024)
    except OSError:
        return "could not be read for format verification"
    mismatch = (
        "does not parse as its claimed raw-data format; a renamed prose "
        "file is not raw evidence"
    )
    no_content = (
        "parses but carries no numeric or boolean measurement content; a "
        "structurally empty artifact is not raw evidence"
    )
    if suffix == ".json":
        try:
            parsed = json.loads(path.read_text(encoding="utf-8"))
        except OSError, ValueError:
            return mismatch
        if not _has_measurement_leaf(parsed):
            return no_content
        return None
    if suffix in (".jsonl", ".ndjson"):
        try:
            parsed_lines = [
                json.loads(line)
                for line in path.read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
        except OSError, ValueError:
            return mismatch
        if not parsed_lines or not _has_measurement_leaf(parsed_lines):
            return no_content
        return None
    if suffix in (".csv", ".tsv"):
        delimiter = b"," if suffix == ".csv" else b"\t"
        first_lines = head.splitlines()[:2]
        if not first_lines or not any(delimiter in line for line in first_lines):
            return mismatch
        return None
    if suffix == ".npy":
        return None if head.startswith(b"\x93NUMPY") else mismatch
    if suffix in (".npz", ".parquet"):
        ok = head.startswith(b"PK") if suffix == ".npz" else head[:4] == b"PAR1"
        return None if ok else mismatch
    return None


def _visual_content_issue(path: "Path") -> "str | None":
    """Why a resolved visual artifact's bytes do not match its claimed type.

    A prose file renamed to `capture.png` satisfies a suffix check, and a
    bare eight-byte signature satisfies a header check; requiring the
    container's structural begin AND end markers (plus a minimum size) means
    the artifact must at least be a complete container of its claimed type.
    The header and the file TAIL are read separately so large legitimate
    files are not falsely reported truncated. Full decoding would need an
    image dependency; that boundary is recorded in the dev-task verification
    log.
    """
    suffix = path.suffix.lower()
    try:
        size = path.stat().st_size
        with path.open("rb") as stream:
            header = stream.read(4096)
            if size > 4096:
                stream.seek(-min(size, 4096), 2)
                tail = stream.read(4096)
            else:
                tail = header
    except OSError:
        return "could not be read for media-signature verification"
    mismatch = (
        "is not a structurally complete media file of its claimed type; a "
        "renamed or truncated artifact is not visual evidence"
    )
    if size < 64:
        return mismatch
    stripped_tail = tail.rstrip()
    if suffix in (".png", ".apng"):
        ok = (
            header.startswith(b"\x89PNG\r\n\x1a\n")
            and b"IHDR" in header[:64]
            and stripped_tail.endswith(b"IEND\xaeB`\x82")
        )
        return None if ok else mismatch
    if suffix in (".jpg", ".jpeg"):
        ok = header.startswith(b"\xff\xd8\xff") and stripped_tail.endswith(b"\xff\xd9")
        return None if ok else mismatch
    if suffix == ".gif":
        ok = header.startswith((b"GIF87a", b"GIF89a")) and stripped_tail.endswith(
            b"\x3b"
        )
        return None if ok else mismatch
    if suffix == ".webp":
        return None if header[:4] == b"RIFF" and header[8:12] == b"WEBP" else mismatch
    if suffix == ".mp4":
        return None if header[4:8] == b"ftyp" else mismatch
    if suffix == ".webm":
        return None if header.startswith(b"\x1a\x45\xdf\xa3") else mismatch
    if suffix == ".svg":
        ok = b"<svg" in header and b"</svg>" in stripped_tail
        return None if ok else mismatch
    return None


def _packet_content_digest(packet: dict) -> str:
    """Digest of the packet minus its review block.

    A review pass binds to this digest; regenerating a packet with different
    content therefore invalidates prior passes instead of silently carrying
    them onto evidence they never reviewed.
    """
    content = {key: value for key, value in packet.items() if key != "review"}
    return (
        "sha256:"
        + hashlib.sha256(
            json.dumps(content, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
    )


def _is_unsupported_leaf(value: object) -> bool:
    """True when a nested value is a typed-unsupported marker."""
    return (
        isinstance(value, dict)
        and value.get("status") == "unsupported"
        and set(value) <= {"status", "reason"}
    )


def _metric_leaves(value: object, path: str = "") -> list[tuple[str, object]]:
    """Flatten a metric value into (dotted-path, leaf) pairs.

    Typed-unsupported markers are returned whole so callers can validate them
    instead of descending into their `status`/`reason` strings.
    """
    if _is_unsupported_leaf(value):
        return [(path, value)]
    if isinstance(value, dict):
        leaves: list[tuple[str, object]] = []
        for key, child in value.items():
            leaves.extend(_metric_leaves(child, f"{path}.{key}" if path else str(key)))
        return leaves
    if isinstance(value, list):
        leaves = []
        for index, child in enumerate(value):
            leaves.extend(_metric_leaves(child, f"{path}[{index}]"))
        return leaves
    return [(path, value)]


def corpus_claim_ids(corpus_text: str) -> list[str]:
    """Extract the ordered unique claim IDs from the corpus markdown table."""
    seen: dict[str, None] = {}
    for match in CORPUS_ROW_RE.finditer(corpus_text):
        seen.setdefault(match.group(1))
    return list(seen)


def metric_group_errors(name: str, group: object) -> list[str]:
    """A metric group is measured-with-method or typed unsupported. Nothing else.

    Inside a measured group, every exact-zero number must be acknowledged: an
    unmeasurable quantity is a typed-unsupported marker
    (`{"status": "unsupported", "reason": ...}`), and a genuinely measured zero
    is listed in `measured_zero_fields` by its dotted path. That is what makes
    "unsupported is never silently zero" an enforced rule instead of a promise:
    a zero cannot reach a packet without its author naming which kind it is.
    """
    errors: list[str] = []
    if not isinstance(group, dict) or not group:
        return [f"metrics.{name} must be a non-empty object"]
    if group.get("status") == "unsupported":
        if not _is_nonempty_str(group.get("reason")):
            errors.append(f"metrics.{name} is unsupported but has no non-empty reason")
        extra = set(group) - {"status", "reason"}
        if extra:
            errors.append(
                f"metrics.{name} mixes unsupported status with values: "
                f"{sorted(extra)}"
            )
        return errors
    if "status" in group:
        errors.append(f"metrics.{name}.status must be 'unsupported' when present")
    if not _is_nonempty_str(group.get("method")):
        errors.append(
            f"metrics.{name} must record a non-empty measurement 'method' "
            "(or be typed unsupported with a reason)"
        )

    declared_zero_fields = group.get("measured_zero_fields", [])
    if not isinstance(declared_zero_fields, list) or not all(
        isinstance(item, str) for item in declared_zero_fields
    ):
        errors.append(
            f"metrics.{name}.measured_zero_fields must be a list of dotted "
            "field paths"
        )
        declared_zero_fields = []

    value_keys = [key for key in group if key not in {"method", "measured_zero_fields"}]
    if not value_keys:
        errors.append(f"metrics.{name} has a method but no measured values")

    observed_zero_fields: list[str] = []
    leaf_count = 0
    measurement_leaves = 0
    for key in value_keys:
        leaves = _metric_leaves(group[key], key)
        leaf_count += len(leaves)
        for path, leaf in leaves:
            if _is_unsupported_leaf(leaf):
                measurement_leaves += 1
                if not _is_nonempty_str(leaf.get("reason")):
                    errors.append(
                        f"metrics.{name}.{path} is typed unsupported but has "
                        "no non-empty reason"
                    )
                continue
            if leaf is None:
                errors.append(
                    f"metrics.{name}.{path} contains null; unsupported values "
                    "must be typed, not null"
                )
            elif (
                isinstance(leaf, str) and leaf.strip().lower() in UNSUPPORTED_SENTINELS
            ):
                errors.append(
                    f"metrics.{name}.{path} uses the placeholder {leaf!r}; "
                    "unsupported values must be typed, not spelled"
                )
            elif isinstance(leaf, str):
                terminal = re.sub(r"(\[\d+\])+$", "", path.rsplit(".", 1)[-1])
                if not (
                    terminal in METRIC_STRING_KEY_EXACT
                    or terminal.endswith(METRIC_STRING_KEY_SUFFIXES)
                ):
                    errors.append(
                        f"metrics.{name}.{path} is prose where a measurement "
                        "is expected; keep strings under semantic keys "
                        "(method, *_note, *_semantics, ...) or type the value "
                        "{'status': 'unsupported', 'reason': ...}"
                    )
                elif not leaf.strip():
                    errors.append(
                        f"metrics.{name}.{path} is a blank string; record the "
                        "annotation or drop the key"
                    )
            elif isinstance(leaf, (int, float)) and not isinstance(leaf, bool):
                measurement_leaves += 1
                if not math.isfinite(leaf):
                    errors.append(f"metrics.{name}.{path} contains a non-finite number")
                elif leaf == 0:
                    observed_zero_fields.append(path)
            elif isinstance(leaf, bool):
                # Boolean findings (stability flags, signature verdicts) are
                # legitimate measured outcomes; accepting them EXPLICITLY
                # here keeps the type chain exhaustive so nothing falls
                # through unvalidated.
                measurement_leaves += 1
            else:
                errors.append(
                    f"metrics.{name}.{path} has unrecognized leaf type "
                    f"{type(leaf).__name__}; a measurement is a finite "
                    "number, a boolean finding, a whitelisted semantic "
                    "string, or a typed-unsupported marker"
                )

    if value_keys and leaf_count > 0 and measurement_leaves == 0:
        errors.append(
            f"metrics.{name} carries only semantic annotations; a measured "
            "group needs at least one numeric/boolean measurement or "
            "typed-unsupported marker"
        )
    if value_keys and leaf_count == 0:
        errors.append(
            f"metrics.{name} has a method but only empty containers; that is "
            "not a measurement"
        )

    unacknowledged = [
        path for path in observed_zero_fields if path not in declared_zero_fields
    ]
    if unacknowledged:
        errors.append(
            f"metrics.{name} reports exact zero at {sorted(set(unacknowledged))} "
            "without acknowledgement; type each unmeasurable value as "
            "{'status': 'unsupported', 'reason': ...} or list a genuinely "
            "measured zero in measured_zero_fields"
        )
    stale = [path for path in declared_zero_fields if path not in observed_zero_fields]
    if stale:
        errors.append(
            f"metrics.{name}.measured_zero_fields lists {sorted(set(stale))} "
            "which are not zero in this packet"
        )
    return errors


def packet_errors(
    packet: object,
    *,
    known_claim_ids: set[str] | None = None,
    expected_branches: tuple[str, ...] = ("main", "release-6.20"),
    base_dir: Path | None = None,
) -> list[str]:
    """Return every fail-closed violation for one evidence packet."""
    if not isinstance(packet, dict):
        return ["packet must be a JSON object"]
    errors: list[str] = []

    if packet.get("schema") != PACKET_SCHEMA:
        errors.append(f"schema must be {PACKET_SCHEMA!r}")
    unknown = set(packet) - PACKET_TOP_LEVEL_KEYS
    if unknown:
        errors.append(f"unknown top-level keys: {sorted(unknown)}")
    missing = REQUIRED_PACKET_KEYS - set(packet)
    if missing:
        errors.append(f"missing required top-level keys: {sorted(missing)}")

    claim_id = packet.get("claim_id")
    if not (isinstance(claim_id, str) and CLAIM_ID_RE.match(claim_id)):
        errors.append("claim_id must match CT-NNN")
    elif known_claim_ids is not None and claim_id not in known_claim_ids:
        errors.append(f"claim_id {claim_id} is not in the claims manifest")
    if not _is_nonempty_str(packet.get("title")):
        errors.append("title must be a non-empty string")

    source = packet.get("source")
    if not isinstance(source, dict):
        errors.append("source must be an object")
    else:
        url = source.get("url")
        if not (_is_nonempty_str(url) and re.match(r"^https?://\S+$", url.strip())):
            errors.append(
                "source.url must be a retrievable http(s) URL; a placeholder "
                "does not bind the claim to its source"
            )
        if not _is_nonempty_str(source.get("claim")):
            errors.append("source.claim must be a non-empty string")

    target = packet.get("target")
    if not isinstance(target, dict):
        errors.append("target must be an object")
    else:
        branch = target.get("branch")
        if branch not in expected_branches:
            errors.append(f"target.branch must be one of {list(expected_branches)}")
        commit = target.get("commit")
        if not (isinstance(commit, str) and COMMIT_RE.match(commit)):
            errors.append("target.commit must be a 40-hex commit hash")
        fetch_hint = target.get("fetch_hint")
        hint_match = (
            FETCH_HINT_RE.match(fetch_hint.strip())
            if _is_nonempty_str(fetch_hint)
            else None
        )
        if hint_match is None:
            errors.append(
                "target.fetch_hint must be the runnable durable PR-ref "
                f"command (matching {FETCH_HINT_RE.pattern!r}); arbitrary "
                "prose or placeholder arguments cannot be executed to reach "
                "target.commit. Reachability itself is guaranteed by GitHub "
                "PR head refs surviving squash-merge and is checked at "
                "packet-writing time via --freshness"
            )
        elif isinstance(commit, str) and hint_match.group(1) != commit:
            errors.append(
                "target.fetch_hint checks out "
                f"{hint_match.group(1)[:12]}... but target.commit is "
                f"{commit[:12]}...; the hint must reproduce THIS packet's "
                "target"
            )

    scene = packet.get("scene")
    if not isinstance(scene, dict):
        errors.append("scene must be an object")
    else:
        if not _is_nonempty_str(scene.get("id")):
            errors.append("scene.id must be a non-empty string")
        digest = scene.get("digest")
        if not (isinstance(digest, str) and SCENE_DIGEST_RE.match(digest)):
            errors.append("scene.digest must match sha256:<64 hex>")
        parameters = scene.get("parameters")
        if isinstance(parameters, dict) and parameters:
            if not any(
                _is_finite_number(leaf) or isinstance(leaf, bool)
                for _, leaf in _metric_leaves(parameters)
            ):
                errors.append(
                    "scene.parameters carries no numeric or boolean values; "
                    "metadata-only parameters do not describe a scene the "
                    "digest could bind"
                )
        if not (isinstance(parameters, dict) and parameters):
            errors.append(
                "scene.parameters must publish the non-empty parameter "
                "object the digest was computed over; a well-formed digest "
                "with no content binds nothing"
            )
        elif isinstance(digest, str) and SCENE_DIGEST_RE.match(digest):
            # When the packet publishes the parameters the digest was taken
            # over, recompute it. A digest that cannot be reproduced from the
            # packet's own scene description binds nothing, and a hand-edited
            # scene would otherwise pass.
            expected = (
                "sha256:"
                + hashlib.sha256(
                    json.dumps(
                        scene["parameters"], sort_keys=True, separators=(",", ":")
                    ).encode("utf-8")
                ).hexdigest()
            )
            if digest != expected:
                errors.append(
                    f"scene.digest {digest} does not match the digest of "
                    f"scene.parameters ({expected}); the scene description "
                    "and its digest disagree"
                )

    configuration = packet.get("configuration")
    if not isinstance(configuration, dict):
        errors.append("configuration must be an object")
    else:
        for side in ("requested", "resolved"):
            value = configuration.get(side)
            if not isinstance(value, dict) or not value:
                errors.append(f"configuration.{side} must be a non-empty object")
                continue
            null_keys = sorted(key for key, item in value.items() if item is None)
            if null_keys:
                errors.append(
                    f"configuration.{side} carries null values at {null_keys}; "
                    "record the identity or omit the key"
                )
            if not _has_identity_key(value):
                errors.append(
                    f"configuration.{side} records no recognizable "
                    "solver/method/detector/integrator/backend identity field; "
                    "an arbitrary placeholder object is not a configuration"
                )
        if not _is_nonempty_str(configuration.get("resolved_provenance")):
            errors.append(
                "configuration.resolved_provenance must name how the resolved "
                "method identity was obtained"
            )
        if not _is_nonempty_str(configuration.get("detector")):
            errors.append("configuration.detector must be a non-empty string")
        timestep = configuration.get("timestep")
        if not (_is_finite_number(timestep) and timestep > 0.0):
            errors.append("configuration.timestep must be a positive number")
        if not _is_nonempty_str(configuration.get("fallback_policy")):
            errors.append("configuration.fallback_policy must be a non-empty string")

    declared_points = 0
    sweep = None
    seeds = None
    ensemble = packet.get("ensemble")
    if not isinstance(ensemble, dict):
        errors.append("ensemble must be an object")
    else:
        if not _is_nonempty_str(ensemble.get("kind")):
            errors.append("ensemble.kind must be a non-empty string")
        repeats = ensemble.get("deterministic_repeats")
        sweep = ensemble.get("sweep")
        seeds = ensemble.get("seeds")
        has_repeats = (
            isinstance(repeats, int) and not isinstance(repeats, bool) and repeats >= 2
        )
        if (
            has_repeats
            and isinstance(repeats, int)
            and repeats > 2
            and not _has_hash_list(packet.get("evidence"), repeats)
        ):
            errors.append(
                f"ensemble.deterministic_repeats={repeats} (> 2) requires a "
                "recorded per-repeat hash list of that length somewhere in "
                "evidence; two verified repeats may rely on the "
                "deterministic_repeats_identical flag plus a trajectory "
                "digest, larger claims must show their repeats"
            )
            has_repeats = False
        if has_repeats and not _has_hash_leaf(packet.get("evidence")):
            errors.append(
                "ensemble.deterministic_repeats is asserted but the "
                "evidence carries no *hash*/sha256 field binding the "
                "repeats to recorded trajectories; an unverifiable repeat "
                "claim is not an ensemble"
            )
            has_repeats = False
        if has_repeats and ensemble.get("deterministic_repeats_identical") is not True:
            errors.append(
                "ensemble.deterministic_repeats is asserted without "
                "deterministic_repeats_identical: true; a repeat count the "
                "writer did not verify bit-identical is a claim, not evidence"
            )
            has_repeats = False
        # A sweep or seed list only counts as an ensemble when its entries are
        # valid and mutually distinct; [null, null] or a duplicated point is
        # one run wearing an ensemble's clothes.
        has_sweep = isinstance(sweep, list) and len(sweep) >= 2
        if has_sweep:
            canonical_points: list[str] = []
            for index, entry in enumerate(sweep):
                if isinstance(entry, dict) and entry:
                    canonical_points.append(json.dumps(entry, sort_keys=True))
                elif _is_finite_number(entry) or _is_nonempty_str(entry):
                    canonical_points.append(json.dumps(entry))
                else:
                    errors.append(
                        f"ensemble.sweep[{index}] must be a non-empty object, "
                        "finite number, or non-empty string sweep point"
                    )
                    has_sweep = False
            if has_sweep and len(set(canonical_points)) < 2:
                errors.append(
                    "ensemble.sweep must contain at least two DISTINCT points"
                )
                has_sweep = False
            if has_sweep:
                declared_points = max(declared_points, len(canonical_points))
        has_seeds = isinstance(seeds, list) and len(seeds) >= 2
        if has_seeds:
            for index, entry in enumerate(seeds):
                if not (
                    (isinstance(entry, int) and not isinstance(entry, bool))
                    or _is_nonempty_str(entry)
                ):
                    errors.append(
                        f"ensemble.seeds[{index}] must be an integer or "
                        "non-empty string seed"
                    )
                    has_seeds = False
            if has_seeds and len({repr(entry) for entry in seeds}) < 2:
                errors.append("ensemble.seeds must contain at least two DISTINCT seeds")
                has_seeds = False
            if has_seeds:
                declared_points = max(declared_points, len(seeds))
        if not (has_repeats or has_sweep or has_seeds):
            errors.append(
                "ensemble must record deterministic_repeats >= 2, a sweep of "
                ">= 2 points, or >= 2 seeds; single runs are not evidence"
            )
        window = ensemble.get("measurement_window")
        if "measurement_window" not in ensemble:
            errors.append("ensemble.measurement_window is required")
        elif isinstance(window, dict) and window:
            bad_values = sorted(
                key for key, item in window.items() if not _is_finite_number(item)
            )
            if bad_values:
                errors.append(
                    "ensemble.measurement_window values at "
                    f"{bad_values} must be finite numbers"
                )
            start = window.get("start_s")
            end = window.get("end_s")
            if _is_finite_number(start) and _is_finite_number(end) and start > end:
                errors.append(
                    "ensemble.measurement_window start_s must not exceed end_s"
                )
            has_time_bounds = {"start_s", "end_s"} <= set(window)
            has_step_bounds = {"warmup_steps", "continuation_steps"} <= set(window)
            if has_step_bounds:
                warmup = window.get("warmup_steps")
                continuation = window.get("continuation_steps")
                if not (
                    isinstance(warmup, int)
                    and not isinstance(warmup, bool)
                    and warmup >= 0
                    and isinstance(continuation, int)
                    and not isinstance(continuation, bool)
                    and continuation >= 1
                ):
                    errors.append(
                        "ensemble.measurement_window step bounds must be "
                        "non-negative integers with continuation_steps >= 1"
                    )
                    has_step_bounds = False
            if not (has_time_bounds or has_step_bounds):
                errors.append(
                    "ensemble.measurement_window must name its bounds "
                    "(start_s/end_s or warmup_steps/continuation_steps); "
                    "unnamed numbers do not record when measurements were "
                    "collected"
                )
        else:
            errors.append(
                "ensemble.measurement_window must be a non-empty object of "
                "finite numeric bounds; prose or truthy placeholders do not "
                "record when measurements were collected"
            )

    metrics = packet.get("metrics")
    if not isinstance(metrics, dict):
        errors.append("metrics must be an object")
    else:
        for name in METRIC_GROUPS:
            if name not in metrics:
                errors.append(
                    f"metrics.{name} is required (measured or typed " "unsupported)"
                )
            else:
                errors.extend(metric_group_errors(name, metrics[name]))

    evidence = packet.get("evidence")
    if not isinstance(evidence, dict):
        errors.append("evidence must be an object")
    else:
        commands = evidence.get("commands")
        if not (
            isinstance(commands, list)
            and commands
            and all(_is_nonempty_str(command) for command in commands)
        ):
            errors.append("evidence.commands must be a non-empty list of commands")
        else:
            for index, command in enumerate(commands):
                if not COMMAND_RE.match(command.strip()):
                    errors.append(
                        f"evidence.commands[{index}] must be the "
                        "reproducible repository form (optional VAR=value "
                        "prefixes followed by 'pixi run ...'); arbitrary "
                        "shell strings are not the promised reproduction "
                        "path"
                    )
        raw_paths = evidence.get("raw_paths")
        raw_rows = evidence.get("raw_rows")
        has_paths = isinstance(raw_paths, list) and bool(raw_paths)
        has_rows = isinstance(raw_rows, list) and bool(raw_rows)
        if has_rows and declared_points > len(raw_rows):
            errors.append(
                f"ensemble declares {declared_points} sweep/seed points but "
                f"evidence.raw_rows records only {len(raw_rows)} rows; every "
                "declared point needs at least one recorded sample"
            )
        # Count parity is not enough: each declared object point must be
        # OBSERVED by a row carrying its exact coordinates, or one point's
        # measurements could be repeated to stand in for the others.
        if has_rows and isinstance(sweep, list):
            for point in sweep:
                if not (isinstance(point, dict) and point):
                    continue
                if not any(
                    isinstance(row, dict)
                    and all(row.get(key) == value for key, value in point.items())
                    for row in raw_rows
                ):
                    errors.append(
                        f"ensemble.sweep point {point} has no matching row "
                        "in evidence.raw_rows recording those coordinates; "
                        "a declared configuration without an observation is "
                        "not swept"
                    )
        if has_rows and isinstance(seeds, list):
            for seed in seeds:
                if not (
                    (isinstance(seed, int) and not isinstance(seed, bool))
                    or _is_nonempty_str(seed)
                ):
                    continue
                if not any(
                    isinstance(row, dict)
                    and any(
                        "seed" in str(key).lower() and row[key] == seed for key in row
                    )
                    for row in raw_rows
                ):
                    errors.append(
                        f"ensemble seed {seed!r} has no row recording it "
                        "under a seed field; a declared seed without an "
                        "observation is not an ensemble member"
                    )
        if not (has_paths or has_rows):
            errors.append("evidence must carry raw_rows inline or non-empty raw_paths")
        if has_rows:
            for index, row in enumerate(raw_rows):
                if not (isinstance(row, dict) and row):
                    errors.append(
                        f"evidence.raw_rows[{index}] must be a non-empty "
                        "structured record; null or scalar placeholders are "
                        "not raw evidence"
                    )
                    continue
                if not any(
                    (_is_finite_number(leaf) or isinstance(leaf, bool))
                    and re.sub(r"(\[\d+\])+$", "", path.rsplit(".", 1)[-1])
                    not in NUMERIC_BOOKKEEPING_KEYS
                    for path, leaf in _metric_leaves(row)
                ):
                    errors.append(
                        f"evidence.raw_rows[{index}] carries no numeric or "
                        "boolean measurement beyond bookkeeping (seed/index/"
                        "id/...); a metadata-only record is not raw evidence"
                    )
        if has_paths:
            for index, raw_path in enumerate(raw_paths):
                if not _is_nonempty_str(raw_path):
                    errors.append(
                        f"evidence.raw_paths[{index}] must be a non-empty string"
                    )
                    continue
                issue = _evidence_path_issue(raw_path, base_dir)
                if issue is not None and "must be a relative path" in issue:
                    errors.append(f"evidence.raw_paths[{index}] {issue}")
                    continue
                if PurePosixPath(raw_path).suffix.lower() not in RAW_DATA_SUFFIXES:
                    errors.append(
                        f"evidence.raw_paths[{index}] {raw_path!r} must name "
                        "a raw-data artifact "
                        f"({', '.join(sorted(RAW_DATA_SUFFIXES))}); prose or "
                        "code files are not raw evidence"
                    )
                    continue
                if issue is not None:
                    errors.append(f"evidence.raw_paths[{index}] {issue}")
                    continue
                resolved = _resolve_evidence_path(raw_path, base_dir)
                if resolved is not None:
                    content_issue = _raw_data_content_issue(resolved)
                    if content_issue is not None:
                        errors.append(
                            f"evidence.raw_paths[{index}] {raw_path!r} "
                            f"{content_issue}"
                        )
        referenced_paths: list[str] = []
        if has_paths:
            referenced_paths.extend(
                raw_path for raw_path in raw_paths if _is_nonempty_str(raw_path)
            )
        visual = evidence.get("visual")
        if isinstance(visual, dict):
            if visual.get("status") != "not-applicable" or not _is_nonempty_str(
                visual.get("reason")
            ):
                errors.append(
                    "evidence.visual object form must be "
                    "{'status': 'not-applicable', 'reason': ...}"
                )
        elif isinstance(visual, list) and visual:
            for index, item in enumerate(visual):
                if _is_nonempty_str(item):
                    item_path = item
                elif (
                    isinstance(item, dict)
                    and _is_nonempty_str(item.get("path"))
                    and _is_nonempty_str(item.get("description"))
                ):
                    item_path = item["path"]
                else:
                    errors.append(
                        f"evidence.visual[{index}] must be an artifact path "
                        "or {'path': ..., 'description': ...}; a placeholder "
                        "cannot stand in for visual evidence"
                    )
                    continue
                if PurePosixPath(item_path).suffix.lower() not in VISUAL_MEDIA_SUFFIXES:
                    errors.append(
                        f"evidence.visual[{index}] {item_path!r} is not a "
                        "recognized visual media artifact "
                        f"({', '.join(VISUAL_MEDIA_SUFFIXES)}); an ordinary "
                        "file cannot stand in for visual evidence"
                    )
                    continue
                issue = _evidence_path_issue(item_path, base_dir)
                if issue is not None:
                    errors.append(f"evidence.visual[{index}] {issue}")
                    continue
                resolved = _resolve_evidence_path(item_path, base_dir)
                if resolved is not None:
                    content_issue = _visual_content_issue(resolved)
                    if content_issue is not None:
                        errors.append(
                            f"evidence.visual[{index}] {item_path!r} "
                            f"{content_issue}"
                        )
        else:
            errors.append(
                "evidence.visual must list visual artifacts or be typed "
                "not-applicable with a reason"
            )
        if isinstance(visual, list):
            for item in visual:
                if _is_nonempty_str(item):
                    referenced_paths.append(item)
                elif isinstance(item, dict) and _is_nonempty_str(item.get("path")):
                    referenced_paths.append(item["path"])
        if referenced_paths:
            digests = evidence.get("artifact_digests")
            if not isinstance(digests, dict):
                errors.append(
                    "evidence.artifact_digests must map every referenced "
                    "artifact path to its sha256; without it, review "
                    "digests do not bind the referenced bytes"
                )
            else:
                for ref in referenced_paths:
                    recorded = digests.get(ref)
                    if not (
                        isinstance(recorded, str) and SCENE_DIGEST_RE.match(recorded)
                    ):
                        errors.append(
                            f"evidence.artifact_digests[{ref!r}] must record "
                            "sha256:<64 hex> for the referenced artifact"
                        )
                        continue
                    resolved = _resolve_evidence_path(ref, base_dir)
                    if resolved is not None:
                        actual = (
                            "sha256:"
                            + hashlib.sha256(resolved.read_bytes()).hexdigest()
                        )
                        if actual != recorded:
                            errors.append(
                                f"evidence.artifact_digests[{ref!r}] does "
                                "not match the referenced file's bytes; a "
                                "swapped artifact invalidates the packet "
                                "and its reviews"
                            )

    result = packet.get("result")
    if not isinstance(result, dict):
        errors.append("result must be an object")
    else:
        if result.get("disposition") not in DISPOSITIONS:
            errors.append(f"result.disposition must be one of {list(DISPOSITIONS)}")
        if not _is_nonempty_str(result.get("claim_boundary")):
            errors.append("result.claim_boundary must be a non-empty string")
        limitations = result.get("limitations")
        if not (
            isinstance(limitations, list)
            and limitations
            and all(_is_nonempty_str(item) for item in limitations)
        ):
            errors.append(
                "result.limitations must be a non-empty list; every packet "
                "has at least one honest limitation"
            )

    review = packet.get("review")
    if not isinstance(review, dict):
        errors.append("review must be an object")
    else:
        passes = review.get("passes")
        if not isinstance(passes, list):
            errors.append("review.passes must be a list")
        else:
            expected_digest = _packet_content_digest(packet)
            for index, entry in enumerate(passes):
                if (
                    not isinstance(entry, dict)
                    or not _is_nonempty_str(entry.get("reviewer"))
                    or not _is_nonempty_str(entry.get("summary"))
                ):
                    errors.append(
                        f"review.passes[{index}] needs non-empty reviewer "
                        "and summary"
                    )
                    continue
                if entry.get("verdict") != "pass":
                    errors.append(
                        f"review.passes[{index}] must record verdict 'pass'; "
                        "an entry without an explicit passing verdict (or "
                        "one recording a failure) cannot count toward "
                        "closure"
                    )
                if entry.get("content_digest") != expected_digest:
                    errors.append(
                        f"review.passes[{index}] is not bound to this "
                        "packet's content (content_digest must equal "
                        f"{expected_digest}); a review recorded against "
                        "earlier evidence does not carry over"
                    )

    return errors


def manifest_errors(manifest: object, corpus_ids: list[str]) -> list[str]:
    """Validate the claims manifest structure and corpus agreement."""
    if not isinstance(manifest, dict):
        return ["manifest must be a JSON object"]
    errors: list[str] = []
    if manifest.get("schema") != MANIFEST_SCHEMA:
        errors.append(f"manifest schema must be {MANIFEST_SCHEMA!r}")

    families = manifest.get("first_wave_families")
    if not (
        isinstance(families, list)
        and len(families) == FIRST_WAVE_FAMILY_CAP
        and all(_is_nonempty_str(family) for family in families)
        and len(set(families)) == FIRST_WAVE_FAMILY_CAP
    ):
        errors.append(
            "first_wave_families must list exactly "
            f"{FIRST_WAVE_FAMILY_CAP} unique families (the cap is a "
            "maintainer decision, not an editable default)"
        )
        families = []

    claims = manifest.get("claims")
    if not isinstance(claims, list) or not claims:
        errors.append("claims must be a non-empty list")
        return errors

    seen_ids: list[str] = []
    for claim in claims:
        if not isinstance(claim, dict):
            errors.append("every claim must be an object")
            continue
        claim_id = claim.get("id", "<missing>")
        if not (isinstance(claim_id, str) and CLAIM_ID_RE.match(claim_id)):
            errors.append(f"claim id {claim_id!r} must match CT-NNN")
            continue
        seen_ids.append(claim_id)
        if not _is_nonempty_str(claim.get("title")):
            errors.append(f"{claim_id}: title must be non-empty")
        if not _is_nonempty_str(claim.get("source")):
            errors.append(f"{claim_id}: source must be non-empty")
        family = claim.get("first_wave_family")
        if family is not None and family not in families:
            errors.append(
                f"{claim_id}: first_wave_family {family!r} is not in the "
                "capped family list"
            )
        lanes = claim.get("lanes")
        if not isinstance(lanes, dict) or set(lanes) != set(LANE_KEYS):
            errors.append(f"{claim_id}: lanes must define exactly {LANE_KEYS}")
            continue
        for lane_name, lane in lanes.items():
            prefix = f"{claim_id}.lanes.{lane_name}"
            if not isinstance(lane, dict):
                errors.append(f"{prefix} must be an object")
                continue
            status = lane.get("status")
            if status not in LANE_STATUSES:
                errors.append(f"{prefix}.status must be one of {list(LANE_STATUSES)}")
                continue
            evidence = lane.get("evidence")
            if not isinstance(evidence, list):
                errors.append(f"{prefix}.evidence must be a list")
                evidence = []
            if status == "not-applicable":
                if not _is_nonempty_str(lane.get("reason")):
                    errors.append(
                        f"{prefix} is not-applicable and must record a reason"
                    )
                na_disposition = lane.get("disposition")
                if na_disposition not in (None, "not-applicable"):
                    errors.append(
                        f"{prefix} is not-applicable and cannot publish "
                        f"disposition {na_disposition!r}; a lane that does "
                        "not apply concludes nothing"
                    )
                if evidence:
                    errors.append(
                        f"{prefix} is not-applicable and must not hold "
                        "evidence packets"
                    )
                continue
            if not _is_nonempty_str(lane.get("owner")):
                errors.append(f"{prefix}.owner must be non-empty")
            disposition = lane.get("disposition")
            if status == "closed":
                if disposition not in DISPOSITIONS:
                    errors.append(f"{prefix} is closed without a valid disposition")
                if not evidence:
                    errors.append(
                        f"{prefix} is closed without evidence packets; prose "
                        "cannot close a row"
                    )
            elif disposition is not None and disposition not in DISPOSITIONS:
                errors.append(
                    f"{prefix}.disposition must be null or one of "
                    f"{list(DISPOSITIONS)}"
                )

    duplicates = sorted({cid for cid in seen_ids if seen_ids.count(cid) > 1})
    if duplicates:
        errors.append(f"duplicate claim ids: {duplicates}")
    if corpus_ids and sorted(seen_ids) != sorted(corpus_ids):
        missing = sorted(set(corpus_ids) - set(seen_ids))
        extra = sorted(set(seen_ids) - set(corpus_ids))
        if missing:
            errors.append(f"claims missing from manifest: {missing}")
        if extra:
            errors.append(f"manifest claims not in corpus table: {extra}")
    return errors


def _reject_nonstandard_constant(constant: str) -> None:
    """Reject NaN/Infinity/-Infinity anywhere in a packet at load time.

    Python's default loader accepts them, but they are not JSON: a packet
    carrying one is unreadable to strict consumers and therefore not the
    portable machine evidence the contract promises.
    """
    raise ValueError(f"non-standard JSON constant {constant!r}")


def _reject_duplicate_keys(pairs: list) -> dict:
    """Different JSON consumers disagree on duplicate keys (first vs last
    wins), so a packet carrying one is not portable machine evidence."""
    seen: dict = {}
    for key, value in pairs:
        if key in seen:
            raise ValueError(f"duplicate JSON object key {key!r}")
        seen[key] = value
    return seen


def _load_json(path: Path, errors: list[str]) -> object | None:
    try:
        return json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_nonstandard_constant,
            object_pairs_hook=_reject_duplicate_keys,
        )
    except (OSError, ValueError) as error:
        # json.JSONDecodeError subclasses ValueError; parse_constant raises
        # a plain ValueError for NaN/Infinity.
        errors.append(f"{path}: unreadable JSON ({error})")
        return None


def _git_head(repo_root: Path) -> str | None:
    try:
        return (
            subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=repo_root,
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
            or None
        )
    except GIT_QUERY_ERRORS:
        return None


def validate_tree(
    plan_dir: Path,
    *,
    freshness_head: str | None = None,
) -> list[str]:
    """Validate the manifest, evidence packets, and negative controls."""
    errors: list[str] = []
    manifest_path = plan_dir / "claims-manifest.json"
    corpus_path = plan_dir / "citation-claim-corpus.md"
    evidence_dir = plan_dir / "evidence"
    negative_dir = evidence_dir / "negative-controls"

    corpus_ids: list[str] = []
    if corpus_path.is_file():
        corpus_ids = corpus_claim_ids(corpus_path.read_text(encoding="utf-8"))
        if not corpus_ids:
            errors.append(f"{corpus_path}: no CT-NNN rows found")
    else:
        errors.append(f"{corpus_path}: missing corpus document")

    manifest = _load_json(manifest_path, errors)
    known_ids: set[str] = set()
    lane_evidence: dict[str, tuple[str, str]] = {}
    lane_dispositions: dict[str, tuple[str, object]] = {}
    if manifest is not None:
        manifest_issues = manifest_errors(manifest, corpus_ids)
        errors.extend(f"{manifest_path}: {issue}" for issue in manifest_issues)
        if isinstance(manifest, dict) and isinstance(manifest.get("claims"), list):
            for claim in manifest["claims"]:
                if not isinstance(claim, dict):
                    continue
                claim_id = claim.get("id")
                if isinstance(claim_id, str):
                    known_ids.add(claim_id)
                lanes = claim.get("lanes")
                if not isinstance(lanes, dict):
                    continue
                for lane_name, lane in lanes.items():
                    if not isinstance(lane, dict):
                        continue
                    lane_paths = lane.get("evidence")
                    if lane_paths is not None and not isinstance(lane_paths, list):
                        errors.append(
                            f"{manifest_path}: {claim_id}.lanes.{lane_name}"
                            ".evidence must be a list"
                        )
                        lane_paths = []
                    for rel in lane_paths or []:
                        if not isinstance(rel, str):
                            errors.append(
                                f"{manifest_path}: {claim_id}.lanes."
                                f"{lane_name}.evidence contains a non-string "
                                f"entry {rel!r}; every entry must be a packet "
                                "path, or a lane could be closed by something "
                                "the packet checks never reach"
                            )
                            continue
                        # Canonicalize before indexing: `evidence/./p.json`
                        # and `evidence/p.json` are one file and must share
                        # one owner/one review record, and an escaping path
                        # must not become a distinct index key.
                        normalized = posixpath.normpath(rel)
                        if (
                            posixpath.isabs(normalized)
                            or normalized.startswith("..")
                            or "\\" in rel
                        ):
                            errors.append(
                                f"{manifest_path}: {claim_id}.lanes."
                                f"{lane_name}.evidence entry {rel!r} escapes "
                                "the plan directory"
                            )
                            continue
                        rel = normalized
                        if True:
                            if rel in lane_evidence:
                                owner = lane_evidence[rel]
                                errors.append(
                                    f"{manifest_path}: packet {rel} is claimed "
                                    f"by both {owner[0]}.{owner[1]} and "
                                    f"{claim_id}.{lane_name}; one packet has "
                                    "one owner"
                                )
                            lane_evidence[rel] = (
                                str(claim_id),
                                str(lane_name),
                            )
                            lane_dispositions[rel] = (
                                str(lane.get("status")),
                                lane.get("disposition"),
                            )

    # Every lane-referenced path is validated as a packet, wherever it sits.
    # Enumerating only `evidence/*.json` would let a lane close a row with a
    # file the packet checks never reach (prose, an empty object, or the
    # negative control itself) simply by living one directory deeper.
    for rel, (claim_id, lane_name) in sorted(lane_evidence.items()):
        packet_path = plan_dir / rel
        if not packet_path.is_file():
            errors.append(
                f"{manifest_path}: {claim_id}.lanes.{lane_name} references "
                f"missing packet {rel}"
            )
            continue
        if packet_path.suffix != ".json":
            errors.append(
                f"{manifest_path}: {claim_id}.lanes.{lane_name} references "
                f"{rel} which is not a .json packet"
            )
        try:
            relative = packet_path.resolve().relative_to(evidence_dir.resolve())
        except ValueError:
            errors.append(
                f"{manifest_path}: {claim_id}.lanes.{lane_name} references "
                f"{rel} outside evidence/"
            )
            continue
        if relative.parts and relative.parts[0] == "negative-controls":
            errors.append(
                f"{manifest_path}: {claim_id}.lanes.{lane_name} references "
                f"negative control {rel}; a control proves the validator "
                "fails closed and can never be a claim's evidence"
            )

    packet_paths = (
        sorted(
            path
            for path in evidence_dir.rglob("*.json")
            if negative_dir.resolve() not in path.resolve().parents
        )
        if evidence_dir.is_dir()
        else []
    )
    for rel in lane_evidence:
        candidate = plan_dir / rel
        if candidate.is_file() and candidate not in packet_paths:
            packet_paths.append(candidate)
    packet_paths = sorted(set(packet_paths))
    for packet_path in packet_paths:
        packet = _load_json(packet_path, errors)
        if packet is None:
            continue
        issues = packet_errors(
            packet, known_claim_ids=known_ids or None, base_dir=plan_dir
        )
        errors.extend(f"{packet_path}: {issue}" for issue in issues)
        if issues or not isinstance(packet, dict):
            continue
        rel = packet_path.relative_to(plan_dir).as_posix()
        linked = lane_evidence.get(rel)
        if linked is None:
            errors.append(
                f"{packet_path}: not referenced by any manifest lane; every "
                "packet needs a claim owner"
            )
        else:
            claim_id, lane_name = linked
            if packet.get("claim_id") != claim_id:
                errors.append(
                    f"{packet_path}: claim_id {packet.get('claim_id')} does "
                    f"not match manifest lane {claim_id}.{lane_name}"
                )
            expected_branch = BRANCH_BY_LANE.get(lane_name)
            branch = (
                packet.get("target", {}).get("branch")
                if isinstance(packet.get("target"), dict)
                else None
            )
            if expected_branch is not None and branch != expected_branch:
                errors.append(
                    f"{packet_path}: target.branch {branch!r} does not match "
                    f"lane {lane_name} branch {expected_branch!r}"
                )
            lane_status, lane_disposition = lane_dispositions.get(rel, (None, None))
            if lane_status == "closed":
                passes = (
                    packet.get("review", {}).get("passes")
                    if isinstance(packet.get("review"), dict)
                    else None
                )
                if not isinstance(passes, list) or len(passes) < 2:
                    errors.append(
                        f"{packet_path}: a packet closing a lane needs at "
                        "least two recorded review passes"
                    )
                else:
                    reviewers = {
                        entry["reviewer"].strip().casefold()
                        for entry in passes
                        if isinstance(entry, dict)
                        and _is_nonempty_str(entry.get("reviewer"))
                    }
                    if len(reviewers) < 2:
                        errors.append(
                            f"{packet_path}: a packet closing a lane needs "
                            "two INDEPENDENT review passes (distinct "
                            "reviewers); a duplicated reviewer is one review"
                        )
            # A lane's published disposition -- open OR closed -- must be the
            # one its evidence records; open lanes may defer (null) but may
            # not contradict.
            if lane_status in ("closed", "in-progress", "audit-required"):
                packet_disposition = (
                    packet.get("result", {}).get("disposition")
                    if isinstance(packet.get("result"), dict)
                    else None
                )
                if lane_status == "closed" or lane_disposition is not None:
                    if lane_disposition != packet_disposition:
                        errors.append(
                            f"{packet_path}: the lane records disposition "
                            f"{lane_disposition!r} but the packet's result "
                            f"is {packet_disposition!r}; the manifest cannot "
                            "publish a conclusion its evidence does not "
                            "support"
                        )
        if freshness_head is not None:
            commit = (
                packet.get("target", {}).get("commit")
                if isinstance(packet.get("target"), dict)
                else None
            )
            if commit != freshness_head:
                errors.append(
                    f"{packet_path}: target.commit {commit} is not the "
                    f"current HEAD {freshness_head} (--freshness)"
                )

    negative_paths = (
        sorted(negative_dir.rglob("*.json")) if negative_dir.is_dir() else []
    )
    negative_paths = [
        path
        for path in negative_paths
        if not path.name.endswith(".expected-errors.json")
    ]
    if not negative_paths:
        errors.append(
            f"{negative_dir}: at least one intentionally incomplete "
            "negative-control packet is required to prove the validator "
            "fails closed"
        )
    for packet_path in negative_paths:
        packet = _load_json(packet_path, errors)
        if packet is None:
            continue
        issues = packet_errors(packet, known_claim_ids=known_ids or None)
        if len(issues) < 3:
            errors.append(
                f"{packet_path}: negative control produced only "
                f"{len(issues)} validation error(s); it must stay clearly "
                "incomplete (>= 3) or the fail-closed proof is vacuous"
            )
        # A sidecar pins each SEEDED defect individually: if a validator
        # check regresses, its expected error disappears and this fails,
        # instead of hiding behind three unrelated survivors.
        sidecar = packet_path.with_name(
            packet_path.name[: -len(".json")] + ".expected-errors.json"
        )
        if sidecar.is_file():
            expected = _load_json(sidecar, errors)
            if not (
                isinstance(expected, list)
                and expected
                and all(_is_nonempty_str(item) for item in expected)
            ):
                errors.append(
                    f"{sidecar}: must be a non-empty list of expected error "
                    "substrings"
                )
            else:
                for needle in expected:
                    if not any(needle in issue for issue in issues):
                        errors.append(
                            f"{packet_path}: seeded defect no longer "
                            f"detected (no error contains {needle!r}); a "
                            "validator check regressed"
                        )
        else:
            errors.append(
                f"{packet_path}: negative control has no "
                ".expected-errors.json sidecar pinning its seeded defects"
            )

    return errors


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--plan-dir",
        type=Path,
        default=PLAN_DIR,
        help="Plan sidecar directory holding the manifest and evidence",
    )
    parser.add_argument(
        "--freshness",
        action="store_true",
        help="Require every evidence packet to record the current HEAD",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    freshness_head: str | None = None
    if args.freshness:
        freshness_head = _git_head(REPO_ROOT)
        if freshness_head is None:
            print(
                "check_citation_evidence: --freshness requires a readable " "git HEAD",
                file=sys.stderr,
            )
            return 1
    errors = validate_tree(args.plan_dir, freshness_head=freshness_head)
    if errors:
        print(
            f"check_citation_evidence: {len(errors)} error(s)",
            file=sys.stderr,
        )
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1
    print("check_citation_evidence: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
