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
import json
import math
import re
import subprocess
import sys
from pathlib import Path

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
LANE_KEYS = ("dart7", "dart6")
BRANCH_BY_LANE = {"dart7": "main", "dart6": "release-6.20"}
FIRST_WAVE_FAMILY_CAP = 6

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
    for key in value_keys:
        leaves = _metric_leaves(group[key], key)
        leaf_count += len(leaves)
        for path, leaf in leaves:
            if _is_unsupported_leaf(leaf):
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
            elif isinstance(leaf, (int, float)) and not isinstance(leaf, bool):
                if not math.isfinite(leaf):
                    errors.append(f"metrics.{name}.{path} contains a non-finite number")
                elif leaf == 0:
                    observed_zero_fields.append(path)

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
        if not _is_nonempty_str(source.get("url")):
            errors.append("source.url must be a non-empty string")
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

    scene = packet.get("scene")
    if not isinstance(scene, dict):
        errors.append("scene must be an object")
    else:
        if not _is_nonempty_str(scene.get("id")):
            errors.append("scene.id must be a non-empty string")
        digest = scene.get("digest")
        if not (isinstance(digest, str) and SCENE_DIGEST_RE.match(digest)):
            errors.append("scene.digest must match sha256:<64 hex>")

    configuration = packet.get("configuration")
    if not isinstance(configuration, dict):
        errors.append("configuration must be an object")
    else:
        for side in ("requested", "resolved"):
            value = configuration.get(side)
            if not isinstance(value, dict) or not value:
                errors.append(f"configuration.{side} must be a non-empty object")
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
        has_sweep = isinstance(sweep, list) and len(sweep) >= 2
        has_seeds = isinstance(seeds, list) and len(seeds) >= 2
        if not (has_repeats or has_sweep or has_seeds):
            errors.append(
                "ensemble must record deterministic_repeats >= 2, a sweep of "
                ">= 2 points, or >= 2 seeds; single runs are not evidence"
            )
        window = ensemble.get("measurement_window")
        if "measurement_window" not in ensemble:
            errors.append("ensemble.measurement_window is required")
        elif not window or (isinstance(window, str) and not window.strip()):
            errors.append(
                "ensemble.measurement_window must record an actual window, "
                "not an empty value"
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
        raw_paths = evidence.get("raw_paths")
        raw_rows = evidence.get("raw_rows")
        has_paths = isinstance(raw_paths, list) and bool(raw_paths)
        has_rows = isinstance(raw_rows, list) and bool(raw_rows)
        if not (has_paths or has_rows):
            errors.append("evidence must carry raw_rows inline or non-empty raw_paths")
        if has_paths:
            for index, raw_path in enumerate(raw_paths):
                if not _is_nonempty_str(raw_path):
                    errors.append(
                        f"evidence.raw_paths[{index}] must be a non-empty string"
                    )
                elif base_dir is not None and not any(
                    (root / raw_path).exists() for root in (base_dir, REPO_ROOT)
                ):
                    errors.append(
                        f"evidence.raw_paths[{index}] {raw_path!r} does not "
                        "resolve to an existing file; a dangling path is prose"
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
        elif not (isinstance(visual, list) and visual):
            errors.append(
                "evidence.visual must list visual artifacts or be typed "
                "not-applicable with a reason"
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


def _load_json(path: Path, errors: list[str]) -> object | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
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
    closed_lane_packets: set[str] = set()
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
                        if isinstance(rel, str):
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
                            if lane.get("status") == "closed":
                                closed_lane_packets.add(rel)

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
            if rel in closed_lane_packets:
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
        sorted(negative_dir.glob("*.json")) if negative_dir.is_dir() else []
    )
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
