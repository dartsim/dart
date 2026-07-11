"""Select a small, non-redundant evidence set tied to explicit claims.

Takes a candidate manifest — artifacts (stills, grids, composites, videos)
annotated with the claims they support, a quality score, and an optional
view azimuth — and picks a minimal high-quality subset: greedy set cover by
claim (most new claims first, quality as the tiebreak), with redundancy
pruning (same kind + same claims + similar viewpoint keeps only the best) and
size budgets. Every selection and
rejection records its rationale so the PR evidence trail explains why each
artifact exists and what it proves.

Input schema (dart.evidence_candidates/v1):
{
  "claims": [{"id": "C1", "text": "..."}],
  "artifacts": [{
      "path": "relative/or/absolute.png",
      "kind": "still" | "grid" | "composite" | "video",
      "claims": ["C1"],
      "caption": "what it shows",
      "quality": 0.0-1.0 (optional; default 0.5),
      "azimuth": radians (optional, for redundancy pruning),
      "observe": "what a reviewer should look at" (optional)
  }]
}
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import sys
from pathlib import Path
from typing import Any

SCHEMA_VERSION = "dart.evidence_selection/v1"
INPUT_SCHEMA_VERSION = "dart.evidence_candidates/v1"

_DEFAULT_MAX_ARTIFACTS = 5
_DEFAULT_MAX_TOTAL_BYTES = 25 * 1024 * 1024
# Views closer than this (radians) show the same information for pruning.
_SIMILAR_AZIMUTH = math.tau / 12.0
_EXACT_SEARCH_MAX_COMBINATIONS = 100_000


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _angular_distance(a: float, b: float) -> float:
    difference = (a - b) % math.tau
    return min(difference, math.tau - difference)


def _validate(manifest: dict[str, Any], base_dir: Path) -> list[dict[str, Any]]:
    if manifest.get("schema_version") != INPUT_SCHEMA_VERSION:
        raise ValueError(
            f"candidates manifest must declare schema_version={INPUT_SCHEMA_VERSION!r}"
        )
    claims = manifest.get("claims") or []
    claim_ids = {claim["id"] for claim in claims}
    if not claim_ids:
        raise ValueError("candidates manifest declares no claims")
    artifacts = manifest.get("artifacts") or []
    if not artifacts:
        raise ValueError("candidates manifest declares no artifacts")
    prepared = []
    for artifact in artifacts:
        path = Path(artifact["path"])
        if not path.is_absolute():
            path = base_dir / path
        if not path.exists():
            raise ValueError(f"artifact does not exist: {path}")
        supported = list(artifact.get("claims") or [])
        unknown = sorted(set(supported) - claim_ids)
        if unknown:
            raise ValueError(
                f"artifact {artifact['path']} references unknown claims {unknown}"
            )
        if not supported:
            raise ValueError(
                f"artifact {artifact['path']} supports no claims; every piece of "
                "evidence must be tied to an explicit claim"
            )
        prepared.append(
            {
                "path": path,
                "declared_path": str(artifact["path"]),
                "kind": str(artifact.get("kind", "still")),
                "claims": supported,
                "caption": str(artifact.get("caption", "")),
                "observe": str(artifact.get("observe", "")),
                "quality": float(artifact.get("quality", 0.5)),
                "azimuth": artifact.get("azimuth"),
                "bytes": path.stat().st_size,
            }
        )
    return prepared


def _find_exact_cover(
    ranked: list[dict[str, Any]],
    claim_ids: set[str],
    max_artifacts: int,
    max_total_bytes: int,
) -> list[dict[str, Any]] | None:
    """Find the smallest feasible cover when the bounded search is tractable."""
    max_count = min(max_artifacts, len(ranked))
    search_size = sum(
        math.comb(len(ranked), count) for count in range(1, max_count + 1)
    )
    if search_size > _EXACT_SEARCH_MAX_COMBINATIONS:
        return None

    for count in range(1, max_count + 1):
        best: tuple[dict[str, Any], ...] | None = None
        best_key: tuple[Any, ...] | None = None
        for candidates in itertools.combinations(ranked, count):
            total_bytes = sum(candidate["bytes"] for candidate in candidates)
            if total_bytes > max_total_bytes:
                continue
            covered = set().union(*(set(candidate["claims"]) for candidate in candidates))
            if not claim_ids <= covered:
                continue
            key = (
                -sum(candidate["quality"] for candidate in candidates),
                total_bytes,
                tuple(candidate["declared_path"] for candidate in candidates),
            )
            if best_key is None or key < best_key:
                best = candidates
                best_key = key
        if best is not None:
            return list(best)
    return None


def select_evidence(
    manifest: dict[str, Any],
    base_dir: Path,
    *,
    max_artifacts: int = _DEFAULT_MAX_ARTIFACTS,
    max_total_bytes: int = _DEFAULT_MAX_TOTAL_BYTES,
) -> dict[str, Any]:
    artifacts = _validate(manifest, base_dir)
    claims = {claim["id"]: claim for claim in manifest["claims"]}

    # Deterministic quality ranking; ties broken by declared path.
    ranked = sorted(artifacts, key=lambda a: (-a["quality"], a["declared_path"]))

    selected: list[dict[str, Any]] = []
    covered: set[str] = set()
    total_bytes = 0

    def redundant_against_selected(candidate: dict[str, Any]) -> str | None:
        for existing in selected:
            if existing["kind"] != candidate["kind"]:
                continue
            if set(existing["claims"]) != set(candidate["claims"]):
                continue
            azimuth_a = existing.get("azimuth")
            azimuth_b = candidate.get("azimuth")
            if (
                azimuth_a is not None
                and azimuth_b is not None
                and _angular_distance(float(azimuth_a), float(azimuth_b))
                > _SIMILAR_AZIMUTH
            ):
                # Distinct viewpoints reveal different information; keep both.
                continue
            return (
                f"redundant with {existing['declared_path']} "
                "(same kind, same claims, similar viewpoint)"
            )
        return None

    # Greedy set cover: each round takes the candidate that covers the most
    # still-uncovered claims, breaking ties by quality then declared path (the
    # ranked order). Coverage leads quality so a tight --max-artifacts budget
    # still finds a covering set when one exists instead of spending the budget
    # on a higher-quality artifact that proves fewer claims.
    remaining = list(ranked)
    while len(selected) < max_artifacts:
        best: dict[str, Any] | None = None
        best_new: set[str] = set()
        for candidate in remaining:
            new_claims = set(candidate["claims"]) - covered
            if not new_claims:
                continue
            if total_bytes + candidate["bytes"] > max_total_bytes:
                continue
            if len(new_claims) > len(best_new):
                best = candidate
                best_new = new_claims
        if best is None:
            break
        rationale_parts = [
            "covers claim(s) " + ", ".join(sorted(best_new)),
            f"quality={best['quality']:.3f}",
        ]
        selected.append(
            {
                **best,
                "rationale": "; ".join(rationale_parts),
                "sha256": _sha256(best["path"]),
            }
        )
        covered |= set(best["claims"])
        total_bytes += best["bytes"]
        remaining.remove(best)

    # Greedy set cover is fast but can spend a tight artifact budget on a
    # locally attractive choice that prevents a complete cover. For the small
    # candidate sets used by evidence packets, replace it with the smallest
    # feasible exact cover (quality, bytes, then path break ties). The search is
    # explicitly bounded so large manifests retain the deterministic greedy
    # fallback instead of growing exponentially.
    exact = _find_exact_cover(
        ranked, set(claims), max_artifacts, max_total_bytes
    )
    if exact is not None:
        selected = []
        covered = set()
        total_bytes = 0
        for candidate in exact:
            new_claims = set(candidate["claims"]) - covered
            selected.append(
                {
                    **candidate,
                    "rationale": (
                        "covers claim(s) " + ", ".join(sorted(new_claims))
                        + f"; quality={candidate['quality']:.3f}"
                    ),
                    "sha256": _sha256(candidate["path"]),
                }
            )
            covered |= set(candidate["claims"])
            total_bytes += candidate["bytes"]

    # Record why each unselected artifact was dropped so the evidence trail is
    # auditable: redundancy, no new coverage, or a budget ceiling.
    rejected: list[dict[str, Any]] = []
    selected_paths = {artifact["declared_path"] for artifact in selected}
    for candidate in ranked:
        if candidate["declared_path"] in selected_paths:
            continue
        new_claims = set(candidate["claims"]) - covered
        redundancy = redundant_against_selected(candidate)
        if redundancy is not None and not new_claims:
            reason = redundancy
        elif not new_claims:
            # Minimality: an extra angle earns its place only through an
            # explicit claim of its own (e.g. "no penetration from the side").
            reason = "adds no claim coverage beyond already-selected evidence"
        elif len(selected) >= max_artifacts:
            reason = f"artifact budget reached ({max_artifacts})"
        else:
            reason = f"size budget reached ({max_total_bytes} bytes total)"
        rejected.append({"path": candidate["declared_path"], "reason": reason})

    uncovered = sorted(set(claims) - covered)
    result = {
        "schema_version": SCHEMA_VERSION,
        "claims": [
            {
                "id": claim_id,
                "text": claims[claim_id]["text"],
                "covered": claim_id in covered,
            }
            for claim_id in sorted(claims)
        ],
        "selected": [
            {
                "path": artifact["declared_path"],
                "kind": artifact["kind"],
                "claims": sorted(artifact["claims"]),
                "caption": artifact["caption"],
                "observe": artifact["observe"],
                "quality": artifact["quality"],
                "bytes": artifact["bytes"],
                "sha256": artifact["sha256"],
                "rationale": artifact["rationale"],
            }
            for artifact in selected
        ],
        "rejected": rejected,
        "total_bytes": total_bytes,
        "uncovered_claims": uncovered,
        "pass": not uncovered,
    }
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("candidates", type=Path, help="candidates manifest JSON")
    parser.add_argument("--out", type=Path, help="write selection manifest here")
    parser.add_argument("--max-artifacts", type=int, default=_DEFAULT_MAX_ARTIFACTS)
    parser.add_argument("--max-total-bytes", type=int, default=_DEFAULT_MAX_TOTAL_BYTES)
    args = parser.parse_args(argv)

    try:
        manifest = json.loads(args.candidates.read_text(encoding="utf-8"))
        result = select_evidence(
            manifest,
            args.candidates.parent,
            max_artifacts=args.max_artifacts,
            max_total_bytes=args.max_total_bytes,
        )
    except (OSError, ValueError, KeyError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
