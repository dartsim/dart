#!/usr/bin/env python3
"""Fetch current, license-vetted MJCF models for the comparison harness.

The MJCF files bundled under ``data/mjcf/openai`` are historical gym-era
models. Per maintainer direction, cross-engine benchmark rows should be able
to use *current* upstream models instead, fetched live from pinned URLs with
recorded licenses and SHA-256 digests (no license surprises, reproducible
bytes). Fetched models are cached locally; both engine runners can then be
pointed at the cached file paths.

Only single-file models are registered for now; mesh-based models (e.g.
MuJoCo Menagerie robot arms) need multi-asset fetching and are tracked as a
follow-up in the WS-G lane doc.
"""

from __future__ import annotations

import argparse
import hashlib
import sys
import urllib.request
from dataclasses import dataclass
from pathlib import Path

DEFAULT_DEST = Path(__file__).resolve().parent / ".model_cache"


@dataclass(frozen=True)
class PinnedModel:
    name: str
    url: str  # pinned to a release tag, never a moving branch
    sha256: str
    license: str
    source: str


PINNED_MODELS: dict[str, PinnedModel] = {
    "humanoid": PinnedModel(
        name="humanoid",
        url=(
            "https://raw.githubusercontent.com/google-deepmind/mujoco/"
            "3.3.0/model/humanoid/humanoid.xml"
        ),
        sha256="acc9167550bf20580717a551467e6e8d0d03d1916d24bbe4b734ce11afb99406",
        license="Apache-2.0 (Copyright 2021 DeepMind Technologies Limited)",
        source="google-deepmind/mujoco @ 3.3.0",
    ),
}


def fetch(model: PinnedModel, dest_dir: Path, force: bool = False) -> Path:
    dest_dir.mkdir(parents=True, exist_ok=True)
    dest = dest_dir / f"{model.name}.xml"
    if dest.exists() and not force:
        data = dest.read_bytes()
    else:
        with urllib.request.urlopen(model.url, timeout=60) as response:
            data = response.read()
        dest.write_bytes(data)
    digest = hashlib.sha256(data).hexdigest()
    if digest != model.sha256:
        dest.unlink(missing_ok=True)
        raise RuntimeError(
            f"SHA-256 mismatch for {model.name}: expected {model.sha256}, "
            f"got {digest}. The pinned URL content changed; re-vet the "
            "license and update the pin deliberately."
        )
    record = dest_dir / f"{model.name}.LICENSE.txt"
    record.write_text(
        f"model: {model.name}\nsource: {model.source}\nurl: {model.url}\n"
        f"sha256: {model.sha256}\nlicense: {model.license}\n"
    )
    return dest


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dest",
        type=Path,
        default=DEFAULT_DEST,
        help=f"Cache directory (default: {DEFAULT_DEST}).",
    )
    parser.add_argument(
        "--model",
        action="append",
        choices=sorted(PINNED_MODELS),
        help="Model(s) to fetch; default: all registered models.",
    )
    parser.add_argument(
        "--force", action="store_true", help="Re-download even if cached."
    )
    args = parser.parse_args(argv)

    names = args.model or sorted(PINNED_MODELS)
    for name in names:
        model = PINNED_MODELS[name]
        path = fetch(model, args.dest, force=args.force)
        print(f"{name}: {path} ({model.license}; {model.source})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
