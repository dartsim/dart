#!/usr/bin/env python3
"""Opt-in golden-image gate for the dartpy offscreen render path.

Renders a fixed canonical DART 7 scene through ``dart.gui.render`` and compares
it against a locally curated golden PNG using the WP-ASV.8 image tooling. This
is the reusable pattern a plan adopts to catch visible render regressions on a
stable backend; it is opt-in and never wired into default CI (see
docs/onboarding/agent-sim-verification.md).

Golden images are backend-specific: llvmpipe and a GPU-backed Filament produce
different pixels. They are therefore curated locally per backend and kept out
of git (the repo's ``*.png`` rule); the default golden path lives under
``tests/fixtures/render_goldens/`` (gitignored). First run ``--update`` on the
backend the gate will run on; the recorded metadata sidecar names that backend
so a later run on a different backend is an explicit, reviewable choice rather
than a silent flake.

Usage:
    python scripts/render_golden_gate.py --update            # curate locally
    python scripts/render_golden_gate.py                     # compare (gate)
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import image_golden

DEFAULT_GOLDEN = ROOT / "tests" / "fixtures" / "render_goldens" / "box_on_ground.png"
DEFAULT_SIZE = (320, 240)


def render_box_on_ground(size: tuple[int, int]) -> bytes:
    """Render the canonical box-on-ground scene and return PNG bytes.

    Deterministic on a fixed backend: same world, default bounds-fit camera,
    and pinned render size produce byte-identical output (verified: two renders
    diff to 0 pixels on llvmpipe).
    """
    import dartpy as dart

    world = dart.World(gravity=(0.0, 0.0, -9.81))
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.1))
    ground.is_static = True
    ground.set_collision_shape(dart.CollisionShape.box((1.0, 1.0, 0.1)))
    box = world.add_rigid_body("box", mass=1.0, position=(0.0, 0.0, 0.3))
    box.set_collision_shape(dart.CollisionShape.box((0.2, 0.2, 0.2)))

    image = dart.gui.render(world, size=size)
    return image.png_bytes()


SCENES = {"box_on_ground": render_box_on_ground}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=sorted(SCENES), default="box_on_ground")
    parser.add_argument("--golden", type=Path, default=DEFAULT_GOLDEN)
    parser.add_argument(
        "--update", action="store_true", help="regenerate the golden from this render"
    )
    parser.add_argument("--width", type=int, default=DEFAULT_SIZE[0])
    parser.add_argument("--height", type=int, default=DEFAULT_SIZE[1])
    parser.add_argument(
        "--backend",
        default="local-filament",
        help="backend label recorded in the golden metadata sidecar",
    )
    parser.add_argument("--fail", type=float, default=image_golden.DEFAULT_FAIL)
    parser.add_argument(
        "--failpercent", type=float, default=image_golden.DEFAULT_FAILPERCENT
    )
    parser.add_argument("--retries", type=int, default=1)
    parser.add_argument(
        "--out",
        type=Path,
        help="write the full JSON verdict here (stdout stays a clean summary "
        "line, since the Filament backend banner prints on stdout at exit)",
    )
    args = parser.parse_args(argv)

    size = (args.width, args.height)
    try:
        png = SCENES[args.scene](size)
    except Exception as exc:  # noqa: BLE001 - surface render failures clearly
        print(f"render_golden_gate.py: render failed: {exc}", file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory() as tmp:
        capture = Path(tmp) / f"{args.scene}.png"
        capture.write_bytes(png)

        metadata = {
            "backend": args.backend,
            "fidelity": "headless-default",
            "scene": args.scene,
            "size": f"{size[0]}x{size[1]}",
        }
        if args.update:
            image_golden.update_golden(
                capture,
                args.golden,
                fail=args.fail,
                failpercent=args.failpercent,
                metadata=metadata,
            )
            print(f"updated golden {args.golden} ({metadata})")
            return 0

        try:
            verdict = image_golden.compare_golden(
                capture,
                args.golden,
                fail=args.fail,
                failpercent=args.failpercent,
                retries=args.retries,
                metadata=metadata,
            )
        except FileNotFoundError as exc:
            # Distinct from a visual-regression FAIL (exit 1): a missing golden
            # means the gate cannot run. Surface it cleanly with exit 2, like
            # image_golden.py, instead of an uncaught traceback.
            print(f"render_golden_gate.py: {exc}", file=sys.stderr)
            return 2
        if args.out is not None:
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(
                json.dumps(verdict, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
        passed = bool(verdict.get("pass"))
        diff = verdict.get("checks", {}).get("diff", {})
        pct = diff.get("pct_pixels_over_threshold")
        print(
            f"render_golden_gate: {'PASS' if passed else 'FAIL'} "
            f"scene={args.scene} diff_pct_over_threshold={pct}"
        )
        return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
