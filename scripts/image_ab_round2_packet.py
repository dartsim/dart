#!/usr/bin/env python3
"""Prepare and score the round-2 image A/B blind-judge packet."""

from __future__ import annotations

import argparse
import hashlib
import json
import random
import sys
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import image_ab_study
import image_sheet
from _image_tools import read_image, write_png

PACKET_SCHEMA_VERSION = "dart.image_ab_round2_packet/v1"
OBSERVATION_SCHEMA_VERSION = "dart.image_ab_round2_observations/v1"
DEFAULT_SIZE = (512, 384)
DEFAULT_STUDY_ID = "agent-sim-image-ab-round2"
VARIANTS = ("single", "multi_view", "turntable", "annotated")


@dataclass(frozen=True)
class BoxSpec:
    name: str
    position: tuple[float, float, float]
    extents: tuple[float, float, float] = (0.24, 0.24, 0.24)


@dataclass(frozen=True)
class CaseSpec:
    case: str
    expected: str
    prompt: str
    boxes: tuple[BoxSpec, ...]


CASES = (
    CaseSpec(
        case="clean_box_on_ground",
        expected="clean",
        prompt="A single box should rest above the ground plane.",
        boxes=(BoxSpec("box", (0.0, 0.0, 0.12)),),
    ),
    CaseSpec(
        case="ground_penetration",
        expected="defect",
        prompt="A single box may be penetrating the ground plane.",
        boxes=(BoxSpec("box", (0.0, 0.0, 0.02)),),
    ),
    CaseSpec(
        case="clean_separated_boxes",
        expected="clean",
        prompt="Two boxes should be separated above the ground plane.",
        boxes=(
            BoxSpec("left_box", (-0.24, 0.0, 0.12)),
            BoxSpec("right_box", (0.24, 0.0, 0.12)),
        ),
    ),
    CaseSpec(
        case="box_interpenetration",
        expected="defect",
        prompt="Two boxes may be interpenetrating above the ground plane.",
        boxes=(
            BoxSpec("left_box", (-0.06, 0.0, 0.12)),
            BoxSpec("right_box", (0.06, 0.0, 0.12)),
        ),
    ),
)


def _camera(azimuth: float, elevation: float = 25.0, distance: float = 2.2) -> Any:
    import dartpy as dart

    return dart.gui.orbit_camera(
        azimuth=azimuth,
        elevation=elevation,
        distance=distance,
        target=(0.0, 0.0, 0.12),
    )


def _make_world(case: CaseSpec) -> Any:
    import dartpy as dart

    world = dart.World(gravity=(0.0, 0.0, -9.81))
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.04))
    ground.is_static = True
    ground.set_collision_shape(dart.CollisionShape.box(_half_extents((1.6, 1.6, 0.08))))
    for box in case.boxes:
        body = world.add_rigid_body(box.name, mass=1.0, position=box.position)
        body.set_collision_shape(dart.CollisionShape.box(_half_extents(box.extents)))
    return world


def _half_extents(extents: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(0.5 * value for value in extents)


def _render_world(
    world: Any, path: Path, *, camera: Any, size: tuple[int, int]
) -> None:
    import dartpy as dart

    path.parent.mkdir(parents=True, exist_ok=True)
    image = dart.gui.render(world, camera=camera, size=size)
    path.write_bytes(image.png_bytes())


def _draw_rect(
    canvas: bytearray,
    canvas_width: int,
    canvas_height: int,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
    color: tuple[int, int, int],
) -> None:
    x0 = max(0, min(canvas_width - 1, x0))
    x1 = max(0, min(canvas_width - 1, x1))
    y0 = max(0, min(canvas_height - 1, y0))
    y1 = max(0, min(canvas_height - 1, y1))
    if x0 > x1:
        x0, x1 = x1, x0
    if y0 > y1:
        y0, y1 = y1, y0
    for x in range(x0, x1 + 1):
        _set_pixel(canvas, canvas_width, canvas_height, x, y0, color)
        _set_pixel(canvas, canvas_width, canvas_height, x, y1, color)
    for y in range(y0, y1 + 1):
        _set_pixel(canvas, canvas_width, canvas_height, x0, y, color)
        _set_pixel(canvas, canvas_width, canvas_height, x1, y, color)


def _draw_line(
    canvas: bytearray,
    canvas_width: int,
    canvas_height: int,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
    color: tuple[int, int, int],
) -> None:
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        _set_pixel(canvas, canvas_width, canvas_height, x, y, color)
        if x == x1 and y == y1:
            break
        twice_err = 2 * err
        if twice_err >= dy:
            err += dy
            x += sx
        if twice_err <= dx:
            err += dx
            y += sy


def _set_pixel(
    canvas: bytearray,
    canvas_width: int,
    canvas_height: int,
    x: int,
    y: int,
    color: tuple[int, int, int],
) -> None:
    if not (0 <= x < canvas_width and 0 <= y < canvas_height):
        return
    offset = (y * canvas_width + x) * 3
    canvas[offset : offset + 3] = bytes(color)


def _draw_label(
    canvas: bytearray,
    canvas_width: int,
    canvas_height: int,
    x: int,
    y: int,
    label: int,
) -> None:
    size = 16
    _draw_rect(
        canvas,
        canvas_width,
        canvas_height,
        x - size // 2,
        y - size // 2,
        x + size // 2,
        y + size // 2,
        (255, 230, 0),
    )
    _draw_line(
        canvas,
        canvas_width,
        canvas_height,
        x - 4,
        y + 5,
        x + 4,
        y + 5,
        (255, 230, 0),
    )
    if label != 0:
        _draw_line(
            canvas,
            canvas_width,
            canvas_height,
            x,
            y - 5,
            x,
            y + 5,
            (255, 230, 0),
        )
    if label == 2:
        _draw_line(
            canvas,
            canvas_width,
            canvas_height,
            x - 4,
            y - 5,
            x + 4,
            y - 5,
            (255, 230, 0),
        )
        _draw_line(
            canvas,
            canvas_width,
            canvas_height,
            x + 4,
            y - 5,
            x - 4,
            y + 5,
            (255, 230, 0),
        )


def annotate_still(source: Path, target: Path, case: CaseSpec) -> None:
    image = read_image(source)
    canvas = bytearray(image.pixels)
    ground_y = int(image.height * 0.66)
    for dy in range(-1, 2):
        _draw_line(
            canvas,
            image.width,
            image.height,
            int(image.width * 0.08),
            ground_y + dy,
            int(image.width * 0.92),
            ground_y + dy,
            (60, 230, 120),
        )
    _draw_label(canvas, image.width, image.height, int(image.width * 0.13), ground_y, 0)

    if len(case.boxes) == 1:
        _draw_rect(
            canvas,
            image.width,
            image.height,
            int(image.width * 0.43),
            int(image.height * 0.39),
            int(image.width * 0.57),
            int(image.height * 0.55),
            (255, 230, 0),
        )
        _draw_label(
            canvas,
            image.width,
            image.height,
            int(image.width * 0.50),
            int(image.height * 0.36),
            1,
        )
    else:
        for label, x_center in ((1, 0.43), (2, 0.57)):
            _draw_rect(
                canvas,
                image.width,
                image.height,
                int(image.width * (x_center - 0.08)),
                int(image.height * 0.39),
                int(image.width * (x_center + 0.08)),
                int(image.height * 0.55),
                (255, 230, 0),
            )
            _draw_label(
                canvas,
                image.width,
                image.height,
                int(image.width * x_center),
                int(image.height * 0.36),
                label,
            )
        _draw_line(
            canvas,
            image.width,
            image.height,
            int(image.width * 0.50),
            int(image.height * 0.47),
            int(image.width * 0.50),
            int(image.height * 0.57),
            (255, 230, 0),
        )
    write_png(target, image.width, image.height, bytes(canvas))


def _artifact_id(study_id: str, case: str, variant: str) -> str:
    digest = hashlib.sha256(f"{study_id}:{case}:{variant}".encode()).hexdigest()
    return f"r2_{digest[:10]}"


def _build_packet_rows(study_id: str, image_dir: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for case in CASES:
        for variant in VARIANTS:
            artifact_id = _artifact_id(study_id, case.case, variant)
            rows.append(
                {
                    "artifact_id": artifact_id,
                    "case": case.case,
                    "variant": variant,
                    "expected": case.expected,
                    "prompt": case.prompt,
                    "image": str(image_dir / f"{artifact_id}.png"),
                }
            )
    return rows


def generate_packet(
    out_dir: Path,
    *,
    study_id: str = DEFAULT_STUDY_ID,
    size: tuple[int, int] = DEFAULT_SIZE,
    seed: int = 20260706,
) -> dict[str, Any]:
    image_dir = out_dir / "images"
    rows = _build_packet_rows(study_id, image_dir)
    by_key = {(row["case"], row["variant"]): row for row in rows}
    for case in CASES:
        world = _make_world(case)
        single = Path(by_key[(case.case, "single")]["image"])
        _render_world(world, single, camera=_camera(-45.0), size=size)

        multi_frames = []
        for token, azimuth, elevation in (
            ("front", -90.0, 0.0),
            ("side", 0.0, 0.0),
            ("top", -90.0, 89.0),
            ("three_quarter", -45.0, 25.0),
        ):
            path = (
                image_dir
                / f"{_artifact_id(study_id, case.case, 'multi_view')}_{token}.png"
            )
            _render_world(world, path, camera=_camera(azimuth, elevation), size=size)
            multi_frames.append(path)
        image_sheet.assemble_sheet(
            multi_frames,
            Path(by_key[(case.case, "multi_view")]["image"]),
            cols=2,
            rows=2,
            tile_size=256,
            labels=True,
        )

        turntable_frames = []
        for index in range(9):
            azimuth = -180.0 + index * 40.0
            path = (
                image_dir
                / f"{_artifact_id(study_id, case.case, 'turntable')}_{index:02d}.png"
            )
            _render_world(world, path, camera=_camera(azimuth), size=size)
            turntable_frames.append(path)
        image_sheet.assemble_sheet(
            turntable_frames,
            Path(by_key[(case.case, "turntable")]["image"]),
            cols=3,
            rows=3,
            tile_size=224,
            labels=True,
        )

        annotated_source = (
            image_dir / f"{_artifact_id(study_id, case.case, 'annotated')}_source.png"
        )
        _render_world(world, annotated_source, camera=_camera(-45.0), size=size)
        annotate_still(
            annotated_source,
            Path(by_key[(case.case, "annotated")]["image"]),
            case,
        )

    answer_key = {
        "schema_version": PACKET_SCHEMA_VERSION,
        "study_id": study_id,
        "rows": rows,
        "notes": [
            "Keep this answer key away from blind judges until observations are complete.",
            "Expected labels use image_ab_study labels: defect or clean.",
        ],
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "answer_key.json").write_text(
        json.dumps(answer_key, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    judge_rows = [
        {
            "artifact_id": row["artifact_id"],
            "image": row["image"],
            "question": (
                "Does this artifact show a static geometry defect such as "
                "ground penetration or solid interpenetration?"
            ),
            "choices": ["defect", "clean", "uncertain"],
        }
        for row in rows
    ]
    random.Random(seed).shuffle(judge_rows)
    judge_form = {
        "schema_version": OBSERVATION_SCHEMA_VERSION,
        "study_id": study_id,
        "rows": judge_rows,
    }
    (out_dir / "judge_form.json").write_text(
        json.dumps(judge_form, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (out_dir / "judge_form.md").write_text(
        render_judge_form_markdown(judge_form),
        encoding="utf-8",
    )
    return {
        "schema_version": PACKET_SCHEMA_VERSION,
        "study_id": study_id,
        "out_dir": str(out_dir),
        "answer_key": str(out_dir / "answer_key.json"),
        "judge_form": str(out_dir / "judge_form.json"),
        "judge_markdown": str(out_dir / "judge_form.md"),
        "artifact_count": len(rows),
    }


def render_judge_form_markdown(judge_form: Mapping[str, Any]) -> str:
    lines = [
        f"# Round-2 Image A/B Blind Judge Form: {judge_form['study_id']}",
        "",
        "For each artifact, answer only from the image: `defect`, `clean`, or `uncertain`.",
        "Do not inspect `answer_key.json` until all observations are recorded.",
        "",
        "| Artifact | Image | Observed | Notes |",
        "| --- | --- | --- | --- |",
    ]
    for row in judge_form["rows"]:
        lines.append(f"| `{row['artifact_id']}` | `{row['image']}` |  |  |")
    lines.append("")
    return "\n".join(lines)


def _load_json(path: Path) -> Mapping[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, Mapping):
        raise ValueError(f"{path}: expected a JSON object")
    return data


def merge_observations(
    answer_key: Mapping[str, Any],
    observations: Mapping[str, Any],
) -> dict[str, Any]:
    keyed = {row["artifact_id"]: row for row in answer_key.get("rows", [])}
    seen_observations: set[tuple[str, str]] = set()
    rows = []
    for raw_row in observations.get("rows", []):
        if not isinstance(raw_row, Mapping):
            raise ValueError(f"observation rows must be objects, got {raw_row!r}")
        artifact_id = raw_row.get("artifact_id")
        judge = raw_row.get("judge")
        observed = raw_row.get("observed")
        if not isinstance(artifact_id, str) or artifact_id not in keyed:
            raise ValueError(f"unknown artifact_id {artifact_id!r}")
        if not isinstance(judge, str) or not judge:
            raise ValueError(f"{artifact_id}: missing non-empty judge")
        observation_key = (artifact_id, judge)
        if observation_key in seen_observations:
            raise ValueError(
                f"{artifact_id}: duplicate observation for judge {judge!r}"
            )
        seen_observations.add(observation_key)
        if not isinstance(observed, str):
            raise ValueError(f"{artifact_id}: missing observed label")
        observed = observed.lower()
        if observed not in image_ab_study.OBSERVED_LABELS:
            raise ValueError(
                f"{artifact_id}: observed must be one of "
                f"{sorted(image_ab_study.OBSERVED_LABELS)}, got {observed!r}"
            )
        key_row = keyed[artifact_id]
        rows.append(
            {
                "case": key_row["case"],
                "variant": key_row["variant"],
                "judge": judge,
                "expected": key_row["expected"],
                "observed": observed,
            }
        )
    return {
        "schema_version": image_ab_study.SCHEMA_VERSION,
        "study_id": str(answer_key.get("study_id", "")),
        "rows": rows,
    }


def score_observations(
    *,
    answer_key_path: Path,
    observations_path: Path,
    out_path: Path,
    results_path: Path | None = None,
    markdown_path: Path | None = None,
    baseline: str = "single",
) -> dict[str, Any]:
    manifest = merge_observations(
        _load_json(answer_key_path),
        _load_json(observations_path),
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    results = image_ab_study.reduce_study(manifest, baseline=baseline)
    if results_path is not None:
        results_path.parent.mkdir(parents=True, exist_ok=True)
        results_path.write_text(
            json.dumps(results, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    if markdown_path is not None:
        markdown_path.parent.mkdir(parents=True, exist_ok=True)
        markdown_path.write_text(
            image_ab_study.render_markdown(results),
            encoding="utf-8",
        )
    return results


def _parse_size(text: str) -> tuple[int, int]:
    if "x" not in text.lower():
        raise ValueError("size must be formatted as WIDTHxHEIGHT")
    width_text, height_text = text.lower().split("x", 1)
    width = int(width_text)
    height = int(height_text)
    if width < 64 or height < 64:
        raise ValueError("size must be at least 64x64")
    return width, height


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate = subparsers.add_parser("generate", help="render a blinded packet")
    generate.add_argument("--out", type=Path, required=True)
    generate.add_argument("--study-id", default=DEFAULT_STUDY_ID)
    generate.add_argument("--size", default=f"{DEFAULT_SIZE[0]}x{DEFAULT_SIZE[1]}")
    generate.add_argument("--seed", type=int, default=20260706)

    score = subparsers.add_parser("score", help="merge observations and reduce")
    score.add_argument("--answer-key", type=Path, required=True)
    score.add_argument("--observations", type=Path, required=True)
    score.add_argument("--out", type=Path, required=True)
    score.add_argument("--results", type=Path)
    score.add_argument("--markdown", type=Path)
    score.add_argument("--baseline", default="single")
    args = parser.parse_args(argv)

    try:
        if args.command == "generate":
            summary = generate_packet(
                args.out,
                study_id=args.study_id,
                size=_parse_size(args.size),
                seed=args.seed,
            )
            print(json.dumps(summary, indent=2, sort_keys=True))
            return 0
        results = score_observations(
            answer_key_path=args.answer_key,
            observations_path=args.observations,
            out_path=args.out,
            results_path=args.results,
            markdown_path=args.markdown,
            baseline=args.baseline,
        )
        print(json.dumps(results, indent=2, sort_keys=True))
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"image_ab_round2_packet.py: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
