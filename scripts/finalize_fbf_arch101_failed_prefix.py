#!/usr/bin/env python3
"""Finalize and verify the Figure 8 expected-failure comparison.

The strict exact-FBF lane for the source-pinned 101-stone scene is expected to
fail closed before the 1,600-substep horizon.  This wrapper preserves that
failure as a scientific negative, pads only the last successfully rendered
frame, and places it beside the complete boxed-LCP capture.  It never presents
the frozen tail as a simulated exact trajectory.

``finalize`` owns the short exact-prefix capture so it can bind the executable
and runner identities before and after the expected failure.  The complete
boxed member must already have been captured by ``run_fbf_visual_evidence.py``.
``seal`` records a human/agent visual inspection after the generated clip and
panel have actually been viewed.  ``verify`` is reuse-only and never launches
the simulation.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import importlib.util
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from types import ModuleType
from typing import Any, Sequence

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = (
    ROOT
    / "docs/dev_tasks/fbf_exact_coulomb_friction/assets"
    / "pr_media_current_head_67073"
)
DEFAULT_DEMO = ROOT / "build/default/cpp/Release/bin/dart-demos"
DEFAULT_RUNNER = ROOT / "scripts/run_fbf_visual_evidence.py"
DEFAULT_FFMPEG = ROOT / ".pixi/envs/gazebo/bin/ffmpeg"
DEFAULT_FFPROBE = ROOT / ".pixi/envs/gazebo/bin/ffprobe"
DEFAULT_PYTHON = Path(sys.executable)
DEFAULT_FONT = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")

RUNNER_TEST = ROOT / "python/tests/unit/test_run_fbf_visual_evidence.py"
PROVENANCE_HELPER = ROOT / "scripts/fbf_scene_provenance.py"
PROVENANCE_TEST = ROOT / "python/tests/unit/test_fbf_scene_provenance.py"
IMAGE_TOOLS_HELPER = ROOT / "scripts/_image_tools.py"
IMAGE_VERDICT_HELPER = ROOT / "scripts/image_verdict.py"
IMAGE_VERDICT_TEST = ROOT / "python/tests/unit/test_image_verdict.py"
FINALIZER_TEST = ROOT / "python/tests/unit/test_finalize_fbf_arch101_failed_prefix.py"

SCHEDULE_ID = "masonry_arch_101_author_standing_current_source"
BOXED_SCHEDULE_ID = f"{SCHEDULE_ID}__boxed"
GROUP_ID = "fig08_arch101_failed_prefix_vs_boxed"

SCHEMA_VERSION = "dart.fbf_arch101_failed_prefix_comparison/v1"
MANUAL_SCHEMA_VERSION = "dart.fbf_arch101_failed_prefix_manual_inspection/v1"
KIND = "expected_failed_prefix_vs_complete_comparison"

EXPECTED_TOTAL_SUBSTEPS = 1600
EXPECTED_TIME_STEP_SECONDS = 1.0 / 240.0
EXPECTED_FRAME_STRIDE = 8
EXPECTED_FAIL_STEP = 209
EXPECTED_LAST_RENDERED_STEP = 208
EXPECTED_TERMINAL_SIGNATURE = {
    "contacts": 208,
    "iterations": 5000,
    "residual": 1.2582804496066107e-6,
    "worst_residual": 1.2582804496066107e-6,
    "exact_attempts": 342,
    "exact_solves": 342,
    "accepted_at_cap": 1,
    "exact_failures": 0,
    "boxed_lcp_fallbacks": 0,
}

OUTPUT_FPS = 30
OUTPUT_FRAME_COUNT = 201
LABEL_BAND_HEIGHT = 24
EXACT_LABEL = "EXACT FBF - FAILED STEP 209; FROZEN AFTER STEP 208"

CLAIM_BOUNDARY = {
    "valid_current_dart_exact_scientific_negative": True,
    "valid_complete_boxed_comparison_capture": True,
    "valid_frozen_prefix_diagnostic": True,
    "complete_exact_trajectory": False,
    "exact_standing_outcome_validated": False,
    "solver_superiority": False,
    "source_trajectory_equivalence": False,
    "source_outcome_equivalence": False,
    "historical_paper_invocation_known": False,
    "fig08_parity": False,
    "video08_parity": False,
    "timing_comparable": False,
    "paper_parity": False,
}


class EvidenceError(RuntimeError):
    """Raised when the Figure 8 diagnostic contract is not satisfied."""


@dataclasses.dataclass(frozen=True)
class CompositionSpec:
    """Immutable media mapping for the failed prefix and complete comparator."""

    source_width: int
    source_height: int
    crop_width: int
    crop_height: int
    crop_x: int
    crop_y: int
    output_fps: int
    output_frame_count: int
    exact_steps: tuple[int, ...]
    boxed_label: str
    font: Path

    @property
    def output_width(self) -> int:
        return 2 * self.crop_width

    @property
    def output_height(self) -> int:
        return self.crop_height + LABEL_BAND_HEIGHT

    @property
    def duration_seconds(self) -> float:
        return self.output_frame_count / self.output_fps

    @property
    def first_frozen_output_frame(self) -> int:
        return len(self.exact_steps)

    @property
    def frozen_output_frame_count(self) -> int:
        return self.output_frame_count - len(self.exact_steps)

    def as_dict(self) -> dict[str, Any]:
        return {
            "source_dimensions": [self.source_width, self.source_height],
            "crop": [
                self.crop_width,
                self.crop_height,
                self.crop_x,
                self.crop_y,
            ],
            "output_dimensions": [self.output_width, self.output_height],
            "output_fps": f"{self.output_fps}/1",
            "output_frame_count": self.output_frame_count,
            "duration_seconds": self.duration_seconds,
            "exact_observed_steps": list(self.exact_steps),
            "exact_observed_frame_count": len(self.exact_steps),
            "first_frozen_output_frame": self.first_frozen_output_frame,
            "frozen_output_frame_count": self.frozen_output_frame_count,
            "frozen_source_step": self.exact_steps[-1],
            "exact_label": EXACT_LABEL,
            "boxed_label": self.boxed_label,
            "label_band_height": LABEL_BAND_HEIGHT,
            "font_path": str(self.font),
        }


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _payload_sha256(payload: Any) -> str:
    encoded = json.dumps(
        payload,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    def reject_nonfinite(value: str) -> None:
        raise EvidenceError(f"{path}: non-finite JSON number {value}")

    try:
        payload = json.loads(
            path.read_text(encoding="utf-8"), parse_constant=reject_nonfinite
        )
    except (OSError, json.JSONDecodeError) as error:
        raise EvidenceError(f"{path}: invalid JSON: {error}") from error
    if not isinstance(payload, dict):
        raise EvidenceError(f"{path}: expected a JSON object")
    return payload


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _require_file(path: Path, label: str) -> Path:
    path = path.resolve()
    if not path.is_file() or path.is_symlink():
        raise EvidenceError(f"{label} is not a regular file: {path}")
    return path


def _load_runner(path: Path) -> ModuleType:
    path = _require_file(path, "visual evidence runner")
    module_name = (
        "_fbf_visual_evidence_for_arch101_"
        + hashlib.sha256(str(path).encode()).hexdigest()[:12]
    )
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise EvidenceError(f"cannot load visual evidence runner: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception as error:
        raise EvidenceError(f"cannot import visual evidence runner: {error}") from error
    return module


def _run(
    argv: Sequence[Path | str],
    *,
    capture_output: bool = False,
    check: bool = True,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(item) for item in argv],
        cwd=ROOT,
        check=check,
        text=True,
        capture_output=capture_output,
    )


def _file_identity(path: Path) -> dict[str, Any]:
    path = _require_file(path, str(path))
    return {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def _tool_identity(path: Path) -> dict[str, Any]:
    path = _require_file(path, "media tool")
    completed = _run((path, "-version"), capture_output=True)
    version = completed.stdout
    if not version:
        raise EvidenceError(f"{path}: version output is empty")
    return {
        **_file_identity(path),
        "version_output_sha256": hashlib.sha256(version.encode()).hexdigest(),
        "version_first_line": version.splitlines()[0],
    }


def _linked_build_library_identities(
    ldd_output: str, build_root: Path
) -> list[dict[str, Any]]:
    build_root = build_root.resolve()
    dependencies: dict[Path, set[str]] = {}
    for raw_line in ldd_output.splitlines():
        line = raw_line.strip()
        if "=> not found" in line:
            raise EvidenceError(f"unresolved demo dependency: {line}")
        if "=>" not in line:
            continue
        soname, remainder = (part.strip() for part in line.split("=>", 1))
        reported_text = remainder.rsplit(" (", 1)[0].strip()
        if not reported_text:
            continue
        reported = Path(reported_text)
        try:
            resolved = reported.resolve(strict=True)
        except OSError as error:
            raise EvidenceError(
                f"cannot resolve linked {soname}: {reported}"
            ) from error
        if resolved.is_file() and resolved.is_relative_to(build_root):
            dependencies.setdefault(resolved, set()).add(soname)

    identities = [
        {
            "sonames": sorted(sonames),
            "resolved_path": str(path),
            "size_bytes": path.stat().st_size,
            "sha256": sha256(path),
        }
        for path, sonames in dependencies.items()
    ]
    identities.sort(key=lambda item: (item["resolved_path"], item["sonames"]))
    if not identities:
        raise EvidenceError("dart-demos has no resolved build-tree dependencies")
    return identities


def _demo_identity(demo: Path) -> dict[str, Any]:
    demo = _require_file(demo, "DART demo binary")
    completed = _run(("ldd", demo), capture_output=True)
    build_root = (ROOT / "build").resolve()
    dependencies = _linked_build_library_identities(completed.stdout, build_root)
    core_candidates = [
        dependency
        for dependency in dependencies
        if any(
            soname == "libdart.so" or soname.startswith("libdart.so.")
            for soname in dependency["sonames"]
        )
    ]
    if len(core_candidates) != 1:
        raise EvidenceError(
            "dart-demos must resolve exactly one build-tree libdart.so, got "
            f"{len(core_candidates)}"
        )
    normalized_resolution = json.dumps(
        dependencies, allow_nan=False, separators=(",", ":"), sort_keys=True
    )
    return {
        **_file_identity(demo),
        "resolved_build_libdart": core_candidates[0],
        "resolved_build_dependencies": dependencies,
        "resolved_build_dependencies_sha256": hashlib.sha256(
            normalized_resolution.encode()
        ).hexdigest(),
    }


def _source_identity(
    *,
    runner_path: Path,
    demo: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
    font: Path,
) -> dict[str, Any]:
    sources = {
        "finalizer": Path(__file__).resolve(),
        "finalizer_test": FINALIZER_TEST,
        "visual_runner": runner_path,
        "visual_runner_test": RUNNER_TEST,
        "semantic_provenance_helper": PROVENANCE_HELPER,
        "semantic_provenance_test": PROVENANCE_TEST,
        "image_tools_helper": IMAGE_TOOLS_HELPER,
        "image_verdict_helper": IMAGE_VERDICT_HELPER,
        "image_verdict_test": IMAGE_VERDICT_TEST,
    }
    return {
        "sources": {name: _file_identity(path) for name, path in sources.items()},
        "demo_runtime": _demo_identity(demo),
        "finalizer_python_runtime": _file_identity(Path(sys.executable)),
        "capture_python": _file_identity(python),
        "ffmpeg": _tool_identity(ffmpeg),
        "ffprobe": _tool_identity(ffprobe),
        "font": _file_identity(font),
    }


def _validate_schedule_contract(runner: ModuleType, schedule: Any) -> None:
    expected = {
        "id": SCHEDULE_ID,
        "scene": "fbf_author_masonry_arch_101_standing_current_source",
        "source_segment": "masonry_arch_101",
        "solver_lane": "exact",
        "supported_solver_lanes": ("exact", "boxed"),
        "total_steps": EXPECTED_TOTAL_SUBSTEPS,
        "frame_stride": EXPECTED_FRAME_STRIDE,
        "time_step_seconds": EXPECTED_TIME_STEP_SECONDS,
        "collision_detector": "native",
        "collision_detector_override": False,
        "exact_fbf_required": True,
        "source_continuation_required": False,
        "actions": (),
        "crop": (660, 506, 260, 58),
    }
    for name, value in expected.items():
        actual = getattr(schedule, name)
        if name == "time_step_seconds":
            matches = math.isclose(actual, value, rel_tol=0.0, abs_tol=1e-15)
        else:
            matches = actual == value
        if not matches:
            raise EvidenceError(
                f"Figure 8 schedule contract changed: {name}={actual!r}"
            )
    if tuple(schedule.video_steps) != tuple(
        range(0, EXPECTED_TOTAL_SUBSTEPS + 1, EXPECTED_FRAME_STRIDE)
    ):
        raise EvidenceError("Figure 8 video-step schedule changed")
    if runner.EXACT_FBF_RESIDUAL_TOLERANCE != 1.0e-6:
        raise EvidenceError("exact-FBF residual tolerance changed")


def _require_terminal_signature(diagnostics: Any) -> dict[str, Any]:
    if not isinstance(diagnostics, dict):
        raise EvidenceError("exact terminal diagnostics are missing")
    signature: dict[str, Any] = {}
    for name, expected in EXPECTED_TERMINAL_SIGNATURE.items():
        actual = diagnostics.get(name)
        if isinstance(expected, float):
            matches = (
                not isinstance(actual, bool)
                and isinstance(actual, (int, float))
                and math.isfinite(float(actual))
                and math.isclose(float(actual), expected, rel_tol=0.0, abs_tol=0.0)
            )
        else:
            matches = (
                not isinstance(actual, bool)
                and isinstance(actual, int)
                and actual == expected
            )
        if not matches:
            raise EvidenceError(f"exact terminal signature changed: {name}={actual!r}")
        signature[name] = actual
    return signature


def _freeze_plan(schedule: Any, fail_step: int) -> dict[str, Any]:
    if fail_step != EXPECTED_FAIL_STEP:
        raise EvidenceError(f"exact fail step changed: {fail_step}")
    steps = tuple(step for step in schedule.capture_steps if step < fail_step)
    expected_steps = tuple(range(0, EXPECTED_LAST_RENDERED_STEP + 1, 8))
    if steps != expected_steps:
        raise EvidenceError(f"exact rendered prefix changed: {steps}")
    first_frozen = len(steps)
    if first_frozen >= OUTPUT_FRAME_COUNT:
        raise EvidenceError("exact prefix unexpectedly reaches the full horizon")
    return {
        "observed_steps": list(steps),
        "observed_frame_count": first_frozen,
        "last_rendered_step": steps[-1],
        "first_frozen_output_frame": first_frozen,
        "frozen_output_frame_count": OUTPUT_FRAME_COUNT - first_frozen,
        "frozen_source_step": steps[-1],
        "output_frame_count": OUTPUT_FRAME_COUNT,
    }


def _validate_frame_manifest(
    runner: ModuleType,
    schedule: Any,
    output_dir: Path,
    steps: Sequence[int],
) -> list[dict[str, Any]]:
    frames_dir = output_dir / "frames"
    if not frames_dir.is_dir() or frames_dir.is_symlink():
        raise EvidenceError(f"exact frame directory is invalid: {frames_dir}")
    expected_names = {f"step_{step:06d}.png" for step in steps}
    actual_names: set[str] = set()
    for entry in frames_dir.iterdir():
        if entry.is_symlink() or not entry.is_file():
            raise EvidenceError(f"exact frame entry is not a regular file: {entry}")
        actual_names.add(entry.name)
    if actual_names != expected_names:
        raise EvidenceError(
            "exact frame membership changed: "
            f"missing={sorted(expected_names - actual_names)}, "
            f"extra={sorted(actual_names - expected_names)}"
        )

    manifest: list[dict[str, Any]] = []
    viewport_hashes: set[str] = set()
    for step in steps:
        path = frames_dir / f"step_{step:06d}.png"
        try:
            verdict = runner._validate_png(path, schedule.width, schedule.height)
            viewport = runner._world_region_report(path, schedule)
        except (OSError, ValueError) as error:
            raise EvidenceError(f"{path}: invalid exact frame: {error}") from error
        viewport_hashes.add(viewport["sha256"])
        manifest.append(
            {
                "step": step,
                "path": str(path),
                "sha256": sha256(path),
                "non_blank": verdict["checks"]["non_blank"],
                "world_viewport": viewport,
            }
        )
    if schedule.expect_motion and len(viewport_hashes) < 2:
        raise EvidenceError("exact rendered prefix contains no visible motion")
    return manifest


def _validate_exact_prefix(
    runner: ModuleType,
    schedule: Any,
    *,
    output_root: Path,
    demo: Path,
) -> dict[str, Any]:
    output_dir = output_root / schedule.id
    try:
        failure = runner._validate_failed_exact_fbf_sidecar(
            schedule, output_dir, expected_demo=demo
        )
    except (OSError, ValueError) as error:
        raise EvidenceError(
            f"invalid exact expected-failure sidecar: {error}"
        ) from error
    if (
        failure.get("completed_steps") != EXPECTED_FAIL_STEP
        or failure.get("reason") != "iteration_cap"
    ):
        raise EvidenceError(
            "exact lane did not reproduce the expected step-209 iteration cap"
        )
    metrics = failure.get("author_masonry_arch_101_scene_state_metrics")
    required_metrics = {
        "sample_count": EXPECTED_FAIL_STEP + 1,
        "completed_substeps": EXPECTED_FAIL_STEP,
        "complete_horizon": False,
        "complete_trace_valid": False,
        "standing_outcome_valid": False,
        "lane_evidence_qualifies": False,
        "comparison_capture_valid": False,
        "physical_outcome_validated": False,
    }
    if not isinstance(metrics, dict) or any(
        metrics.get(name) != value for name, value in required_metrics.items()
    ):
        raise EvidenceError("exact failed-prefix physical outcome contract changed")

    timeline_path = output_dir / "timeline.json"
    timeline = read_json(timeline_path)
    terminal_signature = _require_terminal_signature(timeline.get("solver_diagnostics"))
    plan = _freeze_plan(schedule, failure["completed_steps"])
    frame_manifest = _validate_frame_manifest(
        runner, schedule, output_dir, plan["observed_steps"]
    )
    sidecar_contract = timeline.get("physics_contract")
    if not isinstance(sidecar_contract, dict):
        raise EvidenceError("exact sidecar physics contract is missing")
    try:
        live_provenance = runner._query_scene_physics_provenance(
            schedule, demo, expected_contract=sidecar_contract
        )
    except (OSError, ValueError) as error:
        raise EvidenceError(
            f"exact live physics provenance changed: {error}"
        ) from error
    if live_provenance.get("sidecar_contract_match") is not True:
        raise EvidenceError("exact live physics contract does not match the sidecar")
    if (output_dir / "metadata.json").exists():
        raise EvidenceError(
            "failed exact prefix must not masquerade as a capture result"
        )
    return {
        "schedule_id": schedule.id,
        "scene": schedule.scene,
        "timeline_path": str(timeline_path),
        "timeline_sha256": sha256(timeline_path),
        "completed_substeps": failure["completed_steps"],
        "required_substeps": schedule.total_steps,
        "fail_fast_reason": failure["reason"],
        "terminal_signature": terminal_signature,
        "scene_state_metrics": metrics,
        "freeze_plan": plan,
        "frame_manifest": frame_manifest,
        "frame_manifest_sha256": _payload_sha256(frame_manifest),
        "physics_contract_validation": failure["physics_contract"],
        "scene_physics_provenance": live_provenance,
        "complete_exact_trajectory": False,
        "standing_outcome_validated": False,
    }


def _boxed_summary(
    runner: ModuleType,
    schedule: Any,
    *,
    output_root: Path,
    demo: Path,
    ffmpeg: Path,
    ffprobe: Path,
) -> dict[str, Any]:
    boxed = runner._derive_boxed_schedule(schedule)
    if boxed.id != BOXED_SCHEDULE_ID:
        raise EvidenceError(f"boxed schedule identity changed: {boxed.id}")
    try:
        verified = runner._verify_existing(
            boxed,
            demo=demo,
            output_root=output_root,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
        )
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        raise EvidenceError(
            f"boxed member reuse verification failed: {error}"
        ) from error

    output_dir = output_root / boxed.id
    metadata_path = output_dir / "metadata.json"
    timeline_path = output_dir / "timeline.json"
    clip_path = output_dir / "clip.mp4"
    metadata = read_json(metadata_path)
    if metadata.get("schema_version") != runner.CAPTURE_RESULT_SCHEMA_VERSION:
        raise EvidenceError("boxed capture is not current provenance schema v2")
    metrics = verified.get("timeline", {}).get(
        "author_masonry_arch_101_scene_state_metrics"
    )
    required = {
        "solver_lane": "boxed",
        "sample_count": EXPECTED_TOTAL_SUBSTEPS + 1,
        "completed_substeps": EXPECTED_TOTAL_SUBSTEPS,
        "complete_horizon": True,
        "complete_trace_valid": True,
        "comparison_capture_valid": True,
    }
    if not isinstance(metrics, dict) or any(
        metrics.get(name) != value for name, value in required.items()
    ):
        raise EvidenceError("boxed complete comparison trace contract changed")
    standing = metrics.get("standing_outcome_valid")
    if not isinstance(standing, bool):
        raise EvidenceError("boxed standing outcome is not boolean")
    media = verified.get("media")
    if (
        not isinstance(media, list)
        or len(media) != 1
        or media[0].get("kind") != "mp4"
        or Path(media[0].get("path", "")) != clip_path
    ):
        raise EvidenceError("boxed verified MP4 contract changed")
    provenance = metadata.get("runtime", {}).get("scene_physics_provenance")
    if (
        not isinstance(provenance, dict)
        or provenance.get("sidecar_contract_match") is not True
    ):
        raise EvidenceError("boxed scene physics provenance is missing")
    return {
        "schedule_id": boxed.id,
        "scene": boxed.scene,
        "metadata_path": str(metadata_path),
        "metadata_sha256": sha256(metadata_path),
        "timeline_path": str(timeline_path),
        "timeline_sha256": sha256(timeline_path),
        "clip_path": str(clip_path),
        "clip_sha256": sha256(clip_path),
        "completed_substeps": metrics["completed_substeps"],
        "required_substeps": EXPECTED_TOTAL_SUBSTEPS,
        "complete_trace_valid": True,
        "comparison_capture_valid": True,
        "standing_outcome_valid": standing,
        "scene_state_metrics": metrics,
        "scene_physics_provenance": provenance,
        "reuse_verification_metadata_sha256": verified["metadata_sha256"],
    }


def _composition_spec(
    schedule: Any, exact: dict[str, Any], boxed: dict[str, Any], font: Path
) -> CompositionSpec:
    steps = tuple(exact["freeze_plan"]["observed_steps"])
    standing = boxed["standing_outcome_valid"]
    boxed_label = "BOXED LCP - COMPLETE 1600; STANDING ORACLE " + (
        "PASS" if standing else "FAIL"
    )
    crop_width, crop_height, crop_x, crop_y = schedule.crop
    spec = CompositionSpec(
        source_width=schedule.width,
        source_height=schedule.height,
        crop_width=crop_width,
        crop_height=crop_height,
        crop_x=crop_x,
        crop_y=crop_y,
        output_fps=OUTPUT_FPS,
        output_frame_count=OUTPUT_FRAME_COUNT,
        exact_steps=steps,
        boxed_label=boxed_label,
        font=_require_file(font, "label font"),
    )
    if (
        spec.first_frozen_output_frame != 27
        or spec.frozen_output_frame_count != 174
        or spec.duration_seconds != 6.7
    ):
        raise EvidenceError("Figure 8 frozen composition mapping changed")
    return spec


def _drawtext_filter(label: str, font: Path) -> str:
    def escape(value: str) -> str:
        return value.replace("\\", "\\\\").replace(":", "\\:").replace("'", "\\'")

    return (
        f"pad=iw:ih+{LABEL_BAND_HEIGHT}:0:{LABEL_BAND_HEIGHT}:color=0x181c20,"
        f"drawtext=fontfile='{escape(str(font))}':text='{escape(label)}':"
        "x=8:y=5:fontsize=14:fontcolor=white"
    )


def _probe_video(ffprobe: Path, path: Path) -> dict[str, Any]:
    completed = _run(
        (
            ffprobe,
            "-v",
            "error",
            "-show_entries",
            "format=duration:stream=codec_name,pix_fmt,width,height,"
            "r_frame_rate,nb_frames",
            "-of",
            "json",
            path,
        ),
        capture_output=True,
    )
    try:
        payload = json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        raise EvidenceError(f"{path}: ffprobe returned invalid JSON") from error
    streams = [stream for stream in payload.get("streams", []) if stream.get("width")]
    if len(streams) != 1:
        raise EvidenceError(f"{path}: expected exactly one video stream")
    return {"stream": streams[0], "format": payload.get("format", {})}


def _validate_video(
    ffmpeg: Path,
    ffprobe: Path,
    path: Path,
    *,
    width: int,
    height: int,
    fps: int,
    frame_count: int,
) -> dict[str, Any]:
    if not path.is_file() or path.is_symlink() or path.stat().st_size == 0:
        raise EvidenceError(f"{path}: video is missing, empty, or not regular")
    probe = _probe_video(ffprobe, path)
    stream = probe["stream"]
    expected = {
        "codec_name": "h264",
        "pix_fmt": "yuv420p",
        "width": width,
        "height": height,
        "r_frame_rate": f"{fps}/1",
        "nb_frames": str(frame_count),
    }
    if any(stream.get(name) != value for name, value in expected.items()):
        raise EvidenceError(f"{path}: video stream contract changed: {stream}")
    duration = probe["format"].get("duration")
    try:
        duration_value = float(duration)
    except (TypeError, ValueError) as error:
        raise EvidenceError(f"{path}: duration is unavailable") from error
    expected_duration = frame_count / fps
    if not math.isfinite(duration_value) or abs(duration_value - expected_duration) > (
        0.5 / fps
    ):
        raise EvidenceError(
            f"{path}: duration {duration_value} differs from {expected_duration}"
        )
    _run(
        (
            ffmpeg,
            "-hide_banner",
            "-loglevel",
            "error",
            "-i",
            path,
            "-f",
            "null",
            os.devnull,
        )
    )
    return {
        "path": path.name,
        "sha256": sha256(path),
        "size_bytes": path.stat().st_size,
        "stream": {name: stream[name] for name in expected},
        "duration_seconds": duration_value,
        "full_decode": "pass",
    }


def _codec_arguments() -> tuple[str, ...]:
    return (
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "medium",
        "-crf",
        "24",
        "-pix_fmt",
        "yuv420p",
        "-threads",
        "1",
        "-map_metadata",
        "-1",
        "-movflags",
        "+faststart",
    )


def _compose_media(
    *,
    exact_frames: Sequence[Path],
    boxed_clip: Path,
    exact_last_frame: Path,
    boxed_final_frame: Path,
    spec: CompositionSpec,
    ffmpeg: Path,
    ffprobe: Path,
    destination: Path,
) -> dict[str, Any]:
    if len(exact_frames) != spec.first_frozen_output_frame:
        raise EvidenceError("exact frame count differs from the frozen mapping")
    destination.mkdir(parents=True, exist_ok=False)
    staged_frames = destination / "exact_frames"
    staged_frames.mkdir()
    for index, source in enumerate(exact_frames):
        source = _require_file(source, "exact prefix frame")
        shutil.copyfile(source, staged_frames / f"frame_{index:06d}.png")

    exact_clip = destination / "_exact_frozen.mp4"
    exact_filter = (
        f"crop={spec.crop_width}:{spec.crop_height}:{spec.crop_x}:{spec.crop_y},"
        f"tpad=stop_mode=clone:stop=-1,trim=end_frame={spec.output_frame_count},"
        f"setpts=N/({spec.output_fps}*TB)"
    )
    _run(
        (
            ffmpeg,
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-framerate",
            str(spec.output_fps),
            "-start_number",
            "0",
            "-i",
            staged_frames / "frame_%06d.png",
            "-vf",
            exact_filter,
            "-r",
            str(spec.output_fps),
            "-frames:v",
            str(spec.output_frame_count),
            *_codec_arguments(),
            exact_clip,
        )
    )
    _validate_video(
        ffmpeg,
        ffprobe,
        exact_clip,
        width=spec.crop_width,
        height=spec.crop_height,
        fps=spec.output_fps,
        frame_count=spec.output_frame_count,
    )

    group_filter = ";".join(
        (
            f"[0:v]{_drawtext_filter(EXACT_LABEL, spec.font)}[exact]",
            f"[1:v]{_drawtext_filter(spec.boxed_label, spec.font)}[boxed]",
            "[exact][boxed]hstack=inputs=2:shortest=1[outv]",
        )
    )
    clip = destination / "clip.mp4"
    _run(
        (
            ffmpeg,
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-i",
            exact_clip,
            "-i",
            boxed_clip,
            "-filter_complex",
            group_filter,
            "-map",
            "[outv]",
            "-r",
            str(spec.output_fps),
            "-frames:v",
            str(spec.output_frame_count),
            *_codec_arguments(),
            clip,
        )
    )
    clip_report = _validate_video(
        ffmpeg,
        ffprobe,
        clip,
        width=spec.output_width,
        height=spec.output_height,
        fps=spec.output_fps,
        frame_count=spec.output_frame_count,
    )

    panel_filter = ";".join(
        (
            f"[0:v]crop={spec.crop_width}:{spec.crop_height}:"
            f"{spec.crop_x}:{spec.crop_y},"
            f"{_drawtext_filter(EXACT_LABEL, spec.font)}[exact]",
            f"[1:v]crop={spec.crop_width}:{spec.crop_height}:"
            f"{spec.crop_x}:{spec.crop_y},"
            f"{_drawtext_filter(spec.boxed_label, spec.font)}[boxed]",
            "[exact][boxed]hstack=inputs=2[outv]",
        )
    )
    panel = destination / "panel.png"
    _run(
        (
            ffmpeg,
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-i",
            exact_last_frame,
            "-i",
            boxed_final_frame,
            "-filter_complex",
            panel_filter,
            "-map",
            "[outv]",
            "-frames:v",
            "1",
            panel,
        )
    )
    if not panel.is_file() or panel.is_symlink() or panel.stat().st_size == 0:
        raise EvidenceError("Figure 8 diagnostic panel is missing or empty")
    panel_report = {
        "path": panel.name,
        "sha256": sha256(panel),
        "size_bytes": panel.stat().st_size,
        "width": spec.output_width,
        "height": spec.output_height,
        "exact_source_step": spec.exact_steps[-1],
        "boxed_source_step": EXPECTED_TOTAL_SUBSTEPS,
    }
    shutil.rmtree(staged_frames)
    exact_clip.unlink()
    return {
        "specification": spec.as_dict(),
        "exact_filter": exact_filter,
        "group_filter": group_filter,
        "panel_filter": panel_filter,
        "codec": {
            "name": "libx264",
            "preset": "medium",
            "crf": 24,
            "pixel_format": "yuv420p",
            "threads": 1,
            "metadata_removed": True,
            "faststart": True,
        },
        "clip": clip_report,
        "panel": panel_report,
    }


def _validate_claim_boundary(value: Any) -> None:
    if value != CLAIM_BOUNDARY:
        raise EvidenceError("Figure 8 claim boundary changed")


def _validate_group_identity(metadata: dict[str, Any], schedule: Any) -> None:
    if (
        metadata.get("schema_version") != SCHEMA_VERSION
        or metadata.get("kind") != KIND
        or metadata.get("group_id") != GROUP_ID
        or metadata.get("scene") != schedule.scene
        or metadata.get("schedule_id") != schedule.id
        or metadata.get("source_segment") != "masonry_arch_101"
        or metadata.get("member_order") != [schedule.id, BOXED_SCHEDULE_ID]
        or metadata.get("pass") is not True
        or metadata.get("actual_simulator") is not True
        or metadata.get("generated_scene_imagery") is not False
        or metadata.get("presentation_frame_repetition") is not True
    ):
        raise EvidenceError("Figure 8 group metadata identity changed")


def _validate_manual_keyframe_coverage(
    keyframes: Any, *, duration_seconds: float
) -> list[float]:
    frame_period = 1.0 / OUTPUT_FPS
    last_frame_time = duration_seconds - frame_period
    if (
        not isinstance(keyframes, list)
        or len(keyframes) < 3
        or any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or value < 0.0
            or value > last_frame_time + 1.0e-12
            for value in keyframes
        )
    ):
        raise EvidenceError("manual inspection keyframes are invalid")
    normalized = [float(value) for value in keyframes]
    if normalized != sorted(set(normalized)):
        raise EvidenceError("manual inspection keyframes must be sorted and unique")

    freeze_transition = 27.0 / OUTPUT_FPS
    if normalized[0] != 0.0:
        raise EvidenceError("manual inspection must include the initial frame")
    if not any(
        freeze_transition <= value <= freeze_transition + frame_period
        for value in normalized
    ):
        raise EvidenceError(
            "manual inspection must include the exact-prefix freeze transition"
        )
    if normalized[-1] < last_frame_time - 1.0:
        raise EvidenceError("manual inspection must include a late frozen-tail frame")
    return normalized


def _validate_manual_inspection(
    path: Path, *, clip: Path, panel: Path, duration_seconds: float
) -> dict[str, Any]:
    payload = read_json(path)
    expected_verdicts = {
        "labels_legible": True,
        "no_gross_cropping_or_render_failure": True,
        "frozen_exact_tail_explicitly_disclosed": True,
        "complete_exact_trajectory": False,
        "solver_superiority": False,
        "fig08_parity": False,
        "paper_parity": False,
    }
    if (
        payload.get("schema_version") != MANUAL_SCHEMA_VERSION
        or payload.get("manual_inspected") is not True
        or payload.get("pass") is not True
        or payload.get("clip_sha256") != sha256(clip)
        or payload.get("panel_sha256") != sha256(panel)
        or payload.get("verdicts") != expected_verdicts
    ):
        raise EvidenceError("Figure 8 manual-inspection contract changed")
    observation = payload.get("observation")
    keyframes = payload.get("inspected_keyframe_seconds")
    if not isinstance(observation, str) or not observation.strip():
        raise EvidenceError("manual inspection observation is empty")
    _validate_manual_keyframe_coverage(keyframes, duration_seconds=duration_seconds)
    return payload


def _comparison_paths(
    output_root: Path,
) -> tuple[Path, Path, Path]:
    exact_dir = output_root / SCHEDULE_ID
    boxed_dir = output_root / BOXED_SCHEDULE_ID
    group_dir = output_root / "groups" / GROUP_ID
    return exact_dir, boxed_dir, group_dir


def _expected_composition_inputs(
    output_root: Path, exact: dict[str, Any]
) -> tuple[list[Path], Path, Path, Path]:
    exact_frames = [Path(item["path"]) for item in exact["frame_manifest"]]
    exact_last = exact_frames[-1]
    boxed_dir = output_root / BOXED_SCHEDULE_ID
    return (
        exact_frames,
        boxed_dir / "clip.mp4",
        exact_last,
        boxed_dir / "frames" / f"step_{EXPECTED_TOTAL_SUBSTEPS:06d}.png",
    )


def _base_metadata(
    *,
    exact: dict[str, Any],
    boxed: dict[str, Any],
    composition: dict[str, Any],
    source_identity: dict[str, Any],
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "kind": KIND,
        "status": "mechanically_valid_pending_manual_inspection",
        "group_id": GROUP_ID,
        "scene": exact["scene"],
        "schedule_id": exact["schedule_id"],
        "source_segment": "masonry_arch_101",
        "member_order": [exact["schedule_id"], boxed["schedule_id"]],
        "exact_failed_prefix": exact,
        "boxed_complete_member": boxed,
        "composition": composition,
        "source_identity": source_identity,
        "claim_boundary": CLAIM_BOUNDARY,
        "actual_simulator": True,
        "generated_scene_imagery": False,
        "presentation_frame_repetition": True,
        "manual_inspection": {"required": True, "status": "pending"},
        "upload_ready": False,
        "pass": True,
    }


def _capture_expected_exact(
    runner: ModuleType,
    schedule: Any,
    *,
    demo: Path,
    output_root: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
) -> str:
    exact_dir = output_root / schedule.id
    if exact_dir.exists():
        raise EvidenceError(
            "exact destination already exists; refuse unsealed prefix reuse: "
            f"{exact_dir}"
        )
    capture_error: str | None = None
    try:
        runner.run_schedule(
            schedule,
            demo=demo,
            output_root=output_root,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
            python=python,
            allow_long=True,
        )
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        capture_error = str(error)
    if capture_error is None:
        raise EvidenceError(
            "exact Figure 8 trajectory unexpectedly completed; do not finalize "
            "it as an expected failure"
        )
    return capture_error


def finalize(args: argparse.Namespace) -> dict[str, Any]:
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    demo = _require_file(args.demo, "DART demo binary")
    runner_path = _require_file(args.runner, "visual evidence runner")
    ffmpeg = _require_file(args.ffmpeg, "ffmpeg")
    ffprobe = _require_file(args.ffprobe, "ffprobe")
    python = _require_file(args.python, "Python interpreter")
    font = _require_file(args.font, "label font")
    runner = _load_runner(runner_path)
    schedule = runner.SCHEDULES[SCHEDULE_ID]
    _validate_schedule_contract(runner, schedule)
    exact_dir, boxed_dir, group_dir = _comparison_paths(output_root)
    if not boxed_dir.is_dir():
        raise EvidenceError(
            "complete boxed capture must exist before Figure 8 finalization"
        )
    if group_dir.exists():
        raise EvidenceError(f"Figure 8 group already exists: {group_dir}")
    if exact_dir.exists():
        raise EvidenceError(
            "exact destination already exists; refuse unsealed prefix reuse: "
            f"{exact_dir}"
        )

    identity_before = _source_identity(
        runner_path=runner_path,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
        font=font,
    )
    capture_error = _capture_expected_exact(
        runner,
        schedule,
        demo=demo,
        output_root=output_root,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
    )
    identity_after = _source_identity(
        runner_path=runner_path,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
        font=font,
    )
    if identity_after != identity_before:
        raise EvidenceError("runtime/source identity changed during exact capture")

    exact = _validate_exact_prefix(
        runner,
        schedule,
        output_root=output_root,
        demo=demo,
    )
    boxed = _boxed_summary(
        runner,
        schedule,
        output_root=output_root,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
    )
    spec = _composition_spec(schedule, exact, boxed, font)
    exact_frames, boxed_clip, exact_last, boxed_final = _expected_composition_inputs(
        output_root, exact
    )

    groups_dir = output_root / "groups"
    groups_dir.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{GROUP_ID}.", dir=groups_dir))
    try:
        composition = _compose_media(
            exact_frames=exact_frames,
            boxed_clip=boxed_clip,
            exact_last_frame=exact_last,
            boxed_final_frame=boxed_final,
            spec=spec,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
            destination=staging / "output",
        )
        output_dir = staging / "output"
        metadata = _base_metadata(
            exact=exact,
            boxed=boxed,
            composition=composition,
            source_identity=identity_before,
        )
        write_json(output_dir / "metadata.json", metadata)
        os.replace(output_dir, group_dir)
    finally:
        if staging.exists():
            shutil.rmtree(staging)

    result = verify_bundle(
        output_root,
        demo=demo,
        runner_path=runner_path,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
        font=font,
        require_manual=False,
    )
    result["capture_error"] = capture_error
    return result


def _recompose_and_compare(
    *,
    output_root: Path,
    group_dir: Path,
    exact: dict[str, Any],
    boxed: dict[str, Any],
    spec: CompositionSpec,
    ffmpeg: Path,
    ffprobe: Path,
) -> dict[str, Any]:
    exact_frames, boxed_clip, exact_last, boxed_final = _expected_composition_inputs(
        output_root, exact
    )
    with tempfile.TemporaryDirectory(prefix="fig08-reverify-") as temporary:
        recomposed = _compose_media(
            exact_frames=exact_frames,
            boxed_clip=boxed_clip,
            exact_last_frame=exact_last,
            boxed_final_frame=boxed_final,
            spec=spec,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
            destination=Path(temporary) / "output",
        )
        stored_clip = group_dir / "clip.mp4"
        stored_panel = group_dir / "panel.png"
        if recomposed["clip"]["sha256"] != sha256(stored_clip) or recomposed["panel"][
            "sha256"
        ] != sha256(stored_panel):
            raise EvidenceError(
                "Figure 8 media does not match deterministic reconstruction"
            )
        return recomposed


def verify_bundle(
    output_root: Path,
    *,
    demo: Path,
    runner_path: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
    font: Path,
    require_manual: bool,
) -> dict[str, Any]:
    output_root = output_root.resolve()
    demo = _require_file(demo, "DART demo binary")
    runner_path = _require_file(runner_path, "visual evidence runner")
    ffmpeg = _require_file(ffmpeg, "ffmpeg")
    ffprobe = _require_file(ffprobe, "ffprobe")
    python = _require_file(python, "Python interpreter")
    font = _require_file(font, "label font")
    runner = _load_runner(runner_path)
    schedule = runner.SCHEDULES[SCHEDULE_ID]
    _validate_schedule_contract(runner, schedule)
    _, _, group_dir = _comparison_paths(output_root)
    if not group_dir.is_dir() or group_dir.is_symlink():
        raise EvidenceError(f"Figure 8 group directory is missing: {group_dir}")
    allowed_files = {"clip.mp4", "panel.png", "metadata.json"}
    if (group_dir / "manual-inspection.json").is_file():
        allowed_files.add("manual-inspection.json")
    actual_files = {
        path.name
        for path in group_dir.iterdir()
        if path.is_file() and not path.is_symlink()
    }
    if actual_files != allowed_files or any(
        path.is_symlink() or not path.is_file() for path in group_dir.iterdir()
    ):
        raise EvidenceError("Figure 8 group membership changed")

    metadata_path = group_dir / "metadata.json"
    metadata = read_json(metadata_path)
    _validate_group_identity(metadata, schedule)
    _validate_claim_boundary(metadata.get("claim_boundary"))

    current_identity = _source_identity(
        runner_path=runner_path,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
        font=font,
    )
    if metadata.get("source_identity") != current_identity:
        raise EvidenceError("Figure 8 runtime/source identity changed")

    exact = _validate_exact_prefix(runner, schedule, output_root=output_root, demo=demo)
    if metadata.get("exact_failed_prefix") != exact:
        raise EvidenceError("stored exact failed-prefix binding changed")
    boxed = _boxed_summary(
        runner,
        schedule,
        output_root=output_root,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
    )
    if metadata.get("boxed_complete_member") != boxed:
        raise EvidenceError("stored boxed member binding changed")
    spec = _composition_spec(schedule, exact, boxed, font)
    recomposed = _recompose_and_compare(
        output_root=output_root,
        group_dir=group_dir,
        exact=exact,
        boxed=boxed,
        spec=spec,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
    )
    if metadata.get("composition") != recomposed:
        raise EvidenceError("stored Figure 8 composition binding changed")

    manual_path = group_dir / "manual-inspection.json"
    manual_binding = metadata.get("manual_inspection")
    if require_manual:
        if (
            metadata.get("status") != "valid_current_dart_blocker_diagnostic"
            or metadata.get("upload_ready") is not True
            or not manual_path.is_file()
        ):
            raise EvidenceError("Figure 8 manual inspection is not sealed")
        manual = _validate_manual_inspection(
            manual_path,
            clip=group_dir / "clip.mp4",
            panel=group_dir / "panel.png",
            duration_seconds=spec.duration_seconds,
        )
        expected_binding = {
            "required": True,
            "status": "passed",
            "path": "manual-inspection.json",
            "sha256": sha256(manual_path),
            "pass": manual["pass"],
        }
        if manual_binding != expected_binding:
            raise EvidenceError("Figure 8 manual-inspection binding changed")
    else:
        pending_metadata = (
            metadata.get("status") == "mechanically_valid_pending_manual_inspection"
            and metadata.get("upload_ready") is False
            and manual_binding == {"required": True, "status": "pending"}
        )
        valid_sealed = (
            metadata.get("status") == "valid_current_dart_blocker_diagnostic"
            and metadata.get("upload_ready") is True
            and manual_path.is_file()
        )
        if not (pending_metadata or valid_sealed):
            raise EvidenceError("Figure 8 manual-inspection state is inconsistent")
        if manual_path.is_file():
            manual = _validate_manual_inspection(
                manual_path,
                clip=group_dir / "clip.mp4",
                panel=group_dir / "panel.png",
                duration_seconds=spec.duration_seconds,
            )
            if valid_sealed:
                expected_binding = {
                    "required": True,
                    "status": "passed",
                    "path": "manual-inspection.json",
                    "sha256": sha256(manual_path),
                    "pass": manual["pass"],
                }
                if manual_binding != expected_binding:
                    raise EvidenceError("Figure 8 manual-inspection binding changed")
    return {
        "schema_version": SCHEMA_VERSION,
        "status": metadata["status"],
        "group_id": GROUP_ID,
        "exact_completed_substeps": exact["completed_substeps"],
        "exact_fail_fast_reason": exact["fail_fast_reason"],
        "boxed_completed_substeps": boxed["completed_substeps"],
        "boxed_standing_outcome_valid": boxed["standing_outcome_valid"],
        "clip_sha256": recomposed["clip"]["sha256"],
        "panel_sha256": recomposed["panel"]["sha256"],
        "metadata_sha256": sha256(metadata_path),
        "manual_inspection_sealed": metadata["upload_ready"],
        "upload_ready": metadata["upload_ready"],
        "pass": True,
    }


def seal(args: argparse.Namespace) -> dict[str, Any]:
    output_root = args.output_root.resolve()
    _, _, group_dir = _comparison_paths(output_root)
    verify_bundle(
        output_root,
        demo=args.demo,
        runner_path=args.runner,
        ffmpeg=args.ffmpeg,
        ffprobe=args.ffprobe,
        python=args.python,
        font=args.font,
        require_manual=False,
    )
    metadata_path = group_dir / "metadata.json"
    metadata_before = read_json(metadata_path)
    if metadata_before.get("upload_ready") is True:
        raise EvidenceError("Figure 8 manual inspection is already sealed")
    keyframes = sorted(set(float(value) for value in args.keyframe_second))
    _validate_manual_keyframe_coverage(
        keyframes, duration_seconds=OUTPUT_FRAME_COUNT / OUTPUT_FPS
    )
    manual = {
        "schema_version": MANUAL_SCHEMA_VERSION,
        "manual_inspected": True,
        "pass": True,
        "clip_sha256": sha256(group_dir / "clip.mp4"),
        "panel_sha256": sha256(group_dir / "panel.png"),
        "inspected_keyframe_seconds": keyframes,
        "observation": args.observation.strip(),
        "verdicts": {
            "labels_legible": True,
            "no_gross_cropping_or_render_failure": True,
            "frozen_exact_tail_explicitly_disclosed": True,
            "complete_exact_trajectory": False,
            "solver_superiority": False,
            "fig08_parity": False,
            "paper_parity": False,
        },
    }
    if not manual["observation"]:
        raise EvidenceError("manual inspection observation is empty")
    manual_path = group_dir / "manual-inspection.json"
    manual_before = read_json(manual_path) if manual_path.is_file() else None
    metadata = dict(metadata_before)
    metadata["status"] = "valid_current_dart_blocker_diagnostic"
    metadata["manual_inspection"] = {
        "required": True,
        "status": "passed",
        "path": "manual-inspection.json",
        "pass": True,
    }
    metadata["upload_ready"] = True
    try:
        write_json(manual_path, manual)
        metadata["manual_inspection"]["sha256"] = sha256(manual_path)
        write_json(metadata_path, metadata)
        return verify_bundle(
            output_root,
            demo=args.demo,
            runner_path=args.runner,
            ffmpeg=args.ffmpeg,
            ffprobe=args.ffprobe,
            python=args.python,
            font=args.font,
            require_manual=True,
        )
    except BaseException:
        write_json(metadata_path, metadata_before)
        if manual_before is None:
            manual_path.unlink(missing_ok=True)
        else:
            write_json(manual_path, manual_before)
        raise


def _add_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--demo", type=Path, default=DEFAULT_DEMO)
    parser.add_argument("--runner", type=Path, default=DEFAULT_RUNNER)
    parser.add_argument("--ffmpeg", type=Path, default=DEFAULT_FFMPEG)
    parser.add_argument("--ffprobe", type=Path, default=DEFAULT_FFPROBE)
    parser.add_argument("--python", type=Path, default=DEFAULT_PYTHON)
    parser.add_argument("--font", type=Path, default=DEFAULT_FONT)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    finalize_parser = subparsers.add_parser(
        "finalize", help="capture the expected exact prefix and compose the diagnostic"
    )
    _add_common_arguments(finalize_parser)
    seal_parser = subparsers.add_parser(
        "seal", help="record a completed manual inspection and seal the diagnostic"
    )
    _add_common_arguments(seal_parser)
    seal_parser.add_argument("--observation", required=True)
    seal_parser.add_argument(
        "--keyframe-second",
        action="append",
        required=True,
        type=float,
        help="inspected clip timestamp; repeat for every inspected keyframe",
    )
    verify_parser = subparsers.add_parser(
        "verify", help="revalidate the sealed diagnostic without simulation"
    )
    _add_common_arguments(verify_parser)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "finalize":
            result = finalize(args)
        elif args.command == "seal":
            result = seal(args)
        else:
            result = verify_bundle(
                args.output_root,
                demo=args.demo,
                runner_path=args.runner,
                ffmpeg=args.ffmpeg,
                ffprobe=args.ffprobe,
                python=args.python,
                font=args.font,
                require_manual=True,
            )
    except (
        EvidenceError,
        OSError,
        ValueError,
        subprocess.CalledProcessError,
    ) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
