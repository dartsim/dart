#!/usr/bin/env python3
"""Capture and validate real DART visual evidence for the FBF paper scenes.

The schedules in this file are the machine-readable visual coverage contract.
They drive the real ``dart-demos`` executable through
DemoHost's deterministic completed-step timeline, validates every PNG and JSON
sidecar, then uses the repository image compositor and ffmpeg to make panels
and motion artifacts.  It never synthesizes or substitutes scene imagery.

Some paper cells do not yet have a parameterized DemoHost adapter.  Those cells
remain first-class schedules, but ``run`` refuses them instead of capturing a
different configuration under the requested label.  ``plan`` and ``--dry-run``
make those gaps machine-readable for the next implementation lane.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import math
import os
import shlex
import subprocess
import sys
import tempfile
from fractions import Fraction
from pathlib import Path
from typing import Any, Iterable, Sequence

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from _image_tools import ImageData, read_image  # noqa: E402
from image_verdict import analyze_non_blank, build_verdict  # noqa: E402

SCHEMA_VERSION = "dart.fbf_visual_evidence/v1"
SOURCE_AUDIT_SCHEMA_VERSION = "dart.fbf_visual_source_audit/v1"
SIDECAR_SCHEMA_VERSION = "dart.demos_headless_timeline/v1"
SOLVER_LANES = ("exact", "boxed")
SOLVER_COMPARISON_SUFFIX = "__exact_vs_boxed"
SOLVER_COMPARISON_LABELS = (
    "EXACT COULOMB FBF",
    "EXISTING BOXED LCP",
)
EXACT_SOLVER_NAME = "ExactCoulombFbfConstraintSolver"
BOXED_SOLVER_NAME = "BoxedLcpConstraintSolver"
BOXED_DIAGNOSTICS_GAP = "active solver does not expose exact-Coulomb FBF diagnostics"
HEADLESS_EXACT_FBF_FAIL_FAST_FLAG = "--headless-exact-fbf-fail-fast"
HEADLESS_EXACT_FBF_SOURCE_CONTINUATION_FLAG = "--headless-exact-fbf-source-continuation"
EXACT_FBF_RESIDUAL_TOLERANCE = 1e-6
LITERAL_MASONRY_ARCH_GEOMETRY_FINGERPRINT = "1ff65f2a99ec96d1"
EXACT_FBF_FAIL_FAST_REASONS = {
    "boxed_fallback",
    "exact_failure",
    "iteration_cap",
    "nonfinite_residual",
    "residual_tolerance_exceeded",
}
SOURCE_CONTINUATION_GATE_REASONS = {
    "source_continuation_not_requested",
    "boxed_fallback",
    "exact_failure",
    "cumulative_accounting_mismatch",
    "nonfinite_world_state",
    "group_history_truncated",
    "group_accounting_mismatch",
    "invalid_group_telemetry",
    "source_continuation_inactive",
    "nonfinite_group_residual",
    "nonfinite_group_step_size",
    "group_step_size_relation_mismatch",
    "termination_timing_mismatch",
    "plateau_telemetry_mismatch",
    "success_tolerance_not_strict",
    "unaccepted_group_outcome",
    "group_counter_mismatch",
    "last_group_telemetry_mismatch",
}
COVERAGE_MATRIX_PATH = (
    ROOT
    / "docs"
    / "dev_tasks"
    / "fbf_exact_coulomb_friction"
    / "PAPER_DEMO_VIDEO_MATRIX.md"
)
DEFAULT_DEMO = ROOT / "build/default/cpp/Release/bin/dart-demos"
DEFAULT_FFMPEG = ROOT / ".pixi/envs/gazebo/bin/ffmpeg"
DEFAULT_FFPROBE = ROOT / ".pixi/envs/gazebo/bin/ffprobe"
DEFAULT_OUTPUT_ROOT = Path("/tmp/fbf_visual_evidence")
AUDITED_VIDEO_SHA256 = (
    "d5356e1b31487be62b75af05efbfecdb70ad5d98501a8efd378fcedf066e4794"
)
AUDITED_TEASER_SHA256 = (
    "99527da7a84f7b9ac0031f794d9b16adadfba846d2165e7da22fd51d986c8db0"
)
AUDITED_PAPER_SHA256 = (
    "af7cb8df58288f4323fa4340e1590b09643b8702116a6c47cefe7ffa9a51e2f4"
)
CAPTURE_SEMANTIC_OUTCOME_GATE = (
    "manual inspection plus the separate physical trace/test contract is "
    "required; image motion/nonblank checks do not prove the expected outcome"
)


@dataclasses.dataclass(frozen=True)
class SourceSegment:
    id: str
    start_seconds: int
    end_seconds: int
    content: str
    layout: str


VIDEO_SEGMENTS = (
    SourceSegment("title", 0, 2, "paper title and authors", "title card"),
    SourceSegment(
        "backspin",
        2,
        24,
        "real-time, slow-motion, then Ours/MuJoCo/Kamino backspin",
        "single view followed by a horizontal three-column solver row",
    ),
    SourceSegment(
        "incline",
        24,
        35,
        "mu=0.4 sliding and mu=0.5 sticking",
        "two parameter cells with persistent labels",
    ),
    SourceSegment(
        "turntable",
        35,
        50,
        "four simultaneous friction/angular-speed cells",
        "source order is top: mu=.2 omega=2,5; bottom: mu=.5 omega=2,5",
    ),
    SourceSegment(
        "painleve",
        50,
        60,
        "mu=0.5 slide/rest and mu=0.55 shorter-travel/tumble",
        "two parameter cells, with solver identity shown in the edit",
    ),
    SourceSegment(
        "card_house_26",
        60,
        67,
        "four-level card-house settle and projectile impact",
        "Ours/MuJoCo/Kamino horizontal three-column row",
    ),
    SourceSegment(
        "masonry_arch_25",
        67,
        74,
        "25-stone arch settle and crown impact",
        "Ours/MuJoCo/Kamino horizontal three-column row",
    ),
    SourceSegment(
        "masonry_arch_101",
        74,
        80,
        "101-stone long-run comparison",
        "Ours/Kamino horizontal two-column row",
    ),
    SourceSegment("closing", 80, 82, "Thank You", "closing card"),
)

# Title and closing cards contain no simulation. Every other source-video
# segment must have an explicit capture schedule. The author-pinned turntable
# matrix is the primary four-cell reconstruction; the earlier paper-proxy
# matrix remains available as a separate diagnostic lane.
REQUIRED_VIDEO_SCHEDULES = {
    "backspin": ("backspin",),
    "incline": ("incline",),
    "turntable": (
        "turntable_author_mu02_omega2",
        "turntable_author_mu02_omega5",
        "turntable_author_mu05_omega2",
        "turntable_author_mu05_omega5",
    ),
    "painleve": ("painleve_author_mu05", "painleve_author_mu055"),
    "card_house_26": (
        "card_house_author_4_impact_current_source",
        "card_house_author_4_impact_source_continuation_current_source",
        "card_house_26",
    ),
    "masonry_arch_25": (
        "masonry_arch_25_literal_standing",
        "masonry_arch_25_author_crown_impact_current_source",
        "masonry_arch_25",
    ),
    "masonry_arch_101": ("masonry_arch_101_author_standing_current_source",),
}


@dataclasses.dataclass(frozen=True)
class ScheduledAction:
    step: int
    key: str
    purpose: str

    def __post_init__(self) -> None:
        if self.step < 0:
            raise ValueError("action step must be nonnegative")
        if len(self.key) != 1:
            raise ValueError("DemoHost actions require one key character")


@dataclasses.dataclass(frozen=True)
class CaptureSchedule:
    id: str
    scene: str
    title: str
    source_segment: str
    total_steps: int
    frame_stride: int
    panel_steps: tuple[int, ...]
    panel_labels: tuple[str, ...]
    configuration: tuple[tuple[str, str], ...]
    mismatches: tuple[str, ...]
    known_gate_blockers: tuple[str, ...] = ()
    boxed_comparison_mismatches: tuple[str, ...] | None = None
    boxed_comparison_gate_blockers: tuple[str, ...] | None = None
    actions: tuple[ScheduledAction, ...] = ()
    time_step_seconds: float = 1.0 / 60.0
    width: int = 1280
    height: int = 720
    collision_detector: str = "dart"
    collision_detector_override: bool = True
    threads: int = 1
    output_fps: int = 30
    encode_mp4: bool = True
    encode_gif: bool = False
    exact_fbf_required: bool = True
    source_continuation_required: bool = False
    expect_motion: bool = True
    long_run: bool = False
    runnable: bool = True
    adapter_gap: str | None = None
    solver_lane: str = "exact"
    supported_solver_lanes: tuple[str, ...] = SOLVER_LANES
    source_schedule_id: str | None = None
    pre_run_actions: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if self.total_steps < 0:
            raise ValueError(f"{self.id}: total_steps must be nonnegative")
        if (
            isinstance(self.time_step_seconds, bool)
            or not isinstance(self.time_step_seconds, (int, float))
            or not math.isfinite(self.time_step_seconds)
            or self.time_step_seconds <= 0.0
        ):
            raise ValueError(
                f"{self.id}: time_step_seconds must be finite and positive"
            )
        if self.frame_stride <= 0:
            raise ValueError(f"{self.id}: frame_stride must be positive")
        if self.width <= 620 or self.height <= 214:
            raise ValueError(f"{self.id}: dimensions cannot expose the world viewport")
        if not self.collision_detector:
            raise ValueError(f"{self.id}: collision detector must be explicit")
        if not isinstance(self.collision_detector_override, bool):
            raise ValueError(f"{self.id}: collision override flag must be boolean")
        if len(self.panel_steps) != len(self.panel_labels):
            raise ValueError(f"{self.id}: panel steps and labels differ")
        if tuple(sorted(set(self.panel_steps))) != self.panel_steps:
            raise ValueError(f"{self.id}: panel steps must be sorted and unique")
        if any(step < 0 or step > self.total_steps for step in self.panel_steps):
            raise ValueError(f"{self.id}: a panel step is outside the trajectory")
        if any(action.step > self.total_steps for action in self.actions):
            raise ValueError(f"{self.id}: an action is outside the trajectory")
        if self.runnable == (self.adapter_gap is not None):
            raise ValueError(
                f"{self.id}: runnable schedules have no adapter gap and blocked "
                "schedules must explain one"
            )
        if self.solver_lane not in SOLVER_LANES:
            raise ValueError(f"{self.id}: unsupported solver lane {self.solver_lane!r}")
        if (
            not self.supported_solver_lanes
            or len(set(self.supported_solver_lanes)) != len(self.supported_solver_lanes)
            or any(lane not in SOLVER_LANES for lane in self.supported_solver_lanes)
        ):
            raise ValueError(f"{self.id}: invalid supported solver lanes")
        if self.solver_lane not in self.supported_solver_lanes:
            raise ValueError(
                f"{self.id}: active solver lane is not listed as supported"
            )
        if self.solver_lane == "boxed" and self.exact_fbf_required:
            raise ValueError(f"{self.id}: boxed lane cannot require exact FBF")
        if self.source_continuation_required and not self.exact_fbf_required:
            raise ValueError(
                f"{self.id}: source continuation requires the exact-FBF lane"
            )
        if self.source_schedule_id == self.id:
            raise ValueError(f"{self.id}: derived source schedule cannot equal its id")
        if any(len(key) != 1 for key in self.pre_run_actions):
            raise ValueError(f"{self.id}: pre-run actions require one key character")
        if not self.mismatches:
            raise ValueError(f"{self.id}: source/reconstruction gaps must be explicit")

    @property
    def video_steps(self) -> tuple[int, ...]:
        if not self.encode_mp4 and not self.encode_gif:
            return ()
        steps = set(range(0, self.total_steps + 1, self.frame_stride))
        steps.add(self.total_steps)
        return tuple(sorted(steps))

    @property
    def capture_steps(self) -> tuple[int, ...]:
        steps = set(self.video_steps)
        steps.update(self.panel_steps)
        steps.update(action.step for action in self.actions)
        return tuple(sorted(steps))

    @property
    def time_step_fraction(self) -> Fraction:
        return Fraction(self.time_step_seconds).limit_denominator(1_000_000_000)

    def time_at_step(self, step: int) -> float:
        return step * self.time_step_seconds

    @property
    def crop(self) -> tuple[int, int, int, int]:
        # DemoHost's evidence captures reserve 260 px on the left, 360 px on
        # the right, 58 px above, and 156 px below the fixed world viewport.
        return (self.width - 620, self.height - 214, 260, 58)

    def configuration_dict(self) -> dict[str, str]:
        return dict(self.configuration)


COMMON_DART_MISMATCH = (
    "DART renders its own float64 scene reconstructions under DART collision/"
    "scene contracts, not outputs from the authors' public Warp/Newton "
    "reference implementation or historical paper rendering setup.",
    "Each artifact contains one declared DART solver lane; faithful synchronized "
    "external-solver renderings and approved source goldens remain separate "
    "evidence.",
)


SCHEDULES: dict[str, CaptureSchedule] = {
    "incline": CaptureSchedule(
        id="incline",
        scene="fbf_paper_incline",
        title="Incline threshold pair",
        source_segment="incline",
        total_steps=120,
        frame_stride=2,
        panel_steps=(0, 30, 60, 90, 120),
        panel_labels=("t=0.00s", "t=0.50s", "t=1.00s", "t=1.50s", "t=2.00s"),
        configuration=(("mu_cells", "0.4,0.5"), ("tan_theta", "0.5")),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The combined DART render reports eight contacts per post-initial "
            "step in aggregate and does not expose a per-cell split; independent "
            "single-cell traces report three contacts per post-initial step, "
            "versus four per cell in the paper timing row.",
            "The paper comparison edit and external panels last 11 seconds; this "
            "artifact is the underlying two-second DART trajectory.",
        ),
    ),
    "backspin": CaptureSchedule(
        id="backspin",
        scene="fbf_paper_backspin",
        title="Backspin reversal",
        source_segment="backspin",
        total_steps=130,
        frame_stride=1,
        panel_steps=(0, 10, 50, 120, 130),
        panel_labels=("t=0.00s", "t=0.17s", "t=0.83s", "t=2.00s", "t=2.17s"),
        configuration=(
            ("radius_m", "0.25"),
            ("initial_linear_velocity_m_s", "4"),
            ("initial_angular_velocity_rad_s", "-200"),
            ("mu", "0.5"),
            (
                "orientation_cue",
                "renderer-applied high-contrast 6x4 checker texture with "
                "coral registration tile",
            ),
        ),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The 22-second source segment adds slow motion, narration cards, and "
            "a horizontal three-solver row; this is the 2.17-second DART "
            "physical run.",
            "Camera, materials, and lighting are DART reconstructions rather than "
            "the source renderer.",
            "The checker-textured UV mesh is VisualAspect-only and does not "
            "participate in collision, inertia, friction, or dynamics.",
        ),
        width=1920,
        encode_gif=True,
    ),
    "turntable_mu02_omega2": CaptureSchedule(
        id="turntable_mu02_omega2",
        scene="fbf_paper_turntable_mu_0_2_omega_2",
        title="Turntable mu=.2 omega=2",
        source_segment="turntable",
        total_steps=240,
        frame_stride=4,
        panel_steps=(0, 60, 120, 180, 240),
        panel_labels=("t=0s", "t=1s", "t=2s", "t=3s", "t=4s"),
        configuration=(("mu", "0.2"), ("omega_rad_s", "2"), ("outcome", "ejected")),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The DART support is square instead of the source's segmented disk.",
            "The one-second angular-speed ramp, cube properties, and four-second "
            "capture horizon are documented reconstruction choices.",
        ),
    ),
    "turntable_mu02_omega5": CaptureSchedule(
        id="turntable_mu02_omega5",
        scene="fbf_paper_turntable_mu_0_2_omega_5",
        title="Turntable mu=.2 omega=5",
        source_segment="turntable",
        total_steps=240,
        frame_stride=4,
        panel_steps=(0, 60, 120, 180, 240),
        panel_labels=("t=0s", "t=1s", "t=2s", "t=3s", "t=4s"),
        configuration=(("mu", "0.2"), ("omega_rad_s", "5"), ("outcome", "ejected")),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The DART support is square instead of the source's segmented disk.",
            "The one-second angular-speed ramp, cube properties, and four-second "
            "capture horizon are documented reconstruction choices.",
        ),
    ),
    "turntable_mu05_omega2": CaptureSchedule(
        id="turntable_mu05_omega2",
        scene="fbf_paper_turntable",
        title="Turntable mu=.5 omega=2",
        source_segment="turntable",
        total_steps=240,
        frame_stride=4,
        panel_steps=(0, 60, 120, 180, 240),
        panel_labels=("t=0s", "t=1s", "t=2s", "t=3s", "t=4s"),
        configuration=(("mu", "0.5"), ("omega_rad_s", "2"), ("outcome", "captured")),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The DART support is square instead of the source's segmented disk.",
            "The one-second angular-speed ramp, cube properties, and four-second "
            "capture horizon are documented reconstruction choices.",
        ),
    ),
    "turntable_mu05_omega5": CaptureSchedule(
        id="turntable_mu05_omega5",
        scene="fbf_paper_turntable_mu_0_5_omega_5",
        title="Turntable mu=.5 omega=5",
        source_segment="turntable",
        total_steps=240,
        frame_stride=4,
        panel_steps=(0, 60, 120, 180, 240),
        panel_labels=("t=0s", "t=1s", "t=2s", "t=3s", "t=4s"),
        configuration=(("mu", "0.5"), ("omega_rad_s", "5"), ("outcome", "ejected")),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The DART support is square instead of the source's segmented disk.",
            "The one-second angular-speed ramp, cube properties, and four-second "
            "capture horizon are documented reconstruction choices.",
        ),
    ),
    "turntable_author_mu02_omega2": CaptureSchedule(
        id="turntable_author_mu02_omega2",
        scene="fbf_author_turntable_mu_0_2_omega_2",
        title="Author-pinned turntable mu=.2 omega=2",
        source_segment="turntable",
        total_steps=360,
        frame_stride=2,
        panel_steps=(0, 30, 60, 180, 360),
        panel_labels=("t=0s", "settled t=.5s", "ramped t=1s", "t=3s", "t=6s"),
        configuration=(
            ("author_commit", "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"),
            ("mu", "0.2"),
            ("omega_rad_s", "2"),
            ("outcome", "ejected"),
            ("drop_height_m", "0.2"),
            ("settle_seconds", "0.5"),
            ("smoothstep_ramp_seconds", "0.5"),
        ),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The DART scene ports the public author geometry and schedule, but "
            "DART's float64 dynamics/collision path is not Warp/Newton trace parity.",
            "The author repository exports USD state but supplies no historical "
            "paper camera, materials, lighting, or approved frame golden.",
            "The 5 mm author collision-gap setting is represented as geometric "
            "separation in DART; backend margin semantics are not equivalent.",
        ),
        collision_detector="native",
        collision_detector_override=False,
        supported_solver_lanes=("exact",),
    ),
    "turntable_author_mu02_omega5": CaptureSchedule(
        id="turntable_author_mu02_omega5",
        scene="fbf_author_turntable_mu_0_2_omega_5",
        title="Author-pinned turntable mu=.2 omega=5",
        source_segment="turntable",
        total_steps=360,
        frame_stride=2,
        panel_steps=(0, 30, 60, 180, 360),
        panel_labels=("t=0s", "settled t=.5s", "ramped t=1s", "t=3s", "t=6s"),
        configuration=(
            ("author_commit", "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"),
            ("mu", "0.2"),
            ("omega_rad_s", "5"),
            ("outcome", "ejected"),
            ("drop_height_m", "0.2"),
            ("settle_seconds", "0.5"),
            ("smoothstep_ramp_seconds", "0.5"),
        ),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The DART scene ports the public author geometry and schedule, but "
            "DART's float64 dynamics/collision path is not Warp/Newton trace parity.",
            "The author repository exports USD state but supplies no historical "
            "paper camera, materials, lighting, or approved frame golden.",
            "The 5 mm author collision-gap setting is represented as geometric "
            "separation in DART; backend margin semantics are not equivalent.",
        ),
        collision_detector="native",
        collision_detector_override=False,
        supported_solver_lanes=("exact",),
    ),
    "turntable_author_mu05_omega2": CaptureSchedule(
        id="turntable_author_mu05_omega2",
        scene="fbf_author_turntable_mu_0_5_omega_2",
        title="Author-pinned turntable mu=.5 omega=2",
        source_segment="turntable",
        total_steps=360,
        frame_stride=2,
        panel_steps=(0, 30, 60, 180, 360),
        panel_labels=("t=0s", "settled t=.5s", "ramped t=1s", "t=3s", "t=6s"),
        configuration=(
            ("author_commit", "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"),
            ("mu", "0.5"),
            ("omega_rad_s", "2"),
            ("outcome", "retained_through_6s"),
            ("drop_height_m", "0.2"),
            ("settle_seconds", "0.5"),
            ("smoothstep_ramp_seconds", "0.5"),
        ),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The DART scene ports the public author geometry and schedule, but "
            "DART's float64 dynamics/collision path is not Warp/Newton trace parity.",
            "The author repository exports USD state but supplies no historical "
            "paper camera, materials, lighting, or approved frame golden.",
            "The 5 mm author collision-gap setting is represented as geometric "
            "separation in DART; backend margin semantics are not equivalent.",
        ),
        collision_detector="native",
        collision_detector_override=False,
        supported_solver_lanes=("exact",),
    ),
    "turntable_author_mu05_omega5": CaptureSchedule(
        id="turntable_author_mu05_omega5",
        scene="fbf_author_turntable_mu_0_5_omega_5",
        title="Author-pinned turntable mu=.5 omega=5",
        source_segment="turntable",
        total_steps=360,
        frame_stride=2,
        panel_steps=(0, 30, 60, 180, 360),
        panel_labels=("t=0s", "settled t=.5s", "ramped t=1s", "t=3s", "t=6s"),
        configuration=(
            ("author_commit", "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"),
            ("mu", "0.5"),
            ("omega_rad_s", "5"),
            ("outcome", "ejected"),
            ("drop_height_m", "0.2"),
            ("settle_seconds", "0.5"),
            ("smoothstep_ramp_seconds", "0.5"),
        ),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The DART scene ports the public author geometry and schedule, but "
            "DART's float64 dynamics/collision path is not Warp/Newton trace parity.",
            "The author repository exports USD state but supplies no historical "
            "paper camera, materials, lighting, or approved frame golden.",
            "The 5 mm author collision-gap setting is represented as geometric "
            "separation in DART; backend margin semantics are not equivalent.",
        ),
        collision_detector="native",
        collision_detector_override=False,
        supported_solver_lanes=("exact",),
    ),
    "painleve_mu05": CaptureSchedule(
        id="painleve_mu05",
        scene="fbf_paper_painleve",
        title="Painleve proxy mu=.5",
        source_segment="painleve",
        total_steps=150,
        frame_stride=2,
        panel_steps=(0, 30, 60, 90, 120, 150),
        panel_labels=("t=0s", "t=.5s", "t=1s", "t=1.5s", "t=2s", "t=2.5s"),
        configuration=(("mu", "0.5"), ("outcome", "slide/rest proxy")),
        mismatches=COMMON_DART_MISMATCH
        + (
            "This is explicitly a Painleve-style proxy: the paper does not publish "
            "the box dimensions, mass, launch state, or absolute timestamps.",
            "The source uses a three-solver semantic edit; this artifact is the "
            "underlying DART proxy only.",
        ),
    ),
    "painleve_mu055": CaptureSchedule(
        id="painleve_mu055",
        scene="fbf_paper_painleve_mu_0_55",
        title="Painleve proxy mu=.55",
        source_segment="painleve",
        total_steps=150,
        frame_stride=2,
        panel_steps=(0, 30, 60, 90, 120, 150),
        panel_labels=("t=0s", "t=.5s", "t=1s", "t=1.5s", "t=2s", "t=2.5s"),
        configuration=(("mu", "0.55"), ("outcome", "shorter-travel/tumble proxy")),
        mismatches=COMMON_DART_MISMATCH
        + (
            "This is explicitly a Painleve-style proxy: the paper does not publish "
            "the box dimensions, mass, launch state, or absolute timestamps.",
            "The source uses a three-solver semantic edit; this artifact would be "
            "the underlying DART proxy only.",
        ),
    ),
    "painleve_author_mu05": CaptureSchedule(
        id="painleve_author_mu05",
        scene="fbf_author_painleve_mu_0_5",
        title="Author-pinned Painleve mu=.5",
        source_segment="painleve",
        total_steps=120,
        frame_stride=2,
        panel_steps=(0, 30, 60, 90, 120),
        panel_labels=("t=0s", "t=.5s", "t=1s", "t=1.5s", "t=2s"),
        configuration=(
            ("author_commit", "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"),
            ("author_run_blob", "afaa03613b0ad0a30290168d2fd64221fc3523b7"),
            ("mu", "0.5"),
            ("source_default_mu", "0.55"),
            ("selection", "source-supported --mu 0.5 0.55 sweep"),
            ("box_size_m", "0.3 x 1.2 x 0.6 (DART xyz)"),
            ("density_kg_m3", "200"),
            ("mass_kg", "43.2"),
            ("initial_velocity_m_s", "4"),
            ("time_step_seconds", "1/60"),
            ("duration_seconds", "2"),
            (
                "outcome",
                "upright_near_rest under the current exact DART adapter",
            ),
        ),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The source-supported mu=.5/.55 sweep is selected for the public "
            "paper-video cells; the public source defaults only to mu=.55 and "
            "does not identify the historical paper invocation.",
            "DART Native FourPointPlanar contact does not implement the source "
            "gap=.005, ke=1e4, or kd=1e3 shape semantics equivalently.",
            "The exact lane maps the public CLI gamma_c=5 request onto DART's "
            "adaptive safe-step convention and uses a 1,000-iteration DART "
            "budget; this is not a port of the authors' Warp/Newton backend.",
        ),
        known_gate_blockers=(),
        boxed_comparison_gate_blockers=(),
        collision_detector="native",
        collision_detector_override=False,
        pre_run_actions=("e", "e"),
    ),
    "painleve_author_mu055": CaptureSchedule(
        id="painleve_author_mu055",
        scene="fbf_author_painleve_mu_0_55",
        title="Author-pinned Painleve mu=.55",
        source_segment="painleve",
        total_steps=120,
        frame_stride=2,
        panel_steps=(0, 30, 60, 90, 120),
        panel_labels=("t=0s", "t=.5s", "t=1s", "t=1.5s", "t=2s"),
        configuration=(
            ("author_commit", "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"),
            ("author_run_blob", "afaa03613b0ad0a30290168d2fd64221fc3523b7"),
            ("mu", "0.55"),
            ("source_default_mu", "0.55"),
            ("selection", "source-supported --mu 0.5 0.55 sweep"),
            ("box_size_m", "0.3 x 1.2 x 0.6 (DART xyz)"),
            ("density_kg_m3", "200"),
            ("mass_kg", "43.2"),
            ("initial_velocity_m_s", "4"),
            ("time_step_seconds", "1/60"),
            ("duration_seconds", "2"),
            (
                "outcome",
                "tumbled_near_rest under the current exact DART adapter",
            ),
        ),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The source-supported mu=.5/.55 sweep is selected for the public "
            "paper-video cells; the public source defaults only to mu=.55 and "
            "does not identify the historical paper invocation.",
            "DART Native FourPointPlanar contact does not implement the source "
            "gap=.005, ke=1e4, or kd=1e3 shape semantics equivalently.",
            "The exact lane maps the public CLI gamma_c=5 request onto DART's "
            "adaptive safe-step convention and uses a 1,000-iteration DART "
            "budget; this is not a port of the authors' Warp/Newton backend.",
        ),
        known_gate_blockers=(),
        boxed_comparison_gate_blockers=(),
        collision_detector="native",
        collision_detector_override=False,
        pre_run_actions=("e", "e"),
    ),
    "card_house_author_4_impact_current_source": CaptureSchedule(
        id="card_house_author_4_impact_current_source",
        scene="fbf_author_card_house_4_impact_current_source",
        title="Author-pinned four-level card-house impact candidate",
        source_segment="card_house_26",
        total_steps=2400,
        # One 30 fps evidence frame per eight 240 Hz substeps preserves the
        # selected source invocation's ten-second presentation.
        frame_stride=8,
        panel_steps=(0, 480, 1600, 1680, 2400),
        panel_labels=(
            "initial source configuration",
            "standing probe t=2s",
            "pre-frame-400 release boundary (t=6.67s)",
            "post-release probe t=7s",
            "selected frame-600 endpoint (t=10s)",
        ),
        configuration=(
            ("author_commit", "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"),
            ("levels", "4"),
            ("cards", "26"),
            ("cubes", "4; initially kinematic"),
            ("mu", "0.8"),
            ("simulation_time_step_seconds", "1/240"),
            ("display_time_step_seconds", "1/60"),
            ("substeps_per_display_frame", "4"),
            ("release_substep", "1600"),
            ("total_substeps", "2400"),
            (
                "capture_invocation",
                "source-supported levels=4 frames=600 drop_frame=400 selection",
            ),
        ),
        mismatches=(
            COMMON_DART_MISMATCH[0],
            "This DART scene ports the public author geometry, initial state, and "
            "source-supported four-level/600-frame argument selection; DART Native "
            "collision and float64 dynamics are not the authors' Warp/Newton "
            "float32 backend.",
            "The public source defaults to five levels and 800 frames. The "
            "four-level/600-frame invocation uses supported run.py arguments but "
            "is not a recovered historical paper command.",
            "The author repository supplies no historical paper camera, materials, "
            "lighting, approved frame golden, or DART trajectory oracle.",
            "The source's 5 mm collision-gap setting is recorded but DART Native "
            "collision does not claim equivalent gap semantics.",
        ),
        known_gate_blockers=(
            "No complete strict DART exact-FBF run of this 2,400-substep release "
            "schedule has yet passed the fail-fast solver and physical-outcome "
            "gates.",
            "No paired exact/boxed media has yet been captured and independently "
            "validated for this source-parameterized adapter.",
        ),
        actions=(ScheduledAction(1600, "p", "release the four existing source cubes"),),
        time_step_seconds=1.0 / 240.0,
        collision_detector="native",
        collision_detector_override=False,
        long_run=True,
    ),
    "card_house_author_4_impact_source_continuation_current_source": CaptureSchedule(
        id="card_house_author_4_impact_source_continuation_current_source",
        scene=("fbf_author_card_house_4_impact_source_continuation_current_source"),
        title="Author-pinned four-level card-house source-continuation lane",
        source_segment="card_house_26",
        total_steps=2400,
        frame_stride=8,
        panel_steps=(0, 480, 1600, 1680, 2400),
        panel_labels=(
            "initial source configuration",
            "standing probe t=2s",
            "pre-frame-400 release boundary (t=6.67s)",
            "post-release probe t=7s",
            "selected frame-600 endpoint (t=10s)",
        ),
        configuration=(
            ("author_commit", "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"),
            ("levels", "4"),
            ("cards", "26"),
            ("cubes", "4; initially kinematic"),
            ("mu", "0.8"),
            ("simulation_time_step_seconds", "1/240"),
            ("display_time_step_seconds", "1/60"),
            ("substeps_per_display_frame", "4"),
            ("release_substep", "1600"),
            ("total_substeps", "2400"),
            ("exact_policy", "source_continuation"),
            ("residual_check_interval", "5"),
            ("plateau_patience", "5"),
            ("plateau_relative_tolerance", "0.01"),
            ("step_size_backtrack_limit", "8"),
            ("coupling_variation_skip_threshold", "1e-10"),
            (
                "capture_invocation",
                "source-supported levels=4 frames=600 drop_frame=400 selection",
            ),
        ),
        mismatches=(
            COMMON_DART_MISMATCH[0],
            "This separate exact lane ports source continuation decisions into "
            "the DART adapter; it does not reproduce the authors' Warp/Newton "
            "float32 collision or linear-algebra backend.",
            "The public source defaults to five levels and 800 frames. The "
            "four-level/600-frame invocation uses supported run.py arguments but "
            "is not a recovered historical paper command.",
            "The author repository supplies no historical paper camera, materials, "
            "lighting, approved frame golden, or DART trajectory oracle.",
            "The source's 5 mm collision-gap setting is recorded but DART Native "
            "collision does not claim equivalent gap semantics.",
        ),
        boxed_comparison_mismatches=(
            COMMON_DART_MISMATCH[0],
            "This same-physics comparison selects DART's existing boxed-LCP "
            "solver; it is comparison media, not exact-FBF evidence or a port of "
            "the authors' Warp/Newton backend.",
            "The public source defaults to five levels and 800 frames. The "
            "four-level/600-frame invocation uses supported run.py arguments but "
            "is not a recovered historical paper command.",
            "The author repository supplies no historical paper camera, materials, "
            "lighting, approved frame golden, or DART trajectory oracle.",
            "The source's 5 mm collision-gap setting is recorded but DART Native "
            "collision does not claim equivalent gap semantics.",
        ),
        boxed_comparison_gate_blockers=(),
        actions=(ScheduledAction(1600, "p", "release the four existing source cubes"),),
        time_step_seconds=1.0 / 240.0,
        collision_detector="native",
        collision_detector_override=False,
        source_continuation_required=True,
        long_run=True,
    ),
    "card_house_26": CaptureSchedule(
        id="card_house_26",
        scene="fbf_paper_card_house_26",
        title="26-card settle and projectile sequence",
        source_segment="card_house_26",
        total_steps=600,
        frame_stride=2,
        panel_steps=(0, 120, 402, 420, 600),
        panel_labels=(
            "initial",
            "settling t=2s",
            "pre-drop t=6.7s",
            "impact",
            "post t=10s",
        ),
        configuration=(
            ("cards", "26"),
            ("levels", "4"),
            ("mu", "0.8"),
            ("projectile_action_step", "402"),
        ),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The repaired DART reconstruction starts with 96 contacts rather than "
            "the 214 contacts in the paper timing scene.",
            "Projectile dimensions, mass, launch path, and speed are reconstructed; "
            "only the published 6.7 s/10 s figure instants are fixed.",
        ),
        known_gate_blockers=(
            "The current strict one-step card run ends at residual 1.244e-5 "
            "after 30000 iterations and uses boxed-LCP fallback.",
            "The current strict ten-step run spans 119-155 contacts and has zero "
            "fallback, but its residual still fails the 1e-6 paper tolerance.",
        ),
        actions=(ScheduledAction(402, "p", "launch four reconstructed projectiles"),),
        long_run=True,
    ),
    "masonry_arch_25_literal_standing": CaptureSchedule(
        id="masonry_arch_25_literal_standing",
        scene="fbf_paper_masonry_arch_25_literal_standing",
        title="Literal 25-stone arch standing reconstruction",
        source_segment="masonry_arch_25",
        total_steps=600,
        frame_stride=10,
        panel_steps=(0, 120, 300, 600),
        panel_labels=("initial", "t=2s", "t=5s", "local long run t=10s"),
        configuration=(
            ("stones", "25 literal convex wedges"),
            ("mobile_stones", "23; both springers pinned"),
            ("mu", "0.8"),
            ("contact_frontend", "Native FourPointPlanar"),
            ("contact_caps", "400 total, 8 per pair"),
            ("projectile", "none"),
            ("local_duration_seconds", "10"),
        ),
        mismatches=COMMON_DART_MISMATCH
        + (
            "This is the literal no-projectile standing phase only; it does not "
            "reproduce the source video's crown impact or its unpublished "
            "projectile parameters.",
            "The paper timing row reports 100 contacts, while this source-derived "
            "DART reconstruction has a different measured natural manifold.",
            "Ten seconds is the preserved local text/capture-oracle horizon, not "
            "a source-published duration.",
        ),
        known_gate_blockers=(
            "Passing this standing run cannot close the separate Fig. 7 impact "
            "parity gate.",
        ),
        collision_detector="native",
        collision_detector_override=False,
        long_run=True,
    ),
    "masonry_arch_25_author_crown_impact_current_source": CaptureSchedule(
        id="masonry_arch_25_author_crown_impact_current_source",
        scene="fbf_author_masonry_arch_25_crown_impact_current_source",
        title="Author-pinned 25-stone arch crown-impact candidate",
        source_segment="masonry_arch_25",
        total_steps=2000,
        # One 30 fps evidence frame per eight 240 Hz substeps preserves the
        # source diagnostic's 8.33 s presentation without making a 2x-slow clip.
        frame_stride=8,
        panel_steps=(0, 1200, 1600, 1945, 2000),
        panel_labels=(
            "initial source configuration",
            "pre-release source frame 300 (t=5s)",
            "pre-release source frame 400 (t=6.67s)",
            "post-release probe (t=8.10s)",
            "declared frame-500 endpoint (t=8.33s)",
        ),
        configuration=(
            ("author_commit", "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"),
            ("stones", "25 author-pinned tapered meshes"),
            ("mobile_stones", "23; both springers fixed"),
            ("cubes", "3; initially kinematic"),
            ("mu", "0.8"),
            ("simulation_time_step_seconds", "1/240"),
            ("display_time_step_seconds", "1/60"),
            ("substeps_per_display_frame", "4"),
            ("release_substep", "1600"),
            ("total_substeps", "2000"),
            ("capture_invocation", "declared 500-frame current-source diagnostic"),
        ),
        mismatches=(
            COMMON_DART_MISMATCH[0],
            "This DART scene ports the public author geometry, initial state, and "
            "500-frame release schedule; DART Native collision and float64 "
            "dynamics are not the authors' Warp/Newton float32 backend.",
            "The checked-in source default is 400 frames with drop_frame=400, so "
            "it ends without releasing the cubes. This 500-frame invocation is a "
            "declared current-source diagnostic, not a historical paper run.",
            "The author repository supplies no historical paper camera, materials, "
            "lighting, approved frame golden, or DART trajectory oracle.",
        ),
        known_gate_blockers=(
            "No current DART exact-FBF run of this 2,000-substep release schedule "
            "has yet passed the fail-fast solver and physical-outcome gates.",
            "No paired exact/boxed media has yet been captured and independently "
            "validated for this source-configuration port.",
        ),
        actions=(
            ScheduledAction(1600, "p", "release the three existing source cubes"),
        ),
        time_step_seconds=1.0 / 240.0,
        collision_detector="native",
        collision_detector_override=False,
        long_run=True,
    ),
    "masonry_arch_25": CaptureSchedule(
        id="masonry_arch_25",
        scene="fbf_paper_masonry_arch_25",
        title="25-stone arch settle and crown impact",
        source_segment="masonry_arch_25",
        total_steps=360,
        frame_stride=2,
        panel_steps=(0, 120, 240, 270, 360),
        panel_labels=(
            "initial",
            "settling t=2s",
            "pre-impact t=4s",
            "impact",
            "post t=6s",
        ),
        configuration=(
            ("stones", "25"),
            ("settle_seconds", "4"),
            ("impact_action_step", "240"),
            ("postimpact_seconds", "2"),
        ),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The production visual scene uses source-derived oriented boxes; it "
            "does pin both endpoints and uses the paper contact profile mu=0.8, "
            "but it does not render the literal tapered wedges.",
            "The separate literal-wedge collision audit uses exact uniform-prism "
            "mass/inertia and pinned endpoints. It finds 24 nominal adjacent "
            "contacts and 26 with a bounded closure plus ground; no audited backend "
            "reaches the paper's 100-contact timing contract.",
            "The GUI retains a 48-contact/two-per-pair interactive cap. Its "
            "vertical row of small cube projectiles matches the visible source "
            "shape and direction, but count, size, mass, speed, and timing remain "
            "reconstructions.",
            "The paper omits absolute simulation times and projectile parameters; "
            "4 s settle plus 2 s postimpact is a declared local protocol.",
        ),
        known_gate_blockers=(
            "No completed strict six-second settle-and-crown-impact trajectory "
            "has yet passed the 1e-6 residual and zero-fallback gates.",
            "The interactive scene is a reduced-contact oriented-box "
            "reconstruction, not the literal-wedge 100-contact paper contract.",
        ),
        actions=(ScheduledAction(240, "p", "drop reconstructed crown projectile row"),),
        long_run=True,
    ),
    "masonry_arch_101": CaptureSchedule(
        id="masonry_arch_101",
        scene="fbf_paper_masonry_arch_101",
        title="Legacy 101-stone visual proxy diagnostic",
        source_segment="masonry_arch_101",
        total_steps=600,
        frame_stride=2,
        panel_steps=(0, 120, 300, 600),
        panel_labels=("initial", "t=2s", "t=5s", "local long run t=10s"),
        configuration=(("stones", "101"), ("local_duration_seconds", "10")),
        mismatches=COMMON_DART_MISMATCH
        + (
            "This legacy reduced-contact proxy is retained only as a diagnostic; "
            "the author-parameterized 101-stone schedule is the required source "
            "capture lane.",
            "The source does not publish its long-run duration; ten seconds is a "
            "declared local observation window, not a source-equivalent time.",
            "The production scene pins both inferred endpoints, uses mu=0.8, and "
            "uses a reduced 38-contact oriented-box approximation rather than the "
            "literal tapered wedges with exact inertia from the collision-only "
            "audit.",
            "No faithful matched Kamino collapse clip is available.",
        ),
        known_gate_blockers=(
            "No completed strict ten-second standing trajectory has yet passed "
            "the 1e-6 residual and zero-fallback gates.",
            "The current literal-wedge exact attempt fails closed on its first "
            "dynamic step; this reduced-contact visual proxy cannot replace it.",
        ),
        long_run=True,
    ),
    "masonry_arch_101_author_standing_current_source": CaptureSchedule(
        id="masonry_arch_101_author_standing_current_source",
        scene="fbf_author_masonry_arch_101_standing_current_source",
        title="Author-pinned 101-stone current-source standing run",
        source_segment="masonry_arch_101",
        total_steps=1600,
        # Four 240 Hz substeps form each source display frame. Capturing every
        # other display frame preserves the 6.67 s source duration at 30 fps.
        frame_stride=8,
        panel_steps=(0, 400, 800, 1200, 1600),
        panel_labels=(
            "initial --stones 101 configuration",
            "source frame 100 (t=1.67s)",
            "source frame 200 (t=3.33s)",
            "source frame 300 (t=5s)",
            "source frame 400 endpoint (t=6.67s)",
        ),
        configuration=(
            ("author_commit", "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"),
            ("source_selection", "--stones 101"),
            ("stones", "101 author-pinned tapered meshes"),
            ("mobile_stones", "99; both springers fixed"),
            ("cubes", "3; remain kinematic through the source endpoint"),
            ("mu", "0.8"),
            ("contact_frontend", "Native FourPointPlanar"),
            ("simulation_time_step_seconds", "1/240"),
            ("display_time_step_seconds", "1/60"),
            ("substeps_per_display_frame", "4"),
            ("source_frames", "400"),
            ("drop_frame", "400"),
            ("release_substep", "1600; endpoint, no release action"),
            ("total_substeps", "1600"),
            ("capture_invocation", "current source --stones 101 selection"),
        ),
        mismatches=(
            COMMON_DART_MISMATCH[0],
            "This DART scene ports the public author's --stones 101 geometry, "
            "initial state, and 400-frame standing schedule; DART Native "
            "collision and float64 dynamics are not the authors' Warp/Newton "
            "float32 backend.",
            "The source supports --stones 101, but no historical Figure 8 "
            "invocation was recovered. This is a current-source parameterized "
            "run, not historical Figure 8 or paper parity.",
            "The source sets drop_frame equal to the 400-frame endpoint, so this "
            "schedule deliberately performs no cube-release action.",
            "The author repository supplies no historical paper camera, "
            "materials, lighting, approved frame golden, or DART trajectory "
            "oracle.",
        ),
        known_gate_blockers=(
            "No current DART exact-FBF run of this 1,600-substep standing "
            "schedule has yet passed the fail-fast solver and physical-outcome "
            "gates.",
            "No paired exact/boxed media has yet been captured and independently "
            "validated for this source-parameterized 101-stone port.",
        ),
        time_step_seconds=1.0 / 240.0,
        collision_detector="native",
        collision_detector_override=False,
        long_run=True,
    ),
    "card_house_10_construction": CaptureSchedule(
        id="card_house_10_construction",
        scene="fbf_paper_card_house_10",
        title="Ten-level construction inspection",
        source_segment="paper_tables_6_7_no_video_segment",
        total_steps=0,
        frame_stride=1,
        panel_steps=(0,),
        panel_labels=("construction-only step 0",),
        configuration=(
            ("levels", "10"),
            ("cards", "155"),
            ("mode", "static construction"),
        ),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The current 155-card scene is static, uses boxed-LCP diagnostics, and "
            "cannot support any dynamic exact-FBF or performance claim.",
            "The source video does not show the paper's ten-level CPU/GPU benchmark; "
            "this still documents construction only.",
        ),
        encode_mp4=False,
        exact_fbf_required=False,
        expect_motion=False,
        solver_lane="boxed",
        supported_solver_lanes=("boxed",),
    ),
    "card_house_author_10_impact_current_source": CaptureSchedule(
        id="card_house_author_10_impact_current_source",
        scene="fbf_author_card_house_10_impact_current_source",
        title="Author-pinned ten-level card-house current-source impact",
        source_segment="paper_tables_6_7_no_video_segment",
        total_steps=3200,
        # The public source advances four 240 Hz substeps for each 60 Hz display
        # frame. One evidence frame every eight substeps gives a 30 fps video
        # with the source-selected 800-frame duration.
        frame_stride=8,
        panel_steps=(0, 120, 1600, 1680, 3200),
        panel_labels=(
            "initial current-source configuration",
            "pinned source-prefix endpoint t=.5s",
            "pre-frame-400 release boundary (t=6.67s)",
            "post-release probe t=7s",
            "source frame-800 endpoint (t=13.33s)",
        ),
        configuration=(
            ("author_commit", "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"),
            ("levels", "10"),
            ("cards", "155"),
            ("cubes", "4; initially kinematic"),
            ("mu", "0.8"),
            ("simulation_time_step_seconds", "1/240"),
            ("display_time_step_seconds", "1/60"),
            ("substeps_per_display_frame", "4"),
            ("source_frames", "800"),
            ("release_substep", "1600"),
            ("total_substeps", "3200"),
            ("max_contacts", "4096"),
            ("max_contacts_per_pair", "4"),
            ("collision_shape_frames", "160"),
            ("ground_contact_gap_m", "0.1 for the ground ShapeFrame"),
            (
                "dynamic_shape_contact_gap_m",
                "0.005 for the 155 card and 4 cube ShapeFrames",
            ),
            ("negative_depth_contacts", "enabled for configured Native gaps"),
            (
                "capture_invocation",
                "current public source --levels 10 --frames 800 selection",
            ),
        ),
        mismatches=(
            COMMON_DART_MISMATCH[0],
            "The public source supports --levels 10 with its 800-frame default, "
            "but no historical Tables 6/7 invocation was recovered. This is a "
            "supported current-source selection, not historical paper parity.",
            "The source assigns gap=.1, stiffness=2500, and damping=100 to the "
            "ground, versus gap=.005, stiffness=1e4, and damping=1e3 for each "
            "of the 159 card/cube shapes. DART Native represents the two gap "
            "values with signed contacts, but not the authors' Warp collision, "
            "Newton/float32 solver, or stiffness/damping semantics.",
            "In the pinned source's 30-frame CPU control, 87 of 120 substeps did "
            "not converge and the source continued. That finite prefix supplies "
            "no strict trajectory oracle for DART.",
        ),
        known_gate_blockers=(
            "No complete strict DART exact-FBF run of this 3,200-substep release "
            "schedule has yet passed the solver and physical-outcome gates.",
            "No paired exact/boxed media has yet been captured and independently "
            "validated for this current-source ten-level adapter.",
            "The pinned source prefix itself has 87 nonconverged substeps out of "
            "120 and provides no strict trajectory oracle.",
        ),
        actions=(ScheduledAction(1600, "p", "release the four existing source cubes"),),
        time_step_seconds=1.0 / 240.0,
        collision_detector="native",
        collision_detector_override=False,
        long_run=True,
    ),
    "card_house_author_5_construction": CaptureSchedule(
        id="card_house_author_5_construction",
        scene="fbf_author_card_house_5_construction",
        title="Author-pinned five-level construction inspection",
        source_segment="paper_tables_6_7_no_video_segment",
        total_steps=0,
        frame_stride=1,
        panel_steps=(0,),
        panel_labels=("configuration-only substep 0",),
        configuration=(
            ("levels", "5"),
            ("mode", "static source-configuration port"),
            ("simulation_time_step_seconds", "1/240"),
            ("display_time_step_seconds", "1/60"),
            ("substeps_per_display_frame", "4"),
            ("release_substep", "1600"),
            ("total_substeps", "3200"),
            ("evidence_scope", "configuration-only"),
            ("parity_scope", "non-parity"),
        ),
        mismatches=(
            COMMON_DART_MISMATCH[0],
            "The step-zero capture validates only the source-configuration port; "
            "it supplies no release, standing, dynamics, outcome, performance, "
            "or paper-parity evidence.",
            "DART Native collision and the configured DART solver are not the "
            "authors' Warp/Newton collision backend or solver implementation.",
            "The paper video does not show the five-level GPU benchmark, and no "
            "approved source render golden is available.",
        ),
        time_step_seconds=1.0 / 240.0,
        collision_detector="native",
        collision_detector_override=False,
        encode_mp4=False,
        exact_fbf_required=False,
        expect_motion=False,
        supported_solver_lanes=("exact",),
    ),
    "card_house_10_dynamics": CaptureSchedule(
        id="card_house_10_dynamics",
        scene="fbf_paper_card_house_10_dynamic",
        title="Ten-level dynamic long run",
        source_segment="paper_tables_6_7_no_video_segment",
        total_steps=600,
        frame_stride=2,
        panel_steps=(0, 120, 300, 600),
        panel_labels=("initial", "t=2s", "t=5s", "local long run t=10s"),
        configuration=(
            ("levels", "10"),
            ("cards", "155"),
            ("mode", "dynamic exact FBF"),
            ("max_contacts", "512"),
            ("max_contacts_per_pair", "8"),
            ("contact_budget_status", "saturated in existing boxed-LCP probe"),
        ),
        mismatches=COMMON_DART_MISMATCH
        + (
            "The paper does not publish the ten-level duration or complete asset "
            "recipe.",
            "The dynamic GUI adapter uses a 512-contact/eight-per-pair cap. The "
            "existing boxed-LCP construction probe reaches exactly 512 contacts, "
            "so this is not a natural-manifold run.",
            "No faithful matched MuJoCo/Kamino dynamic clips are available.",
        ),
        known_gate_blockers=(
            "No completed dynamic exact-FBF ten-level trajectory has yet shown "
            "zero exact failures and zero boxed-LCP fallbacks.",
            "The 512-contact budget is known to saturate, so even a clean visual "
            "trajectory cannot prove full-manifold stability or paper timing.",
        ),
        long_run=True,
    ),
}


TURN_TABLE_MEMBERS = (
    "turntable_mu02_omega2",
    "turntable_mu02_omega5",
    "turntable_mu05_omega2",
    "turntable_mu05_omega5",
)
AUTHOR_TURN_TABLE_MEMBERS = (
    "turntable_author_mu02_omega2",
    "turntable_author_mu02_omega5",
    "turntable_author_mu05_omega2",
    "turntable_author_mu05_omega5",
)
AUTHOR_TURN_TABLE_VISUAL_RESOURCES = (
    (
        "turntable_disc_obj",
        ROOT / "data" / "obj" / "fbf_author_turntable_disc.obj",
    ),
    (
        "turntable_disc_mtl",
        ROOT / "data" / "obj" / "fbf_author_turntable_disc.mtl",
    ),
)
PAINLEVE_MEMBERS = ("painleve_mu05", "painleve_mu055")
AUTHOR_PAINLEVE_MEMBERS = ("painleve_author_mu05", "painleve_author_mu055")
AUTHOR_PAINLEVE_OUTCOME_SCHEMA_VERSION = (
    "dart.fbf_author_painleve_dart_adapter_outcome/v1"
)
AUTHOR_PAINLEVE_COMPARISON_SCHEMA_VERSION = (
    "dart.fbf_author_painleve_solver_comparison/v1"
)
AUTHOR_PAINLEVE_OUTCOME_CLAIM_SCOPE = "current_dart_adapter_only"
AUTHOR_PAINLEVE_TERMINAL_WINDOW = (105, 120)
AUTHOR_PAINLEVE_MAX_LINEAR_SPEED_M_S = 0.02
AUTHOR_PAINLEVE_MAX_ANGULAR_SPEED_RAD_S = 0.10
AUTHOR_PAINLEVE_UPRIGHTNESS_MINIMUM = 0.90
AUTHOR_PAINLEVE_UPRIGHT_HEIGHT_RANGE_M = (0.25, 0.35)
AUTHOR_PAINLEVE_TUMBLED_ABS_UPRIGHTNESS_MAXIMUM = 0.25
AUTHOR_PAINLEVE_TUMBLED_HEIGHT_RANGE_M = (0.08, 0.22)
AUTHOR_PAINLEVE_MIN_HORIZONTAL_TRAVEL_M = 1.0
AUTHOR_PAINLEVE_EXPECTED_OUTCOMES = {
    ("painleve_author_mu05", "exact"): "upright_near_rest",
    ("painleve_author_mu05", "boxed"): "upright_near_rest",
    ("painleve_author_mu055", "exact"): "tumbled_near_rest",
    ("painleve_author_mu055", "boxed"): "upright_near_rest",
}
AUTHOR_PAINLEVE_SCENE_STATE_FIELDS = (
    "world_time_seconds",
    "position_x_m",
    "position_y_m",
    "position_z_m",
    "rotation_00",
    "rotation_01",
    "rotation_02",
    "rotation_10",
    "rotation_11",
    "rotation_12",
    "rotation_20",
    "rotation_21",
    "rotation_22",
    "body_up_x",
    "body_up_y",
    "body_up_z",
    "uprightness_cosine",
    "pitch_rad",
    "linear_velocity_x_m_s",
    "linear_velocity_y_m_s",
    "linear_velocity_z_m_s",
    "angular_velocity_x_rad_s",
    "angular_velocity_y_rad_s",
    "angular_velocity_z_rad_s",
)
AUTHOR_MASONRY_ARCH_101_SCENE_STATE_SCHEMA_VERSION = (
    "dart.fbf_author_masonry_arch_101_standing_scene_state/v1"
)
AUTHOR_MASONRY_ARCH_101_REQUIRED_SUBSTEPS = 1600
AUTHOR_MASONRY_ARCH_101_REQUIRED_WORLD_TIME_SECONDS = 1600.0 / 240.0
AUTHOR_MASONRY_ARCH_101_HORIZON_TIME_TOLERANCE_SECONDS = 0.5 / 240.0
AUTHOR_MASONRY_ARCH_101_MAX_MOBILE_BODY_ORIGIN_DISPLACEMENT = 3.0
AUTHOR_MASONRY_ARCH_101_MAX_MOBILE_ROTATION_DELTA_RAD = math.pi / 12.0
AUTHOR_MASONRY_ARCH_101_MAX_CROWN_HEIGHT_LOSS = 3.0
AUTHOR_MASONRY_ARCH_101_MAX_KINEMATIC_CUBE_POSE_ERROR = 1.0e-12
AUTHOR_MASONRY_ARCH_101_SCENE_STATE_FIELDS = (
    "solver_lane_exact_fbf",
    "solver_lane_boxed_lcp",
    "world_time_seconds",
    "world_skeleton_count",
    "observed_stone_count",
    "observed_mobile_stone_count",
    "mobility_matching_stone_count",
    "finite_stone_count",
    "observed_cube_count",
    "finite_cube_count",
    "kinematic_cube_count",
    "ground_valid",
    "crown_observed",
    "crown_body_origin_initial_z",
    "crown_body_origin_z",
    "crown_body_origin_displacement",
    "crown_rotation_delta_rad",
    "max_mobile_body_origin_displacement",
    "max_mobile_rotation_delta_rad",
    "max_kinematic_cube_body_origin_displacement",
    "max_kinematic_cube_rotation_delta_rad",
    "standing_horizon_complete",
    "standing_inventory_valid",
    "standing_all_bodies_finite",
    "standing_cubes_remain_kinematic",
    "standing_cubes_remain_pinned",
    "standing_mobile_displacement_bounded",
    "standing_mobile_rotation_bounded",
    "standing_crown_height_preserved",
    "standing_complete_trace_valid",
    "standing_outcome_valid",
    "standing_lane_evidence_qualifies",
)


@dataclasses.dataclass(frozen=True)
class GroupOutputSpec:
    id: str
    source_segment: str
    members: tuple[str, ...]
    labels: tuple[str, ...]
    layout: str
    panel_step: int | None = None
    panel_steps: tuple[int, ...] | None = None
    panel_labels: tuple[str, ...] | None = None
    solver_lane: str = "exact"
    source_group_id: str | None = None

    def __post_init__(self) -> None:
        if len(self.members) != len(self.labels):
            raise ValueError(f"{self.id}: group members and labels differ")
        if len(set(self.members)) != len(self.members):
            raise ValueError(f"{self.id}: group members must be unique")
        if self.layout not in ("2x2", "side-by-side"):
            raise ValueError(f"{self.id}: unsupported group layout {self.layout!r}")
        if self.layout == "2x2" and len(self.members) != 4:
            raise ValueError(f"{self.id}: 2x2 output needs four members")
        if self.layout == "side-by-side" and len(self.members) != 2:
            raise ValueError(f"{self.id}: side-by-side output needs two members")
        if (self.panel_step is None) == (self.panel_steps is None):
            raise ValueError(
                f"{self.id}: specify either one shared panel step or one step "
                "per member"
            )
        if self.panel_step is not None and self.panel_step < 0:
            raise ValueError(f"{self.id}: shared panel step must be nonnegative")
        if self.panel_steps is not None:
            if len(self.panel_steps) != len(self.members):
                raise ValueError(
                    f"{self.id}: per-member panel steps and members differ"
                )
            if any(step < 0 for step in self.panel_steps):
                raise ValueError(
                    f"{self.id}: per-member panel steps must be nonnegative"
                )
        if self.panel_labels is not None and len(self.panel_labels) != len(
            self.members
        ):
            raise ValueError(f"{self.id}: panel labels and members differ")
        if self.solver_lane not in (*SOLVER_LANES, "both"):
            raise ValueError(f"{self.id}: unsupported solver lane {self.solver_lane!r}")
        if self.solver_lane == "both":
            if self.layout != "side-by-side" or len(self.members) != 2:
                raise ValueError(
                    f"{self.id}: solver comparison must be a two-member "
                    "side-by-side group"
                )
            if self.members[1] != f"{self.members[0]}__boxed":
                raise ValueError(
                    f"{self.id}: solver comparison members must be exact then boxed"
                )
            if self.labels != SOLVER_COMPARISON_LABELS:
                raise ValueError(
                    f"{self.id}: solver comparison labels must identify only "
                    "the solver lanes"
                )
            if (
                self.panel_labels is not None
                and self.panel_labels != SOLVER_COMPARISON_LABELS
            ):
                raise ValueError(
                    f"{self.id}: solver comparison panel labels must identify "
                    "only the solver lanes"
                )
        if self.source_group_id == self.id:
            raise ValueError(f"{self.id}: derived source group cannot equal its id")

    @property
    def resolved_panel_steps(self) -> tuple[int, ...]:
        if self.panel_steps is not None:
            return self.panel_steps
        assert self.panel_step is not None
        return (self.panel_step,) * len(self.members)

    @property
    def resolved_panel_labels(self) -> tuple[str, ...]:
        return self.panel_labels if self.panel_labels is not None else self.labels


GROUP_OUTPUTS: dict[str, GroupOutputSpec] = {
    "turntable": GroupOutputSpec(
        id="turntable",
        source_segment="turntable",
        members=TURN_TABLE_MEMBERS,
        labels=(
            "MU .2 OMEGA 2",
            "MU .2 OMEGA 5",
            "MU .5 OMEGA 2",
            "MU .5 OMEGA 5",
        ),
        layout="2x2",
        panel_step=240,
    ),
    "turntable_author": GroupOutputSpec(
        id="turntable_author",
        source_segment="turntable",
        members=AUTHOR_TURN_TABLE_MEMBERS,
        labels=(
            "mu=0.2, omega=2 rad/s",
            "mu=0.2, omega=5 rad/s",
            "mu=0.5, omega=2 rad/s",
            "mu=0.5, omega=5 rad/s",
        ),
        layout="2x2",
        panel_steps=(136, 120, 360, 90),
        panel_labels=(
            "MU 0.2 OMEGA 2 T 2.27S",
            "MU 0.2 OMEGA 5 T 2.00S",
            "MU 0.5 OMEGA 2 T 6.00S",
            "MU 0.5 OMEGA 5 T 1.50S",
        ),
    ),
    "painleve": GroupOutputSpec(
        id="painleve",
        source_segment="painleve",
        members=PAINLEVE_MEMBERS,
        labels=("MU .5 SLIDE REST", "MU .55 SHORT TRAVEL TUMBLE"),
        layout="side-by-side",
        panel_step=150,
    ),
    "painleve_author": GroupOutputSpec(
        id="painleve_author",
        source_segment="painleve",
        members=AUTHOR_PAINLEVE_MEMBERS,
        labels=("MU .5", "MU .55"),
        layout="side-by-side",
        panel_step=120,
        panel_labels=("MU .5 | T 2.00S", "MU .55 | T 2.00S"),
    ),
}


def _boxed_schedule_id(schedule_id: str) -> str:
    return f"{schedule_id}__boxed"


def _expected_solver(schedule: CaptureSchedule) -> str:
    return EXACT_SOLVER_NAME if schedule.solver_lane == "exact" else BOXED_SOLVER_NAME


def _derive_boxed_schedule(schedule: CaptureSchedule) -> CaptureSchedule:
    if (
        schedule.solver_lane != "exact"
        or "boxed" not in schedule.supported_solver_lanes
    ):
        raise ValueError(f"{schedule.id}: a boxed comparison cannot be derived")
    configuration = schedule.configuration
    if schedule.source_continuation_required:
        exact_only_keys = {
            "exact_policy",
            "residual_check_interval",
            "plateau_patience",
            "plateau_relative_tolerance",
            "step_size_backtrack_limit",
            "coupling_variation_skip_threshold",
        }
        configuration = tuple(
            item for item in configuration if item[0] not in exact_only_keys
        ) + (
            ("comparison_counterpart", "same_physics_boxed_lcp"),
            ("source_exact_policy", "source_continuation"),
            ("active_exact_policy", "not_applicable"),
        )
    if schedule.id in AUTHOR_PAINLEVE_MEMBERS:
        boxed_outcome = AUTHOR_PAINLEVE_EXPECTED_OUTCOMES[(schedule.id, "boxed")]
        configuration = tuple(
            (
                key,
                (
                    f"{boxed_outcome} under the current boxed DART adapter"
                    if key == "outcome"
                    else value
                ),
            )
            for key, value in configuration
        )
    mismatches = schedule.boxed_comparison_mismatches
    if mismatches is None:
        mismatches = (
            *schedule.mismatches,
            "The boxed-LCP lane is a comparison capture, not exact-FBF evidence.",
        )
    gate_blockers = schedule.boxed_comparison_gate_blockers
    if gate_blockers is None:
        gate_blockers = schedule.known_gate_blockers
    return dataclasses.replace(
        schedule,
        id=_boxed_schedule_id(schedule.id),
        title=f"{schedule.title} (boxed LCP comparison)",
        mismatches=mismatches,
        known_gate_blockers=gate_blockers,
        boxed_comparison_mismatches=None,
        boxed_comparison_gate_blockers=None,
        configuration=configuration,
        exact_fbf_required=False,
        source_continuation_required=False,
        solver_lane="boxed",
        supported_solver_lanes=("boxed",),
        source_schedule_id=schedule.id,
        pre_run_actions=("e",),
    )


def _resolve_solver_lanes(
    schedules: Sequence[CaptureSchedule], requested: str
) -> tuple[list[CaptureSchedule], list[dict[str, str]]]:
    if requested not in (*SOLVER_LANES, "both"):
        raise ValueError(f"unsupported solver lane {requested!r}")
    requested_lanes = SOLVER_LANES if requested == "both" else (requested,)
    resolved: list[CaptureSchedule] = []
    skips: list[dict[str, str]] = []
    for lane in requested_lanes:
        for schedule in schedules:
            if lane not in schedule.supported_solver_lanes:
                skips.append(
                    {
                        "kind": "schedule",
                        "schedule_id": schedule.id,
                        "requested_solver_lane": lane,
                        "reason": (
                            "scene is exact-only"
                            if schedule.supported_solver_lanes == ("exact",)
                            else "scene is boxed-only"
                        ),
                    }
                )
                continue
            if schedule.solver_lane == lane:
                resolved.append(schedule)
            elif lane == "boxed":
                resolved.append(_derive_boxed_schedule(schedule))
            else:
                raise ValueError(
                    f"{schedule.id}: cannot derive {lane!r} from "
                    f"{schedule.solver_lane!r}"
                )
    return resolved, skips


def _capture_output_dir(output_root: Path, schedule: CaptureSchedule) -> Path:
    return output_root / schedule.id


def _derive_boxed_group(group: GroupOutputSpec) -> GroupOutputSpec:
    if group.id == "turntable_author":
        raise ValueError(f"{group.id}: author-pinned output is exact-only")
    return dataclasses.replace(
        group,
        id=f"{group.id}__boxed",
        members=tuple(_boxed_schedule_id(member) for member in group.members),
        labels=tuple(
            f"EXISTING BOXED LCP | {SCHEDULES[member].title.upper()}"
            for member in group.members
        ),
        panel_labels=tuple(
            f"EXISTING BOXED LCP | {SCHEDULES[member].title.upper()}"
            for member in group.members
        ),
        solver_lane="boxed",
        source_group_id=group.id,
    )


def _derive_solver_comparison_group(schedule: CaptureSchedule) -> GroupOutputSpec:
    if (
        schedule.solver_lane != "exact"
        or set(schedule.supported_solver_lanes) != set(SOLVER_LANES)
        or not schedule.encode_mp4
        or not schedule.panel_steps
    ):
        raise ValueError(
            f"{schedule.id}: an exact-versus-boxed video comparison cannot be derived"
        )
    return GroupOutputSpec(
        id=f"{schedule.id}{SOLVER_COMPARISON_SUFFIX}",
        source_segment=schedule.source_segment,
        members=(schedule.id, _boxed_schedule_id(schedule.id)),
        labels=SOLVER_COMPARISON_LABELS,
        layout="side-by-side",
        panel_step=schedule.panel_steps[-1],
        solver_lane="both",
    )


def _solver_comparison_groups(
    schedules: Sequence[CaptureSchedule], requested: str
) -> list[GroupOutputSpec]:
    if requested != "both":
        return []
    return [
        _derive_solver_comparison_group(schedule)
        for schedule in schedules
        if schedule.solver_lane == "exact"
        and set(schedule.supported_solver_lanes) == set(SOLVER_LANES)
        and schedule.encode_mp4
    ]


def _group_outputs_for_solver_lane(
    requested: str,
) -> tuple[list[GroupOutputSpec], list[dict[str, str]]]:
    if requested not in (*SOLVER_LANES, "both"):
        raise ValueError(f"unsupported solver lane {requested!r}")
    groups: list[GroupOutputSpec] = []
    skips: list[dict[str, str]] = []
    if requested in ("exact", "both"):
        groups.extend(GROUP_OUTPUTS.values())
    if requested in ("boxed", "both"):
        groups.extend(
            _derive_boxed_group(GROUP_OUTPUTS[group_id])
            for group_id in ("turntable", "painleve", "painleve_author")
        )
        skips.append(
            {
                "kind": "group",
                "group_id": "turntable_author",
                "requested_solver_lane": "boxed",
                "reason": "author-pinned turntable output is exact-only",
            }
        )
    return groups, skips


def _group_output_dir(output_root: Path, group: GroupOutputSpec) -> Path:
    return output_root / "groups" / group.id


def _schedule_for_id(schedule_id: str) -> CaptureSchedule:
    if schedule_id in SCHEDULES:
        return SCHEDULES[schedule_id]
    suffix = "__boxed"
    if schedule_id.endswith(suffix):
        source_id = schedule_id[: -len(suffix)]
        if source_id in SCHEDULES:
            return _derive_boxed_schedule(SCHEDULES[source_id])
    raise KeyError(schedule_id)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _visual_resource_snapshot(
    schedule: CaptureSchedule,
) -> dict[str, dict[str, str]] | None:
    if schedule.id not in AUTHOR_TURN_TABLE_MEMBERS:
        return None
    snapshot: dict[str, dict[str, str]] = {}
    for key, resource in AUTHOR_TURN_TABLE_VISUAL_RESOURCES:
        resource = resource.resolve()
        if not resource.is_file():
            raise FileNotFoundError(resource)
        snapshot[key] = {
            "path": str(resource),
            "sha256": _sha256(resource),
        }
    return snapshot


def _bind_visual_resource_snapshots(
    before: dict[str, dict[str, str]] | None,
    after: dict[str, dict[str, str]] | None,
) -> dict[str, dict[str, str | bool]] | None:
    if before is None and after is None:
        return None
    if before is None or after is None or before.keys() != after.keys():
        raise ValueError("author-turntable visual resource set changed during capture")
    binding: dict[str, dict[str, str | bool]] = {}
    for key in before:
        before_item = before[key]
        after_item = after[key]
        if before_item != after_item:
            raise ValueError(
                f"author-turntable visual resource {key} changed during capture"
            )
        binding[key] = {
            "path": before_item["path"],
            "sha256_before_capture": before_item["sha256"],
            "sha256_after_capture": after_item["sha256"],
            "unchanged_during_capture": True,
        }
    return binding


def _run(
    argv: Sequence[str],
    *,
    cwd: Path | None = None,
    capture_output: bool = False,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(item) for item in argv],
        cwd=cwd,
        check=True,
        text=True,
        capture_output=capture_output,
    )


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _frame_path(output_dir: Path, step: int) -> Path:
    return output_dir / "frames" / f"step_{step:06d}.png"


def _panel_frame_path(output_dir: Path, step: int) -> Path:
    return output_dir / "panel_frames" / f"step_{step:06d}.png"


def build_demo_command(
    schedule: CaptureSchedule, demo: Path, output_dir: Path
) -> list[str]:
    if not schedule.runnable:
        raise ValueError(f"{schedule.id}: {schedule.adapter_gap}")
    command = [
        str(demo),
        "--scene",
        schedule.scene,
        "--headless",
        "--steps",
        str(schedule.total_steps),
        "--width",
        str(schedule.width),
        "--height",
        str(schedule.height),
        "--threads",
        str(schedule.threads),
        "--headless-sidecar",
        str(output_dir / "timeline.json"),
    ]
    if schedule.collision_detector_override:
        threads_index = command.index("--threads")
        command[threads_index:threads_index] = [
            "--collision-detector",
            schedule.collision_detector,
        ]
    if schedule.source_continuation_required:
        command.append(HEADLESS_EXACT_FBF_SOURCE_CONTINUATION_FLAG)
    elif schedule.exact_fbf_required:
        command.append(HEADLESS_EXACT_FBF_FAIL_FAST_FLAG)

    for key in schedule.pre_run_actions:
        command.extend(("--headless-action", key))

    # DemoHost guarantees captures-before-actions at a completed step.  Keep
    # the CLI in the same readable order so a 6.7 s pre-impact frame cannot be
    # confused with the action that follows it.
    for step in schedule.capture_steps:
        command.extend(
            ("--headless-shot-at", f"{step}:{_frame_path(output_dir, step)}")
        )
    for action in schedule.actions:
        command.extend(("--headless-action-at", f"{action.step}:{action.key}"))
    return command


def _shell_command(argv: Sequence[str]) -> str:
    return shlex.join(str(item) for item in argv)


def schedule_plan(
    schedule: CaptureSchedule, demo: Path, output_root: Path
) -> dict[str, Any]:
    output_dir = _capture_output_dir(output_root, schedule)
    command = (
        build_demo_command(schedule, demo, output_dir) if schedule.runnable else None
    )
    return {
        "id": schedule.id,
        "title": schedule.title,
        "scene": schedule.scene,
        "source_segment": schedule.source_segment,
        "solver_lane": schedule.solver_lane,
        "supported_solver_lanes": list(schedule.supported_solver_lanes),
        "source_schedule_id": schedule.source_schedule_id,
        "expected_solver": _expected_solver(schedule),
        "collision_detector": schedule.collision_detector,
        "collision_detector_override": schedule.collision_detector_override,
        "configuration": schedule.configuration_dict(),
        "total_steps": schedule.total_steps,
        "time_step_seconds": schedule.time_step_seconds,
        "capture_steps": list(schedule.capture_steps),
        "panel_steps": list(schedule.panel_steps),
        "panel_labels": list(schedule.panel_labels),
        "actions": [dataclasses.asdict(action) for action in schedule.actions],
        "pre_run_actions": list(schedule.pre_run_actions),
        "output": {
            "directory": str(output_dir),
            "timeline": str(output_dir / "timeline.json"),
            "panel": str(output_dir / "panel.png"),
            "mp4": str(output_dir / "clip.mp4") if schedule.encode_mp4 else None,
            "gif": str(output_dir / "clip.gif") if schedule.encode_gif else None,
        },
        "runnable": schedule.runnable,
        "adapter_gap": schedule.adapter_gap,
        "long_run": schedule.long_run,
        "actual_simulator_required": True,
        "generated_imagery_allowed": False,
        "paper_comparable": False,
        "known_mismatches": list(schedule.mismatches),
        "known_gate_blockers": list(schedule.known_gate_blockers),
        "comparison_capture": schedule.solver_lane == "boxed",
        "evidence_ready": (
            schedule.solver_lane == "exact"
            and schedule.runnable
            and not schedule.known_gate_blockers
        ),
        "demo_argv": command,
        "demo_command": _shell_command(command) if command else None,
    }


def _claim_value_matches(actual: Any, expected: Any) -> bool:
    if isinstance(expected, bool):
        return actual is expected
    return actual == expected


def _contract_value_matches(actual: Any, expected: Any) -> bool:
    """Compare decoded JSON contracts without depending on decimal spelling."""
    if isinstance(expected, bool):
        return actual is expected
    if isinstance(expected, int):
        return (
            not isinstance(actual, bool)
            and isinstance(actual, int)
            and actual == expected
        )
    if isinstance(expected, float):
        return _is_finite_number(actual) and math.isclose(
            float(actual), expected, rel_tol=1e-12, abs_tol=1e-15
        )
    if isinstance(expected, list):
        return (
            isinstance(actual, list)
            and len(actual) == len(expected)
            and all(
                _contract_value_matches(actual_item, expected_item)
                for actual_item, expected_item in zip(actual, expected)
            )
        )
    if isinstance(expected, dict):
        return (
            isinstance(actual, dict)
            and actual.keys() == expected.keys()
            and all(
                _contract_value_matches(actual[key], expected_value)
                for key, expected_value in expected.items()
            )
        )
    return actual == expected


def _validate_capture_claim_boundary(
    metadata: dict[str, Any],
    schedule: CaptureSchedule,
    *,
    metadata_path: Path,
) -> None:
    plan = metadata.get("schedule", {})
    plan_claims = {
        "source_segment": schedule.source_segment,
        "solver_lane": schedule.solver_lane,
        "supported_solver_lanes": list(schedule.supported_solver_lanes),
        "source_schedule_id": schedule.source_schedule_id,
        "expected_solver": _expected_solver(schedule),
        "configuration": schedule.configuration_dict(),
        "total_steps": schedule.total_steps,
        "time_step_seconds": schedule.time_step_seconds,
        "capture_steps": list(schedule.capture_steps),
        "panel_steps": list(schedule.panel_steps),
        "panel_labels": list(schedule.panel_labels),
        "pre_run_actions": list(schedule.pre_run_actions),
        "collision_detector": schedule.collision_detector,
        "collision_detector_override": schedule.collision_detector_override,
        "actual_simulator_required": True,
        "generated_imagery_allowed": False,
        "paper_comparable": False,
        "known_mismatches": list(schedule.mismatches),
        "known_gate_blockers": list(schedule.known_gate_blockers),
        "comparison_capture": schedule.solver_lane == "boxed",
        "evidence_ready": (
            schedule.solver_lane == "exact"
            and schedule.runnable
            and not schedule.known_gate_blockers
        ),
    }
    for key, expected in plan_claims.items():
        if not _claim_value_matches(plan.get(key), expected):
            raise ValueError(f"{metadata_path}: schedule claim boundary {key} changed")

    capture_claims = {
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": CAPTURE_SEMANTIC_OUTCOME_GATE,
        "known_mismatches": list(schedule.mismatches),
    }
    if _is_author_painleve_schedule(schedule):
        timeline_outcome = metadata.get("timeline_validation", {}).get(
            "dart_adapter_outcome"
        )
        if (
            not isinstance(timeline_outcome, dict)
            or timeline_outcome.get("pass") is not True
        ):
            raise ValueError(
                f"{metadata_path}: current DART adapter outcome is unavailable"
            )
        capture_claims.update(
            {
                "automated_current_dart_adapter_outcome_validated": True,
                "current_dart_adapter_outcome": timeline_outcome,
            }
        )
    for key, expected in capture_claims.items():
        if not _claim_value_matches(metadata.get(key), expected):
            raise ValueError(f"{metadata_path}: capture claim boundary {key} changed")


def _validate_runtime_visual_resources(
    schedule: CaptureSchedule,
    runtime: Any,
    *,
    metadata_path: Path,
) -> None:
    if not isinstance(runtime, dict):
        raise ValueError(f"{metadata_path}: runtime metadata is unavailable")
    snapshot = _visual_resource_snapshot(schedule)
    if snapshot is None:
        if "visual_resources" in runtime:
            raise ValueError(
                f"{metadata_path}: unexpected runtime visual-resource claim"
            )
        return
    expected = _bind_visual_resource_snapshots(snapshot, snapshot)
    if runtime.get("visual_resources") != expected:
        raise ValueError(
            f"{metadata_path}: author-turntable visual resource binding changed"
        )


def build_plan(
    schedules: Iterable[CaptureSchedule],
    demo: Path,
    output_root: Path,
    *,
    solver_lane: str = "exact",
    groups: Iterable[GroupOutputSpec] | None = None,
    solver_lane_skips: Sequence[dict[str, str]] = (),
) -> dict[str, Any]:
    schedules = list(schedules)
    groups = list(GROUP_OUTPUTS.values() if groups is None else groups)
    plans = [schedule_plan(schedule, demo, output_root) for schedule in schedules]
    return {
        "schema_version": SCHEMA_VERSION,
        "kind": "capture_plan",
        "source_contract": {
            "paper_video": "https://youtu.be/5THad4PAGmI",
            "paper": "https://www.cs.ubc.ca/research/fbf-friction/paper.pdf",
            "coverage_matrix": str(COVERAGE_MATRIX_PATH),
            "video_segments": [
                dataclasses.asdict(segment) for segment in VIDEO_SEGMENTS
            ],
        },
        "output_root": str(output_root),
        "requested_solver_lane": solver_lane,
        "solver_lane_skips": list(solver_lane_skips),
        "schedules": plans,
        "group_outputs": {
            group.id: {
                "members": list(group.members),
                "solver_lane": group.solver_lane,
                "source_group_id": group.source_group_id,
                "labels": list(group.labels),
                "layout": (
                    "2x2 in source order"
                    if group.layout == "2x2"
                    else (
                        "two synchronized solver lanes"
                        if group.solver_lane == "both"
                        else "two synchronized parameter cells"
                    )
                ),
                **(
                    {"panel_step": group.panel_step}
                    if group.panel_step is not None
                    else {
                        "panel_sources": [
                            {
                                "member": member,
                                "step": step,
                                "time_seconds": _schedule_for_id(member).time_at_step(
                                    step
                                ),
                            }
                            for member, step in zip(
                                group.members, group.resolved_panel_steps
                            )
                        ]
                    }
                ),
                "panel_labels": list(group.resolved_panel_labels),
                "output_directory": str(_group_output_dir(output_root, group)),
                "created_when_all_members_selected_and_successful": True,
            }
            for group in groups
        },
        "pass": True,
        "all_schedules_runnable": all(plan["runnable"] for plan in plans),
        "blocked_schedules": [plan["id"] for plan in plans if not plan["runnable"]],
    }


def _validate_png(path: Path, width: int, height: int) -> dict[str, Any]:
    verdict = build_verdict(path)
    dimensions = verdict["image"]
    if (dimensions["width"], dimensions["height"]) != (width, height):
        raise ValueError(
            f"{path}: dimensions {dimensions['width']}x{dimensions['height']} "
            f"do not match {width}x{height}"
        )
    if not verdict["checks"]["non_blank"]["pass"]:
        raise ValueError(f"{path}: blank/uniform capture: {verdict['reasons']}")
    return verdict


def _world_region_report(path: Path, schedule: CaptureSchedule) -> dict[str, Any]:
    image = read_image(path)
    crop_width, crop_height, x0, y0 = schedule.crop
    pixels = bytearray(crop_width * crop_height * 3)
    for row in range(crop_height):
        source = ((y0 + row) * image.width + x0) * 3
        destination = row * crop_width * 3
        pixels[destination : destination + crop_width * 3] = image.pixels[
            source : source + crop_width * 3
        ]
    region = ImageData(
        path=path,
        width=crop_width,
        height=crop_height,
        pixels=bytes(pixels),
    )
    non_blank = analyze_non_blank(region)
    if not non_blank["pass"]:
        raise ValueError(f"{path}: world viewport is blank/uniform")
    return {
        "x": x0,
        "y": y0,
        "width": crop_width,
        "height": crop_height,
        "sha256": hashlib.sha256(region.pixels).hexdigest(),
        "non_blank": non_blank,
    }


def _validate_diagnostics(
    diagnostics: dict[str, Any],
    *,
    exact_required: bool,
    solver_lane: str,
    label: str,
) -> None:
    expected_solver = EXACT_SOLVER_NAME if solver_lane == "exact" else BOXED_SOLVER_NAME
    if diagnostics.get("solver") != expected_solver:
        raise ValueError(
            f"{label}: active solver {diagnostics.get('solver')!r} differs from "
            f"the {solver_lane} lane"
        )
    if solver_lane == "boxed":
        if diagnostics.get("available") is not False:
            raise ValueError(f"{label}: boxed lane exposes exact-FBF diagnostics")
        if diagnostics.get("gap") != BOXED_DIAGNOSTICS_GAP:
            raise ValueError(f"{label}: boxed diagnostic gap is unavailable or stale")
        return
    if not exact_required:
        return
    if not diagnostics.get("available"):
        raise ValueError(f"{label}: exact-FBF diagnostics are unavailable")

    counter_names = (
        "exact_attempts",
        "exact_solves",
        "accepted_at_cap",
        "exact_failures",
        "boxed_lcp_fallbacks",
    )
    counters: dict[str, int] = {}
    for name in counter_names:
        value = diagnostics.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"{label}: {name} is unavailable/invalid")
        counters[name] = value

    _validate_last_failure_diagnostics(diagnostics, label=label)

    if counters["exact_attempts"] != (
        counters["exact_solves"] + counters["exact_failures"]
    ):
        raise ValueError(f"{label}: exact group accounting is inconsistent")
    if counters["accepted_at_cap"] > counters["exact_solves"]:
        raise ValueError(f"{label}: accepted-at-cap accounting is inconsistent")
    if counters["accepted_at_cap"] != 0:
        raise ValueError(f"{label}: accepted_at_cap={counters['accepted_at_cap']}")
    if counters["exact_failures"] != 0:
        raise ValueError(f"{label}: exact_failures={counters['exact_failures']}")
    if counters["boxed_lcp_fallbacks"] != 0:
        raise ValueError(
            f"{label}: boxed_lcp_fallbacks=" f"{counters['boxed_lcp_fallbacks']}"
        )

    attempts = counters["exact_attempts"]
    contacts = diagnostics.get("contacts")
    if isinstance(contacts, bool) or not isinstance(contacts, int) or contacts < 0:
        raise ValueError(f"{label}: contact diagnostics are unavailable/invalid")
    worst_residual = diagnostics.get("worst_residual")
    if attempts == 0:
        if contacts != 0:
            raise ValueError(f"{label}: contact exists before any exact attempt")
        if worst_residual is not None:
            raise ValueError(f"{label}: worst residual exists before any attempt")
        if diagnostics.get("status") != "not_run":
            raise ValueError(f"{label}: exact solver ran without an attempt")
        if diagnostics.get("fbf_status") != "not_run":
            raise ValueError(f"{label}: FBF solver ran without an attempt")
        if diagnostics.get("residual") is not None:
            raise ValueError(f"{label}: exact residual exists before any attempt")
        return

    if diagnostics.get("status") != "success":
        raise ValueError(f"{label}: exact solver status={diagnostics.get('status')!r}")
    if diagnostics.get("fbf_status") != "success":
        raise ValueError(f"{label}: FBF status={diagnostics.get('fbf_status')!r}")
    residual = diagnostics.get("residual")
    if not isinstance(residual, (int, float)) or not math.isfinite(residual):
        raise ValueError(f"{label}: exact residual is unavailable/non-finite")
    if residual > 1e-6:
        raise ValueError(f"{label}: exact residual {residual} exceeds 1e-6")
    if (
        not isinstance(worst_residual, (int, float))
        or isinstance(worst_residual, bool)
        or not math.isfinite(worst_residual)
    ):
        raise ValueError(f"{label}: worst residual is unavailable/non-finite")
    if worst_residual > 1e-6:
        raise ValueError(
            f"{label}: trajectory worst residual {worst_residual} exceeds 1e-6"
        )


def _validate_source_continuation_diagnostics(
    diagnostics: dict[str, Any], *, label: str
) -> dict[str, Any]:
    _validate_diagnostics(
        diagnostics, exact_required=False, solver_lane="exact", label=label
    )
    if diagnostics.get("available") is not True:
        raise ValueError(f"{label}: exact-FBF diagnostics are unavailable")

    try:
        reason, continuation = _evaluate_source_continuation_gate(diagnostics)
    except ValueError as error:
        raise ValueError(f"{label}: {error}") from error
    if reason is not None:
        raise ValueError(f"{label}: source-continuation gate would trigger: {reason}")
    if continuation is None:
        raise ValueError(f"{label}: source-continuation telemetry is missing")

    _validate_last_failure_diagnostics(diagnostics, label=label)
    cumulative = continuation.get("cumulative")
    if not isinstance(cumulative, dict):
        raise ValueError(
            f"{label}: source-continuation cumulative counters are missing"
        )
    for name in (
        "plateaus_accepted",
        "max_iterations_accepted",
        "line_search_shrink_caps",
    ):
        value = cumulative.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(
                f"{label}: source-continuation cumulative.{name} is invalid"
            )
    if cumulative["max_iterations_accepted"] != diagnostics.get("accepted_at_cap"):
        raise ValueError(f"{label}: cumulative max-iteration counter differs")

    last_attempt = continuation.get("last_attempt")
    if not isinstance(last_attempt, dict):
        raise ValueError(f"{label}: source-continuation last attempt is missing")
    groups = continuation["group_outcomes"]
    if groups:
        last = groups[-1]
        if diagnostics.get("status") != last.get("status") or diagnostics.get(
            "fbf_status"
        ) != last.get("fbf_status"):
            raise ValueError(f"{label}: last-group status telemetry differs")
    elif (
        diagnostics.get("exact_attempts") == 0
        and continuation.get("last_active") is not False
    ):
        raise ValueError(f"{label}: continuation active before the first attempt")
    return continuation


def _new_source_continuation_trajectory_state() -> dict[str, Any]:
    return {
        "aggregate": {
            name: 0
            for name in (
                "exact_attempts",
                "exact_solves",
                "accepted_at_cap",
                "exact_failures",
                "boxed_lcp_fallbacks",
            )
        },
        "continuation": {
            name: 0
            for name in (
                "plateaus_accepted",
                "max_iterations_accepted",
                "line_search_shrink_caps",
            )
        },
        "worst_residual": None,
        "latest_group": None,
        "next_solve_index": 0,
    }


def _validate_source_continuation_trajectory_step(
    diagnostics: dict[str, Any],
    continuation: dict[str, Any],
    *,
    step_index: int,
    state: dict[str, Any],
    label: str,
    require_accepted_outcome: bool,
) -> None:
    prior_aggregate = state["aggregate"]
    current_aggregate: dict[str, int] = {}
    for name, previous in prior_aggregate.items():
        value = diagnostics.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"{label}: cumulative {name} is invalid")
        if value < previous:
            raise ValueError(f"{label}: cumulative {name} regressed")
        current_aggregate[name] = value

    contacts = diagnostics.get("contacts")
    if isinstance(contacts, bool) or not isinstance(contacts, int) or contacts < 0:
        raise ValueError(f"{label}: contact diagnostics are unavailable")
    attempt_delta = (
        current_aggregate["exact_attempts"] - prior_aggregate["exact_attempts"]
    )
    solve_delta = current_aggregate["exact_solves"] - prior_aggregate["exact_solves"]
    if step_index == 0 and any(current_aggregate.values()):
        raise ValueError(f"{label}: exact activity exists before simulation")
    if require_accepted_outcome and step_index > 0 and contacts > 0:
        if attempt_delta <= 0 or solve_delta <= 0:
            raise ValueError(
                f"{label}: contact step did not advance exact attempt/solve counters"
            )

    cumulative = continuation.get("cumulative")
    per_step = continuation.get("step")
    groups = continuation.get("group_outcomes")
    if (
        not isinstance(cumulative, dict)
        or not isinstance(per_step, dict)
        or not isinstance(groups, list)
    ):
        raise ValueError(f"{label}: continuation trajectory telemetry is missing")
    prior_continuation = state["continuation"]
    current_continuation: dict[str, int] = {}
    for name, previous in prior_continuation.items():
        value = cumulative.get(name)
        step_value = per_step.get(name)
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < 0
            or isinstance(step_value, bool)
            or not isinstance(step_value, int)
            or step_value < 0
        ):
            raise ValueError(f"{label}: continuation counter {name} is invalid")
        if value < previous:
            raise ValueError(f"{label}: cumulative {name} regressed")
        if step_value != value - previous:
            raise ValueError(
                f"{label}: per-step {name} differs from its cumulative delta"
            )
        current_continuation[name] = value
    if (
        per_step.get("exact_attempts") != attempt_delta
        or per_step.get("exact_solves") != solve_delta
    ):
        raise ValueError(f"{label}: continuation attempt/solve deltas differ")

    skip_last_binding = bool(groups) and not require_accepted_outcome
    latest_group = groups[-1] if groups else state["latest_group"]
    next_solve_index = state["next_solve_index"]
    for group in groups:
        solve_index = group.get("solve_index")
        if solve_index != next_solve_index:
            raise ValueError(
                f"{label}: group solve indices are not globally contiguous"
            )
        next_solve_index += 1
    if skip_last_binding:
        pass
    elif latest_group is not None:
        last_attempt = continuation.get("last_attempt")
        if not isinstance(last_attempt, dict):
            raise ValueError(f"{label}: continuation last attempt is missing")
        cap_count = latest_group["line_search_shrink_cap_count"]
        if (
            continuation.get("last_active")
            != latest_group.get("source_continuation_active")
            or last_attempt.get("line_search_shrink_cap_count") != cap_count
            or last_attempt.get("line_search_shrink_cap_reached") != (cap_count > 0)
            or last_attempt.get("correction_step_size")
            != latest_group.get("correction_step_size")
            or last_attempt.get("last_inner_solve_step_size")
            != latest_group.get("last_inner_solve_step_size")
        ):
            raise ValueError(f"{label}: last-attempt telemetry lost prior binding")
        if require_accepted_outcome and (
            diagnostics.get("status") != latest_group.get("status")
            or diagnostics.get("fbf_status") != latest_group.get("fbf_status")
        ):
            raise ValueError(f"{label}: last status lost prior group binding")
    else:
        last_attempt = continuation.get("last_attempt")
        if not isinstance(last_attempt, dict) or (
            continuation.get("last_active") is not False
            or last_attempt.get("line_search_shrink_cap_reached") is not False
            or last_attempt.get("line_search_shrink_cap_count") != 0
            or last_attempt.get("correction_step_size") is not None
            or last_attempt.get("last_inner_solve_step_size") is not None
        ):
            raise ValueError(f"{label}: pre-solve last-attempt defaults differ")
        if require_accepted_outcome and (
            diagnostics.get("status") != "not_run"
            or diagnostics.get("fbf_status") != "not_run"
        ):
            raise ValueError(f"{label}: pre-solve status defaults differ")

    current_worst = diagnostics.get("worst_residual")
    prior_worst = state["worst_residual"]
    if current_worst is not None:
        if not _is_finite_number(current_worst) or current_worst < 0.0:
            raise ValueError(f"{label}: cumulative worst residual is invalid")
        if prior_worst is not None and current_worst < prior_worst:
            raise ValueError(f"{label}: cumulative worst residual regressed")
        state["worst_residual"] = float(current_worst)
    state["aggregate"] = current_aggregate
    state["continuation"] = current_continuation
    if groups:
        state["latest_group"] = groups[-1]
    state["next_solve_index"] = next_solve_index


def _validate_last_failure_diagnostics(
    diagnostics: dict[str, Any], *, label: str
) -> None:
    exact_failures = diagnostics.get("exact_failures")
    if (
        isinstance(exact_failures, bool)
        or not isinstance(exact_failures, int)
        or exact_failures < 0
    ):
        raise ValueError(f"{label}: exact_failures is unavailable/invalid")

    last_failure = diagnostics.get("last_failure")
    if exact_failures == 0:
        if last_failure is not None:
            raise ValueError(f"{label}: last_failure exists without a failure")
        return
    if not isinstance(last_failure, dict):
        raise ValueError(f"{label}: last_failure is unavailable/invalid")

    status = last_failure.get("status")
    build_status = last_failure.get("build_status")
    fbf_status = last_failure.get("fbf_status")
    if status not in {"invalid_options", "unsupported_problem", "fbf_failed"}:
        raise ValueError(f"{label}: last_failure.status is unavailable/invalid")
    if build_status not in {
        "success",
        "empty_input",
        "null_constraint",
        "invalid_options",
        "unsupported_dimension",
        "unsupported_bounds",
        "unsupported_friction_coupling",
        "unsupported_anisotropic_friction",
        "non_finite_data",
    }:
        raise ValueError(f"{label}: last_failure.build_status is unavailable/invalid")
    if fbf_status not in {
        "success",
        "max_iterations",
        "invalid_input",
        "inner_solver_failed",
        "step_size_underflow",
        "non_finite_value",
    }:
        raise ValueError(f"{label}: last_failure.fbf_status is unavailable/invalid")
    for name in (
        "contact_count",
        "best_iteration",
        "iterations",
        "shrink_iterations",
    ):
        value = last_failure.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"{label}: last_failure.{name} is unavailable/invalid")
    contact_count = last_failure["contact_count"]
    best_iteration = last_failure["best_iteration"]
    iterations = last_failure["iterations"]
    if best_iteration > iterations:
        raise ValueError(f"{label}: last_failure best iteration exceeds iterations")
    if status == "fbf_failed":
        if build_status != "success" or contact_count == 0:
            raise ValueError(f"{label}: last_failure FBF failure is inconsistent")
    elif contact_count != 0 or fbf_status != "invalid_input":
        raise ValueError(f"{label}: last_failure adapter failure is inconsistent")
    if status == "invalid_options" and build_status != "empty_input":
        raise ValueError(f"{label}: last_failure invalid-options state is inconsistent")
    if status == "unsupported_problem" and build_status == "success":
        raise ValueError(f"{label}: last_failure unsupported state is inconsistent")

    worst_contact_fields = (
        "worst_primal_contact",
        "worst_dual_contact",
        "worst_complementarity_contact",
    )
    for name in worst_contact_fields:
        value = last_failure.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value < -1:
            raise ValueError(f"{label}: last_failure.{name} is unavailable/invalid")
        if value >= contact_count and value != -1:
            raise ValueError(f"{label}: last_failure.{name} is out of range")

    residual_fields = (
        "residual",
        "primal_feasibility",
        "dual_feasibility",
        "complementarity",
    )
    numeric_fields = residual_fields + (
        "best_residual",
        "step_size",
        "safe_step_size",
        "coupling_variation_ratio",
    )
    for name in numeric_fields:
        value = last_failure.get(name)
        if value is not None and (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 0.0
        ):
            raise ValueError(f"{label}: last_failure.{name} is invalid")
    residual_values = [last_failure.get(name) for name in residual_fields]
    if any(value is None for value in residual_values) and not all(
        value is None for value in residual_values
    ):
        raise ValueError(f"{label}: last_failure residual terms are inconsistent")
    if fbf_status not in {"invalid_input", "non_finite_value"} and any(
        value is None for value in residual_values
    ):
        raise ValueError(f"{label}: last_failure residual terms are unavailable")
    if not any(value is None for value in residual_values) and not math.isclose(
        last_failure["residual"],
        max(
            last_failure["primal_feasibility"],
            last_failure["dual_feasibility"],
            last_failure["complementarity"],
        ),
        rel_tol=1e-12,
        abs_tol=1e-15,
    ):
        raise ValueError(f"{label}: last_failure residual aggregate is inconsistent")
    step_size = last_failure.get("step_size")
    if step_size is not None and (
        step_size < 0.0 or (step_size == 0.0 and fbf_status != "step_size_underflow")
    ):
        raise ValueError(f"{label}: last_failure.step_size is invalid")
    safe_step_size = last_failure.get("safe_step_size")
    if safe_step_size is not None and safe_step_size <= 0.0:
        raise ValueError(f"{label}: last_failure.safe_step_size is invalid")
    if status != "fbf_failed":
        if any(
            last_failure[name] != 0
            for name in ("best_iteration", "iterations", "shrink_iterations")
        ) or any(last_failure.get(name) is not None for name in numeric_fields):
            raise ValueError(f"{label}: last_failure adapter failure is inconsistent")


def _legacy_demo_command(command: Sequence[str]) -> list[str]:
    return [item for item in command if item != HEADLESS_EXACT_FBF_FAIL_FAST_FLAG]


def _is_literal_masonry_arch_schedule(schedule: CaptureSchedule) -> bool:
    return (
        schedule.source_schedule_id or schedule.id
    ) == "masonry_arch_25_literal_standing"


def _author_masonry_arch_source_schedule_id(
    schedule: CaptureSchedule,
) -> str | None:
    schedule_id = schedule.source_schedule_id or schedule.id
    if schedule_id in {
        "masonry_arch_25_author_crown_impact_current_source",
        "masonry_arch_101_author_standing_current_source",
    }:
        return schedule_id
    return None


def _is_author_card_house_impact_schedule(schedule: CaptureSchedule) -> bool:
    return (schedule.source_schedule_id or schedule.id) in {
        "card_house_author_4_impact_current_source",
        "card_house_author_4_impact_source_continuation_current_source",
        "card_house_author_10_impact_current_source",
    }


def _author_card_house_dynamics_selection(
    schedule: CaptureSchedule,
) -> dict[str, Any]:
    schedule_id = schedule.source_schedule_id or schedule.id
    if schedule_id in {
        "card_house_author_4_impact_current_source",
        "card_house_author_4_impact_source_continuation_current_source",
    }:
        return {
            "levels": 4,
            "frames": 600,
            "cards": 26,
            "leaning_cards": 20,
            "bridge_cards": 6,
            "cube_initial_height_m": 10.66642467908744,
            "collision_shape_frames": 31,
            "contact_gap_shape_frames": 0,
            "ground_contact_gap_m": 0.0,
            "dynamic_shape_contact_gap_m": 0.0,
            "ground_contact_gap_shape_frames": 0,
            "dynamic_contact_gap_shape_frames": 0,
            "source_contact_gap_values_represented": False,
        }
    if schedule_id == "card_house_author_10_impact_current_source":
        return {
            "levels": 10,
            "frames": 800,
            "cards": 155,
            "leaning_cards": 110,
            "bridge_cards": 45,
            "cube_initial_height_m": 25.1660616977186,
            "collision_shape_frames": 160,
            "contact_gap_shape_frames": 160,
            "ground_contact_gap_m": 0.1,
            "dynamic_shape_contact_gap_m": 0.005,
            "ground_contact_gap_shape_frames": 1,
            "dynamic_contact_gap_shape_frames": 159,
            "source_contact_gap_values_represented": True,
        }
    raise ValueError(f"{schedule.id}: unknown author card-house dynamics selection")


def _is_author_painleve_schedule(schedule: CaptureSchedule) -> bool:
    return (schedule.source_schedule_id or schedule.id) in AUTHOR_PAINLEVE_MEMBERS


def _is_author_card_house_source_continuation_schedule(
    schedule: CaptureSchedule,
) -> bool:
    return (
        schedule.source_schedule_id or schedule.id
    ) == "card_house_author_4_impact_source_continuation_current_source"


def _is_finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(value)
    )


def _validate_author_painleve_scene_state_trace(
    schedule: CaptureSchedule,
    trajectory_steps: Any,
    *,
    sidecar_path: Path,
    expected_last_step: int,
) -> dict[str, Any] | None:
    if not _is_author_painleve_schedule(schedule):
        return None
    if (
        isinstance(expected_last_step, bool)
        or not isinstance(expected_last_step, int)
        or expected_last_step < 0
        or expected_last_step > schedule.total_steps
    ):
        raise ValueError(f"{sidecar_path}: invalid Painleve trace endpoint")
    if not isinstance(trajectory_steps, list) or len(trajectory_steps) != (
        expected_last_step + 1
    ):
        raise ValueError(f"{sidecar_path}: author Painleve scene-state trace length")

    samples: list[dict[str, Any]] = []
    initial_position: list[float] | None = None
    maximum_horizontal_displacement = 0.0
    maximum_absolute_pitch = 0.0
    minimum_uprightness = math.inf
    maximum_angular_speed = 0.0
    for expected_step, item in enumerate(trajectory_steps):
        if not isinstance(item, dict) or item.get("step") != expected_step:
            raise ValueError(
                f"{sidecar_path}: author Painleve scene-state steps are out of order"
            )
        sim_time = item.get("sim_time")
        expected_time = schedule.time_at_step(expected_step)
        if not _is_finite_number(sim_time) or not math.isclose(
            float(sim_time), expected_time, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(
                f"{sidecar_path}: author Painleve scene-state time mismatch at "
                f"step {expected_step}"
            )

        state = item.get("scene_state")
        label = f"{sidecar_path}: author Painleve scene state at step {expected_step}"
        if not isinstance(state, dict):
            raise ValueError(f"{label} is missing")
        if set(state) != set(AUTHOR_PAINLEVE_SCENE_STATE_FIELDS) or not all(
            _is_finite_number(state.get(name))
            for name in AUTHOR_PAINLEVE_SCENE_STATE_FIELDS
        ):
            raise ValueError(f"{label} fields changed or are non-finite")
        world_time = state.get("world_time_seconds")
        if not math.isclose(
            float(world_time), expected_time, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(f"{label} world time changed")

        position = [float(state[f"position_{axis}_m"]) for axis in ("x", "y", "z")]
        rotation = [
            [float(state[f"rotation_{row}{column}"]) for column in range(3)]
            for row in range(3)
        ]
        body_up = [float(state[f"body_up_{axis}"]) for axis in ("x", "y", "z")]
        linear_velocity = [
            float(state[f"linear_velocity_{axis}_m_s"]) for axis in ("x", "y", "z")
        ]
        angular_velocity = [
            float(state[f"angular_velocity_{axis}_rad_s"]) for axis in ("x", "y", "z")
        ]
        uprightness = float(state["uprightness_cosine"])
        pitch = float(state["pitch_rad"])

        for row in range(3):
            for column in range(3):
                dot = sum(
                    rotation[index][row] * rotation[index][column] for index in range(3)
                )
                expected_dot = 1.0 if row == column else 0.0
                if not math.isclose(dot, expected_dot, rel_tol=0.0, abs_tol=1e-9):
                    raise ValueError(f"{label} rotation is not orthonormal")
        determinant = (
            rotation[0][0]
            * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
            - rotation[0][1]
            * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
            + rotation[0][2]
            * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
        )
        if not math.isclose(determinant, 1.0, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(f"{label} rotation is not proper")

        expected_body_up = [rotation[row][2] for row in range(3)]
        if any(
            not math.isclose(
                body_up[index],
                expected_body_up[index],
                rel_tol=0.0,
                abs_tol=1e-12,
            )
            for index in range(3)
        ):
            raise ValueError(f"{label} body-up vector disagrees with rotation")
        if not math.isclose(
            uprightness, expected_body_up[2], rel_tol=0.0, abs_tol=1e-12
        ):
            raise ValueError(f"{label} uprightness disagrees with rotation")
        expected_pitch = math.atan2(rotation[0][2], rotation[2][2])
        if not math.isclose(pitch, expected_pitch, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(f"{label} pitch disagrees with rotation")

        position_values = position
        linear_values = linear_velocity
        angular_values = angular_velocity
        if initial_position is None:
            initial_position = position_values
        horizontal_displacement = math.hypot(
            position_values[0] - initial_position[0],
            position_values[1] - initial_position[1],
        )
        angular_speed = math.sqrt(sum(value * value for value in angular_values))
        maximum_horizontal_displacement = max(
            maximum_horizontal_displacement, horizontal_displacement
        )
        maximum_absolute_pitch = max(maximum_absolute_pitch, abs(pitch))
        minimum_uprightness = min(minimum_uprightness, uprightness)
        maximum_angular_speed = max(maximum_angular_speed, angular_speed)
        samples.append(
            {
                "step": expected_step,
                "time_seconds": float(sim_time),
                "position_m": position_values,
                "uprightness_cosine": uprightness,
                "pitch_rad": pitch,
                "linear_velocity_m_s": linear_values,
                "angular_velocity_rad_s": angular_values,
            }
        )

    initial = samples[0]
    final = samples[-1]
    displacement = [
        final["position_m"][index] - initial["position_m"][index] for index in range(3)
    ]
    return {
        "schema_version": "dart.fbf_author_painleve_scene_state_metrics/v1",
        "scene_id": schedule.scene,
        "body_name": "painleve_author_box",
        "sample_count": len(samples),
        "initial": initial,
        "final": final,
        "displacement_m": displacement,
        "horizontal_displacement_m": math.hypot(displacement[0], displacement[1]),
        "maximum_horizontal_displacement_from_initial_m": (
            maximum_horizontal_displacement
        ),
        "maximum_absolute_pitch_rad": maximum_absolute_pitch,
        "minimum_uprightness_cosine": minimum_uprightness,
        "maximum_angular_speed_rad_s": maximum_angular_speed,
    }


def _validate_author_masonry_arch_101_scene_state_trace(
    schedule: CaptureSchedule,
    trajectory_steps: Any,
    *,
    sidecar_path: Path,
    expected_last_step: int,
) -> dict[str, Any] | None:
    if (
        _author_masonry_arch_source_schedule_id(schedule)
        != "masonry_arch_101_author_standing_current_source"
    ):
        return None
    if (
        isinstance(expected_last_step, bool)
        or not isinstance(expected_last_step, int)
        or expected_last_step < 0
        or expected_last_step > schedule.total_steps
    ):
        raise ValueError(f"{sidecar_path}: invalid author arch trace endpoint")
    if not isinstance(trajectory_steps, list) or len(trajectory_steps) != (
        expected_last_step + 1
    ):
        raise ValueError(f"{sidecar_path}: author arch scene-state trace length")

    expected_lane = {
        "solver_lane_exact_fbf": 1.0 if schedule.solver_lane == "exact" else 0.0,
        "solver_lane_boxed_lcp": 1.0 if schedule.solver_lane == "boxed" else 0.0,
    }
    initial_crown_z: float | None = None
    maximum_mobile_displacement = 0.0
    maximum_mobile_rotation = 0.0
    minimum_crown_z = math.inf
    trace_integrity_valid = True
    standing_components_valid = True
    final_state: dict[str, Any] | None = None
    for expected_step, item in enumerate(trajectory_steps):
        if not isinstance(item, dict) or item.get("step") != expected_step:
            raise ValueError(
                f"{sidecar_path}: author arch scene-state steps are out of order"
            )
        expected_time = schedule.time_at_step(expected_step)
        sim_time = item.get("sim_time")
        if not _is_finite_number(sim_time) or not math.isclose(
            float(sim_time), expected_time, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(
                f"{sidecar_path}: author arch scene-state time mismatch at "
                f"step {expected_step}"
            )

        state = item.get("scene_state")
        label = f"{sidecar_path}: author arch scene state at step {expected_step}"
        if not isinstance(state, dict):
            raise ValueError(f"{label} is missing")
        if set(state) != set(AUTHOR_MASONRY_ARCH_101_SCENE_STATE_FIELDS) or not all(
            _is_finite_number(state.get(name))
            for name in AUTHOR_MASONRY_ARCH_101_SCENE_STATE_FIELDS
        ):
            raise ValueError(f"{label} fields changed or are non-finite")
        if any(
            float(state[name]) != expected for name, expected in expected_lane.items()
        ):
            raise ValueError(f"{label} solver lane changed")
        if not math.isclose(
            float(state["world_time_seconds"]),
            expected_time,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError(f"{label} world time changed")

        crown_initial_z = float(state["crown_body_origin_initial_z"])
        crown_z = float(state["crown_body_origin_z"])
        crown_displacement = float(state["crown_body_origin_displacement"])
        if initial_crown_z is None:
            initial_crown_z = crown_initial_z
        elif not math.isclose(
            crown_initial_z, initial_crown_z, rel_tol=0.0, abs_tol=1e-12
        ):
            raise ValueError(f"{label} crown reference height changed")
        if crown_displacement + 1e-9 < abs(crown_z - crown_initial_z):
            raise ValueError(f"{label} crown displacement is inconsistent")

        inventory_valid = (
            float(state["world_skeleton_count"]) == 105.0
            and float(state["observed_stone_count"]) == 101.0
            and float(state["observed_mobile_stone_count"]) == 99.0
            and float(state["mobility_matching_stone_count"]) == 101.0
            and float(state["observed_cube_count"]) == 3.0
            and float(state["ground_valid"]) == 1.0
            and float(state["crown_observed"]) == 1.0
        )
        all_bodies_finite = (
            float(state["finite_stone_count"]) == 101.0
            and float(state["finite_cube_count"]) == 3.0
        )
        cubes_remain_kinematic = float(state["kinematic_cube_count"]) == 3.0
        cubes_remain_pinned = (
            cubes_remain_kinematic
            and float(state["max_kinematic_cube_body_origin_displacement"])
            <= AUTHOR_MASONRY_ARCH_101_MAX_KINEMATIC_CUBE_POSE_ERROR
            and float(state["max_kinematic_cube_rotation_delta_rad"])
            <= AUTHOR_MASONRY_ARCH_101_MAX_KINEMATIC_CUBE_POSE_ERROR
        )
        mobile_displacement_bounded = (
            float(state["max_mobile_body_origin_displacement"])
            <= AUTHOR_MASONRY_ARCH_101_MAX_MOBILE_BODY_ORIGIN_DISPLACEMENT
        )
        mobile_rotation_bounded = (
            float(state["max_mobile_rotation_delta_rad"])
            <= AUTHOR_MASONRY_ARCH_101_MAX_MOBILE_ROTATION_DELTA_RAD
        )
        crown_height_preserved = (
            crown_z >= crown_initial_z - AUTHOR_MASONRY_ARCH_101_MAX_CROWN_HEIGHT_LOSS
        )
        horizon_complete = (
            expected_time
            >= AUTHOR_MASONRY_ARCH_101_REQUIRED_WORLD_TIME_SECONDS
            - AUTHOR_MASONRY_ARCH_101_HORIZON_TIME_TOLERANCE_SECONDS
        )
        complete_trace_valid = (
            horizon_complete
            and inventory_valid
            and all_bodies_finite
            and cubes_remain_pinned
        )
        standing_outcome_valid = (
            complete_trace_valid
            and mobile_displacement_bounded
            and mobile_rotation_bounded
            and crown_height_preserved
        )
        expected_flags = {
            "standing_horizon_complete": horizon_complete,
            "standing_inventory_valid": inventory_valid,
            "standing_all_bodies_finite": all_bodies_finite,
            "standing_cubes_remain_kinematic": cubes_remain_kinematic,
            "standing_cubes_remain_pinned": cubes_remain_pinned,
            "standing_mobile_displacement_bounded": mobile_displacement_bounded,
            "standing_mobile_rotation_bounded": mobile_rotation_bounded,
            "standing_crown_height_preserved": crown_height_preserved,
            "standing_complete_trace_valid": complete_trace_valid,
            "standing_outcome_valid": standing_outcome_valid,
            "standing_lane_evidence_qualifies": standing_outcome_valid,
        }
        if any(
            float(state[name]) != (1.0 if expected else 0.0)
            for name, expected in expected_flags.items()
        ):
            raise ValueError(f"{label} standing oracle disagrees with state")

        trace_integrity_valid = (
            trace_integrity_valid
            and inventory_valid
            and all_bodies_finite
            and cubes_remain_pinned
        )
        standing_components_valid = (
            standing_components_valid
            and mobile_displacement_bounded
            and mobile_rotation_bounded
            and crown_height_preserved
        )

        maximum_mobile_displacement = max(
            maximum_mobile_displacement,
            float(state["max_mobile_body_origin_displacement"]),
        )
        maximum_mobile_rotation = max(
            maximum_mobile_rotation,
            float(state["max_mobile_rotation_delta_rad"]),
        )
        minimum_crown_z = min(minimum_crown_z, crown_z)
        final_state = state

    assert final_state is not None and initial_crown_z is not None
    complete_horizon = expected_last_step == AUTHOR_MASONRY_ARCH_101_REQUIRED_SUBSTEPS
    complete_trace_valid = complete_horizon and trace_integrity_valid
    standing_outcome_valid = complete_trace_valid and standing_components_valid
    return {
        "schema_version": AUTHOR_MASONRY_ARCH_101_SCENE_STATE_SCHEMA_VERSION,
        "scene_id": schedule.scene,
        "solver_lane": schedule.solver_lane,
        "sample_count": len(trajectory_steps),
        "completed_substeps": expected_last_step,
        "complete_horizon": complete_horizon,
        "initial_crown_body_origin_z": initial_crown_z,
        "final_crown_body_origin_z": float(final_state["crown_body_origin_z"]),
        "minimum_crown_body_origin_z": minimum_crown_z,
        "maximum_mobile_body_origin_displacement": maximum_mobile_displacement,
        "maximum_mobile_rotation_delta_rad": maximum_mobile_rotation,
        "complete_trace_valid": complete_trace_valid,
        "standing_outcome_valid": standing_outcome_valid,
        "lane_evidence_qualifies": standing_outcome_valid,
        "comparison_capture_valid": complete_trace_valid,
        "physical_outcome_validated": standing_outcome_valid,
    }


def _require_author_masonry_arch_101_complete_outcome(
    schedule: CaptureSchedule,
    metrics: dict[str, Any] | None,
    *,
    sidecar_path: Path,
) -> None:
    if metrics is None:
        return
    if schedule.solver_lane == "exact" and not metrics["standing_outcome_valid"]:
        raise ValueError(
            f"{sidecar_path}: exact author arch did not pass the standing oracle"
        )
    if schedule.solver_lane == "boxed" and not metrics["comparison_capture_valid"]:
        raise ValueError(
            f"{sidecar_path}: boxed author arch comparison trace is incomplete"
        )


def _validate_author_painleve_dart_adapter_outcome(
    schedule: CaptureSchedule,
    trajectory_steps: Any,
    *,
    sidecar_path: Path,
    expected_last_step: int,
) -> dict[str, Any] | None:
    """Classify only the complete, current-DART author-Painleve trace."""
    if not _is_author_painleve_schedule(schedule):
        return None

    source_schedule_id = schedule.source_schedule_id or schedule.id
    expected_class = AUTHOR_PAINLEVE_EXPECTED_OUTCOMES.get(
        (source_schedule_id, schedule.solver_lane)
    )
    if expected_class is None:
        raise ValueError(
            f"{sidecar_path}: unsupported author Painleve schedule/solver lane"
        )
    terminal_start, terminal_end = AUTHOR_PAINLEVE_TERMINAL_WINDOW
    if (
        schedule.total_steps != terminal_end
        or expected_last_step != terminal_end
        or not isinstance(trajectory_steps, list)
        or len(trajectory_steps) != terminal_end + 1
    ):
        raise ValueError(
            f"{sidecar_path}: author Painleve outcome requires the complete "
            f"{terminal_end + 1}-sample trace"
        )

    state_metrics = _validate_author_painleve_scene_state_trace(
        schedule,
        trajectory_steps,
        sidecar_path=sidecar_path,
        expected_last_step=expected_last_step,
    )
    assert state_metrics is not None

    initial_state = trajectory_steps[0]["scene_state"]
    expected_initial_state = {
        "world_time_seconds": 0.0,
        "position_x_m": 0.0,
        "position_y_m": 0.0,
        "position_z_m": 0.3,
        "rotation_00": 1.0,
        "rotation_01": 0.0,
        "rotation_02": 0.0,
        "rotation_10": 0.0,
        "rotation_11": 1.0,
        "rotation_12": 0.0,
        "rotation_20": 0.0,
        "rotation_21": 0.0,
        "rotation_22": 1.0,
        "body_up_x": 0.0,
        "body_up_y": 0.0,
        "body_up_z": 1.0,
        "uprightness_cosine": 1.0,
        "pitch_rad": 0.0,
        "linear_velocity_x_m_s": 4.0,
        "linear_velocity_y_m_s": 0.0,
        "linear_velocity_z_m_s": 0.0,
        "angular_velocity_x_rad_s": 0.0,
        "angular_velocity_y_rad_s": 0.0,
        "angular_velocity_z_rad_s": 0.0,
    }
    if any(
        not _contract_value_matches(initial_state.get(key), expected)
        for key, expected in expected_initial_state.items()
    ):
        raise ValueError(f"{sidecar_path}: author Painleve initial state changed")

    horizontal_travel = float(state_metrics["horizontal_displacement_m"])
    if horizontal_travel < AUTHOR_PAINLEVE_MIN_HORIZONTAL_TRAVEL_M:
        raise ValueError(
            f"{sidecar_path}: author Painleve horizontal travel is below "
            f"{AUTHOR_PAINLEVE_MIN_HORIZONTAL_TRAVEL_M} m"
        )

    terminal_samples = trajectory_steps[terminal_start : terminal_end + 1]
    linear_speeds: list[float] = []
    angular_speeds: list[float] = []
    uprightness_values: list[float] = []
    heights: list[float] = []
    for item in terminal_samples:
        state = item["scene_state"]
        linear_speeds.append(
            math.sqrt(
                sum(
                    float(state[f"linear_velocity_{axis}_m_s"]) ** 2
                    for axis in ("x", "y", "z")
                )
            )
        )
        angular_speeds.append(
            math.sqrt(
                sum(
                    float(state[f"angular_velocity_{axis}_rad_s"]) ** 2
                    for axis in ("x", "y", "z")
                )
            )
        )
        uprightness_values.append(float(state["uprightness_cosine"]))
        heights.append(float(state["position_z_m"]))

    max_linear_speed = max(linear_speeds)
    max_angular_speed = max(angular_speeds)
    if (
        max_linear_speed > AUTHOR_PAINLEVE_MAX_LINEAR_SPEED_M_S
        or max_angular_speed > AUTHOR_PAINLEVE_MAX_ANGULAR_SPEED_RAD_S
    ):
        raise ValueError(
            f"{sidecar_path}: author Painleve terminal window is not near rest"
        )

    upright_height_minimum, upright_height_maximum = (
        AUTHOR_PAINLEVE_UPRIGHT_HEIGHT_RANGE_M
    )
    tumbled_height_minimum, tumbled_height_maximum = (
        AUTHOR_PAINLEVE_TUMBLED_HEIGHT_RANGE_M
    )
    upright = all(
        uprightness >= AUTHOR_PAINLEVE_UPRIGHTNESS_MINIMUM
        and upright_height_minimum <= height <= upright_height_maximum
        for uprightness, height in zip(uprightness_values, heights)
    )
    tumbled = all(
        abs(uprightness) <= AUTHOR_PAINLEVE_TUMBLED_ABS_UPRIGHTNESS_MAXIMUM
        and tumbled_height_minimum <= height <= tumbled_height_maximum
        for uprightness, height in zip(uprightness_values, heights)
    )
    if upright == tumbled:
        raise ValueError(
            f"{sidecar_path}: author Painleve terminal pose is unclassified"
        )
    observed_class = "upright_near_rest" if upright else "tumbled_near_rest"
    if observed_class != expected_class:
        raise ValueError(
            f"{sidecar_path}: author Painleve outcome class {observed_class!r} "
            f"does not match expected {expected_class!r}"
        )

    return {
        "schema_version": AUTHOR_PAINLEVE_OUTCOME_SCHEMA_VERSION,
        "claim_scope": AUTHOR_PAINLEVE_OUTCOME_CLAIM_SCOPE,
        "source_schedule_id": source_schedule_id,
        "schedule_id": schedule.id,
        "solver_lane": schedule.solver_lane,
        "expected_class": expected_class,
        "observed_class": observed_class,
        "terminal_window": {
            "start_step": terminal_start,
            "end_step": terminal_end,
            "sample_count": len(terminal_samples),
            "max_linear_speed_m_s": max_linear_speed,
            "max_angular_speed_rad_s": max_angular_speed,
            "minimum_uprightness_cosine": min(uprightness_values),
            "maximum_uprightness_cosine": max(uprightness_values),
            "maximum_absolute_uprightness_cosine": max(
                abs(value) for value in uprightness_values
            ),
            "minimum_height_m": min(heights),
            "maximum_height_m": max(heights),
            "thresholds": {
                "max_linear_speed_m_s": AUTHOR_PAINLEVE_MAX_LINEAR_SPEED_M_S,
                "max_angular_speed_rad_s": AUTHOR_PAINLEVE_MAX_ANGULAR_SPEED_RAD_S,
                "uprightness_minimum": AUTHOR_PAINLEVE_UPRIGHTNESS_MINIMUM,
                "upright_height_range_m": list(AUTHOR_PAINLEVE_UPRIGHT_HEIGHT_RANGE_M),
                "tumbled_absolute_uprightness_maximum": (
                    AUTHOR_PAINLEVE_TUMBLED_ABS_UPRIGHTNESS_MAXIMUM
                ),
                "tumbled_height_range_m": list(AUTHOR_PAINLEVE_TUMBLED_HEIGHT_RANGE_M),
            },
        },
        "initial_state_gate": {
            "expected": expected_initial_state,
            "pass": True,
        },
        "horizontal_travel_m": horizontal_travel,
        "minimum_horizontal_travel_m": AUTHOR_PAINLEVE_MIN_HORIZONTAL_TRAVEL_M,
        "automated_current_dart_adapter_outcome_validated": True,
        "source_backend_equivalent": False,
        "trajectory_equivalent": False,
        "paper_figure_parity": False,
        "solver_superiority": False,
        "pass": True,
    }


def _is_author_painleve_solver_comparison_group(group: GroupOutputSpec) -> bool:
    return (
        group.solver_lane == "both"
        and len(group.members) == 2
        and group.members[0] in AUTHOR_PAINLEVE_MEMBERS
        and group.members[1] == _boxed_schedule_id(group.members[0])
    )


def _validate_author_painleve_solver_comparison(
    group: GroupOutputSpec,
    schedules: Sequence[CaptureSchedule],
    outcome_reports: Any,
    *,
    label: str,
) -> dict[str, Any] | None:
    if not _is_author_painleve_solver_comparison_group(group):
        return None
    if (
        len(schedules) != 2
        or tuple(schedule.id for schedule in schedules) != group.members
        or tuple(schedule.solver_lane for schedule in schedules) != SOLVER_LANES
    ):
        raise ValueError(f"{label}: author Painleve comparison lanes changed")
    if not isinstance(outcome_reports, list) or len(outcome_reports) != 2:
        raise ValueError(f"{label}: author Painleve comparison outcomes are missing")

    source_schedule_id = schedules[0].id
    expected_classes = [
        AUTHOR_PAINLEVE_EXPECTED_OUTCOMES[(source_schedule_id, lane)]
        for lane in SOLVER_LANES
    ]
    observed_classes: list[str] = []
    travel_by_lane: dict[str, float] = {}
    for schedule, report, expected_class in zip(
        schedules, outcome_reports, expected_classes
    ):
        if not isinstance(report, dict):
            raise ValueError(f"{label}: author Painleve comparison outcome is invalid")
        expected_claims = {
            "schema_version": AUTHOR_PAINLEVE_OUTCOME_SCHEMA_VERSION,
            "claim_scope": AUTHOR_PAINLEVE_OUTCOME_CLAIM_SCOPE,
            "source_schedule_id": source_schedule_id,
            "schedule_id": schedule.id,
            "solver_lane": schedule.solver_lane,
            "expected_class": expected_class,
            "observed_class": expected_class,
            "automated_current_dart_adapter_outcome_validated": True,
            "source_backend_equivalent": False,
            "trajectory_equivalent": False,
            "paper_figure_parity": False,
            "solver_superiority": False,
            "pass": True,
        }
        if any(report.get(key) != value for key, value in expected_claims.items()):
            raise ValueError(
                f"{label}: author Painleve comparison outcome claim changed"
            )
        travel = report.get("horizontal_travel_m")
        if (
            not _is_finite_number(travel)
            or float(travel) < AUTHOR_PAINLEVE_MIN_HORIZONTAL_TRAVEL_M
        ):
            raise ValueError(f"{label}: author Painleve comparison travel is invalid")
        observed_classes.append(expected_class)
        travel_by_lane[schedule.solver_lane] = float(travel)

    expected_relation = (
        "same_upright_near_rest"
        if source_schedule_id == "painleve_author_mu05"
        else "exact_tumbled_boxed_upright"
    )
    if source_schedule_id == "painleve_author_mu05":
        relation_valid = observed_classes == [
            "upright_near_rest",
            "upright_near_rest",
        ]
    else:
        relation_valid = (
            observed_classes == ["tumbled_near_rest", "upright_near_rest"]
            and observed_classes[0] != observed_classes[1]
        )
    if not relation_valid:
        raise ValueError(
            f"{label}: author Painleve exact/boxed outcome relation changed"
        )

    return {
        "schema_version": AUTHOR_PAINLEVE_COMPARISON_SCHEMA_VERSION,
        "claim_scope": AUTHOR_PAINLEVE_OUTCOME_CLAIM_SCOPE,
        "source_schedule_id": source_schedule_id,
        "group_id": group.id,
        "expected_relation": expected_relation,
        "observed_relation": expected_relation,
        "members": [
            {
                "schedule_id": schedule.id,
                "solver_lane": schedule.solver_lane,
                "outcome_class": observed_class,
                "horizontal_travel_m": travel_by_lane[schedule.solver_lane],
            }
            for schedule, observed_class in zip(schedules, observed_classes)
        ],
        "boxed_minus_exact_horizontal_travel_m": (
            travel_by_lane["boxed"] - travel_by_lane["exact"]
        ),
        "automated_current_dart_adapter_outcome_validated": True,
        "source_backend_equivalent": False,
        "trajectory_equivalent": False,
        "paper_figure_parity": False,
        "solver_superiority": False,
        "pass": True,
    }


def _validate_contract_vector(value: Any, size: int, *, label: str) -> None:
    if (
        not isinstance(value, list)
        or len(value) != size
        or not all(_is_finite_number(item) for item in value)
    ):
        raise ValueError(f"{label}: expected {size} finite numbers")


def _validate_contract_matrix(value: Any, *, label: str) -> None:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{label}: expected a 3x3 matrix")
    for row in value:
        _validate_contract_vector(row, 3, label=label)


def _validate_contract_pose(value: Any, *, label: str) -> None:
    if not isinstance(value, dict):
        raise ValueError(f"{label}: expected a pose object")
    _validate_contract_vector(value.get("translation"), 3, label=label)
    _validate_contract_matrix(value.get("rotation"), label=label)


def _identity_contract_pose() -> dict[str, list[Any]]:
    return {
        "translation": [0.0, 0.0, 0.0],
        "rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    }


def _literal_masonry_arch_geometry_fingerprint(contract: dict[str, Any]) -> str:
    value = 14695981039346656037

    def append_bytes(payload: bytes) -> None:
        nonlocal value
        for byte in payload:
            value ^= byte
            value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF

    def append_unsigned(item: int) -> None:
        if isinstance(item, bool) or not isinstance(item, int) or item < 0:
            raise ValueError("literal arch fingerprint expected an unsigned integer")
        append_bytes(item.to_bytes(8, byteorder="little", signed=False))

    def append_boolean(item: bool) -> None:
        if not isinstance(item, bool):
            raise ValueError("literal arch fingerprint expected a boolean")
        append_bytes(b"\x01" if item else b"\x00")

    def append_double(item: Any) -> None:
        if not _is_finite_number(item):
            raise ValueError("literal arch fingerprint expected a finite number")
        scaled = float(item) * 1e10
        quantized = (
            math.floor(scaled + 0.5) if scaled >= 0.0 else math.ceil(scaled - 0.5)
        )
        if not -(1 << 63) <= quantized < (1 << 63):
            raise ValueError("literal arch fingerprint number is out of range")
        append_bytes(quantized.to_bytes(8, byteorder="little", signed=True))

    def append_string(item: Any) -> None:
        if not isinstance(item, str):
            raise ValueError("literal arch fingerprint expected a string")
        payload = item.encode("utf-8")
        append_unsigned(len(payload))
        append_bytes(payload)

    def append_vector(item: Any) -> None:
        _validate_contract_vector(item, 3, label="literal arch fingerprint vector")
        for component in item:
            append_double(component)

    def append_matrix(item: Any) -> None:
        _validate_contract_matrix(item, label="literal arch fingerprint matrix")
        for row in item:
            append_vector(row)

    def append_pose(item: Any) -> None:
        _validate_contract_pose(item, label="literal arch fingerprint pose")
        append_vector(item["translation"])
        append_matrix(item["rotation"])

    append_string(
        "dart.fbf_literal_masonry_arch_physical_geometry/" "fnv1a64_q1e-10_le_v1"
    )
    append_unsigned(contract["world"]["skeleton_count"])
    ground = contract["ground"]
    append_boolean(ground["mobile"])
    append_boolean(ground["plane_shape_observed"])
    append_double(ground["friction"])
    append_double(ground["primary_friction"])
    append_double(ground["secondary_friction"])
    append_double(ground["restitution"])
    append_double(ground["primary_slip_compliance"])
    append_double(ground["secondary_slip_compliance"])
    append_vector(ground["first_friction_direction"])
    append_boolean(ground["default_friction_direction_frame"])
    append_vector(ground["local_center_of_mass"])
    append_pose(ground["body_pose"])
    append_vector(ground["linear_velocity"])
    append_vector(ground["angular_velocity"])
    append_unsigned(ground["collision_shape_count"])
    append_pose(ground["collision_shape_local_pose"])
    append_vector(ground["plane_normal"])
    append_double(ground["plane_offset"])
    stones = contract["stones"]
    append_unsigned(len(stones))
    for stone in stones:
        append_string(stone["name"])
        append_boolean(stone["mobile"])
        append_double(stone["friction"])
        append_double(stone["primary_friction"])
        append_double(stone["secondary_friction"])
        append_double(stone["restitution"])
        append_double(stone["primary_slip_compliance"])
        append_double(stone["secondary_slip_compliance"])
        append_vector(stone["first_friction_direction"])
        append_boolean(stone["default_friction_direction_frame"])
        append_double(stone["mass_kg"])
        append_vector(stone["local_center_of_mass"])
        append_matrix(stone["moment_kg_m2"])
        append_pose(stone["body_pose"])
        append_vector(stone["linear_velocity"])
        append_vector(stone["angular_velocity"])
        append_unsigned(stone["collision_shape_count"])
        append_pose(stone["collision_shape_local_pose"])
        vertices = stone["vertices"]
        append_unsigned(len(vertices))
        for vertex in vertices:
            append_vector(vertex)
        triangles = stone["triangles"]
        append_unsigned(len(triangles))
        for triangle in triangles:
            for vertex in triangle:
                append_unsigned(vertex)
    return f"{value:016x}"


def _validate_literal_masonry_arch_contract(
    schedule: CaptureSchedule,
    data: dict[str, Any],
    *,
    sidecar_path: Path,
) -> dict[str, Any] | None:
    if not _is_literal_masonry_arch_schedule(schedule):
        return None

    contract = data.get("physics_contract")
    if not isinstance(contract, dict):
        raise ValueError(f"{sidecar_path}: literal arch physics contract is missing")
    if (
        contract.get("schema_version")
        != ("dart.fbf_literal_masonry_arch_physics_contract/v1")
        or contract.get("kind") != "physics_control"
    ):
        raise ValueError(f"{sidecar_path}: unexpected literal arch contract schema")

    source_binding = contract.get("source_binding")
    if not isinstance(source_binding, dict):
        raise ValueError(f"{sidecar_path}: literal arch source binding is missing")
    expected_hashes = {
        "spec_sha256": _sha256(
            ROOT / "examples/demos/scenes/FbfLiteralMasonryArchSpec.hpp"
        ),
        "implementation_sha256": _sha256(
            ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp"
        ),
        "geometry_sha256": _sha256(ROOT / "dart/math/detail/MasonryArchGeometry.hpp"),
        "solver_options_sha256": _sha256(
            ROOT / "dart/constraint/ExactCoulombFbfConstraintSolver.hpp"
        ),
    }
    if source_binding != expected_hashes:
        raise ValueError(f"{sidecar_path}: literal arch source hashes changed")

    world = contract.get("world")
    if not isinstance(world, dict) or world.get("name") != (
        "fbf_literal_masonry_arch_25"
    ):
        raise ValueError(f"{sidecar_path}: literal arch world identity changed")
    if not _is_finite_number(world.get("time_step_seconds")) or not math.isclose(
        world["time_step_seconds"], 1.0 / 60.0, rel_tol=0.0, abs_tol=1e-15
    ):
        raise ValueError(f"{sidecar_path}: literal arch time step changed")
    gravity = world.get("gravity_m_s2")
    _validate_contract_vector(gravity, 3, label=f"{sidecar_path}: gravity")
    if any(
        not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-15)
        for actual, expected in zip(gravity, (0.0, 0.0, -9.81))
    ):
        raise ValueError(f"{sidecar_path}: literal arch gravity changed")
    if (
        world.get("simulation_threads") != schedule.threads
        or world.get("deactivation_enabled") is not False
        or world.get("skeleton_count") != 26
    ):
        raise ValueError(f"{sidecar_path}: literal arch world policy changed")

    declared = contract.get("declared_spec")
    expected_declared = {
        "density_kg_m3": 1000.0,
        "friction": 0.8,
        "barrier_gap_policy": "omit_source_offsets",
        "end_face_expansion_m": 1e-6,
        "downward_shift_m": 0.001001,
        "contact_erp": 0.0,
    }
    if declared != expected_declared:
        raise ValueError(f"{sidecar_path}: literal arch declared spec changed")

    collision = contract.get("collision")
    if not isinstance(collision, dict) or collision != {
        "detector": "native",
        "contact_manifold": "four_point_planar",
        "native_detector_observed": True,
        "max_contacts": 400,
        "max_contacts_per_pair": 8,
    }:
        raise ValueError(f"{sidecar_path}: literal arch collision contract changed")

    solver = contract.get("solver")
    expected_lane = "exact_fbf" if schedule.solver_lane == "exact" else "boxed_lcp"
    if not isinstance(solver, dict) or solver.get("lane") != expected_lane:
        raise ValueError(f"{sidecar_path}: literal arch solver lane changed")
    if solver.get("split_impulse_enabled") is not True:
        raise ValueError(f"{sidecar_path}: literal arch split impulse changed")
    exact_options = solver.get("exact_options")
    if schedule.solver_lane == "boxed":
        if (
            solver.get("colored_block_gauss_seidel_enabled") is not False
            or solver.get("participant_affinity_enabled") is not False
            or exact_options is not None
        ):
            raise ValueError(f"{sidecar_path}: boxed literal arch solver changed")
    else:
        if (
            solver.get("colored_block_gauss_seidel_enabled") is not True
            or solver.get("participant_affinity_enabled") is not True
        ):
            raise ValueError(f"{sidecar_path}: exact literal arch schedule changed")
        expected_options = {
            "fallback_to_boxed_lcp_enabled": False,
            "constraint_regularization_enabled": False,
            "matrix_free_operator_enabled": False,
            "contact_row_operator_enabled": True,
            "dense_contact_row_snapshot_enabled": False,
            "warm_start_enabled": True,
            "step_size_persistence_enabled": False,
            "step_size_recovery_growth_factor": 1.05,
            "warm_start_match_distance": 0.025,
            "diagonal_seed_enabled": False,
            "matrix_free_seed_enabled": False,
            "projected_gradient_retry_enabled": False,
            "dense_residual_polish_enabled": False,
            "max_outer_iterations": 5000,
            "accept_outer_max_iterations": True,
            "tolerance": 1e-6,
            "initial_step_size": None,
            "cap_initial_step_size_at_safe_bound": True,
            "step_size_scale": 35.0,
            "outer_relaxation": 1.1,
            "coupling_variation_tolerance": 0.9,
            "shrink_factor": 0.7,
            "max_step_shrink_iterations": 20,
            "adaptive_step_size_enabled": True,
            "spectral_iterations": 10,
            "inner_max_sweeps": 30,
            "inner_local_solver": "exact_metric_projection",
            "run_fixed_inner_sweeps": True,
            "accept_inner_max_iterations": True,
            "inner_local_iterations": 1,
            "inner_tolerance": 1e-10,
            "inner_local_tolerance": 1e-12,
            "inner_diagonal_regularization": 0.0,
            "projected_gradient_max_iterations": 400,
            "projected_gradient_tolerance": 1e-12,
            "dense_residual_polish_iterations": 8,
            "dense_residual_polish_line_search_iterations": 8,
            "dense_residual_polish_regularization": 1e-9,
            "max_residual_history_samples": 0,
            "max_residual_history_records": 0,
        }
        if exact_options != expected_options:
            raise ValueError(f"{sidecar_path}: exact literal arch options changed")

    cross_step_options = solver.get("cross_step_options")
    if schedule.solver_lane == "boxed":
        if cross_step_options is not None:
            raise ValueError(f"{sidecar_path}: boxed literal arch policy changed")
    elif cross_step_options != {
        "warm_start_match_mode": "either_body_local_feature",
        "warm_start_normal_cosine": 0.9,
        "strict_warm_start_match_distance": False,
        "warm_start_max_age": -1,
        "persistent_step_size_safe_bound_scale": 1.0,
        "minimum_step_size": None,
        "maximum_step_size": None,
        "warm_start_residual_threshold": None,
        "warm_start_step_size_cap": None,
        "persist_uncapped_step_size_on_warm_start_cap": False,
        "require_residual_improvement_for_unconverged_cache_save": False,
    }:
        raise ValueError(
            f"{sidecar_path}: exact literal arch cross-step policy changed"
        )

    process_state = contract.get("process_state")
    if (
        not isinstance(process_state, dict)
        or not _is_finite_number(process_state.get("observed_contact_erp"))
        or process_state["observed_contact_erp"] != 0.0
    ):
        raise ValueError(f"{sidecar_path}: literal arch scoped ERP is not zero")

    ground = contract.get("ground")
    if not isinstance(ground, dict):
        raise ValueError(f"{sidecar_path}: literal arch ground is missing")
    if (
        ground.get("mobile") is not False
        or ground.get("plane_shape_observed") is not True
        or ground.get("friction") != 0.8
        or ground.get("primary_friction") != 0.8
        or ground.get("secondary_friction") != 0.8
        or ground.get("restitution") != 0.0
        or ground.get("primary_slip_compliance") != -1.0
        or ground.get("secondary_slip_compliance") != -1.0
        or ground.get("first_friction_direction") != [0.0, 0.0, 0.0]
        or ground.get("default_friction_direction_frame") is not True
        or ground.get("local_center_of_mass") != [0.0, 0.0, 0.0]
        or ground.get("linear_velocity") != [0.0, 0.0, 0.0]
        or ground.get("angular_velocity") != [0.0, 0.0, 0.0]
        or ground.get("collision_shape_count") != 1
        or ground.get("plane_normal") != [0.0, 0.0, 1.0]
        or ground.get("plane_offset") != 0.0
    ):
        raise ValueError(f"{sidecar_path}: literal arch ground changed")
    _validate_contract_pose(
        ground.get("collision_shape_local_pose"), label=f"{sidecar_path}: ground"
    )
    _validate_contract_pose(ground.get("body_pose"), label=f"{sidecar_path}: ground")
    if ground["collision_shape_local_pose"] != _identity_contract_pose():
        raise ValueError(f"{sidecar_path}: literal arch ground transform changed")
    if ground["body_pose"] != _identity_contract_pose():
        raise ValueError(f"{sidecar_path}: literal arch ground body pose changed")

    stones = contract.get("stones")
    if not isinstance(stones, list) or len(stones) != 25:
        raise ValueError(f"{sidecar_path}: literal arch stone count changed")
    for index, stone in enumerate(stones):
        label = f"{sidecar_path}: stone {index}"
        if not isinstance(stone, dict) or stone.get("name") != (
            f"masonry_arch_stone_{index}"
        ):
            raise ValueError(f"{label}: identity changed")
        expected_mobile = index not in (0, 24)
        if (
            stone.get("mobile") is not expected_mobile
            or stone.get("friction") != 0.8
            or stone.get("primary_friction") != 0.8
            or stone.get("secondary_friction") != 0.8
            or stone.get("restitution") != 0.0
            or stone.get("primary_slip_compliance") != -1.0
            or stone.get("secondary_slip_compliance") != -1.0
            or stone.get("first_friction_direction") != [0.0, 0.0, 0.0]
            or stone.get("default_friction_direction_frame") is not True
            or not _is_finite_number(stone.get("mass_kg"))
            or stone["mass_kg"] <= 0.0
            or stone.get("local_center_of_mass") != [0.0, 0.0, 0.0]
            or stone.get("linear_velocity") != [0.0, 0.0, 0.0]
            or stone.get("angular_velocity") != [0.0, 0.0, 0.0]
            or stone.get("collision_shape_count") != 1
        ):
            raise ValueError(f"{label}: physical properties changed")
        _validate_contract_matrix(stone.get("moment_kg_m2"), label=label)
        _validate_contract_pose(stone.get("body_pose"), label=label)
        _validate_contract_pose(stone.get("collision_shape_local_pose"), label=label)
        if stone["collision_shape_local_pose"] != _identity_contract_pose():
            raise ValueError(f"{label}: collision transform changed")
        vertices = stone.get("vertices")
        if not isinstance(vertices, list) or len(vertices) != 8:
            raise ValueError(f"{label}: wedge vertices changed")
        for vertex in vertices:
            _validate_contract_vector(vertex, 3, label=label)
        triangles = stone.get("triangles")
        if (
            not isinstance(triangles, list)
            or len(triangles) != 12
            or any(
                not isinstance(triangle, list)
                or len(triangle) != 3
                or any(
                    isinstance(vertex, bool)
                    or not isinstance(vertex, int)
                    or vertex < 0
                    or vertex >= 8
                    for vertex in triangle
                )
                for triangle in triangles
            )
        ):
            raise ValueError(f"{label}: wedge triangles changed")
    reported_fingerprint = contract.get("physical_geometry_fingerprint")
    computed_fingerprint = _literal_masonry_arch_geometry_fingerprint(contract)
    if reported_fingerprint != {
        "algorithm": "fnv1a64_q1e-10_le_v1",
        "value": computed_fingerprint,
    }:
        raise ValueError(
            f"{sidecar_path}: literal arch physical geometry fingerprint changed"
        )
    if computed_fingerprint != LITERAL_MASONRY_ARCH_GEOMETRY_FINGERPRINT:
        raise ValueError(f"{sidecar_path}: literal arch physical geometry changed")
    return {
        "schema_version": contract["schema_version"],
        "source_binding": source_binding,
        "solver_lane": expected_lane,
        "stone_count": len(stones),
        "scoped_erp": process_state["observed_contact_erp"],
        "physical_geometry_fingerprint": computed_fingerprint,
        "pass": True,
    }


def _expected_author_masonry_arch_solver_contract(solver_lane: str) -> dict[str, Any]:
    if solver_lane not in SOLVER_LANES:
        raise ValueError(f"unsupported author arch solver lane {solver_lane!r}")
    exact = solver_lane == "exact"
    return {
        "lane": "exact_fbf" if exact else "boxed_lcp",
        "split_impulse_enabled": True,
        "colored_block_gauss_seidel_enabled": exact,
        "participant_affinity_enabled": exact,
        "exact_options": (
            {
                "fallback_to_boxed_lcp_enabled": False,
                "constraint_regularization_enabled": False,
                "matrix_free_operator_enabled": False,
                "contact_row_operator_enabled": True,
                "dense_contact_row_snapshot_enabled": False,
                "warm_start_enabled": True,
                "step_size_persistence_enabled": False,
                "step_size_recovery_growth_factor": 1.05,
                "warm_start_match_distance": 0.025,
                "diagonal_seed_enabled": False,
                "matrix_free_seed_enabled": False,
                "projected_gradient_retry_enabled": False,
                "dense_residual_polish_enabled": False,
                "max_outer_iterations": 5000,
                "accept_outer_max_iterations": True,
                "tolerance": 1e-6,
                "initial_step_size": None,
                "cap_initial_step_size_at_safe_bound": True,
                "step_size_scale": 35.0,
                "outer_relaxation": 1.1,
                "coupling_variation_tolerance": 0.9,
                "shrink_factor": 0.7,
                "max_step_shrink_iterations": 20,
                "adaptive_step_size_enabled": True,
                "spectral_iterations": 10,
                "inner_max_sweeps": 30,
                "inner_local_solver": "exact_metric_projection",
                "run_fixed_inner_sweeps": True,
                "accept_inner_max_iterations": True,
                "inner_local_iterations": 1,
                "inner_tolerance": 1e-10,
                "inner_local_tolerance": 1e-12,
                "inner_diagonal_regularization": 0.0,
                "projected_gradient_max_iterations": 400,
                "projected_gradient_tolerance": 1e-12,
                "dense_residual_polish_iterations": 8,
                "dense_residual_polish_line_search_iterations": 8,
                "dense_residual_polish_regularization": 1e-9,
                "max_residual_history_samples": 0,
                "max_residual_history_records": 0,
            }
            if exact
            else None
        ),
        "cross_step_options": (
            {
                "warm_start_match_mode": "either_body_local_feature",
                "warm_start_normal_cosine": 0.9,
                "strict_warm_start_match_distance": False,
                "warm_start_max_age": -1,
                "persistent_step_size_safe_bound_scale": 1.0,
                "minimum_step_size": None,
                "maximum_step_size": None,
                "warm_start_residual_threshold": None,
                "warm_start_step_size_cap": None,
                "persist_uncapped_step_size_on_warm_start_cap": False,
                "require_residual_improvement_for_unconverged_cache_save": False,
            }
            if exact
            else None
        ),
    }


def _expected_author_card_house_solver_contract(
    solver_lane: str, *, source_continuation: bool = False
) -> dict[str, Any]:
    if solver_lane not in SOLVER_LANES:
        raise ValueError(f"unsupported author card-house solver lane {solver_lane!r}")
    exact = solver_lane == "exact"
    contract = {
        "lane": "exact_fbf" if exact else "boxed_lcp",
        "split_impulse_enabled": False,
        "source_inner_initialization_requested": exact,
        "source_inner_initialization_active": exact,
        "colored_block_gauss_seidel_enabled": False,
        "participant_affinity_enabled": False,
        "exact_options": (
            {
                "fallback_to_boxed_lcp_enabled": False,
                "constraint_regularization_enabled": False,
                "matrix_free_operator_enabled": False,
                "contact_row_operator_enabled": True,
                "dense_contact_row_snapshot_enabled": False,
                "warm_start_enabled": True,
                "step_size_persistence_enabled": True,
                "step_size_recovery_growth_factor": 1.0 / 0.7,
                "warm_start_match_distance": 0.02,
                "diagonal_seed_enabled": True,
                "matrix_free_seed_enabled": True,
                "projected_gradient_retry_enabled": False,
                "dense_residual_polish_enabled": False,
                "max_outer_iterations": 200,
                "accept_outer_max_iterations": False,
                "tolerance": 1e-6,
                "initial_step_size": None,
                "cap_initial_step_size_at_safe_bound": True,
                "step_size_scale": 10.0,
                "outer_relaxation": 1.0,
                "project_after_correction": False,
                "restart_inner_from_current_outer_reaction": True,
                "project_inner_initial_reaction": False,
                "coupling_variation_tolerance": 0.9,
                "shrink_factor": 0.7,
                "max_step_shrink_iterations": 8,
                "adaptive_step_size_enabled": True,
                "spectral_iterations": 10,
                "inner_max_sweeps": 10,
                "inner_local_solver": "exact_metric_projection",
                "run_fixed_inner_sweeps": True,
                "accept_inner_max_iterations": True,
                "inner_local_iterations": 8,
                "inner_tolerance": 1e-6,
                "inner_local_tolerance": 1e-12,
                "inner_diagonal_regularization": 0.0,
                "projected_gradient_max_iterations": 400,
                "projected_gradient_tolerance": 1e-12,
                "dense_residual_polish_iterations": 8,
                "dense_residual_polish_line_search_iterations": 8,
                "dense_residual_polish_regularization": 1e-9,
                "max_residual_history_samples": 0,
                "max_residual_history_records": 0,
            }
            if exact
            else None
        ),
        "cross_step_options": (
            {
                "warm_start_match_mode": "either_body_local_feature",
                "warm_start_normal_cosine": 0.9,
                "strict_warm_start_match_distance": False,
                "warm_start_max_age": -1,
                "persistent_step_size_safe_bound_scale": 1.0,
                "minimum_step_size": None,
                "maximum_step_size": None,
                "warm_start_residual_threshold": None,
                "warm_start_step_size_cap": None,
                "persist_uncapped_step_size_on_warm_start_cap": False,
                "require_residual_improvement_for_unconverged_cache_save": False,
            }
            if exact
            else None
        ),
        "boxed_baseline": (
            None
            if exact
            else {
                "primary_solver": "DantzigBoxedLcpSolver",
                "secondary_solver": "PgsBoxedLcpSolver",
                "matrix_free_options": {
                    "enabled": False,
                    "min_rows": 193,
                    "max_iterations": 30,
                    "sor": 0.9,
                    "delta_tolerance": 1e-6,
                    "relative_delta_tolerance": 1e-3,
                    "epsilon_for_division": 1e-9,
                },
            }
        ),
    }
    if not source_continuation:
        return contract

    contract["source_continuation"] = {
        "policy": "source_continuation",
        "options_available": exact,
        "requested": exact,
        "last_active": False,
        "numeric_settings": {
            "residual_check_interval": 5,
            "plateau_patience": 5,
            "plateau_relative_tolerance": 0.01,
            "step_size_backtrack_limit": 8,
            "coupling_variation_skip_threshold": 1e-10,
        },
        "fixed_semantics": {
            "strict_convergence_comparison": "<",
            "iteration_zero_residual": "natural_map_unscaled",
            "sampled_termination_residual": "coulomb_rel_dimensionless",
            "plateau_metric": "natural",
            "small_change_armijo_action": "accept",
            "line_search_cap_action": "shrink_cap",
        },
    }
    if exact:
        contract["exact_options"]["max_residual_history_samples"] = 64
        contract["exact_options"]["max_residual_history_records"] = 4096
        contract["cross_step_options"] = {
            "warm_start_match_mode": "ordered_body_b_local_feature",
            "warm_start_normal_cosine": 0.9,
            "strict_warm_start_match_distance": True,
            "warm_start_max_age": 3,
            "persistent_step_size_safe_bound_scale": 10.0,
            "minimum_step_size": 1e-6,
            "maximum_step_size": 1e6,
            "warm_start_residual_threshold": 1e-4,
            "warm_start_step_size_cap": 1e4,
            "persist_uncapped_step_size_on_warm_start_cap": True,
            "require_residual_improvement_for_unconverged_cache_save": True,
        }
    return contract


def _validate_author_card_house_adapter_contract(
    schedule: CaptureSchedule,
    data: dict[str, Any],
    *,
    sidecar_path: Path,
) -> dict[str, Any] | None:
    if not _is_author_card_house_impact_schedule(schedule):
        return None
    selection = _author_card_house_dynamics_selection(schedule)

    contract = data.get("physics_contract")
    if not isinstance(contract, dict):
        raise ValueError(
            f"{sidecar_path}: author card-house adapter contract is missing"
        )
    if (
        contract.get("schema_version")
        != "dart.fbf_author_card_house_dynamics_adapter/v1"
        or contract.get("kind") != "source_configuration_dynamics_adapter"
    ):
        raise ValueError(f"{sidecar_path}: unexpected author card-house schema")

    source_binding = contract.get("source_binding")
    expected_binding = {
        "repository": "https://github.com/matthcsong/fbf-sca-2026",
        "commit": "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0",
        "tree": "ffcdafb61adeda2239c8366d054b548b50d26685e",
        "card_house_run_blob": "35f33651bc9674a259071ac723e47755504152db",
        "card_house_run_py_sha256": (
            "18c58c85eaad865aeef480b46e880a52088f266b79c90226f624637221ee36f8"
        ),
        "fbf_config_py_sha256": (
            "88f3f9ffd758eccce8496f7897192587a05907109e313c7a86bcf8f9de8cc248"
        ),
        "solver_fbf_py_sha256": (
            "8ec32aa20bf8d6c1173ed6c7f3735e2926fbb4b5059ee2236e26ad27eb22f941"
        ),
        "configuration_spec_sha256": _sha256(
            ROOT / "examples/demos/scenes/FbfAuthorCardHouseSpec.hpp"
        ),
        "demo_implementation_sha256": _sha256(
            ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp"
        ),
    }
    if source_binding != expected_binding:
        raise ValueError(f"{sidecar_path}: author card-house source hashes changed")

    if contract.get("source_defaults") != {
        "levels": 5,
        "frames": 800,
        "drop_frame": 400,
        "num_cubes": 4,
        "mu": 0.8,
        "cube_half_size_m": 0.4,
        "cube_density_kg_m3": 500.0,
        "drop_height_m": 1.0,
    }:
        raise ValueError(f"{sidecar_path}: author card-house source defaults changed")

    if contract.get("selected_source_invocation") != {
        "provenance": "source_supported_cli_parameterization",
        "historical_paper_invocation_known": False,
        "arguments": {
            "solvers": ["fbf"],
            "levels": selection["levels"],
            "frames": selection["frames"],
            "drop_frame": 400,
            "num_cubes": 4,
            "mu": 0.8,
            "cube_half_size_m": 0.4,
            "cube_density_kg_m3": 500.0,
            "drop_height_m": 1.0,
            "device": "cpu",
            "profile": True,
            "usd": True,
        },
    }:
        raise ValueError(
            f"{sidecar_path}: author card-house selected source invocation changed"
        )

    expected_source_contact: dict[str, Any] = {
        "friction": 0.8,
        "gap_m": 0.005,
        "shape_stiffness": 10000.0,
        "shape_damping": 1000.0,
    }
    if selection["source_contact_gap_values_represented"]:
        expected_source_contact = {
            "friction": 0.8,
            "dynamic_shape_contact": {
                "gap_m": 0.005,
                "shape_stiffness": 10000.0,
                "shape_damping": 1000.0,
            },
            "ground_contact": {
                "gap_m": 0.1,
                "shape_stiffness": 2500.0,
                "shape_damping": 100.0,
            },
        }
    expected_source_configuration = {
        "cards": {
            "count": selection["cards"],
            "leaning_count": selection["leaning_cards"],
            "bridge_count": selection["bridge_cards"],
            "lean_size_m": [0.04, 1.25, 2.5],
            "bridge_size_m": [2.5, 1.25, 0.04],
            "density_kg_m3": 200.0,
            "mass_kg": 25.0,
            "lean_from_vertical_degrees": 25.0,
            "bridge_angle_degrees": -1.0,
            "tent_half_gap_m": 0.55,
            "tent_width_m": 2.2,
            "tent_height_m": 2.41660616977186,
        },
        "cubes": {
            "count": 4,
            "edge_m": 0.8,
            "density_kg_m3": 500.0,
            "mass_kg": 256.0,
            "initial_height_m": selection["cube_initial_height_m"],
            "initially_kinematic": True,
            "initial_velocity_m_s": [0, 0, 0],
        },
        "contact": expected_source_contact,
        "schedule": {
            "display_time_step_seconds": 1.0 / 60.0,
            "substeps_per_frame": 4,
            "runtime_time_step_seconds": 1.0 / 240.0,
            "release_frame": 400,
            "release_substep": 1600,
            "total_frames": selection["frames"],
            "total_substeps": selection["frames"] * 4,
        },
        "solver": {
            "type": "fbf_exact_coulomb",
            "max_contacts": 4096,
            "max_outer": 200,
            "outer_tol": 1e-6,
            "residual_check_interval": 5,
            "inner_solver": "block_gs",
            "inner_gs_sweeps": 10,
            "inner_max_iter": 200,
            "inner_tol": 1e-6,
            "adaptive_gamma": True,
            "gamma_c": 5.0,
            "gamma_max": 1e6,
            "armijo_rho_high": 0.9,
            "armijo_shrink": 0.7,
            "armijo_max_backtracks": 8,
            "warm_start": True,
            "project_after_correction": False,
            "restart_inner_from_current_outer_reaction": True,
            "project_inner_initial_reaction": False,
            "baumgarte_erp": 0.0,
            "termination_residual": "coulomb_rel",
            "termination_tol": 1e-6,
        },
    }
    if contract.get("source_configuration") != expected_source_configuration:
        raise ValueError(
            f"{sidecar_path}: author card-house source configuration changed"
        )

    adapter = contract.get("dart_adapter")
    if not isinstance(adapter, dict) or adapter.get("scene_id") != schedule.scene:
        raise ValueError(f"{sidecar_path}: author card-house adapter identity changed")
    world = adapter.get("world")
    if not isinstance(world, dict):
        raise ValueError(f"{sidecar_path}: author card-house adapter world is missing")
    if (
        not _is_finite_number(world.get("time_step_seconds"))
        or not math.isclose(
            world["time_step_seconds"], 1.0 / 240.0, rel_tol=0.0, abs_tol=1e-15
        )
        or world.get("current_time_seconds") != 0.0
        or world.get("gravity_m_s2") != [0.0, 0.0, -9.81]
        or world.get("simulation_threads") != schedule.threads
        or world.get("deactivation_enabled") is not False
    ):
        raise ValueError(f"{sidecar_path}: author card-house world policy changed")
    expected_collision = {
        "detector": "native",
        "contact_manifold": "four_point_planar",
        "max_contacts": 4096,
        "max_contacts_per_pair": 4,
        "enable_contact": True,
        "allow_negative_penetration_depth_contacts": selection[
            "source_contact_gap_values_represented"
        ],
        "default_empty_body_node_filter": True,
    }
    if selection["source_contact_gap_values_represented"]:
        expected_collision.update(
            {
                "ground_contact_gap_m": selection["ground_contact_gap_m"],
                "dynamic_shape_contact_gap_m": selection["dynamic_shape_contact_gap_m"],
                "collision_shape_frames": selection["collision_shape_frames"],
                "collision_shape_frames_with_contact_gap": selection[
                    "contact_gap_shape_frames"
                ],
                "ground_shape_frames_with_contact_gap": selection[
                    "ground_contact_gap_shape_frames"
                ],
                "dynamic_shape_frames_with_contact_gap": selection[
                    "dynamic_contact_gap_shape_frames"
                ],
            }
        )
    if adapter.get("collision") != expected_collision:
        raise ValueError(
            f"{sidecar_path}: author card-house collision contract changed"
        )
    if adapter.get("solver") != _expected_author_card_house_solver_contract(
        schedule.solver_lane,
        source_continuation=_is_author_card_house_source_continuation_schedule(
            schedule
        ),
    ):
        raise ValueError(f"{sidecar_path}: author card-house solver contract changed")
    if adapter.get("contact_material") != {
        "primary_friction": 0.8,
        "secondary_friction": 0.8,
        "restitution": 0.0,
        "primary_slip_compliance": -1.0,
        "secondary_slip_compliance": -1.0,
        "first_friction_direction": [0.0, 0.0, 0.0],
        "uses_default_friction_direction_frame": True,
    }:
        raise ValueError(f"{sidecar_path}: author card-house contact material changed")
    if adapter.get("process_state") != {"observed_contact_erp": 0.0}:
        raise ValueError(f"{sidecar_path}: author card-house scoped ERP is not zero")
    if adapter.get("inventory") != {
        "cards": selection["cards"],
        "cubes": 4,
        "released_cubes": 0,
        "finite_state": True,
    }:
        raise ValueError(f"{sidecar_path}: author card-house inventory changed")
    if adapter.get("schedule") != {
        "evidence_total_substeps": selection["frames"] * 4,
        "evidence_runner_action_completed_step": 1600,
        "release_action_key": "p",
        "interactive_action_semantics": "immediate_on_invocation",
    }:
        raise ValueError(f"{sidecar_path}: author card-house release schedule changed")

    expected_adapter_boundaries = {
        "source_contact_gap_recorded_m": 0.005,
        "source_contact_gap_semantics_implemented": False,
        "source_shape_stiffness_semantics_implemented": False,
        "source_shape_damping_semantics_implemented": False,
        "source_collision_backend_implemented": False,
        "source_solver_backend_semantics_implemented": False,
        "source_float32_semantics_implemented": False,
        "dart_native_four_point_planar_is_adapter_choice": True,
    }
    if selection["source_contact_gap_values_represented"]:
        expected_adapter_boundaries.pop("source_contact_gap_recorded_m")
        expected_adapter_boundaries.update(
            {
                "source_dynamic_shape_contact_gap_recorded_m": 0.005,
                "source_ground_contact_gap_recorded_m": 0.1,
                "source_contact_gap_values_represented": True,
            }
        )
    if contract.get("adapter_boundaries") != expected_adapter_boundaries:
        raise ValueError(
            f"{sidecar_path}: author card-house adapter boundaries changed"
        )

    expected_claim_boundary = {
        "current_source_parameterized_configuration_port": True,
        "source_release_action_ported_to_dart": True,
        "source_release_schedule_declared_for_evidence_runner": True,
        "interactive_demo_auto_releases_at_source_step": False,
        "historical_paper_invocation_known": False,
        "trajectory_valid": False,
        "physical_outcome_valid": False,
        "trajectory_equivalence": False,
        "solver_equivalence": False,
        "physical_outcome_equivalence": False,
        "fig06_parity": False,
        "video06_parity": False,
        "timing_comparability": False,
        "paper_parity": False,
    }
    if selection["source_contact_gap_values_represented"]:
        expected_claim_boundary["historical_tables_6_7_invocation_known"] = False
    if contract.get("claim_boundary") != expected_claim_boundary:
        raise ValueError(f"{sidecar_path}: author card-house claim boundary changed")

    return {
        "schema_version": contract["schema_version"],
        "source_binding": source_binding,
        "solver_lane": (
            "exact_fbf" if schedule.solver_lane == "exact" else "boxed_lcp"
        ),
        "card_count": adapter["inventory"]["cards"],
        "cube_count": adapter["inventory"]["cubes"],
        "release_action_completed_step": adapter["schedule"][
            "evidence_runner_action_completed_step"
        ],
        "scoped_erp": adapter["process_state"]["observed_contact_erp"],
        "physical_outcome_validated": False,
        "pass": True,
    }


def _validate_author_painleve_adapter_contract(
    schedule: CaptureSchedule,
    data: dict[str, Any],
    *,
    sidecar_path: Path,
) -> dict[str, Any] | None:
    if not _is_author_painleve_schedule(schedule):
        return None

    contract = data.get("physics_contract")
    if not isinstance(contract, dict):
        raise ValueError(f"{sidecar_path}: author Painleve adapter contract is missing")
    if (
        contract.get("schema_version") != "dart.fbf_author_painleve_dynamics_adapter/v1"
        or contract.get("kind") != "source_configuration_dynamics_adapter"
    ):
        raise ValueError(f"{sidecar_path}: unexpected author Painleve schema")

    source_binding = contract.get("source_binding")
    expected_binding = {
        "repository": "https://github.com/matthcsong/fbf-sca-2026",
        "commit": "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0",
        "tree": "ffcdafb61adeda2239c8366d054b548b50d26685e",
        "painleve_run_blob": "afaa03613b0ad0a30290168d2fd64221fc3523b7",
        "painleve_run_py_sha256": (
            "818fa8f75c2c73e2dd08f0e0e9f9f5d58f63d8073dce38f874e2da24b2aa46e3"
        ),
        "configuration_spec_sha256": _sha256(
            ROOT / "examples/demos/scenes/FbfAuthorPainleveSpec.hpp"
        ),
        "exact_solver_options_sha256": _sha256(
            ROOT / "dart/constraint/ExactCoulombFbfConstraintSolver.hpp"
        ),
        "demo_implementation_sha256": _sha256(
            ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp"
        ),
    }
    if source_binding != expected_binding:
        raise ValueError(f"{sidecar_path}: author Painleve source hashes changed")

    mu = float(schedule.configuration_dict()["mu"])
    source_configuration = contract.get("source_configuration")
    if not _contract_value_matches(
        source_configuration,
        {
            "gravity_m_s2": 9.81,
            "box_width_m": 0.3,
            "box_height_m": 0.6,
            "box_depth_m": 1.2,
            "density_kg_m3": 200.0,
            "mass_kg": 43.2,
            "initial_velocity_m_s": 4.0,
            "critical_friction": 0.5,
            "time_step_seconds": 1.0 / 60.0,
            "duration_seconds": 2.0,
            "total_steps": 120,
            "source_default_mu_values": [0.55],
            "selected_source_supported_mu_sweep": [0.5, 0.55],
            "ground_half_extents_m": [5.0, 1.5, 0.05],
            "box_initial_pose": {
                "translation_m": [0.0, 0.0, 0.3],
                "rotation": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
            },
            "contact": {
                "friction": mu,
                "gap_m": 0.005,
                "shape_stiffness": 10000.0,
                "shape_damping": 1000.0,
            },
        },
    ):
        raise ValueError(
            f"{sidecar_path}: author Painleve source configuration changed"
        )

    adapter = contract.get("dart_adapter")
    if not isinstance(adapter, dict) or adapter.get("scene_id") != schedule.scene:
        raise ValueError(f"{sidecar_path}: author Painleve adapter identity changed")
    if not _contract_value_matches(
        adapter.get("world"),
        {
            "time_step_seconds": 1.0 / 60.0,
            "gravity_m_s2": [0.0, 0.0, -9.81],
            "simulation_threads": schedule.threads,
            "deactivation_enabled": False,
        },
    ):
        raise ValueError(f"{sidecar_path}: author Painleve world policy changed")
    if not _contract_value_matches(
        adapter.get("collision"),
        {
            "detector": "native",
            "contact_manifold": "four_point_planar",
            "max_contacts": 4,
            "max_contacts_per_pair": 4,
        },
    ):
        raise ValueError(f"{sidecar_path}: author Painleve collision contract changed")

    exact = schedule.solver_lane == "exact"
    expected_solver = {
        "lane": "exact_fbf" if exact else "boxed_lcp",
        "configuration_policy": "source_gamma_c_5_strict_dart_adapter",
        "split_impulse_enabled": False,
        "exact_options": (
            {
                "max_outer_iterations": 1000,
                "tolerance": 1e-6,
                "inner_max_sweeps": 120,
                "inner_local_iterations": 32,
                "step_size_scale": 10.0,
                "step_size_persistence_enabled": True,
                "persistent_step_size_safe_bound_scale": 10.0,
                "post_correction_projection_enabled": True,
                "fallback_to_boxed_lcp_enabled": True,
            }
            if exact
            else None
        ),
    }
    if not _contract_value_matches(adapter.get("solver"), expected_solver):
        raise ValueError(f"{sidecar_path}: author Painleve solver contract changed")

    identity = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
    if not _contract_value_matches(
        adapter.get("ground"),
        {
            "mobile": False,
            "size_m": [10.0, 3.0, 0.1],
            "initial_pose": {
                "translation": [0.0, 0.0, -0.05],
                "rotation": identity,
            },
            "friction": mu,
        },
    ):
        raise ValueError(f"{sidecar_path}: author Painleve ground changed")
    if not _contract_value_matches(
        adapter.get("box"),
        {
            "mobile": True,
            "size_m": [0.3, 1.2, 0.6],
            "initial_pose": {
                "translation": [0.0, 0.0, 0.3],
                "rotation": identity,
            },
            "initial_linear_velocity_m_s": [4.0, 0.0, 0.0],
            "initial_angular_velocity_rad_s": [0.0, 0.0, 0.0],
            "friction": mu,
            "mass_kg": 43.2,
            "moment_kg_m2": [
                [6.48, 0.0, 0.0],
                [0.0, 1.62, 0.0],
                [0.0, 0.0, 5.508],
            ],
        },
    ):
        raise ValueError(f"{sidecar_path}: author Painleve box changed")

    if contract.get("adapter_boundaries") != {
        "source_contact_gap_recorded_m": 0.005,
        "source_contact_gap_semantics_implemented": False,
        "source_shape_stiffness_semantics_implemented": False,
        "source_shape_damping_semantics_implemented": False,
        "source_collision_backend_implemented": False,
        "source_solver_backend_semantics_implemented": False,
        "source_float32_semantics_implemented": False,
        "dart_native_four_point_planar_is_adapter_choice": True,
    }:
        raise ValueError(f"{sidecar_path}: author Painleve adapter boundaries changed")
    if contract.get("claim_boundary") != {
        "current_source_parameterized_configuration_port": True,
        "historical_paper_invocation_known": False,
        "trajectory_valid": False,
        "physical_outcome_valid": False,
        "trajectory_equivalence": False,
        "solver_equivalence": False,
        "physical_outcome_equivalence": False,
        "fig05_parity": False,
        "video05_parity": False,
        "timing_comparability": False,
        "paper_parity": False,
    }:
        raise ValueError(f"{sidecar_path}: author Painleve claim boundary changed")

    return {
        "schema_version": contract["schema_version"],
        "source_binding": source_binding,
        "solver_lane": expected_solver["lane"],
        "friction": mu,
        "mass_kg": adapter["box"]["mass_kg"],
        "physical_outcome_validated": False,
        "pass": True,
    }


def _validate_author_masonry_arch_adapter_contract(
    schedule: CaptureSchedule,
    data: dict[str, Any],
    *,
    sidecar_path: Path,
) -> dict[str, Any] | None:
    source_schedule_id = _author_masonry_arch_source_schedule_id(schedule)
    if source_schedule_id is None:
        return None

    scenarios = {
        "masonry_arch_25_author_crown_impact_current_source": {
            "schema_version": (
                "dart.fbf_author_masonry_arch_crown_impact_dart_adapter/v1"
            ),
            "mesh_tree": "2552017df4061c49f7df064d847a4268a6e02d1a",
            "mesh_tree_sha256": (
                "a3f4e35073a2f4e74837fff277cd923f104b6af57f2cf995cf7524fe498e483d"
            ),
            "mesh_directory": "meshes/arch/num_stones=25",
            "selected_cli_arguments": "--stones 25",
            "stones": 25,
            "mobile_stones": 23,
            "source_obj_record_arch_top_z": 65.273375,
            "source_runtime_arch_top_z_float32": 65.27337646484375,
            "cube_initial_z": 75.27337646484375,
            "evidence_frames": 500,
            "evidence_substeps": 2000,
            "release_within_evidence_horizon": True,
            "release_action_scheduled": True,
            "release_action_completed_step": 1600,
            "release_action_key": "p",
            "interactive_action_semantics": "immediate_on_invocation",
        },
        "masonry_arch_101_author_standing_current_source": {
            "schema_version": ("dart.fbf_author_masonry_arch_standing_dart_adapter/v1"),
            "mesh_tree": "e0c209235673d2f69c3c5de7708ab1dfadec96e3",
            "mesh_tree_sha256": (
                "7198f71730d06dd70af8703065541765bd6b6f5da137f28f9befdf7acc5f96bf"
            ),
            "mesh_directory": "meshes/arch/num_stones=101",
            "selected_cli_arguments": "--stones 101",
            "stones": 101,
            "mobile_stones": 99,
            "source_obj_record_arch_top_z": 69.628159,
            "source_runtime_arch_top_z_float32": 69.62815856933594,
            "cube_initial_z": 79.62815856933594,
            "evidence_frames": 400,
            "evidence_substeps": 1600,
            "release_within_evidence_horizon": False,
            "release_action_scheduled": False,
            "release_action_completed_step": None,
            "release_action_key": None,
            "interactive_action_semantics": "not_registered_for_standing_lane",
        },
    }
    scenario = scenarios[source_schedule_id]

    contract = data.get("physics_contract")
    if not isinstance(contract, dict):
        raise ValueError(f"{sidecar_path}: author arch adapter contract is missing")
    if (
        contract.get("schema_version") != scenario["schema_version"]
        or contract.get("kind") != "source_configuration_dynamics_adapter"
    ):
        raise ValueError(f"{sidecar_path}: unexpected author arch adapter schema")

    source_binding = contract.get("source_binding")
    expected_binding = {
        "repository": "https://github.com/matthcsong/fbf-sca-2026",
        "commit": "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0",
        "tree": "ffcdafb61adeda2239c8366d054b548b50d26685",
        "run_py_blob": "35a052d7ef0975e7c828c9678d163054dfbb3ef2",
        "run_py_sha256": (
            "7e9158240267bb0ec1d0316b1badd4f3c8e1cd10270322de2e205cfea96f6f73"
        ),
        "mesh_tree": scenario["mesh_tree"],
        "mesh_tree_sha256": scenario["mesh_tree_sha256"],
        "mesh_directory": scenario["mesh_directory"],
        "configuration_spec_sha256": _sha256(
            ROOT / "examples/demos/scenes/FbfAuthorMasonryArchSpec.hpp"
        ),
        "dart_adapter_sha256": _sha256(
            ROOT / "examples/demos/scenes/FbfAuthorMasonryArchDartAdapter.hpp"
        ),
        "demo_implementation_sha256": _sha256(
            ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp"
        ),
    }
    if source_binding != expected_binding:
        raise ValueError(f"{sidecar_path}: author arch adapter source hashes changed")

    if contract.get("source_selection") != {
        "source_default_stones": 25,
        "selected_cli_arguments": scenario["selected_cli_arguments"],
        "selected_stones": scenario["stones"],
        "historical_paper_invocation_known": False,
    }:
        raise ValueError(f"{sidecar_path}: author arch source selection changed")

    expected_source_configuration = {
        "coordinate_scale": 1.0,
        "coordinate_units": "author_raw_numeric_values",
        "stones": scenario["stones"],
        "fixed_springers": 2,
        "cubes": 3,
        "cube_edge": 3.0,
        "cube_mass": 54000.0,
        "source_obj_record_arch_top_z": scenario["source_obj_record_arch_top_z"],
        "source_runtime_arch_top_z_float32": scenario[
            "source_runtime_arch_top_z_float32"
        ],
        "cube_initial_z": scenario["cube_initial_z"],
        "friction": 0.8,
        "contact_gap": 0.005,
        "shape_stiffness": 10000.0,
        "shape_damping": 1000.0,
        "display_time_step_seconds": 1.0 / 60.0,
        "substeps_per_frame": 4,
        "runtime_time_step_seconds": 1.0 / 240.0,
        "release_frame": 400,
        "release_substep": 1600,
        "evidence_frames": scenario["evidence_frames"],
        "evidence_substeps": scenario["evidence_substeps"],
    }
    if contract.get("source_configuration") != expected_source_configuration:
        raise ValueError(f"{sidecar_path}: author arch source configuration changed")

    adapter = contract.get("dart_adapter")
    if not isinstance(adapter, dict) or adapter.get("scene_id") != schedule.scene:
        raise ValueError(f"{sidecar_path}: author arch adapter identity changed")
    world = adapter.get("world")
    if not isinstance(world, dict):
        raise ValueError(f"{sidecar_path}: author arch adapter world is missing")
    time_step = world.get("time_step_seconds")
    if not _is_finite_number(time_step) or not math.isclose(
        time_step, 1.0 / 240.0, rel_tol=0.0, abs_tol=1e-15
    ):
        raise ValueError(f"{sidecar_path}: author arch adapter time step changed")
    if (
        world.get("gravity_coordinate_units_per_s2") != [0.0, 0.0, -9.81]
        or world.get("simulation_threads") != schedule.threads
        or world.get("deactivation_enabled") is not False
    ):
        raise ValueError(f"{sidecar_path}: author arch adapter world policy changed")

    if adapter.get("collision") != {
        "detector": "native",
        "contact_manifold": "four_point_planar",
        "observed_four_point_planar": True,
        "max_contacts": 4096,
        "max_contacts_per_pair": 8,
    }:
        raise ValueError(f"{sidecar_path}: author arch collision contract changed")

    solver = adapter.get("solver")
    expected_lane = "exact_fbf" if schedule.solver_lane == "exact" else "boxed_lcp"
    if solver != _expected_author_masonry_arch_solver_contract(schedule.solver_lane):
        raise ValueError(f"{sidecar_path}: author arch solver contract changed")
    if adapter.get("process_state") != {"observed_contact_erp": 0.0}:
        raise ValueError(f"{sidecar_path}: author arch scoped ERP is not zero")
    if adapter.get("inventory") != {
        "stones": scenario["stones"],
        "mobile_stones": scenario["mobile_stones"],
        "cubes": 3,
        "cubes_released": False,
    }:
        raise ValueError(f"{sidecar_path}: author arch initial inventory changed")
    if adapter.get("schedule") != {
        "evidence_frames": scenario["evidence_frames"],
        "evidence_substeps": scenario["evidence_substeps"],
        "source_release_frame": 400,
        "source_release_substep": 1600,
        "source_release_within_evidence_horizon": scenario[
            "release_within_evidence_horizon"
        ],
        "evidence_runner_release_action_scheduled": scenario[
            "release_action_scheduled"
        ],
        "evidence_runner_action_completed_step": scenario[
            "release_action_completed_step"
        ],
        "release_action_key": scenario["release_action_key"],
        "interactive_action_semantics": scenario["interactive_action_semantics"],
    }:
        raise ValueError(f"{sidecar_path}: author arch release schedule changed")

    expected_standing_oracle = None
    if scenario["stones"] == 101:
        expected_standing_oracle = {
            "scene_state_schema": (AUTHOR_MASONRY_ARCH_101_SCENE_STATE_SCHEMA_VERSION),
            "required_completed_substeps": (AUTHOR_MASONRY_ARCH_101_REQUIRED_SUBSTEPS),
            "required_world_time_seconds": (
                AUTHOR_MASONRY_ARCH_101_REQUIRED_WORLD_TIME_SECONDS
            ),
            "horizon_time_tolerance_seconds": (
                AUTHOR_MASONRY_ARCH_101_HORIZON_TIME_TOLERANCE_SECONDS
            ),
            "max_mobile_body_origin_displacement": (
                AUTHOR_MASONRY_ARCH_101_MAX_MOBILE_BODY_ORIGIN_DISPLACEMENT
            ),
            "max_mobile_rotation_delta_rad": (
                AUTHOR_MASONRY_ARCH_101_MAX_MOBILE_ROTATION_DELTA_RAD
            ),
            "max_crown_height_loss": AUTHOR_MASONRY_ARCH_101_MAX_CROWN_HEIGHT_LOSS,
            "max_kinematic_cube_pose_error": (
                AUTHOR_MASONRY_ARCH_101_MAX_KINEMATIC_CUBE_POSE_ERROR
            ),
            "requires_exact_inventory": True,
            "requires_all_bodies_finite": True,
            "requires_cubes_kinematic": True,
            "requires_cubes_at_pinned_poses": True,
            "positive_standing_qualification_requires_standing_in_both_lanes": True,
            "comparison_capture_validity_is_runner_level": True,
            "complete_trace_valid_is_not_positive_standing_qualification": True,
            "current_dart_adapter_outcome_only": True,
            "source_outcome_equivalence": False,
        }
    if not _contract_value_matches(
        adapter.get("standing_outcome_oracle"), expected_standing_oracle
    ):
        raise ValueError(f"{sidecar_path}: author arch standing oracle changed")

    expected_actions = [(1600, "p")] if scenario["release_action_scheduled"] else []
    if (
        schedule.total_steps != scenario["evidence_substeps"]
        or not math.isclose(
            schedule.time_step_seconds, 1.0 / 240.0, rel_tol=0.0, abs_tol=1e-15
        )
        or [(action.step, action.key) for action in schedule.actions]
        != expected_actions
    ):
        raise ValueError(f"{sidecar_path}: author arch evidence schedule changed")

    expected_claim_boundary = {
        "source_numeric_geometry_mass_friction_and_initial_state_ported_to_dart": True,
        "source_release_action_ported_to_dart": scenario["release_action_scheduled"],
        "source_release_schedule_declared_for_evidence_runner": scenario[
            "release_action_scheduled"
        ],
        "interactive_demo_auto_releases_at_source_step": False,
        "historical_paper_invocation_known": False,
        "current_dart_adapter_standing_outcome_oracle_declared": (
            scenario["stones"] == 101
        ),
        "source_collision_semantics_equivalent": False,
        "source_contact_gap_semantics_equivalent": False,
        "source_solver_backend_equivalent": False,
        "source_float32_semantics_equivalent": False,
        "trajectory_equivalent": False,
        "physical_outcome_equivalent": False,
        "fig07_parity": False,
        "fig08_parity": False,
        "video07_parity": False,
        "video08_parity": False,
        "timing_comparable": False,
        "paper_parity": False,
    }
    if contract.get("claim_boundary") != expected_claim_boundary:
        raise ValueError(f"{sidecar_path}: author arch claim boundary changed")
    return {
        "schema_version": contract["schema_version"],
        "source_binding": source_binding,
        "solver_lane": expected_lane,
        "stone_count": adapter["inventory"]["stones"],
        "cube_count": adapter["inventory"]["cubes"],
        "source_selection": contract["source_selection"],
        "evidence_substeps": adapter["schedule"]["evidence_substeps"],
        "release_action_scheduled": adapter["schedule"][
            "evidence_runner_release_action_scheduled"
        ],
        "release_action_completed_step": adapter["schedule"][
            "evidence_runner_action_completed_step"
        ],
        "scoped_erp": adapter["process_state"]["observed_contact_erp"],
        "physical_outcome_validated": False,
        "pass": True,
    }


def _validate_schedule_physics_contract(
    schedule: CaptureSchedule,
    data: dict[str, Any],
    *,
    sidecar_path: Path,
) -> dict[str, Any] | None:
    report = _validate_literal_masonry_arch_contract(
        schedule, data, sidecar_path=sidecar_path
    )
    if report is not None:
        return report
    report = _validate_author_painleve_adapter_contract(
        schedule, data, sidecar_path=sidecar_path
    )
    if report is not None:
        return report
    report = _validate_author_card_house_adapter_contract(
        schedule, data, sidecar_path=sidecar_path
    )
    if report is not None:
        return report
    return _validate_author_masonry_arch_adapter_contract(
        schedule, data, sidecar_path=sidecar_path
    )


def _validate_fail_fast_state(
    data: dict[str, Any],
    *,
    sidecar_path: Path,
    triggered: bool,
) -> dict[str, Any]:
    state = data.get("headless_exact_fbf_fail_fast")
    if not isinstance(state, dict):
        raise ValueError(f"{sidecar_path}: exact-FBF fail-fast state is missing")
    if state.get("enabled") is not True or state.get("triggered") is not triggered:
        raise ValueError(f"{sidecar_path}: invalid exact-FBF fail-fast state")
    tolerance = state.get("residual_tolerance")
    if (
        isinstance(tolerance, bool)
        or not isinstance(tolerance, (int, float))
        or not math.isfinite(tolerance)
        or not math.isclose(
            tolerance, EXACT_FBF_RESIDUAL_TOLERANCE, rel_tol=0.0, abs_tol=0.0
        )
    ):
        raise ValueError(f"{sidecar_path}: invalid exact-FBF residual tolerance")
    step = state.get("step")
    reason = state.get("reason")
    if triggered:
        if isinstance(step, bool) or not isinstance(step, int) or step < 0:
            raise ValueError(f"{sidecar_path}: invalid exact-FBF fail-fast step")
        if reason not in EXACT_FBF_FAIL_FAST_REASONS:
            raise ValueError(f"{sidecar_path}: invalid exact-FBF fail-fast reason")
    elif step is not None or reason is not None:
        raise ValueError(f"{sidecar_path}: successful fail-fast state is inconsistent")
    return state


def _validate_source_continuation_state(
    data: dict[str, Any], *, sidecar_path: Path, triggered: bool
) -> dict[str, Any]:
    state = data.get("headless_exact_fbf_source_continuation")
    if not isinstance(state, dict):
        raise ValueError(f"{sidecar_path}: source-continuation gate state is missing")
    if (
        state.get("enabled") is not True
        or state.get("requested") != "source_continuation"
        or state.get("allowed_outcomes")
        != ["success", "plateau_accepted", "max_iterations_accepted"]
        or state.get("line_search_cap_action") != "shrink_cap"
        or state.get("triggered") is not triggered
    ):
        raise ValueError(f"{sidecar_path}: invalid source-continuation gate state")
    step = state.get("step")
    reason = state.get("reason")
    if triggered:
        if isinstance(step, bool) or not isinstance(step, int) or step < 0:
            raise ValueError(f"{sidecar_path}: invalid source-continuation gate step")
        if reason not in SOURCE_CONTINUATION_GATE_REASONS:
            raise ValueError(f"{sidecar_path}: invalid source-continuation gate reason")
    elif step is not None or reason is not None:
        raise ValueError(
            f"{sidecar_path}: successful source-continuation state is inconsistent"
        )
    return state


def _evaluate_source_continuation_gate(
    diagnostics: dict[str, Any],
) -> tuple[str | None, dict[str, Any] | None]:
    """Mirror the C++ source-continuation gate in stable priority order."""

    continuation_value = diagnostics.get("source_continuation")
    continuation = continuation_value if isinstance(continuation_value, dict) else None
    if continuation is None or continuation.get("requested") is not True:
        return "source_continuation_not_requested", continuation

    def nonnegative_integer(mapping: dict[str, Any], name: str) -> int:
        value = mapping.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"invalid source-continuation {name}")
        return value

    def integer(mapping: dict[str, Any], name: str) -> int:
        value = mapping.get(name)
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"invalid source-continuation {name}")
        return value

    fallbacks = nonnegative_integer(diagnostics, "boxed_lcp_fallbacks")
    failures = nonnegative_integer(diagnostics, "exact_failures")
    attempts = nonnegative_integer(diagnostics, "exact_attempts")
    solves = nonnegative_integer(diagnostics, "exact_solves")
    if fallbacks > 0:
        return "boxed_fallback", continuation
    if failures > 0:
        return "exact_failure", continuation
    if attempts != solves + failures:
        return "cumulative_accounting_mismatch", continuation
    if continuation.get("world_state_finite") is not True:
        return "nonfinite_world_state", continuation
    if continuation.get("group_history_truncated") is not False:
        return "group_history_truncated", continuation

    step = continuation.get("step")
    groups = continuation.get("group_outcomes")
    if not isinstance(step, dict) or not isinstance(groups, list):
        raise ValueError("invalid source-continuation step/group telemetry")
    step_attempts = nonnegative_integer(step, "exact_attempts")
    step_solves = nonnegative_integer(step, "exact_solves")
    if step_attempts != step_solves or len(groups) != step_attempts:
        return "group_accounting_mismatch", continuation

    plateaus = 0
    max_iterations = 0
    shrinks = 0
    shrink_caps = 0
    prior_solve_index: int | None = None
    for group in groups:
        if not isinstance(group, dict):
            raise ValueError("invalid source-continuation group")
        solve_index = nonnegative_integer(group, "solve_index")
        contact_count = nonnegative_integer(group, "contact_count")
        iterations = integer(group, "iterations")
        group_shrinks = integer(group, "line_search_shrinks")
        group_caps = integer(group, "line_search_shrink_cap_count")
        if contact_count == 0 or (
            prior_solve_index is not None and solve_index != prior_solve_index + 1
        ):
            return "invalid_group_telemetry", continuation
        prior_solve_index = solve_index
        if group.get("source_continuation_active") is not True:
            return "source_continuation_inactive", continuation
        if iterations < 0 or group_shrinks < 0 or group_caps < 0:
            return "invalid_group_telemetry", continuation
        if (
            group_caps > group_shrinks
            or (group_shrinks > 0 and iterations == 0)
            or group_shrinks > 8 * iterations
            or group_caps > iterations
        ):
            return "invalid_group_telemetry", continuation

        residual = group.get("final_residual")
        natural = group.get("final_natural_map_residual")
        if (
            not _is_finite_number(residual)
            or residual < 0.0
            or not _is_finite_number(natural)
            or natural < 0.0
        ):
            return "nonfinite_group_residual", continuation
        correction_gamma = group.get("correction_step_size")
        inner_gamma = group.get("last_inner_solve_step_size")
        if iterations > 0 and (
            not _is_finite_number(correction_gamma)
            or correction_gamma <= 0.0
            or not _is_finite_number(inner_gamma)
            or inner_gamma <= 0.0
        ):
            return "nonfinite_group_step_size", continuation
        if iterations > 0 and (
            (group_caps == 0 and correction_gamma != inner_gamma)
            or (group_caps > 0 and correction_gamma > inner_gamma)
        ):
            return "group_step_size_relation_mismatch", continuation

        status_pair = (group.get("status"), group.get("fbf_status"))
        is_plateau = False
        if status_pair == ("success", "success"):
            if iterations > 200 or (iterations > 0 and iterations % 5 != 0):
                return "termination_timing_mismatch", continuation
            metric = natural if iterations == 0 else residual
            if metric >= EXACT_FBF_RESIDUAL_TOLERANCE:
                return "success_tolerance_not_strict", continuation
        elif status_pair == ("plateau_accepted", "plateau"):
            is_plateau = True
            if iterations < 30 or iterations > 200 or iterations % 5 != 0:
                return "termination_timing_mismatch", continuation
            reference = group.get("plateau_reference_natural_map_residual")
            improvement = group.get("plateau_relative_improvement")
            if (
                not _is_finite_number(reference)
                or reference <= 0.0
                or not _is_finite_number(improvement)
            ):
                return "plateau_telemetry_mismatch", continuation
            expected_improvement = (reference - natural) / reference
            scale = max(1.0, abs(expected_improvement))
            if (
                abs(improvement - expected_improvement)
                > 8.0 * sys.float_info.epsilon * scale
                or not improvement < 0.01
            ):
                return "plateau_telemetry_mismatch", continuation
            plateaus += 1
        elif status_pair == ("max_iterations_accepted", "max_iterations"):
            if iterations != 200:
                return "termination_timing_mismatch", continuation
            max_iterations += 1
        else:
            return "unaccepted_group_outcome", continuation
        if not is_plateau and (
            group.get("plateau_reference_natural_map_residual") is not None
            or group.get("plateau_relative_improvement") is not None
        ):
            return "plateau_telemetry_mismatch", continuation
        shrinks += group_shrinks
        shrink_caps += group_caps

    if (
        plateaus != nonnegative_integer(step, "plateaus_accepted")
        or max_iterations != nonnegative_integer(step, "max_iterations_accepted")
        or shrinks != nonnegative_integer(step, "line_search_shrinks")
        or shrink_caps != nonnegative_integer(step, "line_search_shrink_caps")
    ):
        return "group_counter_mismatch", continuation
    if groups:
        last = groups[-1]
        last_attempt = continuation.get("last_attempt")
        if not isinstance(last_attempt, dict):
            raise ValueError("invalid source-continuation last-attempt telemetry")
        cap_count = last["line_search_shrink_cap_count"]
        if (
            continuation.get("last_active") != last.get("source_continuation_active")
            or last_attempt.get("line_search_shrink_cap_count") != cap_count
            or last_attempt.get("line_search_shrink_cap_reached") != (cap_count > 0)
            or last_attempt.get("correction_step_size")
            != last.get("correction_step_size")
            or last_attempt.get("last_inner_solve_step_size")
            != last.get("last_inner_solve_step_size")
        ):
            return "last_group_telemetry_mismatch", continuation
    return None, continuation


def _expected_fail_fast_reason(diagnostics: dict[str, Any]) -> str | None:
    counter_reasons = (
        ("boxed_lcp_fallbacks", "boxed_fallback"),
        ("exact_failures", "exact_failure"),
        ("accepted_at_cap", "iteration_cap"),
    )
    for counter_name, reason in counter_reasons:
        counter = diagnostics.get(counter_name)
        if isinstance(counter, bool) or not isinstance(counter, int) or counter < 0:
            raise ValueError(f"invalid {counter_name} fail-fast diagnostic")
        if counter > 0:
            return reason

    attempts = diagnostics.get("exact_attempts")
    if isinstance(attempts, bool) or not isinstance(attempts, int) or attempts < 0:
        raise ValueError("invalid exact_attempts fail-fast diagnostic")
    residuals = []
    for name in ("residual", "worst_residual"):
        if name not in diagnostics:
            raise ValueError(f"missing {name} fail-fast diagnostic")
        value = diagnostics[name]
        if value is not None and (
            isinstance(value, bool) or not isinstance(value, (int, float))
        ):
            raise ValueError(f"invalid {name} fail-fast diagnostic")
        residuals.append(value)
    if attempts == 0:
        return None
    if any(value is None or not math.isfinite(value) for value in residuals):
        return "nonfinite_residual"
    if any(
        value is not None
        and math.isfinite(value)
        and value > EXACT_FBF_RESIDUAL_TOLERANCE
        for value in residuals
    ):
        return "residual_tolerance_exceeded"
    return None


def _validate_failed_exact_fbf_sidecar(
    schedule: CaptureSchedule,
    output_dir: Path,
    *,
    expected_demo: Path,
) -> dict[str, Any]:
    sidecar_path = output_dir / "timeline.json"
    data = _read_json(sidecar_path)
    if data.get("schema_version") != SIDECAR_SCHEMA_VERSION:
        raise ValueError(f"{sidecar_path}: unexpected schema")
    if (
        data.get("scene") != schedule.scene
        or data.get("active_scene") != schedule.scene
    ):
        raise ValueError(f"{sidecar_path}: requested/active scene does not match")
    if data.get("total_steps") != schedule.total_steps:
        raise ValueError(f"{sidecar_path}: total step mismatch")
    if data.get("event_order") != "captures_before_actions_at_each_completed_step":
        raise ValueError(f"{sidecar_path}: unsupported event ordering")
    if (data.get("width"), data.get("height")) != (
        schedule.width,
        schedule.height,
    ):
        raise ValueError(f"{sidecar_path}: capture dimensions do not match")
    if data.get("collision_detector") != schedule.collision_detector:
        raise ValueError(
            f"{sidecar_path}: collision detector differs from the schedule"
        )
    try:
        runtime_argv = shlex.split(data["runtime_command"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{sidecar_path}: invalid runtime command") from error
    expected_runtime_argv = build_demo_command(schedule, expected_demo, output_dir)
    if runtime_argv != expected_runtime_argv:
        raise ValueError(f"{sidecar_path}: runtime command differs from the schedule")
    physics_contract_report = _validate_schedule_physics_contract(
        schedule, data, sidecar_path=sidecar_path
    )

    state = _validate_fail_fast_state(data, sidecar_path=sidecar_path, triggered=True)
    trigger_step = state["step"]
    completed_steps = data.get("completed_steps")
    if (
        isinstance(completed_steps, bool)
        or not isinstance(completed_steps, int)
        or completed_steps != trigger_step
        or trigger_step > schedule.total_steps
    ):
        raise ValueError(
            f"{sidecar_path}: fail-fast step and completed steps are inconsistent"
        )
    trajectory_steps = data.get("steps")
    if not isinstance(trajectory_steps, list) or any(
        not isinstance(item, dict) for item in trajectory_steps
    ):
        raise ValueError(f"{sidecar_path}: completed-step diagnostics are invalid")
    if any(
        isinstance(item.get("step"), bool) or not isinstance(item.get("step"), int)
        for item in trajectory_steps
    ) or [item.get("step") for item in trajectory_steps] != list(
        range(trigger_step + 1)
    ):
        raise ValueError(
            f"{sidecar_path}: partial completed-step diagnostics are missing or "
            "out of order"
        )
    counter_names = (
        "exact_attempts",
        "exact_solves",
        "accepted_at_cap",
        "exact_failures",
        "boxed_lcp_fallbacks",
    )
    prior_counters = {name: 0 for name in counter_names}
    diagnostics_by_step: dict[int, dict[str, Any]] = {}
    reason_by_step: dict[int, str | None] = {}
    for item in trajectory_steps:
        step = item["step"]
        sim_time = item.get("sim_time")
        if (
            isinstance(sim_time, bool)
            or not isinstance(sim_time, (int, float))
            or not math.isclose(
                sim_time, schedule.time_at_step(step), rel_tol=0.0, abs_tol=1e-9
            )
        ):
            raise ValueError(
                f"{sidecar_path}: completed-step time mismatch at step {step}"
            )
        diagnostics = item.get("solver_diagnostics")
        if not isinstance(diagnostics, dict):
            raise ValueError(
                f"{sidecar_path}: fail-fast diagnostics at step {step} are invalid"
            )
        try:
            reason = _expected_fail_fast_reason(diagnostics)
        except ValueError as error:
            raise ValueError(f"{sidecar_path}: step {step}: {error}") from error
        if step < trigger_step and reason is not None:
            raise ValueError(
                f"{sidecar_path}: exact-FBF fail-fast should have triggered "
                f"earlier at step {step}: {reason}"
            )
        current_counters: dict[str, int] = {}
        for name in counter_names:
            value = diagnostics.get(name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(
                    f"{sidecar_path}: step {step}: invalid cumulative {name}"
                )
            if value < prior_counters[name]:
                raise ValueError(
                    f"{sidecar_path}: step {step}: cumulative {name} regressed"
                )
            current_counters[name] = value
        _validate_last_failure_diagnostics(
            diagnostics, label=f"{sidecar_path}: step {step}"
        )
        prior_counters = current_counters
        diagnostics_by_step[step] = diagnostics
        reason_by_step[step] = reason

    scene_state_metrics = _validate_author_painleve_scene_state_trace(
        schedule,
        trajectory_steps,
        sidecar_path=sidecar_path,
        expected_last_step=trigger_step,
    )
    author_arch_scene_state_metrics = (
        _validate_author_masonry_arch_101_scene_state_trace(
            schedule,
            trajectory_steps,
            sidecar_path=sidecar_path,
            expected_last_step=trigger_step,
        )
    )

    for collection_name in ("shots", "actions", "events"):
        collection = data.get(collection_name)
        if not isinstance(collection, list) or any(
            not isinstance(item, dict) for item in collection
        ):
            raise ValueError(f"{sidecar_path}: partial {collection_name} are invalid")
        if any(
            isinstance(item.get("step"), bool)
            or not isinstance(item.get("step"), int)
            or item["step"] >= trigger_step
            for item in collection
        ):
            raise ValueError(
                f"{sidecar_path}: {collection_name} exist at or after the "
                "fail-fast step"
            )

    expected_shots: list[dict[str, Any]] = []
    expected_actions: list[dict[str, Any]] = []
    expected_events: list[dict[str, Any]] = []
    sequence = 0
    for step in range(trigger_step):
        if step in schedule.capture_steps:
            shot = {
                "sequence": sequence,
                "step": step,
                "path": str(_frame_path(output_dir, step)),
            }
            expected_shots.append(shot)
            expected_events.append({**shot, "type": "shot"})
            sequence += 1
        for action in schedule.actions:
            if action.step != step:
                continue
            scheduled_action = {
                "sequence": sequence,
                "step": step,
                "key": action.key,
            }
            expected_actions.append(scheduled_action)
            expected_events.append({**scheduled_action, "type": "action"})
            sequence += 1

    shots = data["shots"]
    if len(shots) != len(expected_shots):
        raise ValueError(
            f"{sidecar_path}: shots do not match the required pre-trigger prefix"
        )
    for actual, expected in zip(shots, expected_shots):
        actual_path = actual.get("path")
        if (
            isinstance(actual.get("sequence"), bool)
            or not isinstance(actual.get("sequence"), int)
            or actual.get("sequence") != expected["sequence"]
            or isinstance(actual.get("step"), bool)
            or not isinstance(actual.get("step"), int)
            or actual.get("step") != expected["step"]
            or not isinstance(actual_path, str)
            or Path(actual_path) != Path(expected["path"])
            or actual.get("success") is not True
        ):
            raise ValueError(f"{sidecar_path}: shot prefix differs from the schedule")
        sim_time = actual.get("sim_time")
        if (
            isinstance(sim_time, bool)
            or not isinstance(sim_time, (int, float))
            or not math.isclose(
                sim_time,
                schedule.time_at_step(expected["step"]),
                rel_tol=0.0,
                abs_tol=1e-9,
            )
        ):
            raise ValueError(f"{sidecar_path}: shot prefix time mismatch")
        if actual.get("solver_diagnostics") != diagnostics_by_step[expected["step"]]:
            raise ValueError(f"{sidecar_path}: shot prefix diagnostics are not bound")

    actions = data["actions"]
    if len(actions) != len(expected_actions):
        raise ValueError(
            f"{sidecar_path}: actions do not match the required pre-trigger prefix"
        )
    for actual, expected in zip(actions, expected_actions):
        if (
            isinstance(actual.get("sequence"), bool)
            or not isinstance(actual.get("sequence"), int)
            or actual.get("sequence") != expected["sequence"]
            or isinstance(actual.get("step"), bool)
            or not isinstance(actual.get("step"), int)
            or actual.get("step") != expected["step"]
            or actual.get("key") != expected["key"]
            or actual.get("success") is not True
        ):
            raise ValueError(f"{sidecar_path}: action prefix differs from the schedule")

    events = data["events"]
    if len(events) != len(expected_events):
        raise ValueError(
            f"{sidecar_path}: events do not match the required pre-trigger prefix"
        )
    for actual, expected in zip(events, expected_events):
        if actual.get("success") is not True or any(
            isinstance(actual.get(name), bool)
            or not isinstance(actual.get(name), int)
            or actual.get(name) != expected[name]
            for name in ("sequence", "step")
        ):
            raise ValueError(f"{sidecar_path}: event prefix differs from the schedule")
        for name in ("type", "path" if expected["type"] == "shot" else "key"):
            if actual.get(name) != expected[name]:
                raise ValueError(
                    f"{sidecar_path}: event prefix differs from the schedule"
                )

    offending_diagnostics = diagnostics_by_step[trigger_step]
    if data.get("solver_diagnostics") != offending_diagnostics:
        raise ValueError(
            f"{sidecar_path}: final diagnostics differ from the fail-fast step"
        )
    expected_reason = reason_by_step[trigger_step]
    if state["reason"] != expected_reason:
        raise ValueError(
            f"{sidecar_path}: fail-fast reason does not match diagnostics priority"
        )
    return {
        "sidecar": str(sidecar_path),
        "completed_steps": completed_steps,
        "reason": state["reason"],
        "headless_exact_fbf_fail_fast": state,
        "physics_contract": physics_contract_report,
        **(
            {"scene_state_metrics": scene_state_metrics}
            if scene_state_metrics is not None
            else {}
        ),
        **(
            {
                "author_masonry_arch_101_scene_state_metrics": (
                    author_arch_scene_state_metrics
                )
            }
            if author_arch_scene_state_metrics is not None
            else {}
        ),
    }


def _validate_failed_source_continuation_sidecar(
    schedule: CaptureSchedule,
    output_dir: Path,
    *,
    expected_demo: Path,
) -> dict[str, Any]:
    sidecar_path = output_dir / "timeline.json"
    data = _read_json(sidecar_path)
    if data.get("schema_version") != SIDECAR_SCHEMA_VERSION:
        raise ValueError(f"{sidecar_path}: unexpected schema")
    if (
        data.get("scene") != schedule.scene
        or data.get("active_scene") != schedule.scene
    ):
        raise ValueError(f"{sidecar_path}: failed continuation identity differs")
    if data.get("total_steps") != schedule.total_steps:
        raise ValueError(f"{sidecar_path}: total step mismatch")
    if data.get("event_order") != "captures_before_actions_at_each_completed_step":
        raise ValueError(f"{sidecar_path}: unsupported event ordering")
    if (data.get("width"), data.get("height")) != (
        schedule.width,
        schedule.height,
    ):
        raise ValueError(f"{sidecar_path}: capture dimensions do not match")
    if data.get("collision_detector") != schedule.collision_detector:
        raise ValueError(
            f"{sidecar_path}: collision detector differs from the schedule"
        )
    if "headless_exact_fbf_fail_fast" in data:
        raise ValueError(f"{sidecar_path}: strict fail-fast state is unexpected")
    try:
        runtime_argv = shlex.split(data["runtime_command"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{sidecar_path}: invalid runtime command") from error
    if runtime_argv != build_demo_command(schedule, expected_demo, output_dir):
        raise ValueError(f"{sidecar_path}: runtime command differs from the schedule")
    physics_contract_report = _validate_schedule_physics_contract(
        schedule, data, sidecar_path=sidecar_path
    )
    state = _validate_source_continuation_state(
        data, sidecar_path=sidecar_path, triggered=True
    )
    trigger_step = state["step"]
    completed_steps = data.get("completed_steps")
    if (
        isinstance(completed_steps, bool)
        or not isinstance(completed_steps, int)
        or completed_steps != trigger_step
        or trigger_step > schedule.total_steps
    ):
        raise ValueError(
            f"{sidecar_path}: continuation gate step and completed steps differ"
        )
    trajectory_steps = data.get("steps")
    if not isinstance(trajectory_steps, list) or any(
        not isinstance(item, dict) for item in trajectory_steps
    ):
        raise ValueError(f"{sidecar_path}: failed trajectory records are invalid")
    if any(
        isinstance(item.get("step"), bool) or not isinstance(item.get("step"), int)
        for item in trajectory_steps
    ) or [item.get("step") for item in trajectory_steps] != list(
        range(trigger_step + 1)
    ):
        raise ValueError(f"{sidecar_path}: failed trajectory prefix is invalid")

    diagnostics_by_step: dict[int, dict[str, Any]] = {}
    continuation_trajectory_state = _new_source_continuation_trajectory_state()
    for item in trajectory_steps:
        step = item["step"]
        sim_time = item.get("sim_time")
        if not _is_finite_number(sim_time) or not math.isclose(
            sim_time, schedule.time_at_step(step), rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(f"{sidecar_path}: failed step time differs")
        diagnostics = item.get("solver_diagnostics")
        if not isinstance(diagnostics, dict):
            raise ValueError(f"{sidecar_path}: failed diagnostics are missing")
        diagnostics_by_step[step] = diagnostics
        if step < trigger_step:
            continuation = _validate_source_continuation_diagnostics(
                diagnostics, label=f"trajectory step {step}"
            )
            _validate_source_continuation_trajectory_step(
                diagnostics,
                continuation,
                step_index=step,
                state=continuation_trajectory_state,
                label=f"trajectory step {step}",
                require_accepted_outcome=True,
            )
    offending = diagnostics_by_step[trigger_step]
    _validate_diagnostics(
        offending,
        exact_required=False,
        solver_lane="exact",
        label=f"trajectory step {trigger_step}",
    )
    if offending.get("available") is not True:
        raise ValueError(
            f"{sidecar_path}: trigger exact-FBF diagnostics are unavailable"
        )
    _validate_last_failure_diagnostics(
        offending, label=f"trajectory step {trigger_step}"
    )
    expected_reason, offending_continuation = _evaluate_source_continuation_gate(
        offending
    )
    if expected_reason is None:
        raise ValueError(
            f"{sidecar_path}: continuation gate triggered on valid diagnostics"
        )
    if state["reason"] != expected_reason:
        raise ValueError(
            f"{sidecar_path}: source-continuation reason does not match "
            "diagnostics priority"
        )
    if offending_continuation is not None and all(
        isinstance(offending_continuation.get(name), expected_type)
        for name, expected_type in (
            ("cumulative", dict),
            ("step", dict),
            ("group_outcomes", list),
        )
    ):
        _validate_source_continuation_trajectory_step(
            offending,
            offending_continuation,
            step_index=trigger_step,
            state=continuation_trajectory_state,
            label=f"trajectory step {trigger_step}",
            require_accepted_outcome=False,
        )

    for collection_name in ("shots", "actions", "events"):
        collection = data.get(collection_name)
        if not isinstance(collection, list) or any(
            not isinstance(item, dict) for item in collection
        ):
            raise ValueError(f"{sidecar_path}: failed {collection_name} are invalid")
        if any(
            isinstance(item.get("step"), bool)
            or not isinstance(item.get("step"), int)
            or item["step"] >= trigger_step
            for item in collection
        ):
            raise ValueError(
                f"{sidecar_path}: {collection_name} exist at or after the "
                "continuation gate step"
            )

    expected_events: list[dict[str, Any]] = []
    expected_shots: list[dict[str, Any]] = []
    expected_actions: list[dict[str, Any]] = []
    sequence = 0
    for step in range(trigger_step):
        for shot_step in schedule.capture_steps:
            if shot_step == step:
                expected_shots.append(
                    {
                        "sequence": sequence,
                        "step": step,
                        "path": str(_frame_path(output_dir, step)),
                    }
                )
                expected_events.append(
                    {
                        "sequence": sequence,
                        "type": "shot",
                        "step": step,
                        "path": str(_frame_path(output_dir, step)),
                    }
                )
                sequence += 1
        for action in schedule.actions:
            if action.step == step:
                expected_actions.append(
                    {
                        "sequence": sequence,
                        "step": step,
                        "key": action.key,
                    }
                )
                expected_events.append(
                    {
                        "sequence": sequence,
                        "type": "action",
                        "step": step,
                        "key": action.key,
                    }
                )
                sequence += 1

    actual_shots = data["shots"]
    if len(actual_shots) != len(expected_shots):
        raise ValueError(f"{sidecar_path}: shot prefix differs from the schedule")
    for actual, expected in zip(actual_shots, expected_shots):
        actual_path = actual.get("path")
        if (
            actual.get("sequence") != expected["sequence"]
            or actual.get("step") != expected["step"]
            or not isinstance(actual_path, str)
            or Path(actual_path) != Path(expected["path"])
            or actual.get("success") is not True
        ):
            raise ValueError(f"{sidecar_path}: shot prefix differs from the schedule")
        sim_time = actual.get("sim_time")
        if not _is_finite_number(sim_time) or not math.isclose(
            sim_time,
            schedule.time_at_step(expected["step"]),
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError(f"{sidecar_path}: shot prefix time mismatch")
        if actual.get("solver_diagnostics") != diagnostics_by_step[expected["step"]]:
            raise ValueError(f"{sidecar_path}: shot prefix diagnostics are not bound")

    actual_actions = data["actions"]
    if len(actual_actions) != len(expected_actions):
        raise ValueError(f"{sidecar_path}: action prefix differs from the schedule")
    for actual, expected in zip(actual_actions, expected_actions):
        if (
            any(actual.get(key) != value for key, value in expected.items())
            or actual.get("success") is not True
            or ("key_code" in actual and actual["key_code"] != ord(expected["key"]))
        ):
            raise ValueError(f"{sidecar_path}: action prefix differs from the schedule")

    events = data["events"]
    if len(events) != len(expected_events):
        raise ValueError(f"{sidecar_path}: events exceed the pre-trigger prefix")
    for actual, expected in zip(events, expected_events):
        if actual.get("success") is not True:
            raise ValueError(f"{sidecar_path}: pre-trigger event failed")
        if any(
            actual.get(key) != value for key, value in expected.items() if key != "path"
        ) or (
            "path" in expected
            and (
                not isinstance(actual.get("path"), str)
                or Path(actual["path"]) != Path(expected["path"])
            )
        ):
            raise ValueError(f"{sidecar_path}: pre-trigger event prefix differs")
    if data.get("solver_diagnostics") != offending:
        raise ValueError(f"{sidecar_path}: final diagnostics differ from trigger")
    return {
        "sidecar": str(sidecar_path),
        "completed_steps": completed_steps,
        "reason": state["reason"],
        "headless_exact_fbf_source_continuation": state,
        "physics_contract": physics_contract_report,
    }


def validate_sidecar(
    schedule: CaptureSchedule,
    output_dir: Path,
    *,
    expected_demo: Path | None = None,
    allow_legacy_fail_fast: bool = False,
) -> dict[str, Any]:
    sidecar_path = output_dir / "timeline.json"
    data = json.loads(sidecar_path.read_text(encoding="utf-8"))
    if data.get("schema_version") != SIDECAR_SCHEMA_VERSION:
        raise ValueError(f"{sidecar_path}: unexpected schema")
    if (
        data.get("scene") != schedule.scene
        or data.get("active_scene") != schedule.scene
    ):
        raise ValueError(f"{sidecar_path}: requested/active scene does not match")
    if data.get("total_steps") != schedule.total_steps:
        raise ValueError(f"{sidecar_path}: total step mismatch")
    if data.get("completed_steps") != schedule.total_steps:
        raise ValueError(f"{sidecar_path}: incomplete trajectory")
    if data.get("event_order") != "captures_before_actions_at_each_completed_step":
        raise ValueError(f"{sidecar_path}: unsupported event ordering")
    if (data.get("width"), data.get("height")) != (
        schedule.width,
        schedule.height,
    ):
        raise ValueError(f"{sidecar_path}: capture dimensions do not match")
    if data.get("collision_detector") != schedule.collision_detector:
        raise ValueError(
            f"{sidecar_path}: collision detector differs from the schedule"
        )
    try:
        runtime_argv = shlex.split(data["runtime_command"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{sidecar_path}: invalid runtime command") from error
    runtime_demo = Path(runtime_argv[0]) if expected_demo is None else expected_demo
    expected_runtime_argv = build_demo_command(schedule, runtime_demo, output_dir)
    fail_fast_field_present = "headless_exact_fbf_fail_fast" in data
    continuation_field_present = "headless_exact_fbf_source_continuation" in data
    legacy_fail_fast = (
        schedule.exact_fbf_required
        and not schedule.source_continuation_required
        and allow_legacy_fail_fast
        and not fail_fast_field_present
        and HEADLESS_EXACT_FBF_FAIL_FAST_FLAG not in runtime_argv
    )
    command_contract = (
        _legacy_demo_command(expected_runtime_argv)
        if legacy_fail_fast
        else expected_runtime_argv
    )
    if runtime_argv != command_contract:
        raise ValueError(f"{sidecar_path}: runtime command differs from the schedule")
    physics_contract_report = _validate_schedule_physics_contract(
        schedule, data, sidecar_path=sidecar_path
    )
    if schedule.source_continuation_required:
        if fail_fast_field_present:
            raise ValueError(f"{sidecar_path}: unexpected strict fail-fast state")
        continuation_state = _validate_source_continuation_state(
            data, sidecar_path=sidecar_path, triggered=False
        )
        fail_fast_state = None
    elif schedule.exact_fbf_required:
        if continuation_field_present:
            raise ValueError(
                f"{sidecar_path}: unexpected source-continuation gate state"
            )
        if legacy_fail_fast:
            fail_fast_state = None
        else:
            fail_fast_state = _validate_fail_fast_state(
                data, sidecar_path=sidecar_path, triggered=False
            )
        continuation_state = None
    else:
        if fail_fast_field_present or continuation_field_present:
            raise ValueError(f"{sidecar_path}: unexpected exact-FBF gate state")
        fail_fast_state = None
        continuation_state = None

    trajectory_steps = data.get("steps", [])
    if [item.get("step") for item in trajectory_steps] != list(
        range(schedule.total_steps + 1)
    ):
        raise ValueError(
            f"{sidecar_path}: completed-step diagnostics are missing or out of order"
        )
    aggregate_counter_names = (
        "exact_attempts",
        "exact_solves",
        "accepted_at_cap",
        "exact_failures",
        "boxed_lcp_fallbacks",
    )
    prior_counters = {name: 0 for name in aggregate_counter_names}
    prior_worst: float | None = None
    continuation_trajectory_state = _new_source_continuation_trajectory_state()
    diagnostics_by_step: dict[int, dict[str, Any]] = {}
    for item in trajectory_steps:
        step = int(item["step"])
        sim_time = item.get("sim_time")
        if not _is_finite_number(sim_time) or not math.isclose(
            sim_time, schedule.time_at_step(step), rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(
                f"{sidecar_path}: completed-step time mismatch at step {step}"
            )
        diagnostics = item.get("solver_diagnostics")
        if not isinstance(diagnostics, dict):
            raise ValueError(f"{sidecar_path}: completed-step diagnostics are invalid")
        if schedule.source_continuation_required:
            continuation = _validate_source_continuation_diagnostics(
                diagnostics, label=f"trajectory step {step}"
            )
            _validate_source_continuation_trajectory_step(
                diagnostics,
                continuation,
                step_index=step,
                state=continuation_trajectory_state,
                label=f"trajectory step {step}",
                require_accepted_outcome=True,
            )
        else:
            _validate_diagnostics(
                diagnostics,
                exact_required=schedule.exact_fbf_required,
                solver_lane=schedule.solver_lane,
                label=f"trajectory step {step}",
            )
            if schedule.exact_fbf_required:
                current_counters = {
                    name: diagnostics[name] for name in aggregate_counter_names
                }
                for name in aggregate_counter_names:
                    current = current_counters[name]
                    if current < prior_counters[name]:
                        raise ValueError(
                            f"trajectory step {step}: cumulative {name} regressed"
                        )
                contacts = diagnostics.get("contacts")
                if (
                    isinstance(contacts, bool)
                    or not isinstance(contacts, int)
                    or contacts < 0
                ):
                    raise ValueError(
                        f"trajectory step {step}: contact diagnostics are unavailable"
                    )
                if step == 0 and current_counters["exact_attempts"] != 0:
                    raise ValueError(
                        "trajectory step 0: exact attempts exist before simulation"
                    )
                if step > 0 and contacts > 0:
                    attempt_delta = (
                        current_counters["exact_attempts"]
                        - prior_counters["exact_attempts"]
                    )
                    solve_delta = (
                        current_counters["exact_solves"]
                        - prior_counters["exact_solves"]
                    )
                    if attempt_delta <= 0 or solve_delta <= 0:
                        raise ValueError(
                            f"trajectory step {step}: contact step did not advance "
                            "exact attempt/solve counters"
                        )
                prior_counters = current_counters
                current_worst = diagnostics.get("worst_residual")
                if current_worst is not None:
                    if prior_worst is not None and current_worst < prior_worst:
                        raise ValueError(
                            f"trajectory step {step}: cumulative worst residual "
                            "regressed"
                        )
                    prior_worst = float(current_worst)
        diagnostics_by_step[step] = diagnostics

    scene_state_metrics = _validate_author_painleve_scene_state_trace(
        schedule,
        trajectory_steps,
        sidecar_path=sidecar_path,
        expected_last_step=schedule.total_steps,
    )
    author_arch_scene_state_metrics = (
        _validate_author_masonry_arch_101_scene_state_trace(
            schedule,
            trajectory_steps,
            sidecar_path=sidecar_path,
            expected_last_step=schedule.total_steps,
        )
    )
    _require_author_masonry_arch_101_complete_outcome(
        schedule,
        author_arch_scene_state_metrics,
        sidecar_path=sidecar_path,
    )
    dart_adapter_outcome = (
        _validate_author_painleve_dart_adapter_outcome(
            schedule,
            trajectory_steps,
            sidecar_path=sidecar_path,
            expected_last_step=schedule.total_steps,
        )
        if schedule.total_steps == AUTHOR_PAINLEVE_TERMINAL_WINDOW[1]
        else None
    )

    shots = data.get("shots", [])
    expected_shots = list(schedule.capture_steps)
    if [shot.get("step") for shot in shots] != expected_shots:
        raise ValueError(f"{sidecar_path}: shots are missing or out of order")
    if [shot.get("sequence") for shot in shots] != [
        event.get("sequence")
        for event in data.get("events", [])
        if event.get("type") == "shot"
    ]:
        raise ValueError(f"{sidecar_path}: shot/event sequences disagree")

    expected_actions = [(action.step, action.key) for action in schedule.actions]
    actual_actions = [
        (action.get("step"), action.get("key")) for action in data.get("actions", [])
    ]
    if actual_actions != expected_actions:
        raise ValueError(f"{sidecar_path}: actions are missing or out of order")
    if any(not action.get("success") for action in data.get("actions", [])):
        raise ValueError(f"{sidecar_path}: at least one scheduled action failed")

    events = data.get("events", [])
    if [event.get("sequence") for event in events] != list(range(len(events))):
        raise ValueError(f"{sidecar_path}: event sequence is not contiguous")
    for step in {action.step for action in schedule.actions}:
        same_step = [event.get("type") for event in events if event.get("step") == step]
        if "action" in same_step and "shot" in same_step:
            if same_step.index("shot") > same_step.index("action"):
                raise ValueError(f"{sidecar_path}: action precedes same-step capture")

    frame_hashes: dict[int, str] = {}
    frame_verdicts: dict[int, dict[str, Any]] = {}
    world_regions: dict[int, dict[str, Any]] = {}
    for shot in shots:
        step = int(shot["step"])
        expected_path = _frame_path(output_dir, step)
        if Path(shot.get("path", "")) != expected_path:
            raise ValueError(f"{sidecar_path}: unexpected shot path at step {step}")
        if not shot.get("success"):
            raise ValueError(f"{sidecar_path}: capture failed at step {step}")
        sim_time = shot.get("sim_time")
        if not isinstance(sim_time, (int, float)) or not math.isclose(
            sim_time, schedule.time_at_step(step), rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(f"{sidecar_path}: simulation time mismatch at step {step}")
        frame_verdicts[step] = _validate_png(
            expected_path, schedule.width, schedule.height
        )
        world_regions[step] = _world_region_report(expected_path, schedule)
        frame_hashes[step] = world_regions[step]["sha256"]
        diagnostics = shot.get("solver_diagnostics", {})
        if diagnostics != diagnostics_by_step[step]:
            raise ValueError(
                f"step {step}: shot diagnostics differ from completed-step record"
            )
        if schedule.source_continuation_required:
            _validate_source_continuation_diagnostics(diagnostics, label=f"step {step}")
        else:
            _validate_diagnostics(
                diagnostics,
                exact_required=schedule.exact_fbf_required,
                solver_lane=schedule.solver_lane,
                label=f"step {step}",
            )

    if schedule.expect_motion and len(set(frame_hashes.values())) < 2:
        raise ValueError(f"{sidecar_path}: every dynamic capture is byte-identical")
    final_diagnostics = data.get("solver_diagnostics", {})
    if final_diagnostics != diagnostics_by_step[schedule.total_steps]:
        raise ValueError(
            f"{sidecar_path}: final diagnostics differ from the final step"
        )
    if schedule.source_continuation_required:
        _validate_source_continuation_diagnostics(
            final_diagnostics, label="final diagnostics"
        )
    else:
        _validate_diagnostics(
            final_diagnostics,
            exact_required=schedule.exact_fbf_required,
            solver_lane=schedule.solver_lane,
            label="final diagnostics",
        )
    if (
        schedule.exact_fbf_required
        and schedule.total_steps > 0
        and final_diagnostics.get("exact_attempts", 0) <= 0
    ):
        raise ValueError(f"{sidecar_path}: exact FBF never attempted a contact group")
    return {
        "sidecar": str(sidecar_path),
        "shot_count": len(shots),
        "action_count": len(actual_actions),
        "unique_frame_hashes": len(set(frame_hashes.values())),
        "step_count": len(trajectory_steps),
        "steps": {
            str(item["step"]): {
                "sim_time": item["sim_time"],
                "solver_diagnostics": item.get("solver_diagnostics", {}),
                **(
                    {"scene_state": item["scene_state"]}
                    if "scene_state" in item
                    else {}
                ),
            }
            for item in trajectory_steps
        },
        "frames": {
            str(step): {
                "path": str(_frame_path(output_dir, step)),
                "sha256": _sha256(_frame_path(output_dir, step)),
                "non_blank": frame_verdicts[step]["checks"]["non_blank"],
                "world_viewport": world_regions[step],
            }
            for step in expected_shots
        },
        "final_solver_diagnostics": final_diagnostics,
        "headless_exact_fbf_fail_fast": {
            "legacy_artifact": legacy_fail_fast,
            "state": fail_fast_state,
        },
        **(
            {
                "headless_exact_fbf_source_continuation": {
                    "state": continuation_state,
                }
            }
            if schedule.source_continuation_required
            else {}
        ),
        "physics_contract": physics_contract_report,
        **(
            {"scene_state_metrics": scene_state_metrics}
            if scene_state_metrics is not None
            else {}
        ),
        **(
            {
                "author_masonry_arch_101_scene_state_metrics": (
                    author_arch_scene_state_metrics
                )
            }
            if author_arch_scene_state_metrics is not None
            else {}
        ),
        **(
            {"dart_adapter_outcome": dart_adapter_outcome}
            if dart_adapter_outcome is not None
            else {}
        ),
        "pass": True,
    }


def _crop_filter(schedule: CaptureSchedule) -> str:
    width, height, x, y = schedule.crop
    return f"crop={width}:{height}:{x}:{y}"


def _prepare_panel_frames(
    schedule: CaptureSchedule, output_dir: Path, ffmpeg: Path
) -> list[Path]:
    panel_paths: list[Path] = []
    for step in schedule.panel_steps:
        destination = _panel_frame_path(output_dir, step)
        destination.parent.mkdir(parents=True, exist_ok=True)
        _run(
            [
                str(ffmpeg),
                "-hide_banner",
                "-loglevel",
                "error",
                "-y",
                "-i",
                str(_frame_path(output_dir, step)),
                "-vf",
                _crop_filter(schedule),
                "-frames:v",
                "1",
                str(destination),
            ]
        )
        _validate_png(destination, schedule.crop[0], schedule.crop[1])
        panel_paths.append(destination)
    return panel_paths


def _compose_panel(
    schedule: CaptureSchedule,
    output_dir: Path,
    panel_paths: Sequence[Path],
    python: Path,
) -> dict[str, Any]:
    command = [
        str(python),
        str(ROOT / "scripts/image_compose.py"),
        "side-by-side",
        *(str(path) for path in panel_paths),
        "--out",
        str(output_dir / "panel.png"),
        "--labels",
        *schedule.panel_labels,
        "--gap",
        "8",
        "--manifest",
        str(output_dir / "panel.compose.json"),
    ]
    _run(command)
    verdict = build_verdict(output_dir / "panel.png")
    if not verdict["pass"]:
        raise ValueError(f"{output_dir / 'panel.png'}: {verdict['reasons']}")
    compose_manifest_path = output_dir / "panel.compose.json"
    panel_path = output_dir / "panel.png"
    return {
        "command": command,
        "path": str(panel_path),
        "sha256": _sha256(panel_path),
        "size_bytes": panel_path.stat().st_size,
        "source_frames": [
            {"path": str(path), "sha256": _sha256(path)} for path in panel_paths
        ],
        "compose_manifest_path": str(compose_manifest_path),
        "compose_manifest_sha256": _sha256(compose_manifest_path),
        "compose_manifest": json.loads(
            compose_manifest_path.read_text(encoding="utf-8")
        ),
        "verdict": verdict,
    }


def _write_ffconcat(schedule: CaptureSchedule, output_dir: Path) -> Path:
    path = output_dir / "video_frames.ffconcat"
    steps = schedule.video_steps
    if not steps:
        raise ValueError(f"{schedule.id}: no video frames are scheduled")
    lines = ["ffconcat version 1.0"]
    for index, step in enumerate(steps):
        lines.append(f"file 'frames/step_{step:06d}.png'")
        if index + 1 < len(steps):
            next_step = steps[index + 1]
            duration = (next_step - step) * schedule.time_step_seconds
            lines.append(f"duration {duration:.17g}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _encode_media(
    schedule: CaptureSchedule, output_dir: Path, ffmpeg: Path
) -> list[dict[str, Any]]:
    if not schedule.encode_mp4 and not schedule.encode_gif:
        return []
    concat = _write_ffconcat(schedule, output_dir)
    outputs: list[dict[str, Any]] = []
    if schedule.encode_mp4:
        destination = output_dir / "clip.mp4"
        command = [
            str(ffmpeg),
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "concat",
            "-safe",
            "0",
            "-i",
            str(concat),
            "-vf",
            f"{_crop_filter(schedule)},fps={schedule.output_fps}",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "24",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            "-frames:v",
            str(_expected_media_stream(schedule, "mp4")["frame_count"]),
            str(destination),
        ]
        _run(command, cwd=output_dir)
        outputs.append({"kind": "mp4", "path": destination, "command": command})
    if schedule.encode_gif:
        destination = output_dir / "clip.gif"
        filter_graph = (
            f"{_crop_filter(schedule)},fps=15,scale=960:-2:flags=lanczos,"
            "split[base][palette_input];[palette_input]palettegen[palette];"
            "[base][palette]paletteuse"
        )
        command = [
            str(ffmpeg),
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "concat",
            "-safe",
            "0",
            "-i",
            str(concat),
            "-filter_complex",
            filter_graph,
            "-loop",
            "0",
            "-frames:v",
            str(_expected_media_stream(schedule, "gif")["frame_count"]),
            str(destination),
        ]
        _run(command, cwd=output_dir)
        outputs.append({"kind": "gif", "path": destination, "command": command})
    return outputs


def _probe_media(path: Path, ffprobe: Path) -> dict[str, Any]:
    result = _run(
        [
            str(ffprobe),
            "-v",
            "error",
            "-show_entries",
            "format=duration:stream=codec_name,pix_fmt,width,height,"
            "r_frame_rate,nb_frames",
            "-of",
            "json",
            str(path),
        ],
        capture_output=True,
    )
    return json.loads(result.stdout)


def _parse_frame_rate(value: Any, *, label: str) -> Fraction:
    if not isinstance(value, str):
        raise ValueError(f"{label}: frame rate is unavailable")
    try:
        rate = Fraction(value)
    except (ValueError, ZeroDivisionError) as error:
        raise ValueError(f"{label}: invalid rational frame rate {value!r}") from error
    if rate <= 0:
        raise ValueError(f"{label}: frame rate must be positive")
    return rate


def _expected_frame_count(schedule: CaptureSchedule, frame_rate: Fraction) -> int:
    simulated_frames = schedule.total_steps * schedule.time_step_fraction * frame_rate
    return simulated_frames.numerator // simulated_frames.denominator + 1


def _scaled_even_dimension(value: Fraction) -> int:
    # ffmpeg's scale=-2 chooses the nearest positive even integer.
    half = value / 2
    rounded_half = (2 * half.numerator + half.denominator) // (2 * half.denominator)
    return max(2, 2 * rounded_half)


def _expected_media_stream(schedule: CaptureSchedule, kind: str) -> dict[str, Any]:
    if kind == "mp4":
        frame_rate = Fraction(schedule.output_fps, 1)
        width, height = schedule.crop[:2]
    elif kind == "gif":
        frame_rate = Fraction(15, 1)
        source_width, source_height = schedule.crop[:2]
        width = 960
        height = _scaled_even_dimension(Fraction(source_height * width, source_width))
    else:
        raise ValueError(f"{schedule.id}: unsupported media kind {kind!r}")
    return {
        "width": width,
        "height": height,
        "frame_rate": frame_rate,
        "frame_rate_rational": f"{frame_rate.numerator}/{frame_rate.denominator}",
        "frame_count": _expected_frame_count(schedule, frame_rate),
    }


def _validate_stream_contract(
    stream: dict[str, Any], expected: dict[str, Any], *, label: str
) -> None:
    if (stream.get("width"), stream.get("height")) != (
        expected["width"],
        expected["height"],
    ):
        raise ValueError(
            f"{label}: dimensions {stream.get('width')}x{stream.get('height')} "
            f"do not match {expected['width']}x{expected['height']}"
        )
    actual_rate = _parse_frame_rate(stream.get("r_frame_rate"), label=label)
    if actual_rate != expected["frame_rate"]:
        raise ValueError(
            f"{label}: frame rate {actual_rate} does not match "
            f"{expected['frame_rate']}"
        )
    frame_count = stream.get("nb_frames")
    try:
        actual_count = int(frame_count)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label}: exact frame count is unavailable") from error
    if actual_count != expected["frame_count"]:
        raise ValueError(
            f"{label}: frame count {actual_count} does not match "
            f"{expected['frame_count']}"
        )


def _validate_mp4_stream_contract(stream: dict[str, Any], *, label: str) -> None:
    if stream.get("codec_name") != "h264":
        raise ValueError(
            f"{label}: codec {stream.get('codec_name')!r} does not match 'h264'"
        )
    if stream.get("pix_fmt") != "yuv420p":
        raise ValueError(
            f"{label}: pixel format {stream.get('pix_fmt')!r} does not match "
            "'yuv420p'"
        )


def _validate_media(
    schedule: CaptureSchedule,
    media: Sequence[dict[str, Any]],
    ffmpeg: Path,
    ffprobe: Path,
) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    for item in media:
        path = Path(item["path"])
        if not path.is_file() or path.stat().st_size == 0:
            raise ValueError(f"{path}: media output is missing/empty")
        probe = _probe_media(path, ffprobe)
        stream = _media_stream(probe, label=str(path))
        expected_stream = _expected_media_stream(schedule, item["kind"])
        _validate_stream_contract(stream, expected_stream, label=str(path))
        if item["kind"] == "mp4":
            _validate_mp4_stream_contract(stream, label=str(path))
        duration = float(probe.get("format", {}).get("duration", 0.0))
        expected_duration = (
            expected_stream["frame_count"] / expected_stream["frame_rate"]
        )
        duration_tolerance = max(
            0.01, float(Fraction(1, 2) / expected_stream["frame_rate"])
        )
        if not math.isfinite(duration) or abs(duration - float(expected_duration)) > (
            duration_tolerance
        ):
            raise ValueError(
                f"{path}: duration {duration} differs from "
                f"{float(expected_duration)}"
            )
        # ffprobe validates headers; this full null decode catches corrupt image
        # payloads or broken frame references.
        _run(
            [
                str(ffmpeg),
                "-hide_banner",
                "-loglevel",
                "error",
                "-i",
                str(path),
                "-f",
                "null",
                os.devnull,
            ]
        )
        reports.append(
            {
                "kind": item["kind"],
                "path": str(path),
                "sha256": _sha256(path),
                "size_bytes": path.stat().st_size,
                "probe": probe,
                "stream_contract": {
                    **expected_stream,
                    "frame_rate": expected_stream["frame_rate_rational"],
                },
                "full_decode": "pass",
                "command": item["command"],
            }
        )
    return reports


def run_schedule(
    schedule: CaptureSchedule,
    *,
    demo: Path,
    output_root: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
    allow_long: bool,
) -> dict[str, Any]:
    if not schedule.runnable:
        raise ValueError(f"{schedule.id}: {schedule.adapter_gap}")
    if schedule.long_run and not allow_long:
        raise ValueError(f"{schedule.id}: pass --allow-long to execute this trajectory")
    output_root = output_root.resolve()
    output_dir = output_root / schedule.id
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "metadata.json").unlink(missing_ok=True)
    (output_dir / "timeline.json").unlink(missing_ok=True)
    for executable in (demo, ffmpeg, ffprobe, python):
        if not executable.is_file():
            raise FileNotFoundError(executable)
    demo = demo.resolve()
    ffmpeg = ffmpeg.resolve()
    ffprobe = ffprobe.resolve()
    python = python.resolve()

    (output_dir / "frames").mkdir(exist_ok=True)
    command = build_demo_command(schedule, demo, output_dir)
    visual_resources_before = _visual_resource_snapshot(schedule)
    try:
        _run(command, cwd=ROOT)
    except subprocess.CalledProcessError as error:
        if not schedule.exact_fbf_required:
            raise
        gate_label = (
            "exact-FBF source-continuation gate"
            if schedule.source_continuation_required
            else "exact-FBF fail-fast"
        )
        try:
            if schedule.source_continuation_required:
                failure = _validate_failed_source_continuation_sidecar(
                    schedule, output_dir, expected_demo=demo
                )
            else:
                failure = _validate_failed_exact_fbf_sidecar(
                    schedule, output_dir, expected_demo=demo
                )
        except (OSError, ValueError) as sidecar_error:
            raise ValueError(
                f"{schedule.id}: demo exited {error.returncode} without a valid "
                f"{gate_label} sidecar: {sidecar_error}"
            ) from error
        raise ValueError(
            f"{schedule.id}: {gate_label} triggered at completed step "
            f"{failure['completed_steps']}: {failure['reason']}"
        ) from error
    visual_resources_after = _visual_resource_snapshot(schedule)
    visual_resources = _bind_visual_resource_snapshots(
        visual_resources_before, visual_resources_after
    )
    timeline_report = validate_sidecar(schedule, output_dir, expected_demo=demo)
    panel_paths = _prepare_panel_frames(schedule, output_dir, ffmpeg)
    panel_report = _compose_panel(schedule, output_dir, panel_paths, python)
    media_items = _encode_media(schedule, output_dir, ffmpeg)
    media_report = _validate_media(schedule, media_items, ffmpeg, ffprobe)

    report = {
        "schema_version": SCHEMA_VERSION,
        "kind": "capture_result",
        "schedule": schedule_plan(schedule, demo, output_root),
        "runtime": {
            "demo_argv": command,
            "demo_path": str(demo),
            "demo_sha256": _sha256(demo),
            "ffmpeg": str(ffmpeg),
            "ffprobe": str(ffprobe),
            **(
                {"visual_resources": visual_resources}
                if visual_resources is not None
                else {}
            ),
        },
        "timeline_validation": timeline_report,
        "panel_validation": panel_report,
        "media_validation": media_report,
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        **(
            {
                "automated_current_dart_adapter_outcome_validated": True,
                "current_dart_adapter_outcome": timeline_report["dart_adapter_outcome"],
            }
            if _is_author_painleve_schedule(schedule)
            else {}
        ),
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": CAPTURE_SEMANTIC_OUTCOME_GATE,
        "known_mismatches": list(schedule.mismatches),
        "pass": True,
    }
    _write_json(output_dir / "metadata.json", report)
    return report


def _read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return data


def _media_stream(probe: dict[str, Any], *, label: str) -> dict[str, Any]:
    streams = [stream for stream in probe.get("streams", []) if stream.get("width")]
    if len(streams) != 1:
        raise ValueError(f"{label}: expected exactly one decoded video stream")
    return streams[0]


def _group_semantic_outcome_gate(group: GroupOutputSpec) -> str:
    if group.solver_lane == "both":
        return (
            "the synchronized exact-versus-boxed composite is a presentation "
            "artifact; it does not establish solver superiority; manual inspection "
            "plus per-member physical trace/test contracts remain required"
        )
    prefix = (
        "the composite video remains synchronized while the outcome-aware "
        "still uses explicitly recorded per-member source times; "
        if group.panel_steps is not None
        else "the synchronized composite is a presentation artifact; "
    )
    return (
        prefix + "manual inspection plus per-member physical trace/test contracts "
        "remain required"
    )


def _validate_group_claim_boundary(
    metadata: dict[str, Any],
    group: GroupOutputSpec,
    *,
    metadata_path: Path,
) -> None:
    claims = {
        "source_segment": group.source_segment,
        "solver_lane": group.solver_lane,
        "source_group_id": group.source_group_id,
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": _group_semantic_outcome_gate(group),
    }
    if _is_author_painleve_solver_comparison_group(group):
        comparison = metadata.get("current_dart_adapter_outcome_comparison")
        if not isinstance(comparison, dict) or comparison.get("pass") is not True:
            raise ValueError(
                f"{metadata_path}: current DART adapter comparison is unavailable"
            )
        claims["automated_current_dart_adapter_outcome_validated"] = True
    for key, expected in claims.items():
        if not _claim_value_matches(metadata.get(key), expected):
            raise ValueError(f"{metadata_path}: group claim boundary {key} changed")


def _group_member_contract(
    group: GroupOutputSpec,
    schedules: Sequence[CaptureSchedule],
    *,
    output_root: Path,
    ffprobe: Path,
) -> dict[str, Any]:
    member_order = tuple(schedule.id for schedule in schedules)
    if member_order != group.members:
        raise ValueError(
            f"{group.id}: members are missing or out of source order; "
            f"expected {group.members}, got {member_order}"
        )

    member_lanes = tuple(schedule.solver_lane for schedule in schedules)
    expected_lanes = (
        SOLVER_LANES
        if group.solver_lane == "both"
        else (group.solver_lane,) * len(schedules)
    )
    if member_lanes != expected_lanes:
        raise ValueError(
            f"{group.id}: solver lanes are missing or out of order; "
            f"expected {expected_lanes}, got {member_lanes}"
        )

    first = schedules[0]
    expected_member_stream = _expected_media_stream(first, "mp4")
    panel_steps = group.resolved_panel_steps
    for schedule, panel_step in zip(schedules, panel_steps):
        if panel_step not in schedule.capture_steps:
            raise ValueError(
                f"{group.id}: panel source step {panel_step} is not a captured "
                f"instant for {schedule.id}"
            )
        if schedule.total_steps != first.total_steps:
            raise ValueError(f"{group.id}: member total-step mismatch")
        if schedule.time_step_fraction != first.time_step_fraction:
            raise ValueError(f"{group.id}: member time-step mismatch")
        if schedule.video_steps != first.video_steps:
            raise ValueError(f"{group.id}: member video-step ordering mismatch")
        if schedule.output_fps != first.output_fps:
            raise ValueError(f"{group.id}: member output-FPS mismatch")
        if schedule.crop[:2] != first.crop[:2]:
            raise ValueError(f"{group.id}: member viewport-size mismatch")

    members: list[dict[str, Any]] = []
    outcome_reports: list[dict[str, Any]] = []
    durations: list[float] = []
    frame_counts: list[int | None] = []
    frame_rates: list[str | None] = []
    for schedule, panel_step in zip(schedules, panel_steps):
        output_dir = output_root / schedule.id
        metadata_path = output_dir / "metadata.json"
        metadata = _read_json(metadata_path)
        if metadata.get("schema_version") != SCHEMA_VERSION:
            raise ValueError(f"{metadata_path}: unexpected schema")
        if metadata.get("kind") != "capture_result" or not metadata.get("pass"):
            raise ValueError(f"{metadata_path}: member capture is not successful")
        plan = metadata.get("schedule", {})
        if plan.get("id") != schedule.id or plan.get("scene") != schedule.scene:
            raise ValueError(f"{metadata_path}: member schedule identity mismatch")
        _validate_capture_claim_boundary(
            metadata, schedule, metadata_path=metadata_path
        )
        if metadata.get("actual_simulator") is not True:
            raise ValueError(
                f"{metadata_path}: member is not actual-simulator evidence"
            )
        if metadata.get("generated_imagery") is not False:
            raise ValueError(f"{metadata_path}: generated imagery is forbidden")
        if metadata.get("automated_semantic_outcome_validated") is not False:
            raise ValueError(
                f"{metadata_path}: semantic outcome must remain a manual gate"
            )
        if _is_author_painleve_solver_comparison_group(group):
            outcome = metadata.get("current_dart_adapter_outcome")
            if not isinstance(outcome, dict):
                raise ValueError(
                    f"{metadata_path}: current DART adapter outcome is unavailable"
                )
            outcome_reports.append(outcome)

        runtime = metadata.get("runtime", {})
        _validate_runtime_visual_resources(
            schedule, runtime, metadata_path=metadata_path
        )
        demo_path_value = runtime.get("demo_path")
        if not isinstance(demo_path_value, str) or not demo_path_value:
            raise ValueError(f"{metadata_path}: member demo path is unavailable")
        demo_path = Path(demo_path_value)
        if not demo_path.is_file() or runtime.get("demo_sha256") != _sha256(demo_path):
            raise ValueError(f"{metadata_path}: member demo hash binding changed")

        timeline_path = output_dir / "timeline.json"
        timeline_validation = metadata.get("timeline_validation", {})
        if timeline_validation.get("sidecar") != str(timeline_path):
            raise ValueError(f"{metadata_path}: member timeline path is not bound")
        panel_time = schedule.time_at_step(panel_step)
        timeline_step = timeline_validation.get("steps", {}).get(str(panel_step), {})
        panel_sim_time = timeline_step.get("sim_time")
        if not isinstance(panel_sim_time, (int, float)) or not math.isclose(
            panel_sim_time, panel_time, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(
                f"{metadata_path}: panel source simulation time changed at step "
                f"{panel_step}"
            )
        frame_path = _frame_path(output_dir, panel_step)
        timeline_frame = timeline_validation.get("frames", {}).get(str(panel_step), {})
        if timeline_frame.get("path") != str(frame_path):
            raise ValueError(f"{metadata_path}: panel source frame path is not bound")
        frame_hash = _sha256(frame_path)
        if timeline_frame.get("sha256") != frame_hash:
            raise ValueError(f"{metadata_path}: panel source frame hash changed")

        mp4_reports = [
            item
            for item in metadata.get("media_validation", [])
            if item.get("kind") == "mp4"
        ]
        if len(mp4_reports) != 1:
            raise ValueError(f"{metadata_path}: expected one validated member MP4")
        mp4_report = mp4_reports[0]
        clip_path = output_dir / "clip.mp4"
        if mp4_report.get("path") != str(clip_path):
            raise ValueError(f"{metadata_path}: member MP4 path is not bound")
        clip_hash = _sha256(clip_path)
        if mp4_report.get("sha256") != clip_hash:
            raise ValueError(f"{metadata_path}: member MP4 hash changed")
        probe = _probe_media(clip_path, ffprobe)
        stream = _media_stream(probe, label=str(clip_path))
        _validate_stream_contract(
            stream,
            _expected_media_stream(schedule, "mp4"),
            label=str(clip_path),
        )
        _validate_mp4_stream_contract(stream, label=str(clip_path))
        duration = float(probe.get("format", {}).get("duration", 0.0))
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError(f"{clip_path}: invalid duration")
        expected_duration = (
            expected_member_stream["frame_count"] / expected_member_stream["frame_rate"]
        )
        duration_tolerance = max(
            0.01,
            float(Fraction(1, 2) / expected_member_stream["frame_rate"]),
        )
        if abs(duration - float(expected_duration)) > duration_tolerance:
            raise ValueError(
                f"{clip_path}: duration {duration} differs from the schedule"
            )
        durations.append(duration)
        frame_counts.append(int(stream["nb_frames"]))
        frame_rates.append(
            str(_parse_frame_rate(stream.get("r_frame_rate"), label=str(clip_path)))
        )
        member = {
            "id": schedule.id,
            "label": group.labels[len(members)],
            "metadata_path": str(metadata_path),
            "metadata_sha256": _sha256(metadata_path),
            "demo_path": str(demo_path),
            "demo_sha256": runtime.get("demo_sha256"),
            "timeline_path": str(timeline_path),
            "timeline_sha256": _sha256(timeline_path),
            "panel_source_frame": str(frame_path),
            "panel_source_frame_sha256": frame_hash,
            "clip_path": str(clip_path),
            "clip_sha256": clip_hash,
            "clip_probe": probe,
        }
        if group.panel_steps is not None:
            member.update(
                {
                    "panel_source_step": panel_step,
                    "panel_source_time_seconds": panel_time,
                }
            )
        members.append(member)

    tolerance = max(0.05, 1.0 / first.output_fps + 0.01)
    if max(durations) - min(durations) > tolerance:
        raise ValueError(
            f"{group.id}: member duration mismatch {durations} exceeds {tolerance}"
        )
    if len(set(frame_counts)) != 1:
        raise ValueError(f"{group.id}: member frame-count mismatch {frame_counts}")
    if any(rate is None for rate in frame_rates) or len(set(frame_rates)) != 1:
        raise ValueError(f"{group.id}: member frame-rate mismatch {frame_rates}")

    outcome_comparison = _validate_author_painleve_solver_comparison(
        group,
        schedules,
        outcome_reports,
        label=group.id,
    )
    return {
        "members": members,
        "total_steps": first.total_steps,
        "video_steps": list(first.video_steps),
        "output_fps": first.output_fps,
        "duration_seconds": sum(durations) / len(durations),
        "duration_tolerance_seconds": tolerance,
        "frame_count": expected_member_stream["frame_count"],
        "frame_rate": expected_member_stream["frame_rate_rational"],
        "tile_width": first.crop[0],
        "tile_height": first.crop[1],
        **(
            {"dart_adapter_outcome_comparison": outcome_comparison}
            if outcome_comparison is not None
            else {}
        ),
    }


def _compose_labeled_row(
    paths: Sequence[Path],
    labels: Sequence[str],
    *,
    destination: Path,
    manifest_path: Path,
    python: Path,
) -> dict[str, Any]:
    command = [
        str(python),
        str(ROOT / "scripts/image_compose.py"),
        "side-by-side",
        *(str(path) for path in paths),
        "--out",
        str(destination),
        "--labels",
        *labels,
        "--gap",
        "8",
        "--manifest",
        str(manifest_path),
    ]
    _run(command)
    verdict = build_verdict(destination)
    if not verdict["pass"]:
        raise ValueError(f"{destination}: {verdict['reasons']}")
    return {
        "command": command,
        "manifest_path": str(manifest_path),
        "manifest_sha256": _sha256(manifest_path),
        "manifest": _read_json(manifest_path),
        "path": str(destination),
        "sha256": _sha256(destination),
        "verdict": verdict,
    }


def _compose_group_panel(
    group: GroupOutputSpec,
    schedules: Sequence[CaptureSchedule],
    *,
    output_root: Path,
    output_dir: Path,
    ffmpeg: Path,
    python: Path,
) -> dict[str, Any]:
    tile_dir = output_dir / "panel_members"
    tile_dir.mkdir(parents=True, exist_ok=True)
    tile_paths: list[Path] = []
    crop_commands: list[list[str]] = []
    panel_steps = group.resolved_panel_steps
    for schedule, panel_step in zip(schedules, panel_steps):
        source = _frame_path(output_root / schedule.id, panel_step)
        destination = tile_dir / f"{schedule.id}.png"
        command = [
            str(ffmpeg),
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-i",
            str(source),
            "-vf",
            _crop_filter(schedule),
            "-frames:v",
            "1",
            str(destination),
        ]
        _run(command)
        _validate_png(destination, schedule.crop[0], schedule.crop[1])
        tile_paths.append(destination)
        crop_commands.append(command)

    row_reports: list[dict[str, Any]] = []
    panel_path = output_dir / "panel.png"
    stack_command: list[str] | None = None
    if group.layout == "side-by-side":
        row_reports.append(
            _compose_labeled_row(
                tile_paths,
                group.resolved_panel_labels,
                destination=panel_path,
                manifest_path=output_dir / "panel.compose.json",
                python=python,
            )
        )
    else:
        row_paths = (output_dir / "panel_top.png", output_dir / "panel_bottom.png")
        for row_index, row_path in enumerate(row_paths):
            start = row_index * 2
            row_reports.append(
                _compose_labeled_row(
                    tile_paths[start : start + 2],
                    group.resolved_panel_labels[start : start + 2],
                    destination=row_path,
                    manifest_path=output_dir / f"panel_row_{row_index}.compose.json",
                    python=python,
                )
            )
        stack_command = [
            str(ffmpeg),
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-i",
            str(row_paths[0]),
            "-i",
            str(row_paths[1]),
            "-filter_complex",
            "[0:v][1:v]vstack=inputs=2[outv]",
            "-map",
            "[outv]",
            "-frames:v",
            "1",
            str(panel_path),
        ]
        _run(stack_command)
        top = row_reports[0]["verdict"]["image"]
        _validate_png(panel_path, top["width"], top["height"] * 2)

    verdict = build_verdict(panel_path)
    if not verdict["pass"]:
        raise ValueError(f"{panel_path}: {verdict['reasons']}")
    decode_command = [
        str(ffmpeg),
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        str(panel_path),
        "-f",
        "null",
        os.devnull,
    ]
    _run(decode_command)
    report = {
        "member_order": list(group.members),
        "labels": list(group.resolved_panel_labels),
        "source_frames": [
            {
                **(
                    {
                        "member": schedule.id,
                        "step": panel_step,
                        "time_seconds": schedule.time_at_step(panel_step),
                    }
                    if group.panel_steps is not None
                    else {}
                ),
                "path": str(_frame_path(output_root / schedule.id, panel_step)),
                "sha256": _sha256(_frame_path(output_root / schedule.id, panel_step)),
            }
            for schedule, panel_step in zip(schedules, panel_steps)
        ],
        "cropped_tiles": [
            {"path": str(path), "sha256": _sha256(path)} for path in tile_paths
        ],
        "crop_commands": crop_commands,
        "row_compositions": row_reports,
        "stack_command": stack_command,
        "path": str(panel_path),
        "sha256": _sha256(panel_path),
        "size_bytes": panel_path.stat().st_size,
        "verdict": verdict,
        "full_decode": "pass",
        "decode_command": decode_command,
    }
    if group.panel_step is not None:
        report["panel_step"] = group.panel_step
    return report


def _drawtext_filter(label: str) -> str:
    escaped = label.replace("\\", "\\\\").replace(":", "\\:").replace("'", "\\'")
    return (
        "setpts=PTS-STARTPTS,pad=iw:ih+24:0:24:color=0x181c20,"
        f"drawtext=text='{escaped}':x=8:y=5:fontsize=14:fontcolor=white"
    )


def _encode_group_media(
    group: GroupOutputSpec,
    contract: dict[str, Any],
    *,
    output_dir: Path,
    ffmpeg: Path,
    ffprobe: Path,
) -> dict[str, Any]:
    clip_paths = [Path(member["clip_path"]) for member in contract["members"]]
    filters = [
        f"[{index}:v]{_drawtext_filter(label)}[v{index}]"
        for index, label in enumerate(group.labels)
    ]
    if group.layout == "side-by-side":
        filters.append("[v0][v1]hstack=inputs=2:shortest=1[outv]")
    else:
        filters.extend(
            (
                "[v0][v1]hstack=inputs=2:shortest=1[top]",
                "[v2][v3]hstack=inputs=2:shortest=1[bottom]",
                "[top][bottom]vstack=inputs=2:shortest=1[outv]",
            )
        )
    destination = output_dir / "clip.mp4"
    command = [str(ffmpeg), "-hide_banner", "-loglevel", "error", "-y"]
    for path in clip_paths:
        command.extend(("-i", str(path)))
    command.extend(
        (
            "-filter_complex",
            ";".join(filters),
            "-map",
            "[outv]",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "24",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(destination),
        )
    )
    _run(command)
    if not destination.is_file() or destination.stat().st_size == 0:
        raise ValueError(f"{destination}: group MP4 is missing/empty")
    probe = _probe_media(destination, ffprobe)
    stream = _media_stream(probe, label=str(destination))
    input_width = contract["tile_width"]
    input_height = contract["tile_height"] + 24
    expected_width = input_width * 2
    expected_height = input_height * (2 if group.layout == "2x2" else 1)
    if (stream.get("width"), stream.get("height")) != (
        expected_width,
        expected_height,
    ):
        raise ValueError(f"{destination}: unexpected group-video dimensions")
    group_stream_contract = {
        "width": expected_width,
        "height": expected_height,
        "frame_rate": Fraction(contract["frame_rate"]),
        "frame_count": contract["frame_count"],
    }
    _validate_stream_contract(stream, group_stream_contract, label=str(destination))
    _validate_mp4_stream_contract(stream, label=str(destination))
    duration = float(probe.get("format", {}).get("duration", 0.0))
    if (
        abs(duration - contract["duration_seconds"])
        > contract["duration_tolerance_seconds"]
    ):
        raise ValueError(
            f"{destination}: duration {duration} differs from synchronized members"
        )
    decode_command = [
        str(ffmpeg),
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        str(destination),
        "-f",
        "null",
        os.devnull,
    ]
    _run(decode_command)
    return {
        "kind": "mp4",
        "path": str(destination),
        "sha256": _sha256(destination),
        "size_bytes": destination.stat().st_size,
        "probe": probe,
        "full_decode": "pass",
        "command": command,
        "decode_command": decode_command,
    }


def run_group_output(
    group: GroupOutputSpec,
    schedules: Sequence[CaptureSchedule],
    *,
    output_root: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
) -> dict[str, Any]:
    output_root = output_root.resolve()
    output_dir = output_root / "groups" / group.id
    output_dir.mkdir(parents=True, exist_ok=True)
    metadata_path = output_dir / "metadata.json"
    metadata_path.unlink(missing_ok=True)
    for executable in (ffmpeg, ffprobe, python):
        if not executable.is_file():
            raise FileNotFoundError(executable)
    contract = _group_member_contract(
        group, schedules, output_root=output_root, ffprobe=ffprobe
    )
    panel_report = _compose_group_panel(
        group,
        schedules,
        output_root=output_root,
        output_dir=output_dir,
        ffmpeg=ffmpeg,
        python=python,
    )
    media_report = _encode_group_media(
        group, contract, output_dir=output_dir, ffmpeg=ffmpeg, ffprobe=ffprobe
    )
    report = {
        "schema_version": SCHEMA_VERSION,
        "kind": "group_capture_result",
        "group_id": group.id,
        "source_segment": group.source_segment,
        "solver_lane": group.solver_lane,
        "source_group_id": group.source_group_id,
        "layout": group.layout,
        "member_order": list(group.members),
        "labels": list(group.labels),
        "synchronization": {
            key: contract[key]
            for key in (
                "total_steps",
                "video_steps",
                "output_fps",
                "duration_seconds",
                "duration_tolerance_seconds",
                "frame_count",
                "frame_rate",
            )
        },
        "members": contract["members"],
        **(
            {
                "automated_current_dart_adapter_outcome_validated": True,
                "current_dart_adapter_outcome_comparison": contract[
                    "dart_adapter_outcome_comparison"
                ],
            }
            if _is_author_painleve_solver_comparison_group(group)
            else {}
        ),
        "panel_validation": panel_report,
        "media_validation": media_report,
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": _group_semantic_outcome_gate(group),
        "pass": True,
    }
    _write_json(metadata_path, report)
    return report


def audit_coverage_contract() -> dict[str, Any]:
    """Validate the in-code source-video-to-demo coverage contract."""

    segment_ids = [segment.id for segment in VIDEO_SEGMENTS]
    if len(segment_ids) != len(set(segment_ids)):
        raise ValueError("source video segment identifiers are not unique")
    if VIDEO_SEGMENTS[0].start_seconds != 0 or VIDEO_SEGMENTS[-1].end_seconds != 82:
        raise ValueError("source video segments must cover 0 through 82 seconds")
    if any(
        left.end_seconds != right.start_seconds
        for left, right in zip(VIDEO_SEGMENTS, VIDEO_SEGMENTS[1:])
    ):
        raise ValueError("source video segments are not contiguous")

    missing_schedules = sorted(
        schedule_id
        for schedule_ids in REQUIRED_VIDEO_SCHEDULES.values()
        for schedule_id in schedule_ids
        if schedule_id not in SCHEDULES
    )
    if missing_schedules:
        raise ValueError(f"missing required visual schedules {missing_schedules}")

    wrong_segments = sorted(
        schedule_id
        for segment_id, schedule_ids in REQUIRED_VIDEO_SCHEDULES.items()
        for schedule_id in schedule_ids
        if SCHEDULES[schedule_id].source_segment != segment_id
    )
    if wrong_segments:
        raise ValueError(
            f"visual schedules mapped to the wrong segment {wrong_segments}"
        )

    required_ids = tuple(
        schedule_id
        for schedule_ids in REQUIRED_VIDEO_SCHEDULES.values()
        for schedule_id in schedule_ids
    )
    non_runnable = [
        schedule_id
        for schedule_id in required_ids
        if not SCHEDULES[schedule_id].runnable
    ]
    without_mp4 = [
        schedule_id
        for schedule_id in required_ids
        if not SCHEDULES[schedule_id].encode_mp4
    ]
    gate_blocked = [
        schedule_id
        for schedule_id in required_ids
        if SCHEDULES[schedule_id].known_gate_blockers
    ]
    return {
        "schema_version": SCHEMA_VERSION,
        "coverage_matrix": str(COVERAGE_MATRIX_PATH),
        "video_segments": {
            f"video.{index:02d}_{segment.id}": (
                segment.start_seconds,
                segment.end_seconds,
            )
            for index, segment in enumerate(VIDEO_SEGMENTS, start=1)
        },
        "required_video_schedules": {
            segment_id: list(schedule_ids)
            for segment_id, schedule_ids in REQUIRED_VIDEO_SCHEDULES.items()
        },
        "required_schedule_count": len(required_ids),
        "non_runnable_schedules": non_runnable,
        "schedules_without_mp4": without_mp4,
        "known_gate_blocked_schedules": gate_blocked,
        "capture_path_complete": not non_runnable and not without_mp4,
        "evidence_complete": not non_runnable and not without_mp4 and not gate_blocked,
        "pass": True,
    }


def audit_sources(
    *,
    video: Path,
    teaser: Path,
    paper: Path | None,
    ffmpeg: Path,
    ffprobe: Path,
) -> dict[str, Any]:
    for path in (video, teaser, ffmpeg, ffprobe):
        if not path.is_file():
            raise FileNotFoundError(path)
    probe = _probe_media(video, ffprobe)
    video_streams = [
        stream for stream in probe.get("streams", []) if stream.get("width")
    ]
    if not video_streams:
        raise ValueError(f"{video}: no video stream")
    video_stream = video_streams[0]
    duration = float(probe.get("format", {}).get("duration", 0.0))
    if not 81.8 <= duration <= 82.0:
        raise ValueError(f"{video}: expected the audited 82-second source")
    if (video_stream.get("width"), video_stream.get("height")) != (640, 360):
        raise ValueError(f"{video}: unexpected source dimensions")

    teaser_verdict = build_verdict(teaser)
    if not teaser_verdict["pass"]:
        raise ValueError(f"{teaser}: {teaser_verdict['reasons']}")
    if (
        teaser_verdict["image"]["width"],
        teaser_verdict["image"]["height"],
    ) != (3840, 2160):
        raise ValueError(f"{teaser}: unexpected teaser dimensions")

    keyframes: list[dict[str, Any]] = []
    with tempfile.TemporaryDirectory(prefix="fbf_source_audit_") as temp:
        temp_dir = Path(temp)
        for segment in VIDEO_SEGMENTS:
            keyframe = temp_dir / f"{segment.start_seconds:02d}_{segment.id}.png"
            _run(
                [
                    str(ffmpeg),
                    "-hide_banner",
                    "-loglevel",
                    "error",
                    "-y",
                    "-ss",
                    str(segment.start_seconds),
                    "-i",
                    str(video),
                    "-frames:v",
                    "1",
                    str(keyframe),
                ]
            )
            verdict = build_verdict(keyframe)
            if not verdict["pass"]:
                raise ValueError(
                    f"{video}: undecodable/blank segment keyframe {segment.id}"
                )
            keyframes.append(
                {
                    "segment": segment.id,
                    "start_seconds": segment.start_seconds,
                    "sha256": _sha256(keyframe),
                    "width": verdict["image"]["width"],
                    "height": verdict["image"]["height"],
                    "non_blank": verdict["checks"]["non_blank"]["pass"],
                }
            )

    paper_report: dict[str, Any] | None = None
    if paper is not None:
        if not paper.is_file():
            raise FileNotFoundError(paper)
        paper_hash = _sha256(paper)
        paper_report = {
            "path": str(paper),
            "sha256": paper_hash,
            "matches_audited_20260711": paper_hash == AUDITED_PAPER_SHA256,
        }
    video_hash = _sha256(video)
    teaser_hash = _sha256(teaser)
    hashes_match = (
        video_hash == AUDITED_VIDEO_SHA256
        and teaser_hash == AUDITED_TEASER_SHA256
        and (
            paper is None
            or (paper_report is not None and paper_report["matches_audited_20260711"])
        )
    )
    if not hashes_match:
        raise ValueError(
            "official source hash mismatch; refusing to audit unrelated or "
            "silently changed media"
        )
    return {
        "schema_version": SOURCE_AUDIT_SCHEMA_VERSION,
        "video": {
            "path": str(video),
            "sha256": video_hash,
            "matches_audited_20260711": video_hash == AUDITED_VIDEO_SHA256,
            "probe": probe,
            "keyframes": keyframes,
        },
        "teaser": {
            "path": str(teaser),
            "sha256": teaser_hash,
            "matches_audited_20260711": teaser_hash == AUDITED_TEASER_SHA256,
            "verdict": teaser_verdict,
            "semantic_layout": (
                "four-way white-diagonal composite: standing 26-card house, "
                "pre-impact arch, impacted card house, and impacted arch"
            ),
        },
        "paper": paper_report,
        "coverage_contract": audit_coverage_contract(),
        "segments": [dataclasses.asdict(segment) for segment in VIDEO_SEGMENTS],
        "semantic_observations": {
            "turntable": VIDEO_SEGMENTS[3].layout,
            "painleve": VIDEO_SEGMENTS[4].layout,
            "card_house_26": VIDEO_SEGMENTS[5].layout,
            "masonry_arch_25": VIDEO_SEGMENTS[6].layout,
            "masonry_arch_101": VIDEO_SEGMENTS[7].layout,
        },
        "pass": True,
    }


def _select_schedules(values: Sequence[str]) -> list[CaptureSchedule]:
    if not values or values == ["all-runnable"]:
        return [schedule for schedule in SCHEDULES.values() if schedule.runnable]
    selected_ids: list[str] = []
    for value in values:
        if value == "all":
            selected_ids.extend(SCHEDULES)
        elif value == "all-runnable":
            selected_ids.extend(
                schedule.id for schedule in SCHEDULES.values() if schedule.runnable
            )
        elif value in SCHEDULES:
            selected_ids.append(value)
        else:
            raise ValueError(f"unknown schedule {value!r}")
    return [SCHEDULES[schedule_id] for schedule_id in dict.fromkeys(selected_ids)]


def _common_capture_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--scenario",
        action="append",
        default=[],
        help="schedule id, all, or all-runnable; repeat to select several",
    )
    parser.add_argument(
        "--solver-lane",
        choices=(*SOLVER_LANES, "both"),
        default="exact",
        help=(
            "capture exact FBF, the existing boxed-LCP comparison, or both; "
            "exact-only and boxed-only scenes are reported as explicit skips"
        ),
    )
    parser.add_argument("--demo", type=Path, default=DEFAULT_DEMO)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    plan_parser = subparsers.add_parser("plan", help="emit schedules and commands")
    _common_capture_arguments(plan_parser)
    plan_parser.add_argument("--out", type=Path)

    run_parser = subparsers.add_parser("run", help="capture, compose, encode, validate")
    _common_capture_arguments(run_parser)
    run_parser.add_argument("--ffmpeg", type=Path, default=DEFAULT_FFMPEG)
    run_parser.add_argument("--ffprobe", type=Path, default=DEFAULT_FFPROBE)
    run_parser.add_argument("--python", type=Path, default=Path(sys.executable))
    run_parser.add_argument("--allow-long", action="store_true")
    run_parser.add_argument("--keep-going", action="store_true")
    run_parser.add_argument(
        "--dry-run",
        action="store_true",
        help="emit the plan without launching the demo or creating output",
    )
    run_parser.add_argument("--out", type=Path, help="summary JSON path")

    verify_parser = subparsers.add_parser(
        "verify", help="revalidate existing frames, sidecars, panels, and media"
    )
    _common_capture_arguments(verify_parser)
    verify_parser.add_argument("--ffmpeg", type=Path, default=DEFAULT_FFMPEG)
    verify_parser.add_argument("--ffprobe", type=Path, default=DEFAULT_FFPROBE)
    verify_parser.add_argument("--out", type=Path)

    audit_parser = subparsers.add_parser(
        "audit-source", help="audit the official local video/teaser and coverage"
    )
    audit_parser.add_argument("--video", type=Path, required=True)
    audit_parser.add_argument("--teaser", type=Path, required=True)
    audit_parser.add_argument("--paper", type=Path)
    audit_parser.add_argument("--ffmpeg", type=Path, default=DEFAULT_FFMPEG)
    audit_parser.add_argument("--ffprobe", type=Path, default=DEFAULT_FFPROBE)
    audit_parser.add_argument("--out", type=Path)
    return parser


def _verify_existing(
    schedule: CaptureSchedule,
    *,
    demo: Path,
    output_root: Path,
    ffmpeg: Path,
    ffprobe: Path,
) -> dict[str, Any]:
    output_dir = output_root.resolve() / schedule.id
    if not demo.is_file():
        raise FileNotFoundError(demo)
    demo = demo.resolve()
    timeline = validate_sidecar(
        schedule,
        output_dir,
        expected_demo=demo,
        allow_legacy_fail_fast=True,
    )
    panel_path = output_dir / "panel.png"
    panel_verdict = build_verdict(panel_path)
    if not panel_verdict["pass"]:
        raise ValueError(f"{panel_path}: {panel_verdict['reasons']}")
    media = []
    if schedule.encode_mp4:
        media.append({"kind": "mp4", "path": output_dir / "clip.mp4", "command": None})
    if schedule.encode_gif:
        media.append({"kind": "gif", "path": output_dir / "clip.gif", "command": None})
    media_report = _validate_media(schedule, media, ffmpeg, ffprobe)
    metadata_path = output_dir / "metadata.json"
    metadata = _read_json(metadata_path)
    if metadata.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"{metadata_path}: unexpected schema")
    if metadata.get("kind") != "capture_result" or not metadata.get("pass"):
        raise ValueError(f"{metadata_path}: capture metadata is not successful")
    expected_command = build_demo_command(schedule, demo, output_dir)
    if timeline.get("headless_exact_fbf_fail_fast", {}).get("legacy_artifact"):
        expected_command = _legacy_demo_command(expected_command)
    plan = metadata.get("schedule", {})
    if plan.get("id") != schedule.id or plan.get("scene") != schedule.scene:
        raise ValueError(f"{metadata_path}: schedule identity mismatch")
    _validate_capture_claim_boundary(metadata, schedule, metadata_path=metadata_path)
    if plan.get("demo_argv") != expected_command:
        raise ValueError(f"{metadata_path}: scheduled demo command is stale")
    runtime = metadata.get("runtime", {})
    _validate_runtime_visual_resources(schedule, runtime, metadata_path=metadata_path)
    if runtime.get("demo_path") != str(demo):
        raise ValueError(f"{metadata_path}: requested demo path is not bound")
    if runtime.get("demo_argv") != expected_command:
        raise ValueError(f"{metadata_path}: runtime demo command is stale")
    if runtime.get("demo_sha256") != _sha256(demo):
        raise ValueError(f"{metadata_path}: requested demo hash changed")

    stored_timeline = metadata.get("timeline_validation", {})
    if stored_timeline != timeline:
        raise ValueError(f"{metadata_path}: timeline validation binding changed")
    if _is_author_painleve_schedule(schedule):
        current_outcome = timeline.get("dart_adapter_outcome")
        if metadata.get("current_dart_adapter_outcome") != current_outcome:
            raise ValueError(
                f"{metadata_path}: current DART adapter outcome binding changed"
            )

    stored_panel = metadata.get("panel_validation", {})
    if stored_panel.get("path") != str(panel_path):
        raise ValueError(f"{metadata_path}: panel path is stale")
    if stored_panel.get("sha256") != _sha256(panel_path):
        raise ValueError(f"{metadata_path}: panel hash changed")
    expected_panel_sources = [
        {
            "path": str(_panel_frame_path(output_dir, step)),
            "sha256": _sha256(_panel_frame_path(output_dir, step)),
        }
        for step in schedule.panel_steps
    ]
    if stored_panel.get("source_frames") != expected_panel_sources:
        raise ValueError(f"{metadata_path}: panel source-frame binding changed")
    compose_manifest_path = output_dir / "panel.compose.json"
    if (
        stored_panel.get("compose_manifest_path") != str(compose_manifest_path)
        or stored_panel.get("compose_manifest_sha256") != _sha256(compose_manifest_path)
        or stored_panel.get("compose_manifest") != _read_json(compose_manifest_path)
    ):
        raise ValueError(f"{metadata_path}: panel composition metadata changed")

    stored_media = {
        item.get("kind"): item for item in metadata.get("media_validation", [])
    }
    if set(stored_media) != {item["kind"] for item in media_report}:
        raise ValueError(f"{metadata_path}: media kinds changed")
    for item in media_report:
        stored = stored_media[item["kind"]]
        if stored.get("path") != item["path"] or stored.get("sha256") != item["sha256"]:
            raise ValueError(f"{metadata_path}: {item['kind']} hash binding changed")
    return {
        "schedule": schedule.id,
        "timeline": timeline,
        "panel": panel_verdict,
        "media": media_report,
        "metadata_path": str(metadata_path),
        "metadata_sha256": _sha256(metadata_path),
        "pass": True,
    }


def _verify_existing_group(
    group: GroupOutputSpec,
    schedules: Sequence[CaptureSchedule],
    *,
    output_root: Path,
    ffmpeg: Path,
    ffprobe: Path,
) -> dict[str, Any]:
    output_root = output_root.resolve()
    contract = _group_member_contract(
        group, schedules, output_root=output_root, ffprobe=ffprobe
    )
    output_dir = output_root / "groups" / group.id
    metadata_path = output_dir / "metadata.json"
    metadata = _read_json(metadata_path)
    if metadata.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"{metadata_path}: unexpected schema")
    if metadata.get("kind") != "group_capture_result" or not metadata.get("pass"):
        raise ValueError(f"{metadata_path}: group capture is not successful")
    if (
        metadata.get("group_id") != group.id
        or metadata.get("layout") != group.layout
        or metadata.get("member_order") != list(group.members)
        or metadata.get("labels") != list(group.labels)
    ):
        raise ValueError(f"{metadata_path}: group identity/order/labels changed")
    _validate_group_claim_boundary(metadata, group, metadata_path=metadata_path)
    if _is_author_painleve_solver_comparison_group(group) and metadata.get(
        "current_dart_adapter_outcome_comparison"
    ) != contract.get("dart_adapter_outcome_comparison"):
        raise ValueError(
            f"{metadata_path}: current DART adapter comparison binding changed"
        )

    stored_members = metadata.get("members", [])
    if [member.get("id") for member in stored_members] != list(group.members):
        raise ValueError(f"{metadata_path}: stored members are out of source order")
    member_binding_keys = (
        "metadata_path",
        "metadata_sha256",
        "demo_path",
        "demo_sha256",
        "timeline_path",
        "timeline_sha256",
        "panel_source_frame",
        "panel_source_frame_sha256",
        "clip_path",
        "clip_sha256",
    )
    if group.panel_steps is not None:
        member_binding_keys += (
            "panel_source_step",
            "panel_source_time_seconds",
        )
    for stored, current in zip(stored_members, contract["members"]):
        for key in member_binding_keys:
            if stored.get(key) != current.get(key):
                raise ValueError(
                    f"{metadata_path}: member {current['id']} {key} binding changed"
                )

    synchronization = metadata.get("synchronization", {})
    for key in (
        "total_steps",
        "video_steps",
        "output_fps",
        "frame_count",
        "frame_rate",
    ):
        if synchronization.get(key) != contract[key]:
            raise ValueError(f"{metadata_path}: synchronization {key} changed")
    if (
        abs(
            synchronization.get("duration_seconds", -1.0) - contract["duration_seconds"]
        )
        > contract["duration_tolerance_seconds"]
    ):
        raise ValueError(f"{metadata_path}: synchronized duration changed")

    panel = metadata.get("panel_validation", {})
    panel_path = output_dir / "panel.png"
    if panel.get("path") != str(panel_path) or panel.get("sha256") != _sha256(
        panel_path
    ):
        raise ValueError(f"{metadata_path}: group panel hash changed")
    if panel.get("member_order") != list(group.members):
        raise ValueError(f"{metadata_path}: group panel member order changed")
    if panel.get("labels") != list(group.resolved_panel_labels):
        raise ValueError(f"{metadata_path}: group panel labels changed")
    if group.panel_step is not None:
        if panel.get("panel_step") != group.panel_step:
            raise ValueError(f"{metadata_path}: shared group panel step changed")
        expected_sources = [
            {
                "path": member["panel_source_frame"],
                "sha256": member["panel_source_frame_sha256"],
            }
            for member in contract["members"]
        ]
    else:
        if "panel_step" in panel:
            raise ValueError(
                f"{metadata_path}: outcome-aware group panel claims one shared step"
            )
        expected_sources = [
            {
                "member": member["id"],
                "step": member["panel_source_step"],
                "time_seconds": member["panel_source_time_seconds"],
                "path": member["panel_source_frame"],
                "sha256": member["panel_source_frame_sha256"],
            }
            for member in contract["members"]
        ]
    if panel.get("source_frames") != expected_sources:
        raise ValueError(f"{metadata_path}: group panel source binding changed")
    tiles = panel.get("cropped_tiles", [])
    if len(tiles) != len(group.members):
        raise ValueError(f"{metadata_path}: group panel tiles are missing")
    expected_tile_paths = [
        str(output_dir / "panel_members" / f"{member}.png") for member in group.members
    ]
    if [tile.get("path") for tile in tiles] != expected_tile_paths:
        raise ValueError(f"{metadata_path}: group panel tiles are out of source order")
    for tile in tiles:
        if _sha256(Path(tile["path"])) != tile.get("sha256"):
            raise ValueError(f"{metadata_path}: group panel tile hash changed")
    rows = panel.get("row_compositions", [])
    expected_row_count = 2 if group.layout == "2x2" else 1
    if len(rows) != expected_row_count:
        raise ValueError(f"{metadata_path}: row compositions are missing")
    flattened_inputs: list[str] = []
    flattened_labels: list[str] = []
    for row in rows:
        manifest_path = Path(row["manifest_path"])
        if (
            row.get("manifest_sha256") != _sha256(manifest_path)
            or row.get("manifest") != _read_json(manifest_path)
            or row.get("sha256") != _sha256(Path(row["path"]))
        ):
            raise ValueError(f"{metadata_path}: row composition binding changed")
        flattened_inputs.extend(row["manifest"].get("inputs", []))
        flattened_labels.extend(row["manifest"].get("labels", []))
    if flattened_inputs != expected_tile_paths or flattened_labels != list(
        group.resolved_panel_labels
    ):
        raise ValueError(f"{metadata_path}: row composition source order changed")
    panel_verdict = build_verdict(panel_path)
    if not panel_verdict["pass"]:
        raise ValueError(f"{panel_path}: {panel_verdict['reasons']}")
    _run(
        [
            str(ffmpeg),
            "-hide_banner",
            "-loglevel",
            "error",
            "-i",
            str(panel_path),
            "-f",
            "null",
            os.devnull,
        ]
    )

    media = metadata.get("media_validation", {})
    clip_path = output_dir / "clip.mp4"
    if media.get("path") != str(clip_path) or media.get("sha256") != _sha256(clip_path):
        raise ValueError(f"{metadata_path}: group MP4 hash changed")
    probe = _probe_media(clip_path, ffprobe)
    stream = _media_stream(probe, label=str(clip_path))
    expected_width = contract["tile_width"] * 2
    expected_height = (contract["tile_height"] + 24) * (
        2 if group.layout == "2x2" else 1
    )
    if (stream.get("width"), stream.get("height")) != (
        expected_width,
        expected_height,
    ):
        raise ValueError(f"{clip_path}: unexpected group-video dimensions")
    _validate_stream_contract(
        stream,
        {
            "width": expected_width,
            "height": expected_height,
            "frame_rate": Fraction(contract["frame_rate"]),
            "frame_count": contract["frame_count"],
        },
        label=str(clip_path),
    )
    _validate_mp4_stream_contract(stream, label=str(clip_path))
    duration = float(probe.get("format", {}).get("duration", 0.0))
    if (
        abs(duration - contract["duration_seconds"])
        > contract["duration_tolerance_seconds"]
    ):
        raise ValueError(f"{clip_path}: duration differs from synchronized members")
    _run(
        [
            str(ffmpeg),
            "-hide_banner",
            "-loglevel",
            "error",
            "-i",
            str(clip_path),
            "-f",
            "null",
            os.devnull,
        ]
    )
    return {
        "group_id": group.id,
        "member_order": list(group.members),
        "panel": {"path": str(panel_path), "sha256": _sha256(panel_path)},
        "media": {"path": str(clip_path), "sha256": _sha256(clip_path), "probe": probe},
        "metadata_path": str(metadata_path),
        "metadata_sha256": _sha256(metadata_path),
        "pass": True,
    }


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        if args.command == "audit-source":
            payload = audit_sources(
                video=args.video,
                teaser=args.teaser,
                paper=args.paper,
                ffmpeg=args.ffmpeg,
                ffprobe=args.ffprobe,
            )
        else:
            source_schedules = _select_schedules(args.scenario)
            schedules, schedule_lane_skips = _resolve_solver_lanes(
                source_schedules, args.solver_lane
            )
            candidate_groups, group_lane_skips = _group_outputs_for_solver_lane(
                args.solver_lane
            )
            candidate_groups.extend(
                _solver_comparison_groups(source_schedules, args.solver_lane)
            )
            selected_ids = {schedule.id for schedule in schedules}
            selected_source_ids = {schedule.id for schedule in source_schedules}
            groups = [
                group
                for group in candidate_groups
                if set(group.members).issubset(selected_ids)
            ]
            relevant_group_lane_skips = [
                skip
                for skip in group_lane_skips
                if set(GROUP_OUTPUTS[skip["group_id"]].members).issubset(
                    selected_source_ids
                )
            ]
            solver_lane_skips = [
                *schedule_lane_skips,
                *relevant_group_lane_skips,
            ]
            if args.command == "plan" or (args.command == "run" and args.dry_run):
                payload = build_plan(
                    schedules,
                    args.demo,
                    args.output_root,
                    solver_lane=args.solver_lane,
                    groups=groups,
                    solver_lane_skips=solver_lane_skips,
                )
            elif args.command == "run":
                results = []
                failures = []
                for schedule in schedules:
                    try:
                        results.append(
                            run_schedule(
                                schedule,
                                demo=args.demo,
                                output_root=args.output_root,
                                ffmpeg=args.ffmpeg,
                                ffprobe=args.ffprobe,
                                python=args.python,
                                allow_long=args.allow_long,
                            )
                        )
                    except (
                        OSError,
                        ValueError,
                        subprocess.CalledProcessError,
                    ) as error:
                        failures.append({"schedule": schedule.id, "error": str(error)})
                        if not args.keep_going:
                            raise
                successful_ids = {
                    result.get("schedule", {}).get("id") for result in results
                }
                group_results = []
                group_skips = []
                for group in groups:
                    failed_members = sorted(set(group.members) - successful_ids)
                    if failed_members:
                        (
                            _group_output_dir(args.output_root.resolve(), group)
                            / "metadata.json"
                        ).unlink(missing_ok=True)
                        group_skips.append(
                            {
                                "group_id": group.id,
                                "reason": "not all selected members succeeded",
                                "failed_members": failed_members,
                            }
                        )
                        continue
                    try:
                        group_results.append(
                            run_group_output(
                                group,
                                [_schedule_for_id(member) for member in group.members],
                                output_root=args.output_root,
                                ffmpeg=args.ffmpeg,
                                ffprobe=args.ffprobe,
                                python=args.python,
                            )
                        )
                    except (
                        OSError,
                        ValueError,
                        subprocess.CalledProcessError,
                    ) as error:
                        failures.append({"group": group.id, "error": str(error)})
                        if not args.keep_going:
                            raise
                payload = {
                    "schema_version": SCHEMA_VERSION,
                    "kind": "capture_run",
                    "requested_solver_lane": args.solver_lane,
                    "solver_lane_skips": solver_lane_skips,
                    "results": results,
                    "group_outputs": group_results,
                    "group_skips": group_skips,
                    "failures": failures,
                    "pass": not failures,
                }
            else:
                results = [
                    _verify_existing(
                        schedule,
                        demo=args.demo,
                        output_root=args.output_root,
                        ffmpeg=args.ffmpeg,
                        ffprobe=args.ffprobe,
                    )
                    for schedule in schedules
                ]
                group_results = [
                    _verify_existing_group(
                        group,
                        [_schedule_for_id(member) for member in group.members],
                        output_root=args.output_root,
                        ffmpeg=args.ffmpeg,
                        ffprobe=args.ffprobe,
                    )
                    for group in groups
                ]
                payload = {
                    "schema_version": SCHEMA_VERSION,
                    "kind": "verification",
                    "requested_solver_lane": args.solver_lane,
                    "solver_lane_skips": solver_lane_skips,
                    "results": results,
                    "group_outputs": group_results,
                    "pass": True,
                }
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    output_path = getattr(args, "out", None)
    if output_path is not None:
        _write_json(output_path, payload)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload.get("pass", False) else 1


if __name__ == "__main__":
    raise SystemExit(main())
