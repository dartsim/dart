#!/usr/bin/env python3
"""Validate PLAN-104's fail-closed VBD and AVBD paper-parity contracts."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from collections import Counter
from datetime import date
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
CONTRACT_DIR = REPO_ROOT / "docs" / "plans" / "104-vertex-block-descent-solver"
MATRIX_PATH = CONTRACT_DIR / "paper-parity-matrix.md"
DEFAULT_CONTRACTS = (
    CONTRACT_DIR / "vbd-paper-coverage-contract.json",
    CONTRACT_DIR / "avbd-paper-coverage-contract.json",
)

ALLOWED_STATUSES = frozenset({"missing", "partial", "blocked", "complete"})
KNOWN_PREDICATES = (
    "artifact_valid",
    "solver_contract_valid",
    "physical_outcome_valid",
    "manual_inspected",
    "media_decoded",
    "performance_comparable",
    "claim_valid",
)
METHOD_PREDICATES = (
    "artifact_valid",
    "solver_contract_valid",
    "physical_outcome_valid",
    "performance_comparable",
    "claim_valid",
)
VISUAL_PREDICATES = (
    "artifact_valid",
    "solver_contract_valid",
    "physical_outcome_valid",
    "manual_inspected",
    "media_decoded",
    "performance_comparable",
    "claim_valid",
)
TABLE_PREDICATES = (
    "artifact_valid",
    "solver_contract_valid",
    "physical_outcome_valid",
    "manual_inspected",
    "performance_comparable",
    "claim_valid",
)
REQUIRED_BACKENDS = ("cpu", "cuda")
MATRIX_LABELS = {
    "vbd": {
        "method": "VBD method",
        "limitations": "VBD limitations",
        "paper_figures": "VBD paper figures",
        "paper_tables": "VBD paper tables",
        "paper_video": "VBD official video",
        "gaia_dynamics": "VBD Gaia dynamics",
        "gaia_cloth": "VBD Gaia cloth",
        "tinyvbd": "VBD TinyVBD",
        "project_page": "VBD project page",
    },
    "avbd": {
        "method": "AVBD method",
        "paper_figures": "AVBD paper figures",
        "paper_tables": "AVBD paper tables",
        "paper_video": "AVBD official video",
        "demo2d": "AVBD demo2d",
        "demo3d": "AVBD demo3d",
        "project_page": "AVBD project page",
    },
}


def _numbered_ids(prefix: str, count: int) -> tuple[str, ...]:
    return tuple(f"{prefix}.{index:02d}" for index in range(1, count + 1))


VBD_GROUPS = (
    (
        "method",
        "method",
        METHOD_PREDICATES,
        tuple(
            f"vbd.method.{name}"
            for name in (
                "variational_backward_euler",
                "local_vertex_block_newton",
                "local_spd_solve",
                "stable_neo_hookean",
                "rayleigh_damping",
                "soft_constraints",
                "penalty_contact",
                "lagged_friction",
                "adaptive_initialization",
                "chebyshev_acceleration",
                "vertex_coloring",
                "dynamic_collision_double_buffer",
                "topology_change",
                "gpu_hierarchical_kernel",
                "particle_blocks",
                "rigid_six_dof_blocks",
                "unified_simulation",
            )
        ),
    ),
    (
        "limitations",
        "limitation",
        VISUAL_PREDICATES,
        tuple(
            f"vbd.limit.{name}"
            for name in (
                "local_information_propagation",
                "high_stiffness_ratio",
                "penetration_contact",
                "codimensional_self_collision",
            )
        ),
    ),
    (
        "paper_figures",
        "paper_figure",
        VISUAL_PREDICATES,
        _numbered_ids("vbd.paper.fig", 24),
    ),
    ("paper_tables", "paper_table", TABLE_PREDICATES, ("vbd.paper.table.01",)),
    (
        "paper_video",
        "paper_video_segment",
        VISUAL_PREDICATES,
        tuple(
            f"vbd.video.{name}"
            for name in (
                "01_title",
                "02_216_squishy_balls",
                "03_10368_models",
                "04_twisting_beams",
                "05_random_initialization",
                "06_flattening_initialization",
                "07_large_residual",
                "08_extreme_stretch",
                "09_elasticity_convergence",
                "10_incline_friction",
                "11_octopus_friction",
                "12_damping",
                "13_topology_change",
                "14_collision_recoloring",
                "15_xpbd_mass_ratio_collision",
                "16_xpbd_timestep",
                "17_particle_mass_ratio",
                "18_rigid_chain",
                "19_rigid_teapot",
            )
        ),
    ),
    (
        "gaia_dynamics",
        "reference_scene",
        VISUAL_PREDICATES,
        tuple(
            f"vbd.gaia.dynamics.{name}"
            for name in (
                "hybrid_64_experiment",
                "hybrid_64",
                "drop_two_squishy_balls",
                "twist_two_beams",
                "many_hybrid_models",
                "drop_216_squishy_balls",
                "friction_cube_mu_0_1",
                "friction_cube_mu_0_5",
                "friction_cube_mu_0_6",
                "friction_cube_mu_0_9",
                "damping_beam_0",
                "damping_beam_2e_5",
                "damping_beam_5e_5",
                "random_teapot",
            )
        ),
    ),
    (
        "gaia_cloth",
        "reference_scene",
        VISUAL_PREDICATES,
        tuple(
            f"vbd.gaia.cloth.{name}"
            for name in (
                "twist_c100",
                "twist_c50",
                "twist_c50_thick",
                "knot_pulling",
                "multilayer_collider",
            )
        ),
    ),
    (
        "tinyvbd",
        "reference_scene",
        VISUAL_PREDICATES,
        ("vbd.tinyvbd.tilted_strand",),
    ),
    (
        "project_page",
        "project_surface",
        VISUAL_PREDICATES,
        (
            "vbd.project.teaser",
            "vbd.project.random_teapot_stability",
            "vbd.project.flattened_armadillo_stability",
        ),
    ),
)

AVBD_GROUPS = (
    (
        "method",
        "method",
        METHOD_PREDICATES,
        tuple(
            f"avbd.method.{name}"
            for name in (
                "augmented_lagrangian_rows",
                "hard_equality_inequality",
                "finite_stiffness_ramping",
                "force_bounds",
                "friction_static_dynamic",
                "alpha_error_regularization",
                "gamma_warm_start",
                "quasi_newton_hessian",
                "rigid_six_dof_blocks",
                "vertex_body_coloring",
                "parallel_dual_stiffness_pass",
                "contact_persistence",
                "joints_and_attachments",
                "motors",
                "fracture",
                "unified_rigid_soft",
                "cpu_solver",
                "cuda_solver",
                "parameter_defaults",
                "baseline_comparisons",
            )
        ),
    ),
    (
        "paper_figures",
        "paper_figure",
        VISUAL_PREDICATES,
        _numbered_ids("avbd.paper.fig", 15),
    ),
    (
        "paper_tables",
        "paper_table",
        TABLE_PREDICATES,
        _numbered_ids("avbd.paper.table", 2),
    ),
    (
        "paper_video",
        "paper_video_segment",
        VISUAL_PREDICATES,
        tuple(
            f"avbd.video.{name}"
            for name in (
                "01_110k_block_pile",
                "02_510k_block_pile",
                "03_unified_cloth_rigid",
                "04_high_stiffness",
                "05_flag_pole",
                "06_card_tower",
                "07_pendulum",
                "08_two_heavy_balls",
                "09_box_stack",
                "10_friction",
                "11_chain_mail",
                "12_breakable_wall",
                "13_parameter_sweeps",
                "14_closing",
            )
        ),
    ),
    (
        "demo2d",
        "reference_scene",
        VISUAL_PREDICATES,
        tuple(
            f"avbd.demo2d.{name}"
            for name in (
                "empty",
                "ground",
                "dynamic_friction",
                "static_friction",
                "pyramid",
                "cards",
                "rope",
                "heavy_rope",
                "hanging_rope",
                "spring",
                "spring_ratio",
                "stack",
                "stack_ratio",
                "rod",
                "soft_body",
                "joint_grid",
                "net",
                "motor",
                "fracture",
            )
        ),
    ),
    (
        "demo3d",
        "reference_scene",
        VISUAL_PREDICATES,
        tuple(
            f"avbd.demo3d.{name}"
            for name in (
                "empty",
                "ground",
                "dynamic_friction",
                "static_friction",
                "pyramid",
                "rope",
                "heavy_rope",
                "spring",
                "spring_ratio",
                "stack",
                "stack_ratio",
                "soft_body",
                "bridge",
                "breakable",
            )
        ),
    ),
    (
        "project_page",
        "project_surface",
        VISUAL_PREDICATES,
        (
            "avbd.project.teaser",
            "avbd.project.online_demo2d",
            "avbd.project.online_demo3d",
            "avbd.project.contact_comparison",
        ),
    ),
)

EXPECTED_GROUPS = {"vbd": VBD_GROUPS, "avbd": AVBD_GROUPS}
EXPECTED_SCHEMAS = {
    "vbd": "dart.vbd_paper_coverage_contract/v1",
    "avbd": "dart.avbd_paper_coverage_contract/v1",
}
EXPECTED_PREDICATE_DEFINITIONS = {
    family: {
        "artifact_valid": (
            "The evidence schema, source/build identity, hashes, ordering, "
            "dimensions, frames, and replay inputs validate."
        ),
        "solver_contract_valid": (
            f"The intended {family.upper()} solver and backend ran every "
            "required step without fallback, invalid state, or an undisclosed "
            "cap."
        ),
        "physical_outcome_valid": (
            "A scenario-specific quantitative oracle passes for the declared "
            "duration and accuracy target."
        ),
        "manual_inspected": (
            "A current-build render or plot was actually inspected and records "
            "the expected qualitative behavior."
        ),
        "media_decoded": (
            "Required image/video media decodes for its declared duration and "
            "is bound to the validated run."
        ),
        "performance_comparable": (
            "Scene, precision, backend, hardware, compiler, timer boundary, "
            "warmup, aggregation, and achieved accuracy are matched."
        ),
        "claim_valid": (
            "Every predicate required by the row passes and the resulting "
            "claim stays within the demonstrated boundary."
        ),
    }
    for family in ("vbd", "avbd")
}
EXPECTED_LOCATOR_DIGESTS = {
    "vbd": "0a9746e4b3c5af8f6207d4c210670116dfab89b6b2a34009549183e6ae0774b7",
    "avbd": "3830039626580ca628c60cceeda942fd811f174e6b640351bd62d405519d67cd",
}

EXPECTED_SOURCE_VALUES = {
    "vbd": {
        (
            "paper",
            "url",
        ): "https://graphics.cs.utah.edu/research/projects/vbd/vbd-siggraph2024.pdf",
        (
            "paper",
            "sha256",
        ): "4ba65c4e49e8e8740aca491c2bf466b6eeb6f8824a152c12d1f7800828959132",
        ("paper", "size_bytes"): 20000116,
        ("project_page", "url"): "https://graphics.cs.utah.edu/research/projects/vbd/",
        (
            "project_page",
            "sha256",
        ): "2c98c3158d94f2d3d6193381e7f266507f7a722c1ae3c6777786d98ccd84c16f",
        ("project_page", "size_bytes"): 7886,
        (
            "project_assets",
            "teaser",
            "url",
        ): "https://graphics.cs.utah.edu/research/projects/vbd/teaser.mp4",
        (
            "project_assets",
            "teaser",
            "sha256",
        ): "096533af2e9f116815b551f133da58b5f059c6724da6b80d9041859b715dda65",
        ("project_assets", "teaser", "size_bytes"): 34479245,
        ("project_assets", "teaser", "duration_seconds"): 23.366667,
        ("project_assets", "teaser", "width"): 1920,
        ("project_assets", "teaser", "height"): 1080,
        ("project_assets", "teaser", "frame_rate"): "60/1",
        ("project_assets", "teaser", "frames"): 1402,
        (
            "project_assets",
            "random_teapot_stability",
            "url",
        ): "https://graphics.cs.utah.edu/research/projects/vbd/teapot-small.gif",
        (
            "project_assets",
            "random_teapot_stability",
            "sha256",
        ): "429101b99fdbab5d8b5b958d03b3f11a5ff34bdcc49a9054f4d4501496c5d94c",
        ("project_assets", "random_teapot_stability", "size_bytes"): 3284707,
        ("project_assets", "random_teapot_stability", "duration_seconds"): 2.72,
        ("project_assets", "random_teapot_stability", "width"): 480,
        ("project_assets", "random_teapot_stability", "height"): 270,
        ("project_assets", "random_teapot_stability", "frames"): 68,
        (
            "project_assets",
            "flattened_armadillo_stability",
            "url",
        ): "https://graphics.cs.utah.edu/research/projects/vbd/armadillo-small.gif",
        (
            "project_assets",
            "flattened_armadillo_stability",
            "sha256",
        ): "451ac464a728e6d273ac17350f7b97eced3a3970df8b810309cd90e2f7479c6f",
        ("project_assets", "flattened_armadillo_stability", "size_bytes"): 3283913,
        ("project_assets", "flattened_armadillo_stability", "duration_seconds"): 3.28,
        ("project_assets", "flattened_armadillo_stability", "width"): 200,
        ("project_assets", "flattened_armadillo_stability", "height"): 270,
        ("project_assets", "flattened_armadillo_stability", "frames"): 164,
        ("paper_video", "url"): "https://www.youtube.com/watch?v=2HCgKfKy3W8",
        ("paper_video", "youtube_id"): "2HCgKfKy3W8",
        ("paper_video", "duration_seconds"): 331,
        ("paper_video", "audited_format"): "YouTube format 18, 640x360",
        (
            "paper_video",
            "sha256",
        ): "fe5e669c2260a507735cd77e10926a3969fd78fee8c983b328274c25ac5682af",
        ("repositories", "gaia", "url"): "https://github.com/AnkaChan/Gaia",
        (
            "repositories",
            "gaia",
            "revision",
        ): "c229692045465a76233f9fba9197fb22bbfb3694",
        ("repositories", "tinyvbd", "url"): "https://github.com/AnkaChan/TinyVBD",
        (
            "repositories",
            "tinyvbd",
            "revision",
        ): "dcd011a5d945172e247ecced90a6c2c4b4313520",
    },
    "avbd": {
        (
            "paper",
            "url",
        ): "https://graphics.cs.utah.edu/research/projects/avbd/Augmented_VBD-SIGGRAPH25.pdf",
        (
            "paper",
            "sha256",
        ): "7957d116b9130cfb0aa5a48ab7cd0d74a64ad79f75c99acd291bdece2be3f2d6",
        ("paper", "size_bytes"): 13803273,
        ("project_page", "url"): "https://graphics.cs.utah.edu/research/projects/avbd/",
        (
            "project_page",
            "sha256",
        ): "602525cc0a21b32b71d742ea38be27f68d6672b7c6968397a2ac44c21a5d3aac",
        ("project_page", "size_bytes"): 10265,
        (
            "project_page",
            "online_demo2d_url",
        ): "https://graphics.cs.utah.edu/research/projects/avbd/avbd_demo2d.html",
        (
            "project_page",
            "online_demo2d_sha256",
        ): "89bd971d6bd0f3b6b79ae1cd3d7a6215d15852dd5f06bd65d7f338a00171e13e",
        ("project_page", "online_demo2d_size_bytes"): 1599470,
        (
            "project_page",
            "online_demo3d_url",
        ): "https://graphics.cs.utah.edu/research/projects/avbd/avbd_demo3d.html",
        (
            "project_page",
            "online_demo3d_sha256",
        ): "64c10c6d63474b0d9a7e14e7778390baca964ab4f95d76aa5c3adef12ba0cf3d",
        ("project_page", "online_demo3d_size_bytes"): 1624308,
        (
            "project_assets",
            "teaser",
            "url",
        ): "https://graphics.cs.utah.edu/research/projects/avbd/teaser.mp4",
        (
            "project_assets",
            "teaser",
            "sha256",
        ): "ab7e34bdb4dd83198a4f6a38634ae7d878a7f2ecb381f7437b75d9ef41145b24",
        ("project_assets", "teaser", "size_bytes"): 15620043,
        ("project_assets", "teaser", "duration_seconds"): 20.733333,
        ("project_assets", "teaser", "width"): 1920,
        ("project_assets", "teaser", "height"): 1080,
        ("project_assets", "teaser", "frame_rate"): "30/1",
        ("project_assets", "teaser", "frames"): 622,
        (
            "project_assets",
            "contact_comparison",
            "url",
        ): "https://graphics.cs.utah.edu/research/projects/avbd/chainmail.mp4",
        (
            "project_assets",
            "contact_comparison",
            "sha256",
        ): "5a9b6e40aabf49b9341cb00fe1c8f3972a973156394b5edad1492a26dfdddd36",
        ("project_assets", "contact_comparison", "size_bytes"): 8972724,
        ("project_assets", "contact_comparison", "duration_seconds"): 6.583333,
        ("project_assets", "contact_comparison", "width"): 1920,
        ("project_assets", "contact_comparison", "height"): 1080,
        ("project_assets", "contact_comparison", "frame_rate"): "60/1",
        ("project_assets", "contact_comparison", "frames"): 395,
        (
            "associated_publication",
            "url",
        ): "https://graphics.cs.utah.edu/research/projects/avbd/Augmented_VBD-SIGGRAPH25_RTL.pdf",
        (
            "associated_publication",
            "sha256",
        ): "00b57241602fb8f5428d0b1effa2e084598d796de731a62cd04ea1047dfbe5af",
        ("associated_publication", "size_bytes"): 2395194,
        ("associated_publication", "pages"): 2,
        (
            "associated_publication",
            "role",
        ): "Real-Time Live extended abstract whose equations, two figures, and performance annotations are a subset of the canonical AVBD paper and project teaser.",
        ("paper_video", "url"): "https://www.youtube.com/watch?v=bwJgifqvd5M",
        ("paper_video", "youtube_id"): "bwJgifqvd5M",
        ("paper_video", "duration_seconds"): 279,
        ("paper_video", "audited_format"): "YouTube format 18, 640x360",
        (
            "paper_video",
            "sha256",
        ): "2f685b9214c8afd1b35bee54c76c80aaa2d69ac5497cfdef5d229be3292e420d",
        (
            "noncanonical_explainer",
            "url",
        ): "https://www.youtube.com/watch?v=TzIKbjuSy2A",
        ("noncanonical_explainer", "youtube_id"): "TzIKbjuSy2A",
        ("noncanonical_explainer", "duration_seconds"): 379,
        (
            "noncanonical_explainer",
            "role",
        ): "Third-party explainer linked by the project page; it reuses source media and is not an independent parity surface.",
        (
            "repositories",
            "avbd_demo2d",
            "url",
        ): "https://github.com/savant117/avbd-demo2d",
        (
            "repositories",
            "avbd_demo2d",
            "revision",
        ): "74699a11f8586d3ac34474c92b1ef8feb5f587de",
        (
            "repositories",
            "avbd_demo3d",
            "url",
        ): "https://github.com/savant117/avbd-demo3d",
        (
            "repositories",
            "avbd_demo3d",
            "revision",
        ): "7701bd427d55ca5d03ea1fdf331912ded9169f4b",
    },
}

EXPECTED_VIDEO_RANGES = {
    "vbd": (
        (0.0, 6.6),
        (6.6, 42.733),
        (42.733, 86.8),
        (86.8, 108.3),
        (108.3, 122.8),
        (122.8, 142.033),
        (142.033, 154.933),
        (154.933, 175.9),
        (175.9, 196.033),
        (196.033, 206.967),
        (206.967, 222.533),
        (222.533, 238.067),
        (238.067, 250.367),
        (250.367, 258.933),
        (258.933, 278.167),
        (278.167, 291.733),
        (291.733, 303.633),
        (303.633, 315.533),
        (315.533, 331.0),
    ),
    "avbd": (
        (0.0, 31.033),
        (31.033, 55.633),
        (55.633, 93.167),
        (93.167, 102.933),
        (102.933, 121.133),
        (121.133, 132.9),
        (132.9, 157.567),
        (157.567, 171.567),
        (171.567, 182.033),
        (182.033, 191.8),
        (191.8, 199.2),
        (199.2, 239.667),
        (239.667, 272.633),
        (272.633, 279.0),
    ),
}

EXPECTED_PROJECT_RANGES = {
    "vbd": {
        "vbd.project.teaser": (0.0, 23.366667),
        "vbd.project.random_teapot_stability": (0.0, 2.72),
        "vbd.project.flattened_armadillo_stability": (0.0, 3.28),
    },
    "avbd": {
        "avbd.project.teaser": (0.0, 20.733333),
        "avbd.project.contact_comparison": (0.0, 6.583333),
    },
}


def _value_at(value: Any, path: tuple[str, ...]) -> Any:
    for key in path:
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def _source_locator_digest(groups: list[Any]) -> str:
    inventory = [
        [requirement.get("id"), requirement.get("source_locator")]
        for group in groups
        if isinstance(group, dict)
        for requirement in group.get("requirements", ())
        if isinstance(requirement, dict)
    ]
    serialized = json.dumps(inventory, ensure_ascii=False, separators=(",", ":"))
    return hashlib.sha256(f"{serialized}\n".encode()).hexdigest()


def _list_of_nonempty_strings(value: Any) -> bool:
    return isinstance(value, list) and all(
        isinstance(item, str) and bool(item.strip()) for item in value
    )


def _result_map_errors(
    value: Any,
    *,
    allowed_keys: tuple[str, ...],
    label: str,
    require_all_true: bool,
) -> list[str]:
    if value is None:
        return [f"{label} is required"] if require_all_true else []
    if not isinstance(value, dict):
        return [f"{label} must be an object"]

    errors: list[str] = []
    unknown = sorted(set(value) - set(allowed_keys))
    if unknown:
        errors.append(f"{label} has unknown keys: {', '.join(unknown)}")
    for key, result in value.items():
        if not isinstance(result, bool):
            errors.append(f"{label}.{key} must be boolean")
    if require_all_true:
        if tuple(value) != allowed_keys:
            errors.append(
                f"{label} must contain exactly, in order: {', '.join(allowed_keys)}"
            )
        false_or_missing = [key for key in allowed_keys if value.get(key) is not True]
        if false_or_missing:
            errors.append(f"{label} must be true for: {', '.join(false_or_missing)}")
    return errors


def _evidence_path_errors(requirement_id: str, evidence: Any) -> list[str]:
    if not _list_of_nonempty_strings(evidence):
        return [f"{requirement_id}: evidence must be a list of non-empty paths"]

    errors: list[str] = []
    for raw_path in evidence:
        path = Path(raw_path)
        if path.is_absolute() or ".." in path.parts:
            errors.append(
                f"{requirement_id}: evidence path must be repository-relative: "
                f"{raw_path}"
            )
            continue
        if not (REPO_ROOT / path).is_file():
            errors.append(f"{requirement_id}: evidence path does not exist: {raw_path}")
    return errors


def _requirement_errors(
    requirement: Any,
    *,
    expected_id: str,
    required_predicates: tuple[str, ...],
) -> list[str]:
    label = expected_id
    if not isinstance(requirement, dict):
        return [f"{label}: requirement must be an object"]

    errors: list[str] = []
    if requirement.get("id") != expected_id:
        errors.append(
            f"{label}: expected id {expected_id!r}, got {requirement.get('id')!r}"
        )
    source_locator = requirement.get("source_locator")
    if not isinstance(source_locator, str) or not source_locator.strip():
        errors.append(f"{label}: source_locator must be a non-empty string")

    status = requirement.get("status")
    if status not in ALLOWED_STATUSES:
        errors.append(f"{label}: invalid status {status!r}")

    evidence = requirement.get("evidence")
    blockers = requirement.get("blockers")
    errors.extend(_evidence_path_errors(label, evidence))
    if not _list_of_nonempty_strings(blockers):
        errors.append(f"{label}: blockers must be a list of non-empty strings")

    if status == "missing" and evidence:
        errors.append(f"{label}: missing rows cannot claim evidence")
    if status == "partial" and not evidence:
        errors.append(f"{label}: partial rows require evidence")
    if status != "complete" and not blockers:
        errors.append(f"{label}: incomplete rows require at least one blocker")
    if status == "complete" and blockers:
        errors.append(f"{label}: complete rows cannot retain blockers")
    if status == "complete" and not evidence:
        errors.append(f"{label}: complete rows require evidence")

    require_all_true = status == "complete"
    predicate_results = requirement.get("predicate_results")
    errors.extend(
        _result_map_errors(
            predicate_results,
            allowed_keys=required_predicates,
            label=f"{label}: predicate_results",
            require_all_true=require_all_true,
        )
    )
    errors.extend(
        _result_map_errors(
            requirement.get("backend_results"),
            allowed_keys=REQUIRED_BACKENDS,
            label=f"{label}: backend_results",
            require_all_true=require_all_true,
        )
    )
    if (
        status != "complete"
        and isinstance(predicate_results, dict)
        and predicate_results.get("claim_valid") is True
    ):
        errors.append(f"{label}: incomplete rows cannot set claim_valid true")
    return errors


def _source_errors(contract: dict[str, Any], family: str) -> list[str]:
    sources = contract.get("sources")
    if not isinstance(sources, dict):
        return [f"{family}: sources must be an object"]

    errors: list[str] = []
    for source_path, expected in EXPECTED_SOURCE_VALUES[family].items():
        actual = _value_at(sources, source_path)
        if actual != expected:
            errors.append(
                f"{family}: source {'.'.join(source_path)} must be "
                f"{expected!r}, got {actual!r}"
            )
    return errors


def _video_errors(family: str, groups_by_id: dict[str, dict[str, Any]]) -> list[str]:
    group = groups_by_id.get("paper_video")
    if not isinstance(group, dict):
        return []
    requirements = group.get("requirements")
    if not isinstance(requirements, list):
        return []

    expected_ranges = EXPECTED_VIDEO_RANGES[family]
    errors: list[str] = []
    for index, (requirement, expected_range) in enumerate(
        zip(requirements, expected_ranges, strict=False)
    ):
        label = (
            requirement.get("id", f"{family}.video[{index}]")
            if isinstance(requirement, dict)
            else f"{family}.video[{index}]"
        )
        source_seconds = (
            requirement.get("source_seconds") if isinstance(requirement, dict) else None
        )
        if (
            not isinstance(source_seconds, list)
            or len(source_seconds) != 2
            or any(
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                for value in source_seconds
            )
        ):
            errors.append(f"{label}: source_seconds must be two finite numbers")
            continue
        actual_range = tuple(float(value) for value in source_seconds)
        if actual_range != expected_range:
            errors.append(
                f"{label}: source_seconds must be {list(expected_range)}, "
                f"got {source_seconds}"
            )
        if actual_range[0] >= actual_range[1]:
            errors.append(f"{label}: source_seconds must have positive duration")
        if index and actual_range[0] != expected_ranges[index - 1][1]:
            errors.append(f"{label}: video inventory is not contiguous")

    expected_duration = float(expected_ranges[-1][1])
    if expected_ranges[0][0] != 0.0:
        errors.append(f"{family}: paper-video inventory must start at 0 seconds")
    if expected_duration != float(
        EXPECTED_SOURCE_VALUES[family][("paper_video", "duration_seconds")]
    ):
        errors.append(f"{family}: internal checker video duration mismatch")
    return errors


def _project_surface_errors(
    family: str, groups_by_id: dict[str, dict[str, Any]]
) -> list[str]:
    group = groups_by_id.get("project_page")
    if not isinstance(group, dict):
        return []
    requirements = group.get("requirements")
    if not isinstance(requirements, list):
        return []

    errors: list[str] = []
    requirements_by_id = {
        requirement.get("id"): requirement
        for requirement in requirements
        if isinstance(requirement, dict)
    }
    for requirement_id, expected_range in EXPECTED_PROJECT_RANGES[family].items():
        requirement = requirements_by_id.get(requirement_id)
        if requirement is None:
            continue
        source_seconds = requirement.get("source_seconds")
        if not isinstance(source_seconds, list) or len(source_seconds) != 2:
            errors.append(
                f"{requirement_id}: source_seconds must be " f"{list(expected_range)}"
            )
            continue
        if any(
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(value)
            for value in source_seconds
        ):
            errors.append(
                f"{requirement_id}: source_seconds must be two finite numbers"
            )
            continue
        actual_range = tuple(float(value) for value in source_seconds)
        if actual_range != expected_range:
            errors.append(
                f"{requirement_id}: source_seconds must be "
                f"{list(expected_range)}, got {source_seconds}"
            )
    return errors


def validate_contract(contract: Any, *, source_name: str = "<contract>") -> list[str]:
    if not isinstance(contract, dict):
        return [f"{source_name}: contract must be a JSON object"]

    errors: list[str] = []
    family = contract.get("solver_family")
    if family not in EXPECTED_GROUPS:
        return [
            f"{source_name}: solver_family must be one of "
            f"{', '.join(sorted(EXPECTED_GROUPS))}"
        ]

    prefix = f"{source_name} ({family})"
    if contract.get("schema_version") != EXPECTED_SCHEMAS[family]:
        errors.append(f"{prefix}: schema_version must be {EXPECTED_SCHEMAS[family]!r}")
    if contract.get("plan_id") != "PLAN-104":
        errors.append(f"{prefix}: plan_id must be 'PLAN-104'")
    if contract.get("completion_rule") != "paper-parity-matrix.md#completion-rule":
        errors.append(
            f"{prefix}: completion_rule must target "
            "'paper-parity-matrix.md#completion-rule'"
        )
    try:
        date.fromisoformat(contract.get("snapshot_date", ""))
    except TypeError, ValueError:
        errors.append(f"{prefix}: snapshot_date must be an ISO calendar date")

    predicate_definitions = contract.get("predicate_definitions")
    if not isinstance(predicate_definitions, dict):
        errors.append(f"{prefix}: predicate_definitions must be an object")
    else:
        if tuple(predicate_definitions) != KNOWN_PREDICATES:
            errors.append(
                f"{prefix}: predicate_definitions must contain exactly, in "
                f"order: {', '.join(KNOWN_PREDICATES)}"
            )
        for predicate, definition in predicate_definitions.items():
            if not isinstance(definition, str) or not definition.strip():
                errors.append(
                    f"{prefix}: predicate definition {predicate!r} must be non-empty"
                )
        for predicate, expected_definition in EXPECTED_PREDICATE_DEFINITIONS[
            family
        ].items():
            actual_definition = predicate_definitions.get(predicate)
            if actual_definition != expected_definition:
                errors.append(
                    f"{prefix}: predicate definition {predicate!r} must be "
                    f"{expected_definition!r}, got {actual_definition!r}"
                )

    errors.extend(_source_errors(contract, family))

    groups = contract.get("coverage_groups")
    if not isinstance(groups, list):
        return errors + [f"{prefix}: coverage_groups must be a list"]
    locator_digest = _source_locator_digest(groups)
    if locator_digest != EXPECTED_LOCATOR_DIGESTS[family]:
        errors.append(
            f"{prefix}: source-locator inventory digest must be "
            f"{EXPECTED_LOCATOR_DIGESTS[family]}, got {locator_digest}"
        )

    expected_groups = EXPECTED_GROUPS[family]
    actual_group_ids = [
        group.get("id") if isinstance(group, dict) else None for group in groups
    ]
    expected_group_ids = [group[0] for group in expected_groups]
    if actual_group_ids != expected_group_ids:
        errors.append(
            f"{prefix}: coverage group ids/order must be {expected_group_ids!r}, "
            f"got {actual_group_ids!r}"
        )

    all_ids: list[str] = []
    groups_by_id: dict[str, dict[str, Any]] = {}
    for index, expected_group in enumerate(expected_groups):
        group_id, expected_kind, expected_predicates, expected_ids = expected_group
        if index >= len(groups) or not isinstance(groups[index], dict):
            errors.append(f"{prefix}: missing object for group {group_id}")
            continue
        group = groups[index]
        groups_by_id[group_id] = group
        label = f"{prefix}: group {group_id}"
        if group.get("id") != group_id:
            errors.append(f"{label}: id must be {group_id!r}")
        if group.get("kind") != expected_kind:
            errors.append(f"{label}: kind must be {expected_kind!r}")
        if tuple(group.get("required_backends", ())) != REQUIRED_BACKENDS:
            errors.append(
                f"{label}: required_backends must be " f"{list(REQUIRED_BACKENDS)!r}"
            )
        if tuple(group.get("required_predicates", ())) != expected_predicates:
            errors.append(
                f"{label}: required_predicates must be "
                f"{list(expected_predicates)!r}"
            )

        requirements = group.get("requirements")
        if not isinstance(requirements, list):
            errors.append(f"{label}: requirements must be a list")
            continue
        actual_ids = [
            requirement.get("id") if isinstance(requirement, dict) else None
            for requirement in requirements
        ]
        if actual_ids != list(expected_ids):
            errors.append(
                f"{label}: requirement ids/order must be {list(expected_ids)!r}, "
                f"got {actual_ids!r}"
            )
        for requirement, expected_id in zip(requirements, expected_ids, strict=False):
            errors.extend(
                _requirement_errors(
                    requirement,
                    expected_id=expected_id,
                    required_predicates=expected_predicates,
                )
            )
        all_ids.extend(
            requirement_id
            for requirement_id in actual_ids
            if isinstance(requirement_id, str)
        )

    duplicates = sorted(
        requirement_id
        for requirement_id, count in Counter(all_ids).items()
        if count > 1
    )
    if duplicates:
        errors.append(f"{prefix}: duplicate requirement ids: {', '.join(duplicates)}")

    errors.extend(_video_errors(family, groups_by_id))
    errors.extend(_project_surface_errors(family, groups_by_id))

    statuses = [
        requirement.get("status")
        for group in groups
        if isinstance(group, dict)
        for requirement in group.get("requirements", ())
        if isinstance(requirement, dict)
    ]
    computed_complete = bool(statuses) and all(
        status == "complete" for status in statuses
    )
    expected_overall_status = "complete" if computed_complete else "incomplete"
    if contract.get("overall_status") != expected_overall_status:
        errors.append(
            f"{prefix}: overall_status must be {expected_overall_status!r} "
            "for the recorded row statuses"
        )
    return errors


def contract_errors(path: Path) -> list[str]:
    if not path.is_file():
        return [f"{path}: contract file not found"]
    try:
        contract = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        return [f"{path}: invalid JSON ({exc})"]
    return validate_contract(contract, source_name=path.name)


def contract_summary(path: Path) -> str:
    contract = json.loads(path.read_text())
    statuses = Counter(
        requirement["status"]
        for group in contract["coverage_groups"]
        for requirement in group["requirements"]
    )
    status_summary = ", ".join(
        f"{status}={statuses[status]}"
        for status in ("complete", "partial", "blocked", "missing")
        if statuses[status]
    )
    total = sum(statuses.values())
    return f"{contract['solver_family'].upper()}: {total} rows ({status_summary})"


def _matrix_table_rows(text: str) -> dict[str, tuple[int, int, int, int, int]]:
    rows: dict[str, tuple[int, int, int, int, int]] = {}
    for line in text.splitlines():
        if not line.startswith("|"):
            continue
        cells = [cell.strip().replace("**", "") for cell in line.split("|")[1:-1]]
        if len(cells) != 6:
            continue
        try:
            counts = tuple(int(cell) for cell in cells[1:])
        except ValueError:
            continue
        rows[cells[0]] = counts
    return rows


def matrix_errors(
    contracts: tuple[dict[str, Any], ...], *, matrix_path: Path = MATRIX_PATH
) -> list[str]:
    if not matrix_path.is_file():
        return [f"{matrix_path}: parity matrix not found"]
    rows = _matrix_table_rows(matrix_path.read_text())
    expected_rows: dict[str, tuple[int, int, int, int, int]] = {}
    grand_counts = Counter()

    for contract in contracts:
        family = contract["solver_family"]
        family_counts = Counter()
        for group in contract["coverage_groups"]:
            counts = Counter(
                requirement["status"] for requirement in group["requirements"]
            )
            label = MATRIX_LABELS[family][group["id"]]
            expected_rows[label] = (
                len(group["requirements"]),
                counts["partial"],
                counts["blocked"],
                counts["missing"],
                counts["complete"],
            )
            family_counts.update(counts)
        family_total = sum(family_counts.values())
        expected_rows[f"{family.upper()} total"] = (
            family_total,
            family_counts["partial"],
            family_counts["blocked"],
            family_counts["missing"],
            family_counts["complete"],
        )
        grand_counts.update(family_counts)

    expected_rows["Grand total"] = (
        sum(grand_counts.values()),
        grand_counts["partial"],
        grand_counts["blocked"],
        grand_counts["missing"],
        grand_counts["complete"],
    )

    errors: list[str] = []
    for label, expected in expected_rows.items():
        actual = rows.get(label)
        if actual != expected:
            errors.append(
                f"{matrix_path.name}: table row {label!r} must be {expected}, "
                f"got {actual}"
            )
    return errors


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--contract",
        action="append",
        type=Path,
        default=None,
        help="Explicit coverage contract to validate (repeatable).",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    paths = tuple(args.contract) if args.contract else DEFAULT_CONTRACTS
    all_errors: list[str] = []
    for path in paths:
        all_errors.extend(contract_errors(path))
    if not args.contract and not all_errors:
        contracts = tuple(json.loads(path.read_text()) for path in paths)
        all_errors.extend(matrix_errors(contracts))

    if all_errors:
        for error in all_errors:
            print(f"ERROR: {error}")
        return 1

    total = 0
    for path in paths:
        print(contract_summary(path))
        contract = json.loads(path.read_text())
        total += sum(
            len(group["requirements"]) for group in contract["coverage_groups"]
        )
    print(f"Validated {len(paths)} PLAN-104 contract(s), {total} canonical rows")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
