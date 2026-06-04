#!/usr/bin/env python3
"""Human-readable display names for the experimental World performance dashboard.

Both the hosted dashboard (``benchmark-action/github-action-benchmark``) and the
local preview use a Google Benchmark row's ``name`` as the chart title *and* the
history key. Raw names such as ``BM_WorldStepParallel/128/32`` are hard to read,
so this module rewrites them into readable titles like
``World step (parallel) · 128 parents · 32 children/parent``.

The mapping is intentionally explicit, one curated entry per dashboard surface,
so published titles stay stable and reviewable. Anything unmapped falls back to a
generic transform that drops the ``BM_`` prefix, splits CamelCase/underscores,
and appends the raw arguments, so newly added benchmarks still render legibly
before they are curated here.

Only the dashboard presentation layer (``merge_benchmark_results.py`` for the
published JSON and ``preview_performance_dashboard.py`` for the local preview)
uses this module. The C++ benchmark registrations and the correctness gate
(``check_compute_graph_benchmarks.py``) keep operating on the raw Google
Benchmark names, so renaming here never affects the run-time ``--benchmark_filter``
or the gate.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

# Section headers used to group charts on the dashboard.
FAMILY_CORE = "Experimental World — core step & scaling"
FAMILY_RIGID = "Rigid-body dynamics solver"
FAMILY_VBD = "Deformable solver (Vertex Block Descent)"
FAMILY_FEM = "Deformable solver (FEM)"
FAMILY_AVBD = "Augmented VBD rigid"
FAMILY_OTHER = "Other benchmarks"


@dataclass(frozen=True)
class Surface:
    """A curated dashboard benchmark: readable title, section, and arg labels."""

    title: str
    family: str
    args: tuple[str, ...] = ()
    # When True a single argument ``n`` is rendered as an ``n×n`` grid instead of
    # ``n <label>`` (used by the square deformable grid benchmarks).
    grid: bool = False


# Base benchmark name (without the ``/arg`` suffix) -> readable surface.
SURFACES: dict[str, Surface] = {
    # --- Experimental World core step & scaling (bm_compute_graph) ---------
    "BM_WorldUpdateKinematics": Surface(
        "Kinematics update", FAMILY_CORE, ("parents", "children/parent")
    ),
    "BM_WorldStepSequential": Surface(
        "World step (sequential)", FAMILY_CORE, ("parents", "children/parent")
    ),
    "BM_WorldStepParallel": Surface(
        "World step (parallel)", FAMILY_CORE, ("parents", "children/parent")
    ),
    "BM_RigidBodyStepSequential": Surface(
        "Rigid-body step (sequential)", FAMILY_CORE, ("bodies",)
    ),
    "BM_RigidBodyStepParallel": Surface(
        "Rigid-body step (parallel)", FAMILY_CORE, ("bodies",)
    ),
    "BM_ContactShapedSequential": Surface(
        "Contact-shaped proxy (sequential)", FAMILY_CORE, ("bodies", "iterations")
    ),
    "BM_ContactShapedParallel": Surface(
        "Contact-shaped proxy (parallel)", FAMILY_CORE, ("bodies", "iterations")
    ),
    "BM_ContactIslandShapedSequential": Surface(
        "Contact-island proxy (sequential)",
        FAMILY_CORE,
        ("islands", "bodies/island", "iterations"),
    ),
    "BM_ContactIslandShapedParallel": Surface(
        "Contact-island proxy (parallel)",
        FAMILY_CORE,
        ("islands", "bodies/island", "iterations"),
    ),
    "BM_Phase5RigidBodyBatchCpuBaseline": Surface(
        "Rigid-body batch (CPU baseline)",
        FAMILY_CORE,
        ("worlds", "bodies", "steps"),
    ),
    # --- Rigid-body dynamics solver (bm_rigid_ipc_solver) -----------------
    "BM_RigidWorldStep_SequentialImpulse": Surface(
        "Rigid world step (sequential impulse)", FAMILY_RIGID, ("boxes",)
    ),
    "BM_RigidWorldStep_Ipc": Surface(
        "Rigid world step (IPC barrier)", FAMILY_RIGID, ("boxes",)
    ),
    # --- Deformable Vertex Block Descent (bm_vbd_world_solver) ------------
    "BM_VbdWorldStepDefault": Surface(
        "Deformable world step (default solver)", FAMILY_VBD, ("side",), grid=True
    ),
    "BM_VbdWorldStepVbd": Surface(
        "Deformable world step (VBD)", FAMILY_VBD, ("side",), grid=True
    ),
    # --- Deformable FEM (bm_deformable_body) ------------------------------
    "BM_DeformableFemBarStep": Surface("FEM bar step", FAMILY_FEM, ("cells",)),
    # --- Augmented VBD rigid (bm_avbd_rigid_fixed_joint) ------------------
    "BM_AvbdRigidFixedJointStep": Surface(
        "AVBD fixed-joint step", FAMILY_AVBD, ("links",)
    ),
}


def _split(raw_name: str) -> tuple[str, list[str]]:
    """Split ``BM_Foo/1/2`` into ``("BM_Foo", ["1", "2"])``."""
    base, _, arg_str = raw_name.partition("/")
    values = [part for part in arg_str.split("/") if part != ""] if arg_str else []
    return base, values


def _generic_title(base: str) -> str:
    """De-prefix ``BM_`` and split CamelCase/underscores for unmapped names."""
    name = base[3:] if base.startswith("BM_") else base
    name = name.replace("_", " ")
    name = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", " ", name)
    return re.sub(r"\s+", " ", name).strip()


def _format_args(values: list[str], surface: Surface | None) -> str:
    if not values:
        return ""
    if surface and surface.grid and len(values) == 1:
        return f"{values[0]}×{values[0]} grid"
    labels = surface.args if surface else ()
    parts = []
    for index, value in enumerate(values):
        label = labels[index] if index < len(labels) else f"arg{index}"
        parts.append(f"{value} {label}")
    return " · ".join(parts)


def humanize_name(raw_name: str) -> str:
    """Return a readable chart title for a raw Google Benchmark name.

    The argument values stay in the title so each parameterized row remains a
    distinct, uniquely keyed series on the dashboard.
    """
    base, values = _split(raw_name)
    surface = SURFACES.get(base)
    title = surface.title if surface else _generic_title(base)
    arg_part = _format_args(values, surface)
    return f"{title} · {arg_part}" if arg_part else title


def family_of(raw_name: str) -> str:
    """Return the dashboard section a benchmark belongs to."""
    base, _ = _split(raw_name)
    surface = SURFACES.get(base)
    return surface.family if surface else FAMILY_OTHER
