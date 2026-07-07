#!/usr/bin/env python3
"""Human-readable display names for the DART 6 performance dashboard."""

from __future__ import annotations

import re
from dataclasses import dataclass

FAMILY_BASELINE = "DART 6 benchmark baselines"
FAMILY_KINEMATICS = "DART 6 kinematics and dynamics"
FAMILY_INVERSE = "DART 6 inverse dynamics"
FAMILY_CONTACT = "DART 6 contact and collision"
FAMILY_DEFORMABLE = "DART 6 deformable bodies"
FAMILY_OTHER = "Other benchmarks"


@dataclass(frozen=True)
class Surface:
    title: str
    family: str
    args: tuple[str, ...] = ()


SURFACES: dict[str, Surface] = {
    "BM_Empty": Surface("Google Benchmark empty baseline", FAMILY_BASELINE),
    "BM_Kinematics": Surface(
        "Skel kinematics update corpus", FAMILY_KINEMATICS, ("iterations",)
    ),
    "BM_Dynamics": Surface("Skel dynamics step corpus", FAMILY_KINEMATICS, ("steps",)),
    "BM_InverseDynamics": Surface(
        "Recursive inverse dynamics", FAMILY_INVERSE, ("links",)
    ),
    "BM_InverseDynamicsViaMassMatrix": Surface(
        "Dense mass-matrix inverse dynamics", FAMILY_INVERSE, ("links",)
    ),
    "BM_ContactInverseDynamics": Surface(
        "Contact inverse dynamics", FAMILY_INVERSE, ("contacts",)
    ),
    "BM_ContactInverseDynamicsBasis": Surface(
        "Contact inverse dynamics basis sweep",
        FAMILY_INVERSE,
        ("friction bases",),
    ),
    "BM_RunBoxes": Surface("Stacked boxes world step", FAMILY_CONTACT, ("grid side",)),
    "BM_ContactContainerActive": Surface(
        "Contact container active step",
        FAMILY_CONTACT,
        ("objects", "engine", "threads"),
    ),
    "BM_ContactContainerLargeActive": Surface(
        "Contact container large active step",
        FAMILY_CONTACT,
        ("objects", "engine", "threads"),
    ),
    "BM_ContactContainerDeactivation": Surface(
        "Contact container deactivation-enabled step",
        FAMILY_CONTACT,
        ("objects", "engine", "threads"),
    ),
    "BM_SoftBodyStep": Surface(
        "Soft-body world step",
        FAMILY_DEFORMABLE,
        ("scene", "threads", "steps"),
    ),
}

GOOGLE_BENCHMARK_METADATA_PREFIXES = (
    "iterations:",
    "repeats:",
    "threads:",
    "thread_per_cpu:",
)


def _split(raw_name: str) -> tuple[str, list[str]]:
    base, _, arg_str = raw_name.partition("/")
    values = [part for part in arg_str.split("/") if part != ""] if arg_str else []
    return base, [value for value in values if not _is_google_benchmark_metadata(value)]


def _is_google_benchmark_metadata(value: str) -> bool:
    return value.startswith(GOOGLE_BENCHMARK_METADATA_PREFIXES)


def _generic_title(base: str) -> str:
    name = base[3:] if base.startswith("BM_") else base
    name = name.replace("_", " ")
    name = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", " ", name)
    return re.sub(r"\s+", " ", name).strip()


def _format_args(values: list[str], surface: Surface | None) -> str:
    if not values:
        return ""

    labels = surface.args if surface else ()
    parts = []
    for index, value in enumerate(values):
        label = labels[index] if index < len(labels) else f"arg{index}"
        parts.append(f"{value} {label}")
    return " - ".join(parts)


def humanize_name(raw_name: str) -> str:
    base, values = _split(raw_name)
    surface = SURFACES.get(base)
    title = surface.title if surface else _generic_title(base)
    arg_part = _format_args(values, surface)
    return f"{title} - {arg_part}" if arg_part else title


def family_of(raw_name: str) -> str:
    base, _ = _split(raw_name)
    surface = SURFACES.get(base)
    return surface.family if surface else FAMILY_OTHER
