#!/usr/bin/env python3
"""Shared schema, JSON IO, and MJCF helpers for the DART-vs-MuJoCo comparison
harness.

This module intentionally has no dependency on ``mujoco`` or ``dartpy`` so it
can be imported unmodified by both ``mujoco_runner.py`` and ``dart_runner.py``
regardless of which pixi environment's Python interpreter runs them. See
``README.md`` for the full methodology these runners implement.

Unit convention: every ``*_half_extent``/``radius``/``half_*`` field in this
module is a half-extent (distance from center to face), matching MuJoCo's own
box/geom ``size`` convention. DART's ``BoxShape`` size is a *full* extent, so
``dart_runner.py`` and ``gen_scenes.py`` double these values when constructing
DART shapes. Getting this conversion wrong would silently simulate
differently-sized boxes in each engine and invalidate the comparison.
"""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Optional

import numpy as np

# --------------------------------------------------------------------------
# Result row schema
# --------------------------------------------------------------------------


@dataclass
class ResultRow:
    """One benchmark measurement, as written by a runner or aggregated by
    ``run_comparison.py``.

    Field meanings:
      scene: fixed scene id, e.g. "PILE-120".
      engine: "dart" or "mujoco".
      config: free-form label distinguishing runs of the same scene, e.g.
        "headline", "model-default-integrator", "rep3".
      steps: number of measured steps actually completed (excludes warmup;
        may be less than requested if a non-finite state was detected and
        the run was stopped early).
      wall_s: wall-clock seconds spent in the measured step loop only
        (model/world construction and warmup are excluded).
      ms_per_step: wall_s / steps * 1000.
      steps_per_s: steps / wall_s.
      rtf: real-time factor, i.e. (steps * timestep) / wall_s.
      ncon_mean: mean number of contacts per measured step.
      ncon_max: maximum number of contacts observed in any measured step.
      finite: whether qpos/qvel (DART: positions/velocities) stayed finite
        for every measured step.
      sleeping_bodies: sleeping/deactivated body count at the last measured
        step if the engine binding exposes it, else None. MuJoCo has no
        sleeping concept, so this is always None for "mujoco" rows.
      metadata: engine/scene specific extras (detector, sleep, timestep,
        integrator, drive parameters, package versions, dof/body counts,
        etc).
    """

    scene: str
    engine: str
    config: str
    steps: int
    wall_s: float
    ms_per_step: float
    steps_per_s: float
    rtf: float
    ncon_mean: float
    ncon_max: int
    finite: bool
    sleeping_bodies: Optional[int]
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @staticmethod
    def from_dict(data: dict[str, Any]) -> "ResultRow":
        return ResultRow(**data)


def make_result_row(
    *,
    scene: str,
    engine: str,
    config: str,
    steps: int,
    wall_s: float,
    timestep: float,
    ncon_values: list[int],
    finite: bool,
    sleeping_bodies: Optional[int],
    metadata: dict[str, Any],
) -> ResultRow:
    """Build a ResultRow from raw per-step telemetry collected by a runner."""
    ms_per_step = (wall_s / steps * 1000.0) if steps else float("nan")
    steps_per_s = (steps / wall_s) if wall_s > 0.0 else float("nan")
    rtf = (steps * timestep / wall_s) if wall_s > 0.0 else float("nan")
    ncon_mean = float(np.mean(ncon_values)) if ncon_values else 0.0
    ncon_max = int(np.max(ncon_values)) if ncon_values else 0
    return ResultRow(
        scene=scene,
        engine=engine,
        config=config,
        steps=steps,
        wall_s=wall_s,
        ms_per_step=ms_per_step,
        steps_per_s=steps_per_s,
        rtf=rtf,
        ncon_mean=ncon_mean,
        ncon_max=ncon_max,
        finite=finite,
        sleeping_bodies=sleeping_bodies,
        metadata=metadata,
    )


def write_result_row(row: ResultRow, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(row.to_dict(), indent=2) + "\n", encoding="utf-8")


def read_result_row(path: Path) -> ResultRow:
    return ResultRow.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


# --------------------------------------------------------------------------
# Generated scene spec (box pile / stirred pile / FFI-overhead probe)
# --------------------------------------------------------------------------


@dataclass
class ContainerSpec:
    """A four-wall, open-top box container plus floor, all static."""

    half_extent_xy: float
    wall_height: float
    wall_thickness: float
    floor_half_thickness: float


@dataclass
class StirrerSpec:
    """A kinematic bar swept about a vertical pivot axis at a fixed angular
    velocity.

    The bar spans the full container diameter (``radius`` is a half-length,
    i.e. the bar runs from ``-radius`` to ``+radius`` from the pivot) so a
    single sweep passes through the entire pile. Both runners recompute the
    bar's pose every step from ``angular_velocity_rad_s`` and elapsed sim
    time; it is never driven by contact forces or torques, and it never
    integrates dynamically (DART: ``Skeleton.setMobile(False)``; MuJoCo: a
    ``mocap`` body).
    """

    radius: float
    half_height: float
    half_thickness: float
    friction: float
    angular_velocity_rad_s: float
    pivot_xyz: tuple[float, float, float]


@dataclass
class GeneratedSceneSpec:
    scene_id: str
    seed: int
    n_objects: int
    box_half_extent: float
    mass: float
    friction: float
    spacing_jitter: float
    drop_height: float
    timestep: float
    gravity: tuple[float, float, float]
    container: Optional[ContainerSpec]
    stirrer: Optional[StirrerSpec]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @staticmethod
    def from_dict(data: dict[str, Any]) -> "GeneratedSceneSpec":
        data = dict(data)
        container = data.get("container")
        stirrer = data.get("stirrer")
        data["container"] = ContainerSpec(**container) if container else None
        if stirrer:
            stirrer = dict(stirrer)
            stirrer["pivot_xyz"] = tuple(stirrer["pivot_xyz"])
            data["stirrer"] = StirrerSpec(**stirrer)
        else:
            data["stirrer"] = None
        data["gravity"] = tuple(data["gravity"])
        return GeneratedSceneSpec(**data)


@dataclass
class BoxBody:
    name: str
    half_extent: float
    mass: float
    friction: float
    position: tuple[float, float, float]


@dataclass
class GeneratedLayout:
    """A materialized, seed-deterministic pile layout: the scene spec plus a
    concrete list of free boxes, ready to be emitted as MJCF XML
    (``gen_scenes.to_mjcf_xml``) or consumed directly to build a DART world
    (``dart_runner._build_world_from_generated``). Both emitters read from
    the *same* ``GeneratedLayout``, so the two engines simulate physically
    identical scenes.
    """

    spec: GeneratedSceneSpec
    boxes: list[BoxBody]


def save_generated_layout(layout: GeneratedLayout, path: Path) -> None:
    data = {
        "spec": layout.spec.to_dict(),
        "boxes": [asdict(b) for b in layout.boxes],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def load_generated_layout(path: Path) -> GeneratedLayout:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    spec = GeneratedSceneSpec.from_dict(data["spec"])
    boxes = [
        BoxBody(
            name=b["name"],
            half_extent=b["half_extent"],
            mass=b["mass"],
            friction=b["friction"],
            position=tuple(b["position"]),
        )
        for b in data["boxes"]
    ]
    return GeneratedLayout(spec=spec, boxes=boxes)


# --------------------------------------------------------------------------
# MJCF <option>/<actuator> parsing (fixed scene files: reacher/pusher/
# humanoid)
# --------------------------------------------------------------------------


@dataclass
class MjcfOption:
    timestep: float
    gravity: tuple[float, float, float]
    integrator: str


_MUJOCO_DEFAULT_TIMESTEP = 0.002
_MUJOCO_DEFAULT_GRAVITY = (0.0, 0.0, -9.81)
_MUJOCO_DEFAULT_INTEGRATOR = "Euler"


def parse_mjcf_option(xml_path: Path) -> MjcfOption:
    """Read the model's own top-level ``<option>`` element.

    This exists because ``dart::utils::MjcfParser::createWorld()`` (see
    ``dart/utils/mjcf/MjcfParser.cpp``) never applies ``<option
    timestep="...">``/``<option gravity="...">`` to the ``World`` it
    builds -- it only converts bodies/joints/geoms. Fairness requires the
    same timestep and gravity in both engines, so ``dart_runner.py`` parses
    this itself and calls ``world.setTimeStep()``/``world.setGravity()``
    explicitly after ``MjcfParser.readWorld()``. Skipping this silently
    would run DART at its own default step (0.001s) while MuJoCo runs at
    the file's own step (e.g. 0.01s for these OpenAI Gym MJCF files),
    which would invalidate any steps-per-second/RTF comparison.
    """
    root = ET.parse(str(xml_path)).getroot()
    option = root.find("option")
    timestep = _MUJOCO_DEFAULT_TIMESTEP
    gravity = _MUJOCO_DEFAULT_GRAVITY
    integrator = _MUJOCO_DEFAULT_INTEGRATOR
    if option is not None:
        if "timestep" in option.attrib:
            timestep = float(option.attrib["timestep"])
        if "gravity" in option.attrib:
            parts = [float(v) for v in option.attrib["gravity"].split()]
            if len(parts) == 3:
                gravity = (parts[0], parts[1], parts[2])
        if "integrator" in option.attrib:
            integrator = option.attrib["integrator"]
    return MjcfOption(timestep=timestep, gravity=gravity, integrator=integrator)


def parse_mjcf_actuator_joint_names(xml_path: Path) -> list[str]:
    """Return the sorted, de-duplicated joint names referenced by
    ``<actuator><motor joint="...">`` elements.

    DART's MJCF parser ignores ``<actuator>`` entirely, so this list is only
    used to decide *which joints* the harness drives with its own synthetic
    sinusoidal torque profile (see ``derive_joint_phases``/
    ``sinusoidal_torque``). It intentionally does not read MuJoCo's
    per-actuator ``gear`` scaling: that scaling has no DART-side equivalent
    to replicate, so both engines instead apply the harness's own uniform
    torque amplitude directly as a generalized force on each named joint's
    (single) degree of freedom.
    """
    root = ET.parse(str(xml_path)).getroot()
    names: set[str] = set()
    actuator = root.find("actuator")
    if actuator is not None:
        for elem in actuator:
            joint = elem.attrib.get("joint")
            if joint:
                names.add(joint)
    return sorted(names)


# --------------------------------------------------------------------------
# Shared sinusoidal drive profile
# --------------------------------------------------------------------------


def derive_joint_phases(joint_names: list[str], seed: int) -> dict[str, float]:
    """Deterministically assign a phase offset in [0, 2*pi) to each joint
    name, drawn from a seeded RNG over the *sorted* name list.

    Both runners call this with the same ``(joint_names, seed)`` so the
    resulting per-joint phase map is identical across engines, independent
    of each engine's own internal joint ordering (MuJoCo and DART do not
    generally enumerate joints in the same order).
    """
    names = sorted(joint_names)
    rng = np.random.default_rng(seed)
    phases = rng.uniform(0.0, 2.0 * math.pi, size=len(names))
    return {name: float(phase) for name, phase in zip(names, phases)}


def sinusoidal_torque(
    t: float, amplitude: float, frequency_hz: float, phase: float
) -> float:
    return amplitude * math.sin(2.0 * math.pi * frequency_hz * t + phase)


def finite_all(*arrays: np.ndarray) -> bool:
    """True if every element of every non-empty array is finite."""
    return all(bool(np.all(np.isfinite(a))) for a in arrays if np.size(a))
