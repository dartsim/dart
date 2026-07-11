#!/usr/bin/env python3
"""Step a DART world and emit one ``common.ResultRow`` telemetry JSON.

CLI mirror of ``mujoco_runner.py``; see ``README.md`` for the full fairness
contract both runners implement. Only this module imports ``dartpy``;
``common.py`` and ``gen_scenes.py`` stay importable from a plain-numpy
environment.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import common
import dartpy as dart
import gen_scenes
import numpy as np

_DETECTOR_CLASS_NAMES = {
    "dart": "DARTCollisionDetector",
    "fcl": "FCLCollisionDetector",
    "bullet": "BulletCollisionDetector",
    "ode": "OdeCollisionDetector",
    "native": "NativeCollisionDetector",
}


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene",
        required=True,
        type=Path,
        help="Path to a .xml MJCF file or a generated-scene spec .json "
        "(see gen_scenes.py).",
    )
    parser.add_argument(
        "--steps", type=int, required=True, help="Number of measured steps."
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=0,
        help="Warmup steps run before timing starts (excluded from telemetry).",
    )
    parser.add_argument(
        "--out", required=True, type=Path, help="Where to write the ResultRow JSON."
    )
    parser.add_argument(
        "--timestep", type=float, default=None, help="Override the scene's timestep."
    )
    parser.add_argument(
        "--detector",
        choices=("default", "dart", "fcl", "bullet", "ode", "native"),
        default="default",
        help="Collision detector to install via "
        "world.getConstraintSolver().setCollisionDetector(); 'default' "
        "leaves the World's built-in default detector untouched.",
    )
    parser.add_argument(
        "--sleep",
        choices=("on", "off"),
        default="on",
        help="Automatic deactivation ('sleeping') of resting islands, via "
        "world.setDeactivationOptions().",
    )
    parser.add_argument(
        "--drive",
        choices=("on", "off"),
        default="off",
        help="Apply the harness's synthetic per-joint sinusoidal torque "
        "drive via DegreeOfFreedom.setForce(). Only valid for MJCF-file "
        "scenes; generated scenes encode their own kinematics (e.g. the "
        "stirrer).",
    )
    parser.add_argument(
        "--drive-amplitude",
        type=float,
        default=5.0,
        help="Torque amplitude (N*m) applied to each actuated joint.",
    )
    parser.add_argument(
        "--drive-frequency",
        type=float,
        default=0.5,
        help="Sinusoid frequency (Hz) for the drive torque.",
    )
    parser.add_argument(
        "--drive-seed", type=int, default=0, help="Seed for per-joint phase offsets."
    )
    parser.add_argument(
        "--scene-id",
        default=None,
        help="Scene id recorded in the result row; defaults to the --scene "
        "file stem.",
    )
    parser.add_argument(
        "--config",
        default="headline",
        help="Free-form config label recorded in the result row.",
    )
    return parser.parse_args(argv)


def _isometry(rotation: np.ndarray, translation: np.ndarray):
    return dart.math.Isometry3(rotation, translation)


def _z_rotation(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _build_container(world, container: common.ContainerSpec, friction: float) -> None:
    he = container.half_extent_xy
    wt = container.wall_thickness
    wh = container.wall_height
    ft = container.floor_half_thickness
    pieces = [
        ("floor", (he, he, ft), (0.0, 0.0, -ft)),
        ("wall_px", (wt, he + wt, wh / 2.0), (he + wt, 0.0, wh / 2.0)),
        ("wall_nx", (wt, he + wt, wh / 2.0), (-(he + wt), 0.0, wh / 2.0)),
        ("wall_py", (he + wt, wt, wh / 2.0), (0.0, he + wt, wh / 2.0)),
        ("wall_ny", (he + wt, wt, wh / 2.0), (0.0, -(he + wt), wh / 2.0)),
    ]
    skel = dart.dynamics.Skeleton("container")
    for name, half_extents, position in pieces:
        joint, body = skel.createFreeJointAndBodyNodePair(None)
        joint.setName(f"{name}_joint")
        body.setName(name)
        full_size = np.array(half_extents) * 2.0
        shape = dart.dynamics.BoxShape(full_size)
        shape_node = body.createShapeNode(shape)
        shape_node.createVisualAspect()
        shape_node.createCollisionAspect()
        shape_node.createDynamicsAspect().setFrictionCoeff(friction)
        joint.setTransform(_isometry(np.eye(3), np.array(position)))
    skel.setMobile(False)
    world.addSkeleton(skel)


def _build_boxes(world, boxes: list[common.BoxBody]) -> None:
    # One Skeleton per box: a Skeleton is DART's constraint-island and
    # sleeping unit, so a shared multi-tree Skeleton would fuse every contact
    # into one dense LCP island and disable per-body deactivation (measured
    # 125 ms/step vs sub-ms for PILE-120). Free bodies as individual
    # Skeletons is also how DART's own contact fixtures are built. Telemetry
    # that iterates skeletons (finite check, sleep count) runs outside the
    # timed loop, so the extra Python objects do not affect measured step
    # time.
    for box in boxes:
        skel = dart.dynamics.Skeleton(box.name)
        joint, body = skel.createFreeJointAndBodyNodePair(None)
        joint.setName(f"{box.name}_joint")
        body.setName(box.name)
        full_size = np.full(3, 2.0 * box.half_extent)
        shape = dart.dynamics.BoxShape(full_size)
        shape_node = body.createShapeNode(shape)
        shape_node.createVisualAspect()
        shape_node.createCollisionAspect()
        shape_node.createDynamicsAspect().setFrictionCoeff(box.friction)
        moment = dart.dynamics.BoxShape.computeInertiaOf(full_size, box.mass)
        body.setInertia(dart.dynamics.Inertia(box.mass, np.zeros(3), moment))
        joint.setTransform(_isometry(np.eye(3), np.array(box.position)))
        world.addSkeleton(skel)


def _build_stirrer(world, stirrer: common.StirrerSpec):
    skel = dart.dynamics.Skeleton("stirrer")
    joint, body = skel.createFreeJointAndBodyNodePair(None)
    body.setName("stirrer")
    size = np.array(
        [2.0 * stirrer.radius, 2.0 * stirrer.half_thickness, 2.0 * stirrer.half_height]
    )
    shape = dart.dynamics.BoxShape(size)
    shape_node = body.createShapeNode(shape)
    shape_node.createVisualAspect()
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect().setFrictionCoeff(stirrer.friction)
    joint.setTransform(_isometry(np.eye(3), np.array(stirrer.pivot_xyz)))
    skel.setMobile(False)
    world.addSkeleton(skel)
    return joint


def _build_world_from_mjcf(scene_path: Path):
    world = dart.utils.MjcfParser.readWorld(str(scene_path))
    if world is None:
        raise SystemExit(f"Failed to parse MJCF scene: {scene_path}")
    option = common.parse_mjcf_option(scene_path)
    world.setTimeStep(option.timestep)
    world.setGravity(list(option.gravity))
    return world, option


def _build_world_from_generated(layout: common.GeneratedLayout):
    world = dart.simulation.World()
    world.setGravity(list(layout.spec.gravity))
    world.setTimeStep(layout.spec.timestep)
    if layout.spec.container is not None:
        _build_container(world, layout.spec.container, layout.spec.friction)
    _build_boxes(world, layout.boxes)
    stirrer_joint = None
    if layout.spec.stirrer is not None:
        stirrer_joint = _build_stirrer(world, layout.spec.stirrer)
    option = common.MjcfOption(
        timestep=layout.spec.timestep, gravity=layout.spec.gravity, integrator="Euler"
    )
    return world, option, stirrer_joint


def _build_world(scene_path: Path):
    """Return (world, MjcfOption, stirrer_joint-or-None, StirrerSpec-or-None)."""
    if scene_path.suffix == ".json":
        layout = common.load_generated_layout(scene_path)
        world, option, stirrer_joint = _build_world_from_generated(layout)
        return world, option, stirrer_joint, layout.spec.stirrer
    world, option = _build_world_from_mjcf(scene_path)
    return world, option, None, None


def _apply_detector(world, name: str) -> None:
    if name == "default":
        return
    class_name = _DETECTOR_CLASS_NAMES[name]
    factory = getattr(dart.collision, class_name, None)
    if factory is None:
        if name == "native":
            raise SystemExit(
                "dartpy.collision.NativeCollisionDetector is not bound yet. "
                "This is tracked separately (bind NativeCollisionDetector in "
                "dartpy) and must land before --detector native can be used "
                "here; see docs/dev_tasks/dart6_dependency_minimization/ for "
                "status."
            )
        raise SystemExit(
            f"dartpy.collision.{class_name} is not available in this build "
            "(likely built without the corresponding optional backend)."
        )
    world.getConstraintSolver().setCollisionDetector(factory())


def _all_dofs_by_name(world) -> dict[str, object]:
    mapping = {}
    for i in range(world.getNumSkeletons()):
        skel = world.getSkeleton(i)
        for dof in skel.getDofs():
            mapping[dof.getName()] = dof
    return mapping


def _finite_check(world) -> bool:
    for i in range(world.getNumSkeletons()):
        skel = world.getSkeleton(i)
        positions = np.asarray(skel.getPositions())
        velocities = np.asarray(skel.getVelocities())
        if not common.finite_all(positions, velocities):
            return False
    return True


def _sleeping_body_count(world) -> int | None:
    """Best-effort sleeping/deactivated skeleton count.

    dart::dynamics::Skeleton::isResting() is not currently bound in dartpy,
    so this returns None unless a future dartpy build exposes it.
    """
    count = 0
    for i in range(world.getNumSkeletons()):
        skel = world.getSkeleton(i)
        is_resting = getattr(skel, "isResting", None)
        if is_resting is None:
            return None
        if is_resting():
            count += 1
    return count


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    if args.drive == "on" and args.scene.suffix == ".json":
        raise SystemExit(
            "--drive on is only supported for MJCF-file scenes; generated "
            "scenes encode their own kinematics (e.g. the stirrer) in the "
            "spec JSON."
        )

    world, option, stirrer_joint, stirrer = _build_world(args.scene)

    timestep = args.timestep if args.timestep is not None else option.timestep
    world.setTimeStep(timestep)
    world.setNumSimulationThreads(1)

    deactivation = dart.simulation.DeactivationOptions()
    deactivation.mEnabled = args.sleep == "on"
    world.setDeactivationOptions(deactivation)

    _apply_detector(world, args.detector)

    drive_dofs: dict[str, object] = {}
    phases: dict[str, float] = {}
    if args.drive == "on":
        joint_names = common.parse_mjcf_actuator_joint_names(args.scene)
        phases = common.derive_joint_phases(joint_names, args.drive_seed)
        all_dofs = _all_dofs_by_name(world)
        drive_dofs = {name: all_dofs[name] for name in joint_names if name in all_dofs}
        missing = sorted(set(joint_names) - set(drive_dofs))
        if missing:
            print(
                f"WARNING: --drive on requested but {len(missing)}/"
                f"{len(joint_names)} actuator joint name(s) were not found "
                f"among the DART-parsed DOFs (silently dropped, not driven): "
                f"{missing}",
                file=sys.stderr,
            )

    def apply_drive(t: float) -> None:
        for name, dof in drive_dofs.items():
            torque = common.sinusoidal_torque(
                t, args.drive_amplitude, args.drive_frequency, phases[name]
            )
            dof.setForce(torque)

    def apply_stirrer(t: float) -> None:
        if stirrer_joint is None:
            return
        angle = stirrer.angular_velocity_rad_s * t
        stirrer_joint.setTransform(
            _isometry(_z_rotation(angle), np.array(stirrer.pivot_xyz))
        )

    t = 0.0
    for _ in range(args.warmup):
        apply_drive(t)
        apply_stirrer(t)
        world.step()
        t += timestep

    # Fairness: keep the timed loop's telemetry symmetric with
    # mujoco_runner.py — only the per-step contact count is recorded inside
    # the timed region. A per-step finite check here would iterate every
    # skeleton through the bindings (hundreds of calls/step on pile scenes)
    # while MuJoCo reads two contiguous arrays, so finiteness is asserted
    # once after timing in both runners instead.
    ncon_values: list[int] = []
    completed = 0
    start = time.perf_counter()
    for _ in range(args.steps):
        apply_drive(t)
        apply_stirrer(t)
        world.step()
        t += timestep
        completed += 1
        ncon_values.append(world.getLastCollisionResult().getNumContacts())
    wall_s = time.perf_counter() - start
    finite = _finite_check(world)

    total_dofs = sum(
        world.getSkeleton(i).getNumDofs() for i in range(world.getNumSkeletons())
    )
    row = common.make_result_row(
        scene=args.scene_id or args.scene.stem,
        engine="dart",
        config=args.config,
        steps=completed,
        wall_s=wall_s,
        timestep=timestep,
        ncon_values=ncon_values,
        finite=finite,
        sleeping_bodies=_sleeping_body_count(world),
        metadata={
            "scene_path": str(args.scene),
            "warmup": args.warmup,
            "detector": args.detector,
            "sleep": args.sleep,
            "timestep_source": (
                "override"
                if args.timestep is not None
                else ("scene" if args.scene.suffix == ".json" else "mjcf-file")
            ),
            "drive": args.drive,
            "drive_amplitude": args.drive_amplitude if args.drive == "on" else None,
            "drive_frequency_hz": args.drive_frequency if args.drive == "on" else None,
            "drive_seed": args.drive_seed if args.drive == "on" else None,
            "driven_joint_count": len(drive_dofs),
            "num_skeletons": world.getNumSkeletons(),
            "num_dofs": total_dofs,
        },
    )
    common.write_result_row(row, args.out)
    print(args.out, flush=True)
    return 0


if __name__ == "__main__":
    _rc = main(sys.argv[1:])
    # dartpy currently segfaults during interpreter teardown after long
    # simulations (destructor-order issue in the bindings). The result row is
    # already written and flushed, so skip Python finalization entirely.
    os._exit(_rc)
