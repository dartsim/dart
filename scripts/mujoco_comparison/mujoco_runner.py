#!/usr/bin/env python3
"""Step a MuJoCo model and emit one ``common.ResultRow`` telemetry JSON.

Mirrors ``dart_runner.py``'s CLI and telemetry so the two engines can be
compared apples-to-apples; see ``README.md`` for the full fairness contract.
Only this module imports ``mujoco``; ``common.py`` and ``gen_scenes.py`` stay
importable from a plain-numpy environment.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import common
import gen_scenes
import mujoco
import numpy as np

_INTEGRATOR_ENUM_NAMES = {
    "euler": "mjINT_EULER",
    "rk4": "mjINT_RK4",
    "implicit": "mjINT_IMPLICIT",
    "implicitfast": "mjINT_IMPLICITFAST",
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
        "--integrator",
        choices=sorted(_INTEGRATOR_ENUM_NAMES),
        default=None,
        help="Override the model's integrator. Headline rows use 'euler'; "
        "omit to keep the model/file's own default for the sensitivity row.",
    )
    parser.add_argument(
        "--timestep", type=float, default=None, help="Override the model's timestep."
    )
    parser.add_argument(
        "--drive",
        choices=("on", "off"),
        default="off",
        help="Apply the harness's synthetic per-joint sinusoidal torque "
        "drive via data.qfrc_applied. Only valid for MJCF-file scenes; "
        "generated scenes encode their own kinematics (e.g. the stirrer).",
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
        help="Free-form config label recorded in the result row (e.g. "
        "'headline', 'model-default-integrator').",
    )
    return parser.parse_args(argv)


def _set_integrator(model: "mujoco.MjModel", name: str) -> None:
    enum_name = _INTEGRATOR_ENUM_NAMES[name]
    value = getattr(mujoco.mjtIntegrator, enum_name, None)
    if value is None:
        raise SystemExit(
            f"mujoco.mjtIntegrator.{enum_name} is not available in this "
            "mujoco version"
        )
    model.opt.integrator = value


def _load_model(scene_path: Path):
    """Return (model, MjcfOption, StirrerSpec-or-None)."""
    if scene_path.suffix == ".json":
        layout = common.load_generated_layout(scene_path)
        xml_str = gen_scenes.to_mjcf_xml(layout)
        model = mujoco.MjModel.from_xml_string(xml_str)
        option = common.MjcfOption(
            timestep=layout.spec.timestep,
            gravity=layout.spec.gravity,
            integrator="Euler",
        )
        return model, option, layout.spec.stirrer
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    option = common.parse_mjcf_option(scene_path)
    return model, option, None


def _mocap_id_for_body(model: "mujoco.MjModel", name: str) -> int | None:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        return None
    mocap_id = int(model.body_mocapid[body_id])
    return mocap_id if mocap_id >= 0 else None


def _drive_joint_dof_addrs(model: "mujoco.MjModel", joint_names: list[str]) -> dict[str, int]:
    """Map joint name -> its (first, and assumed only) DOF address.

    Every joint the harness drives (reacher/pusher/humanoid actuator
    targets) is a single-DOF hinge or slide joint, so ``jnt_dofadr`` alone
    is sufficient; this would need to gather a range of DOFs for a
    multi-DOF joint (e.g. ball/free), which none of the driven joints are.
    """
    addrs = {}
    for name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            continue
        addrs[name] = int(model.jnt_dofadr[joint_id])
    return addrs


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    if args.drive == "on" and args.scene.suffix == ".json":
        raise SystemExit(
            "--drive on is only supported for MJCF-file scenes; generated "
            "scenes encode their own kinematics (e.g. the stirrer) in the "
            "spec JSON."
        )

    model, option, stirrer = _load_model(args.scene)

    timestep = args.timestep if args.timestep is not None else option.timestep
    model.opt.timestep = timestep
    model.opt.gravity[:] = option.gravity

    if args.integrator is not None:
        _set_integrator(model, args.integrator)

    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    drive_addrs: dict[str, int] = {}
    phases: dict[str, float] = {}
    if args.drive == "on":
        joint_names = common.parse_mjcf_actuator_joint_names(args.scene)
        phases = common.derive_joint_phases(joint_names, args.drive_seed)
        drive_addrs = _drive_joint_dof_addrs(model, joint_names)
        missing = sorted(set(joint_names) - set(drive_addrs))
        if missing:
            print(
                f"WARNING: --drive on requested but {len(missing)}/"
                f"{len(joint_names)} actuator joint name(s) were not found "
                f"in the loaded model (silently dropped, not driven): "
                f"{missing}",
                file=sys.stderr,
            )

    stirrer_mocap_id = _mocap_id_for_body(model, "stirrer") if stirrer is not None else None

    def apply_drive(t: float) -> None:
        if not drive_addrs:
            return
        data.qfrc_applied[:] = 0.0
        for name, addr in drive_addrs.items():
            torque = common.sinusoidal_torque(
                t, args.drive_amplitude, args.drive_frequency, phases[name]
            )
            data.qfrc_applied[addr] = torque

    def apply_stirrer(t: float) -> None:
        if stirrer_mocap_id is None:
            return
        angle = stirrer.angular_velocity_rad_s * t
        data.mocap_pos[stirrer_mocap_id] = stirrer.pivot_xyz
        half = angle * 0.5
        data.mocap_quat[stirrer_mocap_id] = (
            np.cos(half),
            0.0,
            0.0,
            np.sin(half),
        )

    t = 0.0
    for _ in range(args.warmup):
        apply_drive(t)
        apply_stirrer(t)
        mujoco.mj_step(model, data)
        t += timestep

    ncon_values: list[int] = []
    # Fairness: mirror dart_runner.py — only the per-step contact count is
    # recorded inside the timed region; finiteness is asserted once after
    # timing in both runners.
    completed = 0
    start = time.perf_counter()
    for _ in range(args.steps):
        apply_drive(t)
        apply_stirrer(t)
        mujoco.mj_step(model, data)
        t += timestep
        completed += 1
        ncon_values.append(int(data.ncon))
    wall_s = time.perf_counter() - start
    finite = common.finite_all(data.qpos, data.qvel)

    row = common.make_result_row(
        scene=args.scene_id or args.scene.stem,
        engine="mujoco",
        config=args.config,
        steps=completed,
        wall_s=wall_s,
        timestep=timestep,
        ncon_values=ncon_values,
        finite=finite,
        sleeping_bodies=None,
        metadata={
            "scene_path": str(args.scene),
            "warmup": args.warmup,
            "integrator": args.integrator or option.integrator,
            "timestep_source": (
                "override"
                if args.timestep is not None
                else ("scene" if args.scene.suffix == ".json" else "mjcf-file")
            ),
            "drive": args.drive,
            "drive_amplitude": args.drive_amplitude if args.drive == "on" else None,
            "drive_frequency_hz": args.drive_frequency if args.drive == "on" else None,
            "drive_seed": args.drive_seed if args.drive == "on" else None,
            "driven_joint_count": len(drive_addrs),
            "mujoco_version": getattr(mujoco, "__version__", None),
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nbody": int(model.nbody),
        },
    )
    common.write_result_row(row, args.out)
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
