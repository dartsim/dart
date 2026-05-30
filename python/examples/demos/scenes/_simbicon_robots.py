"""Robot models and SIMBICON configs for the py-demos SIMBICON scenes.

Provides :func:`make_atlas_config`/:func:`make_g1_config` (the per-robot role
names and gait parameters consumed by :class:`._simbicon.SimbiconController`)
and loaders for the Atlas (bundled) and Unitree G1 (fetched + cached locally,
since dartpy has no HTTP resource retriever) skeletons.
"""

from __future__ import annotations

import math
import pathlib
import re
import urllib.request

import numpy as np

import dartpy as dart

from ._simbicon import SimbiconConfig, SimbiconController
from ._z_up import reorient_to_z_up

_GROUND_URI = "dart://sample/skel/ground.skel"

_DEG = math.pi / 180.0

_ATLAS_URI = "dart://sample/sdf/atlas/atlas_v5_no_head.urdf"

# Unitree G1 (29-DOF) lives in a remote ROS package; dartpy can't fetch it, so
# we download the URDF + meshes once into a local cache and load by path.
_G1_BASE = (
    "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
    "master/robots/g1_description"
)
_G1_URDF = "g1_29dof.urdf"
_G1_CACHE = pathlib.Path.home() / ".cache" / "dart_demos" / "g1_description"


# ----------------------------------------------------------------- Atlas -----
def make_atlas_config() -> SimbiconConfig:
    """SIMBICON config for the bundled Atlas v5 model.

    Mirrors the gain/target values of the C++ ``atlas_simbicon`` walking-in-place
    controller (including the corrected positive sagittal torso gain).
    """
    return SimbiconConfig(
        name="atlas",
        pelvis="pelvis",
        left_foot="l_foot",
        right_foot="r_foot",
        left_thigh="l_uleg",
        right_thigh="r_uleg",
        torso="back_bky",
        left_hip_sagittal="l_leg_hpy",
        right_hip_sagittal="r_leg_hpy",
        left_hip_coronal="l_leg_hpx",
        right_hip_coronal="r_leg_hpx",
        left_knee="l_leg_kny",
        right_knee="r_leg_kny",
        left_ankle_sagittal="l_leg_aky",
        right_ankle_sagittal="r_leg_aky",
        kp=1.0e3,
        kd=1.0,
        cd=0.5,
        cv=0.2,
        torso_kp=5000.0,
        torso_kd=1.0,
        min_pelvis_height=-0.70,
        max_pelvis_height=0.30,
        foot_contact_height=0.04,
        ground_height=-0.93,
        walk_torso=4.75 * _DEG,
        # Final (signed) joint targets; signs already baked in -> sign mults = 1.
        swing_hip_up=-0.50,
        swing_knee_up=1.10,
        swing_ankle_up=-0.60,
        stance_knee_up=0.05,
        stance_ankle_up=0.0,
        swing_hip_down=0.10,
        swing_knee_down=0.05,
        swing_ankle_down=-0.15,
        stance_knee_down=0.10,
        stance_ankle_down=0.0,
    )


def load_atlas_skeleton() -> "dart.dynamics.Skeleton":
    robot = dart.io.UrdfParser().parse_skeleton(_ATLAS_URI)
    if robot is None:
        raise RuntimeError(f"Failed to load Atlas from {_ATLAS_URI}")
    robot.set_name("atlas")
    return robot


# ------------------------------------------------------------------- G1 ------
def _ensure_g1_cached() -> pathlib.Path:
    """Download the G1 URDF + referenced meshes into the local cache (once)."""
    _G1_CACHE.mkdir(parents=True, exist_ok=True)

    def fetch(rel: str) -> None:
        dst = _G1_CACHE / rel
        if dst.exists() and dst.stat().st_size > 0:
            return
        dst.parent.mkdir(parents=True, exist_ok=True)
        urllib.request.urlretrieve(f"{_G1_BASE}/{rel}", dst)

    fetch(_G1_URDF)
    text = (_G1_CACHE / _G1_URDF).read_text()
    for mesh in sorted(set(re.findall(r'filename="(meshes/[^"]+)"', text))):
        fetch(mesh)
    return _G1_CACHE / _G1_URDF


def load_g1_skeleton() -> "dart.dynamics.Skeleton":
    urdf_path = _ensure_g1_cached()
    robot = dart.io.UrdfParser().parse_skeleton(str(urdf_path))
    if robot is None:
        raise RuntimeError("Failed to load G1 from cached URDF")
    robot.set_name("g1")
    return robot


def make_g1_config() -> SimbiconConfig:
    """SIMBICON config for the Unitree G1 (29-DOF).

    G1 is much lighter/smaller than Atlas, so gains are scaled down and the gait
    targets are a starting point for tuning (Unitree joint-axis conventions are
    absorbed by the per-joint sign multipliers).
    """
    return SimbiconConfig(
        name="g1",
        pelvis="pelvis",
        left_foot="left_ankle_roll_link",
        right_foot="right_ankle_roll_link",
        left_thigh="left_hip_pitch_link",
        right_thigh="right_hip_pitch_link",
        torso="waist_pitch_joint",
        left_hip_sagittal="left_hip_pitch_joint",
        right_hip_sagittal="right_hip_pitch_joint",
        left_hip_coronal="left_hip_roll_joint",
        right_hip_coronal="right_hip_roll_joint",
        left_knee="left_knee_joint",
        right_knee="right_knee_joint",
        left_ankle_sagittal="left_ankle_pitch_joint",
        right_ankle_sagittal="right_ankle_pitch_joint",
        kp=200.0,
        kd=8.0,
        cd=0.5,
        cv=0.2,
        torso_kp=400.0,
        torso_kd=1.0,
        # The shared ground.skel slab sits at world z=-0.95 (top), so the G1
        # pelvis operates around -0.19 once standing on it.
        min_pelvis_height=-0.60,
        max_pelvis_height=0.30,
        foot_contact_height=0.07,
        ground_height=-0.95,
        walk_torso=0.0,
        swing_hip_up=0.50,
        swing_knee_up=-1.10,
        swing_ankle_up=0.50,
        stance_knee_up=-0.05,
        stance_ankle_up=0.0,
        swing_hip_down=-0.10,
        swing_knee_down=-0.30,
        swing_ankle_down=0.15,
        stance_knee_down=-0.10,
        stance_ankle_down=0.0,
    )


# ----------------------------------------------------------- scene builder ---
def build_simbicon_setup(robots):
    """Build a Z-up world with a static ground and one or more SIMBICON robots.

    ``robots`` is an iterable of ``(loader, config, x_offset)`` tuples. Returns
    ``(world, controllers, pre_step)`` where ``pre_step`` advances every
    controller and ``controllers`` is the list of :class:`SimbiconController`.
    """
    # The bundled ground.skel is a welded (static) Y-up slab; reorient it to the
    # canonical Z-up convention (this also sets gravity to -Z).
    world = dart.io.SkelParser.read_world(_GROUND_URI)
    if world is None:
        raise RuntimeError(f"Failed to load ground from {_GROUND_URI}")
    reorient_to_z_up(world)
    if (ground := world.get_skeleton(0)) is not None:
        ground.set_name("simbicon_ground")

    controllers = []
    for loader, config, x_offset in robots:
        robot = loader()
        if x_offset:
            positions = np.asarray(robot.get_positions())
            # Root is a FreeJoint: DOFs 3,4,5 are the base translation.
            positions[3] += float(x_offset)
            robot.set_positions(positions.tolist())
        world.add_skeleton(robot)
        controllers.append(SimbiconController(world, robot, config))

    def pre_step() -> None:
        for controller in controllers:
            controller.pre_step()

    return world, controllers, pre_step
