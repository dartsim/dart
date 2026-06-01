"""Robot models and SIMBICON configs for the py-demos SIMBICON scenes.

Provides :func:`make_atlas_config`/:func:`make_g1_config` (the per-robot role
names and gait parameters consumed by :class:`._simbicon.SimbiconController`)
and loaders for the Atlas (bundled) and Unitree G1 (fetched + cached locally,
since dartpy has no HTTP resource retriever) skeletons.
"""

from __future__ import annotations

from collections import deque
import math
import os
import pathlib
import re
import urllib.error
import urllib.request

import numpy as np

import dartpy as dart

from ..runner import ScenePanel
from ._simbicon import SimbiconConfig, SimbiconController
from ._z_up import reorient_to_z_up

_GROUND_URI = "dart://sample/skel/ground.skel"

_DEG = math.pi / 180.0

_ATLAS_URI = "dart://sample/sdf/atlas/atlas_v5_no_head.urdf"

# Unitree G1 (29-DOF) lives in a remote ROS package; dartpy can't fetch it, so
# we download the URDF + meshes once into a local cache and load by path. Pin a
# specific upstream commit (not a moving branch) so the fetched assets are
# reproducible and don't drift or 404 if `master` is rewritten.
_G1_REF = "7a48a28ab993614b8cd2059ddcb9b91ba4780d6f"
_G1_BASE = (
    "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
    f"{_G1_REF}/robots/g1_description"
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
        # Stop the gradual crouch-collapse: extend the stance knee toward the
        # standing pelvis height. 2.0 is the sweep sweet spot (Atlas survives
        # 3000+ steps; higher values over-correct and destabilize the knee).
        height_kp=2.0,
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
    """Download the G1 URDF + referenced meshes into the local cache (once).

    DART can't bundle the third-party G1 meshes and dartpy has no HTTP resource
    retriever, so the assets are fetched once from the pinned upstream commit
    into ``_G1_CACHE``. On a network failure (offline CI, proxy, rate limit)
    this raises a clear :class:`RuntimeError` instead of leaving a half-built
    cache; the demo runner turns that into a logged skip rather than a silent
    empty world. Pre-populate ``_G1_CACHE`` on a networked machine to run the
    G1 demos offline.
    """
    _G1_CACHE.mkdir(parents=True, exist_ok=True)

    def fetch(rel: str) -> None:
        dst = _G1_CACHE / rel
        if dst.exists() and dst.stat().st_size > 0:
            return
        dst.parent.mkdir(parents=True, exist_ok=True)
        url = f"{_G1_BASE}/{rel}"
        # Download to a sibling temp file and atomically rename so an
        # interrupted fetch never leaves a truncated file that the size check
        # above would later mistake for a complete asset.
        tmp = dst.with_name(dst.name + ".part")
        try:
            urllib.request.urlretrieve(url, tmp)
        except (urllib.error.URLError, OSError) as exc:
            tmp.unlink(missing_ok=True)
            raise RuntimeError(
                f"Could not fetch the Unitree G1 asset '{rel}' from {url}. The "
                "G1 SIMBICON demos need a one-time download of the G1 "
                f"description; pre-populate {_G1_CACHE} on a networked machine "
                "to run them offline."
            ) from exc
        os.replace(tmp, dst)

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
        # Stop the gradual crouch-collapse (see make_atlas_config). 2.0 is the
        # sweep sweet spot for G1 too (survival nearly doubles).
        height_kp=2.0,
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


def make_simbicon_panel(title: str, controllers: list[SimbiconController]) -> ScenePanel:
    histories = {
        controller.cfg.name: {
            "pelvis_height": deque(maxlen=120),
            "balance_sagittal": deque(maxlen=120),
            "balance_coronal": deque(maxlen=120),
        }
        for controller in controllers
    }

    def build_panel(builder: object, context: object) -> None:
        for index, controller in enumerate(controllers):
            diag = controller.diagnostics()
            name = str(diag["name"])
            history = histories[name]
            pelvis_height = float(diag["pelvis_height"])
            balance_sagittal = float(diag["balance_sagittal"])
            balance_coronal = float(diag["balance_coronal"])
            history["pelvis_height"].append(pelvis_height)
            history["balance_sagittal"].append(balance_sagittal)
            history["balance_coronal"].append(balance_coronal)
            builder.text(f"{name}: state {diag['state']} swing {diag['swing']}")
            builder.text(f"control enabled: {diag['control_enabled']}")
            builder.text(f"state time: {float(diag['state_time']):.3f} s")
            builder.text(f"pelvis height: {pelvis_height:.3f} m")
            builder.text(
                f"balance d/v sag: {balance_sagittal:.3f} m, "
                f"{float(diag['balance_sagittal_velocity']):.3f} m/s"
            )
            builder.text(
                f"balance d/v cor: {balance_coronal:.3f} m, "
                f"{float(diag['balance_coronal_velocity']):.3f} m/s"
            )
            builder.separator()
            builder.plot_lines(f"{name} z", list(history["pelvis_height"]))
            builder.plot_lines(f"{name} sag", list(history["balance_sagittal"]))
            builder.plot_lines(f"{name} cor", list(history["balance_coronal"]))
            if index + 1 < len(controllers):
                builder.separator()

    return ScenePanel(title, build_panel)
