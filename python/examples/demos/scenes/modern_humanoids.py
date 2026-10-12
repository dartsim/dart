"""Fixed-base inspection of verified Atlas v5 and Unitree G1 bundles."""

from __future__ import annotations

import math

import dartpy as dart
import numpy as np

MODEL_URIS = {
    "atlas-v5": (
        "model://atlas-v5/0d92c25f336db51049a9516de46886838e6ea596/"
        "atlas_v5_no_head.urdf"
    ),
    "unitree-g1": (
        "model://unitree-g1/5994d4faef0a9cadd3287f8de0199a67eeb2a259/"
        "g1_29dof_mode_15.urdf"
    ),
}


class MissingModelsError(RuntimeError):
    """The verified model cache lacks a model; the message says how to fetch it."""


def _world(model_id: str, root_height: float):
    uri = MODEL_URIS[model_id]
    retriever = dart.utils.ModelResourceRetriever("", True)
    try:
        if not retriever.getFilePath(uri):
            raise RuntimeError("Verified model cache is unavailable.")
    except RuntimeError as error:
        raise MissingModelsError(
            f"{error} Prefetch with: pixi run fetch-robot-assets {model_id}"
        ) from error
    package = dart.utils.PackageResourceRetriever(retriever)
    package.addPackageDirectory(
        "atlas_description" if model_id == "atlas-v5" else "g1_description",
        uri.rsplit("/", 1)[0] + "/",
    )
    composite = dart.utils.CompositeResourceRetriever()
    composite.addSchemaRetriever("model", retriever)
    composite.addSchemaRetriever("package", package)
    loader = dart.utils.DartLoader()
    loader.setOptions(
        dart.utils.DartLoader.Options(
            composite, dart.utils.DartLoader.RootJointType.FIXED
        )
    )
    robot = loader.parseSkeleton(uri)
    if robot is None or robot.getNumDofs() == 0:
        raise RuntimeError(f"Unable to parse cached model: {uri}")
    transform = np.eye(4)
    transform[2, 3] = root_height
    robot.getRootJoint().setTransformFromParentBodyNode(transform)
    robot.setMobile(False)
    world = dart.simulation.World()
    world.setGravity([0, 0, 0])
    world.addSkeleton(robot)
    return world


def atlas_world():
    """Return the cached fixed-base Atlas world for offscreen capture."""
    return _world("atlas-v5", 1.0)


def g1_world():
    """Return the cached fixed-base G1 world for offscreen capture."""
    return _world("unitree-g1", 0.8)


def _build(world, root_height) -> dart.viewer.SceneSetup:
    robot = world.getSkeleton(0)
    initial_pose = robot.getPositions().copy()
    pending_joint = -1
    pending_position = 0.0
    reset = False

    panel = dart.viewer.Panel()
    panel.add_text("Fixed base; kinematic joint inspection.")
    panel.add_text(
        f"{robot.getNumBodyNodes()} bodies, {robot.getNumDofs()} joints to pose"
    )
    joint = panel.add_slider(
        "Joint", 0, min=0, max=robot.getNumDofs() - 1, integer=True
    )
    joint_name = panel.add_text(robot.getDof(0).getName())
    position = panel.add_slider("Position (rad)", 0.0, min=-math.pi, max=math.pi)
    reset_pose = panel.add_button("Reset pose")

    def pose_joint():
        nonlocal pending_joint, pending_position
        pending_joint = joint.value
        pending_position = position.value

    def reset_joint_positions():
        nonlocal reset, pending_joint
        reset = True
        pending_joint = -1

    def pre_refresh():
        nonlocal reset, pending_joint
        if reset:
            robot.setPositions(initial_pose)
            reset = False
        if pending_joint >= 0:
            robot.getDof(pending_joint).setPosition(pending_position)
            pending_joint = -1
        dof = robot.getDof(joint.value)
        joint_name.label = dof.getName()
        lower = dof.getPositionLowerLimit()
        upper = dof.getPositionUpperLimit()
        position.set_range(
            lower if math.isfinite(lower) else -math.pi,
            upper if math.isfinite(upper) else math.pi,
        )
        position.value = dof.getPosition()

    position.on_change = pose_joint
    reset_pose.on_change = reset_joint_positions
    pre_refresh()

    return dart.viewer.SceneSetup(
        world,
        pre_refresh=pre_refresh,
        shadows=False,
        camera_home=dart.viewer.CameraHome(
            eye=[6.0, 5.0, 3.0],
            center=[0.0, 0.0, root_height - 0.6],
            up=[0.0, 0.0, 1.0],
        ),
        allow_simulation=False,
        panel=panel,
    )


def _scene_world(load):
    # World factories stay usable without the app host (agent-capture); the
    # host reports missing models as a skipped scene.
    try:
        return load()
    except MissingModelsError as error:
        raise dart.viewer.AssetsNotPrefetchedError(str(error)) from error


def build_atlas() -> dart.viewer.SceneSetup:
    return _build(_scene_world(atlas_world), 1.0)


def build_g1() -> dart.viewer.SceneSetup:
    return _build(_scene_world(g1_world), 0.8)


def scenes():
    from . import atlas_v5, unitree_g1

    return [*atlas_v5.scenes(), *unitree_g1.scenes()]
