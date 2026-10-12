"""Fixed-base inspection of verified Atlas v5 and Unitree G1 bundles."""

from __future__ import annotations

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
    if robot is None:
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


def _build(world) -> dart.viewer.SceneSetup:
    robot = world.getSkeleton(0)
    initial_pose = robot.getPositions().copy()
    selected = 0

    def select_joint(delta):
        nonlocal selected
        selected = (selected + delta) % robot.getNumDofs()
        print(f"Selected joint: {robot.getDof(selected).getName()}")

    def pose_joint(delta):
        dof = robot.getDof(selected)
        position = min(
            dof.getPositionUpperLimit(),
            max(dof.getPositionLowerLimit(), dof.getPosition() + delta),
        )
        dof.setPosition(position)
        print(f"{dof.getName()}: {position:.3f} rad")

    panel = dart.viewer.Panel()
    panel.add_text(
        "Fixed base: [ / ] select a joint, - / = change its angle, t resets.",
        wrap=True,
    )
    panel.add_text(f"Initially selected joint: {robot.getDof(0).getName()}", wrap=True)

    return dart.viewer.SceneSetup(
        world,
        shadows=False,
        camera_home=dart.viewer.CameraHome(
            eye=[3.0, 2.5, 1.8],
            center=[
                0.0,
                0.0,
                robot.getRootBodyNode().getWorldTransform().translation()[2],
            ],
            up=[0.0, 0.0, 1.0],
        ),
        allow_simulation=False,
        key_actions=[
            dart.viewer.KeyAction("[", "Previous joint", lambda: select_joint(-1)),
            dart.viewer.KeyAction("]", "Next joint", lambda: select_joint(1)),
            dart.viewer.KeyAction(
                "-", "Decrease joint angle", lambda: pose_joint(-0.1)
            ),
            dart.viewer.KeyAction(
                "=", "Increase joint angle", lambda: pose_joint(0.1)
            ),
            dart.viewer.KeyAction(
                "t", "Reset pose", lambda: robot.setPositions(initial_pose)
            ),
        ],
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
    return _build(_scene_world(atlas_world))


def build_g1() -> dart.viewer.SceneSetup:
    return _build(_scene_world(g1_world))


def scenes():
    return [
        dart.viewer.Scene(
            "atlas_v5",
            title="Atlas v5 (No Head)",
            category="Robots",
            summary="Inspect a verified cached humanoid and pose its joints.",
            factory=build_atlas,
        ),
        dart.viewer.Scene(
            "unitree_g1",
            title="Unitree G1 (29 DOF)",
            category="Robots",
            summary="Inspect a verified cached humanoid and pose its joints.",
            factory=build_g1,
        ),
    ]
