"""A draggable WAM arm posed with native analytical whole-body IK."""

from pathlib import Path

import dartpy as dart


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()
    world.setGravity([0.0, 0.0, 0.0])

    uri = "dart://sample/urdf/wam/wam.urdf"
    model_path = dart.utils.DartResourceRetriever().getFilePath(uri)
    if not model_path:
        raise RuntimeError(f"failed to load {uri}")
    loader = dart.utils.DartLoader()
    loader.addPackageDirectory("herb_description", str(Path(model_path).parent))
    wam = loader.parseSkeleton(uri)
    if wam is None:
        raise RuntimeError(f"failed to load {uri}")
    for name in ("/j1", "/j2", "/j3", "/j4", "/j5", "/j6", "/j7"):
        dof = wam.getDof(name)
        if dof is not None:
            dof.setPosition(0.0)

    hand = dart.math.Isometry3()
    hand.set_translation([0.0, 0.0, -0.09])
    wam7 = wam.getBodyNode("/wam7")
    if wam7 is None:
        raise RuntimeError("wam.urdf: missing body node /wam7")
    ee = wam7.createEndEffector("ee")
    ee.setDefaultRelativeTransform(hand, True)
    target = dart.gui.InteractiveFrame(dart.dynamics.Frame.World(), "lh_target")
    ik = ee.getIK(True)
    ik.setTarget(target)
    dart._demos.set_wam_ikfast(ik)
    infinity = [float("inf")] * 3
    negative_infinity = [-float("inf")] * 3
    ik.getErrorMethod().setLinearBounds(negative_infinity, infinity)
    ik.getErrorMethod().setAngularBounds(negative_infinity, infinity)
    world.addSkeleton(wam)

    ground = dart.dynamics.Skeleton("ground")
    joint, body = ground.createWeldJointAndBodyNodePair()
    ground_transform = dart.math.Isometry3()
    ground_transform.set_translation([0.0, 0.0, -0.005])
    joint.setTransformFromParentBodyNode(ground_transform)
    shape = body.createShapeNode(dart.dynamics.BoxShape([10.0, 10.0, 0.01]))
    shape.createVisualAspect().setColor([0.1, 0.1, 0.9, 0.2])
    shape.createCollisionAspect()
    shape.createDynamicsAspect()
    world.addSkeleton(ground)

    rest_config = wam.getPositions().copy()
    default_bounds = tuple(bound.copy() for bound in ik.getErrorMethod().getBounds())
    default_target_tf = target.getRelativeTransform()
    constraint_active = False
    log = None

    panel = dart.viewer.Panel()
    status = panel.add_text("IK target: inactive")
    panel.add_text(
        "Purely kinematic (physics stepping is disabled for this scene). "
        "Alt+drag translates a body, Ctrl+drag rotates it, Shift+drag "
        "moves it via its parent joint. '1' toggles the draggable IK "
        "target on the end effector; 't' resets to the rest pose; 'p' "
        "prints DOF values to the Diagnostics log.",
        wrap=True,
    )

    def pre_refresh():
        wam.getIK(True).solveAndApply(True)
        status.label = f"IK target: {'active' if constraint_active else 'inactive'}"

    def on_activate(ctx):
        nonlocal log
        log = ctx.log
        wam.getIK(True).solveAndApply(True)
        for index in range(wam.getNumBodyNodes()):
            ctx.enable_drag_and_drop(
                wam.getBodyNode(index), use_external_ik=False, use_whole_body=False
            )
        ctx.enable_drag_and_drop(target)

    def print_dof_values():
        if log is not None:
            for index in range(wam.getNumDofs()):
                dof = wam.getDof(index)
                log(f"{dof.getName()}: {dof.getPosition():.6g}")

    def toggle_target():
        nonlocal constraint_active
        constraint_active = not constraint_active
        if constraint_active:
            ik.getErrorMethod().setBounds()
            target.setTransform(ee.getTransform())
            world.addSimpleFrame(target)
        else:
            ik.getErrorMethod().setBounds(*default_bounds)
            target.setRelativeTransform(default_target_tf)
            world.removeSimpleFrame(target)

    return dart.viewer.SceneSetup(
        world,
        pre_refresh=pre_refresh,
        on_activate=on_activate,
        key_actions=[
            dart.viewer.KeyAction("p", "Print DOF values", print_dof_values),
            dart.viewer.KeyAction(
                "t", "Reset to rest pose", lambda: wam.setPositions(rest_config)
            ),
            dart.viewer.KeyAction("1", "Toggle IK target", toggle_target),
        ],
        panel=panel,
        allow_simulation=False,
        camera_home=dart.viewer.CameraHome(
            eye=[5.34, 3.0, 1.91], center=[0.0, 0.0, 0.5], up=[0.0, 0.0, 0.98]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "wam_ikfast",
            title="WAM IkFast",
            category="Control & IK",
            summary="WAM arm posed by IkFast analytical whole-body IK, fully draggable.",
            factory=build,
        )
    ]
