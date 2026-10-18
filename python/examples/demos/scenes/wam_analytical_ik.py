"""A draggable WAM arm posed with ssik's closed-form 7R analytical IK."""

import importlib
import weakref
from pathlib import Path

import dartpy as dart
import numpy as np


def ssik_chain(ssik, wam, ee):
    """Build ssik's chain from DART's own joint transforms, axes, and limits."""
    joints = [
        wam.getJoint(i)
        for i in range(wam.getNumJoints())
        if wam.getJoint(i).getNumDofs()
    ]
    trafos = []
    child_to_joint = np.eye(4)
    for joint in joints:
        trafos.append(
            child_to_joint @ joint.getTransformFromParentBodyNode().matrix()
        )
        child_to_joint = np.linalg.inv(
            joint.getTransformFromChildBodyNode().matrix()
        )
    trafos.append(child_to_joint @ ee.getRelativeTransform().matrix())
    return ssik.Manipulator.from_transforms(
        trafos,
        [joint.getAxis() for joint in joints],
        limits=np.column_stack(
            [wam.getPositionLowerLimits(), wam.getPositionUpperLimits()]
        ),
    )


# A held swivel that would turn a joint this far is ill-conditioned, as at the
# straight-up rest pose, where the shoulder-wrist line is the base axis.
FLIP = 1.0
# Keep the held swivel unless Newton continuation from the current pose reaches
# the target with this much less joint motion, as near wrist singularities.
JUMP = 0.5
# Fallback swivels are searched coarse to fine, because ssik's native
# Chart.in_limits can stall on the degenerate chart of a singular target, such
# as the straight-up rest pose turned in place.
SAMPLES = 64
LEVELS = 8
# Charts close forward kinematics to about 1e-7 at a fold, but can miss by far
# more near a wrist singularity.
FK_TOLERANCE = 1e-6


def swivel_solver(chain, wam):
    """Hold the current elbow's swivel, so redundancy follows body drags.

    Prefers Newton continuation from the current pose where the held swivel is
    ill-conditioned. When neither helps, also offers each branch's in-limit
    swivel nearest the current pose, then ssik's seeded solve. DART applies the
    valid solution nearest the current pose.
    """
    refinement = importlib.import_module("ssik.refinement")
    lower = wam.getPositionLowerLimits()
    upper = wam.getPositionUpperLimits()
    center = 0.5 * (lower + upper)
    # The skeleton owns this callback, so a strong reference would leak it.
    wam = weakref.ref(wam)
    # DART calls again with the same pose while it iterates; reuse the answer.
    last = [None, []]

    def into_limits(q):
        # Charts return any turn of each angle; DART checks limits literally,
        # so also snap roundoff past a limit onto it.
        q = q - 2.0 * np.pi * np.round((q - center) / (2.0 * np.pi))
        bounded = np.clip(q, lower, upper)
        return np.where(np.abs(q - bounded) < 1e-9, bounded, q)

    def within_limits(q):
        # Also rejects NaN, which marks a swivel where a branch does not exist.
        return np.all((lower <= q) & (q <= upper), axis=-1)

    def usable(q, desired):
        return within_limits(q) and np.abs(chain.fk(q) - desired).max() < FK_TOLERANCE

    def nearest_in_limits(chart, q, desired):
        # Zoom in on the least limit violation until some swivel is in limits,
        # then on the in-limit solution nearest q; arcs near limits are narrow.
        swivels = np.linspace(-np.pi, np.pi, SAMPLES, endpoint=False)
        best, best_distance = None, np.inf
        for _ in range(LEVELS):
            samples = into_limits(chart.q(swivels))
            valid = within_limits(samples)
            if valid.any():
                distances = np.where(valid, np.linalg.norm(samples - q, axis=1), np.inf)
                index = np.argmin(distances)
                if distances[index] < best_distance and usable(samples[index], desired):
                    best, best_distance = samples[index], distances[index]
            else:
                excess = np.maximum(lower - samples, 0) + np.maximum(samples - upper, 0)
                index = np.argmin(np.nan_to_num(excess.sum(axis=1), nan=np.inf))
            step = swivels[1] - swivels[0]
            swivels = swivels[index] + np.linspace(-step, step, SAMPLES)
        return best

    def jacobian(q):
        return refinement.kinbody_jacobian(chain.kinbody, q)

    def candidates(desired, q):
        motion = chain.self_motion(desired)
        # Measure the current elbow in the target's swivel basis, which ssik
        # rebuilds per pose and switches near a vertical shoulder-wrist line.
        swivel = motion.param_of(q)
        charts = list(motion)
        held = [into_limits(chart.q(swivel)) for chart in charts]
        held = [c for c in held if usable(c, desired)]
        move = min((np.abs(c - q).max() for c in held), default=np.inf)
        # Newton-continue from the current pose, as ssik's own trackers do: it
        # stays continuous where the swivel is ill-conditioned or near limits.
        tracked = refinement.seeded_track(q, chain.fk, jacobian, desired)
        if tracked is not None:
            tracked = into_limits(tracked.q)
            if usable(tracked, desired) and np.abs(tracked - q).max() + JUMP < move:
                return [tracked]
        if move < FLIP:
            return held
        nearest = (nearest_in_limits(chart, q, desired) for chart in charts)
        found = [c for c in nearest if c is not None]
        if not found:
            # The seeded solve also handles the singular continua charts miss;
            # an SRS arm has at most eight branches.
            seeded = chain.solve(
                desired, q_seed=q, max_solutions=8, allow_rescue=False
            )
            found = [into_limits(s.q) for s in seeded]
            found = [c for c in found if usable(c, desired)]
        return held + found

    def solve(desired):
        desired = np.asarray(desired)
        q = wam().getPositions()
        key = desired.tobytes() + q.tobytes()
        if last[0] != key:
            last[:] = [key, candidates(desired, q)]
        return last[1]

    return solve


def build() -> dart.viewer.SceneSetup:
    try:
        ssik = importlib.import_module("ssik")
        ssik.Manipulator.self_motion
    except (ImportError, AttributeError) as error:
        raise dart.viewer.AssetsNotPrefetchedError(
            "Install ssik>=8.1.0 in this Python environment with: "
            "pixi run install-ssik"
        ) from error
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
    ik.setPythonAnalytical(
        swivel_solver(ssik_chain(ssik, wam, ee), wam), methodName="ssik SRS"
    )
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
            "wam_analytical_ik",
            title="WAM Analytical IK",
            category="Control & IK",
            summary=(
                "WAM arm posed by ssik closed-form analytical IK (optional ssik "
                "package), fully draggable."
            ),
            factory=build,
        )
    ]
