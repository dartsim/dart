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
# Newton continuation from the current pose converges only linearly near full
# extension and aligned axes, so it gets a looser gate and more iterations
# than ssik's own tracker. Damping bounds steps along active limits.
TRACK_TOLERANCE = 1e-9
TRACK_ITERATIONS = 100
BOUNDARY_ITERATIONS = 20
TRACK_DAMPING = 1e-2
# A bounded step this small has stalled against the limits.
STALL = 1e-14
TRACK_REACH = 0.5
# The WAM's elbow, /j4: the shoulder-wrist distance alone fixes its angle.
ELBOW = 3


def swivel_solver(chain, wam):
    """Hold the current elbow's swivel, so redundancy follows body drags.

    Prefers Newton continuation from the current pose where the held swivel is
    ill-conditioned. When neither helps, also offers each branch's in-limit
    swivel nearest the current pose, then ssik's seeded solve. DART applies the
    valid solution nearest the current pose.
    """
    refinement = importlib.import_module("ssik.refinement")
    # SciPy is an ssik dependency.
    optimize = importlib.import_module("scipy.optimize")
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

    def zoom(chart, q, desired, swivels):
        # Zoom in on the least limit violation until some swivel is in limits,
        # then on the in-limit solution nearest q; arcs near limits are narrow.
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
        return best, best_distance

    def nearest_in_limits(chart, q, desired, swivel):
        # Also zoom from the held swivel: the coarse grid can lock onto a
        # distant arc and miss a narrow one next to the current pose.
        step = 2.0 * np.pi / SAMPLES
        starts = (
            np.linspace(-np.pi, np.pi, SAMPLES, endpoint=False),
            swivel + np.linspace(-step, step, SAMPLES),
        )
        return min((zoom(chart, q, desired, s) for s in starts), key=lambda r: r[1])[0]

    def jacobian(q):
        return refinement.kinbody_jacobian(chain.kinbody, q)

    def near(x, q):
        return x is not None and np.abs(x - q).max() <= TRACK_REACH

    def track(q, desired, iterations):
        # ssik's damped Newton continuation from the current pose, with a
        # looser gate and a larger budget than its tracker.
        result = refinement.lm_refine(
            q,
            chain.fk,
            desired,
            fk_atol=TRACK_TOLERANCE,
            max_iters=iterations,
            jacobian_fn=jacobian,
            divergence_factor=np.inf,
            stall_patience=iterations,
        )
        x = None if result is None else into_limits(result[0])
        if not near(x, q) or not within_limits(x):
            x = track_within_limits(q, desired, iterations)
        return x if near(x, q) else None

    def track_within_limits(q, desired, iterations):
        # Where the unconstrained steps cross a limit or stall, continue again
        # with bounded least-squares steps that keep every joint in its limits.
        # Damping shrinks with the residual, as in Levenberg-Marquardt, and the
        # augmented system stays well posed where the Jacobian loses rank.
        x, best = q.copy(), None
        for _ in range(iterations):
            pose = chain.fk(x)
            error = np.abs(pose - desired).max()
            if error < TRACK_TOLERANCE:
                return x
            if error < FK_TOLERANCE:
                best = x.copy()
            twist = refinement.se3_log_residual(desired @ np.linalg.inv(pose))
            damping = np.sqrt(min(TRACK_DAMPING, twist @ twist)) * np.eye(x.size)
            step = optimize.lsq_linear(
                np.vstack([jacobian(x), damping]),
                np.concatenate([twist, np.zeros(x.size)]),
                bounds=(lower - x, upper - x),
                method="bvls",
            ).x
            moved = np.clip(x + np.clip(step, -TRACK_REACH, TRACK_REACH), lower, upper)
            if np.abs(moved - x).max() < STALL:
                break
            x = moved
        return x if np.abs(chain.fk(x) - desired).max() < FK_TOLERANCE else best

    def candidates(desired, q):
        motion = chain.self_motion(desired)
        # Measure the current elbow in the target's swivel basis, which ssik
        # rebuilds per pose and switches near a vertical shoulder-wrist line.
        swivel = motion.param_of(q)
        charts = list(motion)
        held = [into_limits(chart.q(swivel)) for chart in charts]
        # The target fixes each branch's elbow angle, whatever the swivel; if
        # none is in limits, nothing within limits reaches the target.
        elbows = {c[ELBOW] for c in held if lower[ELBOW] <= c[ELBOW] <= upper[ELBOW]}
        if charts and not elbows:
            return []
        held = [c for c in held if usable(c, desired)]
        move = min((np.abs(c - q).max() for c in held), default=np.inf)
        if move <= JUMP:
            return held
        # Newton continuation, as ssik's own trackers use, stays continuous
        # where the swivel is ill-conditioned or near limits. Without a chart
        # the target is out of reach unless it lies on the boundary at full
        # extension, so keep that attempt short.
        tracked = track(q, desired, TRACK_ITERATIONS if charts else BOUNDARY_ITERATIONS)
        # From the elbow fold at full extension, continuation can settle on the
        # wrong side of active limits; retry from each of the target's elbows.
        for elbow in sorted(elbows) if tracked is None else ():
            start = q.copy()
            start[ELBOW] = elbow
            tracked = track_within_limits(start, desired, TRACK_ITERATIONS)
            if near(tracked, q):
                break
            tracked = None
        if tracked is not None and np.abs(tracked - q).max() + JUMP < move:
            return [tracked]
        if move < FLIP:
            return held
        nearest = (nearest_in_limits(chart, q, desired, swivel) for chart in charts)
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
            try:
                found = candidates(desired, q)
            except np.linalg.LinAlgError:
                # A numerical failure must not disable the scene's refresh.
                found = []
            last[:] = [key, found]
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
