"""Atlas whole-body IK puppet with draggable end-effector targets and supports."""

from functools import partial

import dartpy as dart
import numpy as np

from examples.demos.models.relaxed_posture import RelaxedPosture


class _Puppet:
    def __init__(self, atlas, world, posture, balance):
        self.atlas = atlas
        self.world = world
        self.posture = posture
        self.balance = balance
        self.rest_config = atlas.getPositions().copy()
        self.end_effectors = [atlas.getEndEffector(index) for index in range(4)]
        self.default_bounds = [
            tuple(bound.copy() for bound in ee.getIK().getErrorMethod().getBounds())
            for ee in self.end_effectors
        ]
        self.default_target_tf = [
            ee.getIK().getTarget().getRelativeTransform() for ee in self.end_effectors
        ]
        self.constraint_active = [False] * 4
        self.movement = {key: False for key in "qweasdfz"}
        self.consecutive_failures = 0
        self.reported_failing = False
        self.log = None
        self.panel = dart.viewer.Panel()
        self.status = self.panel.add_text("")
        self.panel.add_text(
            "Purely kinematic (physics stepping is disabled for this scene). "
            "Alt+drag translates a body, Ctrl+drag rotates it, Shift+drag "
            "moves it via its parent joint. W/A/S/D move the robot, Q/E "
            "rotate it, F/Z change elevation; hold 'r' to optimize posture "
            "and balance. Green = support polygon, blue/red ball = center of "
            "mass, green ball = centroid.",
            wrap=True,
        )
        self._refresh_status()

    def _refresh_status(self):
        self.status.label = "Targets: " + "  ".join(
            f"{ee.getName()} {'on' if active else 'off'}"
            for ee, active in zip(self.end_effectors, self.constraint_active)
        )

    def pre_refresh(self):
        if any(self.movement.values()):
            old_tf = self.atlas.getBodyNode(0).getWorldTransform()
            forward = old_tf.rotation()[:, 0].copy()
            left = old_tf.rotation()[:, 1].copy()
            for direction in (forward, left):
                direction[2] = 0.0
                norm = np.linalg.norm(direction)
                direction[:] = direction / norm if norm > 1e-10 else 0.0
            movement = self.movement
            delta = (
                0.01 * (movement["w"] - movement["s"]) * forward
                + 0.01 * (movement["a"] - movement["d"]) * left
                + np.array([0.0, 0.0, 0.002 * (movement["f"] - movement["z"])])
            )
            rotation = dart.math.eulerXYZToMatrix(
                [0.0, 0.0, np.deg2rad(2.0) * (movement["q"] - movement["e"])]
            )
            tf = dart.math.Isometry3()
            tf.set_translation(delta + old_tf.translation())
            tf.set_rotation(rotation @ old_tf.rotation())
            self.atlas.getJoint(0).setPositions(
                dart.dynamics.FreeJoint.convertToPositions(tf)
            )
        if self.atlas.getIK(True).solveAndApply(True):
            self.consecutive_failures = 0
            self.reported_failing = False
        else:
            self.consecutive_failures += 1
        if self.consecutive_failures >= 1000 and not self.reported_failing:
            self.reported_failing = True
            if self.log is not None:
                self.log("atlas_puppet: IK solver has failed 1000 consecutive times.")
        self._refresh_status()

    def handle_event(self, event):
        event_type = event.getEventType()
        down = event_type == dart.gui.GUIEventAdapter.KEYDOWN
        if not down and event_type != dart.gui.GUIEventAdapter.KEYUP:
            return False
        key = event.getKey()
        if key == ord("r"):
            self.posture.enforce_ideal_posture = down
            method = dart.constraint.BalanceConstraint.ErrorMethod
            self.balance.setErrorMethod(
                method.OPTIMIZE_BALANCE if down else method.FROM_CENTROID
            )
            return True
        if key in map(ord, self.movement):
            self.movement[chr(key)] = down
            return True
        return False

    def toggle_target(self, index):
        ee = self.end_effectors[index]
        ik = ee.getIK()
        target = ik.getTarget()
        self.constraint_active[index] = not self.constraint_active[index]
        if self.constraint_active[index]:
            ik.getErrorMethod().setBounds()
            target.setTransform(ee.getTransform())
            self.world.addSimpleFrame(target)
        else:
            ik.getErrorMethod().setBounds(*self.default_bounds[index])
            target.setRelativeTransform(self.default_target_tf[index])
            self.world.removeSimpleFrame(target)

    def toggle_support(self, name):
        support = self.atlas.getEndEffector(name).getSupport()
        support.setActive(not support.isActive())

    def reset_to_rest(self):
        for index in range(self.atlas.getNumDofs()):
            if index < 2 or index > 4:
                self.atlas.getDof(index).setPosition(self.rest_config[index])

    def print_dof_values(self):
        if self.log is not None:
            for index in range(self.atlas.getNumDofs()):
                dof = self.atlas.getDof(index)
                self.log(f"{dof.getName()}: {dof.getPosition():.6g}")

    def on_activate(self, ctx):
        self.log = ctx.log
        for index in range(self.atlas.getNumBodyNodes()):
            ctx.enable_drag_and_drop(
                self.atlas.getBodyNode(index),
                use_external_ik=False,
                use_whole_body=False,
            )
        for ee in self.end_effectors:
            ctx.enable_drag_and_drop(ee.getIK().getTarget())
        ctx.add_attachment(dart.gui.SupportPolygonVisual(self.atlas, elevation=0.05))
        ctx.add_event_handler(self.handle_event)


def _create_ground():
    ground = dart.dynamics.Skeleton("ground")
    joint, body = ground.createWeldJointAndBodyNodePair()
    tf = dart.math.Isometry3()
    tf.set_translation([0.0, 0.0, -0.005])
    joint.setTransformFromParentBodyNode(tf)
    shape = body.createShapeNode(dart.dynamics.BoxShape([10.0, 10.0, 0.01]))
    shape.createVisualAspect().setColor([0.1, 0.1, 0.9, 0.2])
    shape.createCollisionAspect()
    shape.createDynamicsAspect()
    return ground


def _create_atlas():
    uri = "dart://sample/sdf/atlas/atlas_v3_no_head.urdf"
    atlas = dart.utils.DartLoader().parseSkeleton(uri)
    if atlas is None:
        raise RuntimeError(f"failed to load {uri}")
    shape = atlas.getBodyNode(0).createShapeNode(
        dart.dynamics.BoxShape([0.25, 0.25, 0.125])
    )
    shape.createVisualAspect().setColor([0.0, 0.0, 0.0, 1.0])
    tf = dart.math.Isometry3()
    tf.set_translation([0.0, 0.0, 0.1])
    shape.setRelativeTransform(tf)
    return atlas


def _setup_start_configuration(atlas):
    atlas.getDof("r_leg_hpy").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("r_leg_kny").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("r_leg_aky").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("l_leg_hpy").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("l_leg_kny").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("l_leg_aky").setPosition(-45.0 * np.pi / 180.0)

    atlas.getDof("r_arm_shx").setPosition(65.0 * np.pi / 180.0)
    atlas.getDof("r_arm_ely").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("r_arm_elx").setPosition(-90.0 * np.pi / 180.0)
    atlas.getDof("r_arm_wry").setPosition(65.0 * np.pi / 180.0)
    atlas.getDof("l_arm_shx").setPosition(-65.0 * np.pi / 180.0)
    atlas.getDof("l_arm_ely").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("l_arm_elx").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("l_arm_wry").setPosition(65.0 * np.pi / 180.0)

    atlas.getDof("r_leg_kny").setPositionLowerLimit(10 * np.pi / 180.0)
    atlas.getDof("l_leg_kny").setPositionLowerLimit(10 * np.pi / 180.0)


def _setup_end_effectors(atlas):
    infinity = np.full(3, np.inf)
    hand_tf = dart.math.Isometry3()
    hand_tf.set_translation([0.0009, 0.1254, 0.012])
    hand_tf.set_rotation(dart.math.eulerXYZToMatrix([0.0, 0.0, np.pi / 2.0]))
    for side, frame_name in (("l", "lh_target"), ("r", "rh_target")):
        if side == "r":
            hand_tf.set_translation([-0.0009, -0.1254, 0.012])
            hand_tf.set_rotation(np.linalg.inv(hand_tf.rotation()))
        ee = atlas.getBodyNode(f"{side}_hand").createEndEffector(f"{side}_hand")
        ee.setDefaultRelativeTransform(hand_tf, True)
        ik = ee.getIK(True)
        ik.setTarget(dart.gui.InteractiveFrame(dart.dynamics.Frame.World(), frame_name))
        ik.useWholeBody()
        ik.getGradientMethod().setComponentWeights(0.01 * np.ones(6))
        ik.getErrorMethod().setLinearBounds(-infinity, infinity)
        ik.getErrorMethod().setAngularBounds(-infinity, infinity)

    support = [
        [-0.03 - 0.186, -0.03, 0.0],
        [0.10 - 0.186, -0.03, 0.0],
        [0.10 - 0.186, 0.03, 0.0],
        [-0.03 - 0.186, 0.03, 0.0],
    ]
    foot_tf = dart.math.Isometry3()
    foot_tf.set_translation([0.186, 0.0, -0.08])
    linear_bounds = np.array([np.inf, np.inf, 1e-8])
    angular_bounds = np.array([1e-8, 1e-8, np.inf])
    for side, frame_name in (("l", "lf_target"), ("r", "rf_target")):
        ee = atlas.getBodyNode(f"{side}_foot").createEndEffector(f"{side}_foot")
        ee.setRelativeTransform(foot_tf)
        ik = ee.getIK(True)
        ik.setTarget(dart.gui.InteractiveFrame(dart.dynamics.Frame.World(), frame_name))
        ik.setHierarchyLevel(1)
        ik.getErrorMethod().setLinearBounds(-linear_bounds, linear_bounds)
        ik.getErrorMethod().setAngularBounds(-angular_bounds, angular_bounds)
        ee.getSupport(True).setGeometry(support)
        ee.getSupport().setActive()
    atlas.getDof(5).setPosition(
        -atlas.getEndEffector("r_foot").getWorldTransform().translation()[2]
    )
    for name in ("l_foot", "r_foot"):
        ee = atlas.getEndEffector(name)
        ee.getIK().getTarget().setTransform(ee.getTransform())


def _setup_whole_body_solver(atlas):
    ik = atlas.getIK(True)
    ik.getSolver().setNumMaxIterations(10)
    weights = 0.01 * np.ones(atlas.getNumDofs())
    weights[2:5] = 0.0
    weights[6:9] *= 0.2
    lower = np.full(atlas.getNumDofs(), -np.inf)
    upper = np.full(atlas.getNumDofs(), np.inf)
    indices = [0, 1, 5, 6, 7, 8]
    lower[indices] = [-0.35, -0.35, 0.600, -0.1, -0.1, -0.1]
    upper[indices] = [0.35, 0.35, 0.885, 0.1, 0.1, 0.1]
    posture = RelaxedPosture(atlas.getPositions(), lower, upper, weights)
    ik.setObjective(posture)
    balance = dart.constraint.BalanceConstraint(ik)
    ik.getProblem().addEqConstraint(balance)
    balance.setErrorMethod(dart.constraint.BalanceConstraint.ErrorMethod.FROM_CENTROID)
    balance.setBalanceMethod(
        dart.constraint.BalanceConstraint.BalanceMethod.SHIFT_SUPPORT
    )
    return posture, balance


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()
    world.setGravity([0.0, 0.0, 0.0])
    atlas = _create_atlas()
    world.addSkeleton(atlas)
    world.addSkeleton(_create_ground())
    _setup_start_configuration(atlas)
    _setup_end_effectors(atlas)
    state = _Puppet(atlas, world, *_setup_whole_body_solver(atlas))
    keys = [
        dart.viewer.KeyAction(
            str(index + 1),
            f"Toggle {ee.getName()} target",
            partial(state.toggle_target, index),
        )
        for index, ee in enumerate(state.end_effectors)
    ]
    keys.extend(
        [
            dart.viewer.KeyAction(
                "x", "Toggle l_foot support", partial(state.toggle_support, "l_foot")
            ),
            dart.viewer.KeyAction(
                "c", "Toggle r_foot support", partial(state.toggle_support, "r_foot")
            ),
            dart.viewer.KeyAction("t", "Reset to rest pose", state.reset_to_rest),
            dart.viewer.KeyAction("p", "Print DOF values", state.print_dof_values),
        ]
    )
    return dart.viewer.SceneSetup(
        world,
        pre_refresh=state.pre_refresh,
        on_activate=state.on_activate,
        key_actions=keys,
        panel=state.panel,
        allow_simulation=False,
        camera_home=dart.viewer.CameraHome(
            eye=[5.34, 3.00, 2.41], center=[0.00, 0.00, 1.00], up=[-0.20, -0.08, 0.98]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "atlas_puppet",
            title="Atlas Puppet",
            category="Control & IK",
            summary="Atlas whole-body IK puppet with balance and support overlays.",
            factory=build,
        )
    ]
