"""Control & IK > atlas_puppet.

Port of ``python/examples/atlas_puppet/main.py``: a purely kinematic Atlas
whole-body IK puppet, with draggable hand/foot targets and WASD-style base
movement.

Significantly reworked from the original, which builds its whole IK rig on
``dart.dynamics.EndEffector`` (per-limb offset transforms, foot Support
polygons feeding a ``BalanceConstraint``). ``EndEffector`` is not bound in
dartpy at all -- no ``createEndEffector``, no ``EndEffector`` class, nothing
beyond ``Skeleton.getNumEndEffectors()``. Since ``BalanceConstraint`` computes
its cost from
``Skeleton::getSupportPolygon()``, which is itself built from active
EndEffector supports, it is unusable with zero EndEffectors (empty support
polygon) -- not merely cosmetically degraded, so this port drops it rather
than risk NaNs from an empty-polygon balance term.

What this port keeps, using only already-bound APIs (no new bindings):
whole-body IK on four BodyNodes directly (l_hand, r_hand, l_foot, r_foot)
via ``BodyNode.getIK(True)`` -- the same ``InverseKinematics`` machinery the
original used, just attached to the raw body instead of an EndEffector --
draggable ``InteractiveFrame`` targets, the F1-F4 per-target
constrain/release toggle, WASD/QE/FZ base movement, and 't' reset to the
relaxed pose.

Dropped entirely (needs EndEffector, unreachable from Python): per-hand/foot
contact-offset transforms (targets track the raw body frame instead), the
foot Support polygons, the 'x'/'c' foot-support toggle, the
``BalanceConstraint``, and ``SupportPolygonVisual`` (the one gap the
standalone example already hits at ``main.py:552``, which is why it's broken
today).

Also dropped: the original's ``RelaxedPosture`` objective + 'r'-key
hold-to-tighten-posture. That objective was installed on
``atlas.getIK(True)`` -- a Skeleton-level ``WholeBodyIK``, not the
per-BodyNode ``InverseKinematics`` used above. dartpy only binds
``Skeleton.getIK()`` with no ``createIfNull`` overload and no way to
construct/attach a ``WholeBodyIK`` from Python, so there is no reachable way
to install a whole-skeleton objective at all. Each of the 4 targets instead
solves independently through its own per-BodyNode IK, each still using
``useWholeBody()``, so simultaneously-dragged targets converge by
alternating rather than one joint optimization.

The standalone example also has three more latent breakages fixed here with
already-bound APIs: a generic ``Skeleton.createJointAndBodyNodePair(parent,
jointProperties, bodyProperties)`` call and a constructible
``WeldJoint.Properties`` type, neither bound (fixed via
``createWeldJointAndBodyNodePair(parent)`` +
``setTransformFromParentBodyNode``); ``BodyNode.createShapeNodeWith(...)``,
also unbound (fixed via ``createShapeNode(shape)`` + individual
``create*Aspect()`` calls); and ``dart.math.eulerToMatrix(...)``, which
dartpy only exposes per-rotation-order (fixed via ``eulerXYZToMatrix``,
order-irrelevant here since every call only rotates about Z).

Key remap (navigator reserves 0-9 for direct scene select, 'p'/'r' for
prev-scene/rebuild): the original's digit keys 1-4 (toggle a target) move to
F1-F4; 'p' (print dof positions) moves to 'v'. w/a/s/d/q/e/f/z/t are
unchanged (no collisions); 'o'/'x'/'c' are gone along with the
posture/foot-support features they drove.
"""

import dartpy as dart
import numpy as np

from ..registry import SceneHandle


class AtlasPuppetNode(dart.gui.osg.WorldNode):
    MOVE_Q, MOVE_W, MOVE_E, MOVE_A, MOVE_S, MOVE_D, MOVE_F, MOVE_Z = range(8)
    NUM_MOVE = 8

    def __init__(self, world, atlas, targets):
        super().__init__(world)
        self.atlas = atlas
        self.targets = targets  # [(bodynode, ik, frame), ...]
        self.iter = 0
        self.move_components = [False] * self.NUM_MOVE
        self.any_movement = False

    def setMovement(self, move_components):
        self.move_components = move_components
        self.any_movement = any(move_components)

    def customPreRefresh(self):
        if self.any_movement:
            old_tf = self.atlas.getBodyNode(0).getWorldTransform()
            new_tf = dart.math.Isometry3()

            forward = old_tf.rotation()[:, 0]
            forward[2] = 0.0
            if np.linalg.norm(forward) > 1e-10:
                forward = forward / np.linalg.norm(forward)
            else:
                forward[:] = 0.0

            left = old_tf.rotation()[:, 1]
            left[2] = 0.0
            if np.linalg.norm(left) > 1e-10:
                left = left / np.linalg.norm(left)
            else:
                left[:] = 0.0

            up = np.array([0.0, 0.0, 1.0])

            linear_step = 0.01
            elevation_step = 0.2 * linear_step
            rotational_step = 2.0 * np.pi / 180.0

            m = self.move_components
            if m[self.MOVE_W]:
                new_tf.set_translation(new_tf.translation() + linear_step * forward)
            if m[self.MOVE_S]:
                new_tf.set_translation(new_tf.translation() - linear_step * forward)
            if m[self.MOVE_A]:
                new_tf.set_translation(new_tf.translation() + linear_step * left)
            if m[self.MOVE_D]:
                new_tf.set_translation(new_tf.translation() - linear_step * left)
            if m[self.MOVE_F]:
                new_tf.set_translation(new_tf.translation() + elevation_step * up)
            if m[self.MOVE_Z]:
                new_tf.set_translation(new_tf.translation() - elevation_step * up)
            if m[self.MOVE_Q]:
                rot = dart.math.eulerXYZToMatrix([0, 0, rotational_step])
                new_tf.set_rotation(np.matmul(rot, new_tf.rotation()))
            if m[self.MOVE_E]:
                rot = dart.math.eulerXYZToMatrix([0, 0, -rotational_step])
                new_tf.set_rotation(np.matmul(rot, new_tf.rotation()))

            new_tf.set_translation(new_tf.translation() + old_tf.translation())
            new_tf.set_rotation(np.matmul(new_tf.rotation(), old_tf.rotation()))

            positions = dart.dynamics.FreeJoint.convertToPositions(new_tf)
            self.atlas.getJoint(0).setPositions(positions)

        # No Skeleton-level WholeBodyIK is reachable from Python (see module
        # docstring), so each target is solved independently rather than as
        # one joint whole-body optimization; each still uses useWholeBody()
        # internally, so simultaneously-engaged targets converge by
        # alternating rather than jointly.
        all_solved = True
        for _body, ik, _frame in self.targets:
            if not ik.solveAndApply(True):
                all_solved = False
        if all_solved:
            self.iter = 0
        else:
            self.iter += 1
        if self.iter == 1000:
            print("Failing!")


class AtlasPuppetInputHandler(dart.gui.osg.GUIEventHandler):
    """Handles atlas_puppet's own keyboard layout. Not registered with the
    OSG event system: the runner calls ``handle(ea, aa)`` on this instance
    directly whenever atlas_puppet is the active scene (dartpy has no
    ``Viewer.removeEventHandler`` binding to un-register a real handler on
    scene switch, so per-scene handlers are plain Python objects instead).
    """

    def __init__(self, atlas, world, teleop, targets):
        super().__init__()
        self.atlas = atlas
        self.world = world
        self.teleop = teleop
        self.targets = targets  # ordered [(bodynode, ik, frame), ...]
        self._initialize()

    def _initialize(self):
        self.rest_config = self.atlas.getPositions()

        self.default_bounds = [t[1].getErrorMethod().getBounds() for t in self.targets]
        self.default_target_tf = [
            t[1].getTarget().getRelativeTransform() for t in self.targets
        ]
        self.constraint_active = [False] * len(self.targets)
        self.move_components = [False] * AtlasPuppetNode.NUM_MOVE

    def handle(self, ea, aa):
        event_type = ea.getEventType()
        ea_cls = dart.gui.osg.GUIEventAdapter

        if event_type == ea_cls.KEYDOWN:
            key = ea.getKey()

            if key == ord("v"):
                for i in range(self.atlas.getNumDofs()):
                    print(
                        f"{self.atlas.getDof(i).getName()}: "
                        f"{self.atlas.getDof(i).getPosition()}"
                    )
                print("  -- -- -- -- -- ")
                return True

            if key == ord("t"):
                for i in range(self.atlas.getNumDofs()):
                    if i < 2 or 4 < i:
                        self.atlas.getDof(i).setPosition(self.rest_config[i])
                return True

            target_keys = (ea_cls.KEY_F1, ea_cls.KEY_F2, ea_cls.KEY_F3, ea_cls.KEY_F4)
            if key in target_keys:
                index = target_keys.index(key)
                if index < len(self.targets):
                    body, ik, _frame = self.targets[index]
                    if self.constraint_active[index]:
                        self.constraint_active[index] = False
                        ik.getErrorMethod().setBounds(*self.default_bounds[index])
                        ik.getTarget().setRelativeTransform(
                            self.default_target_tf[index]
                        )
                        self.world.removeSimpleFrame(ik.getTarget())
                    else:
                        self.constraint_active[index] = True
                        ik.getErrorMethod().setBounds()
                        ik.getTarget().setTransform(body.getTransform())
                        self.world.addSimpleFrame(ik.getTarget())
                return True

            move_keys = {
                ord("w"): AtlasPuppetNode.MOVE_W,
                ord("a"): AtlasPuppetNode.MOVE_A,
                ord("s"): AtlasPuppetNode.MOVE_S,
                ord("d"): AtlasPuppetNode.MOVE_D,
                ord("q"): AtlasPuppetNode.MOVE_Q,
                ord("e"): AtlasPuppetNode.MOVE_E,
                ord("f"): AtlasPuppetNode.MOVE_F,
                ord("z"): AtlasPuppetNode.MOVE_Z,
            }
            if key in move_keys:
                self.move_components[move_keys[key]] = True
                self.teleop.setMovement(self.move_components)
                return True

        if event_type == ea_cls.KEYUP:
            key = ea.getKey()

            move_keys = {
                ord("w"): AtlasPuppetNode.MOVE_W,
                ord("a"): AtlasPuppetNode.MOVE_A,
                ord("s"): AtlasPuppetNode.MOVE_S,
                ord("d"): AtlasPuppetNode.MOVE_D,
                ord("q"): AtlasPuppetNode.MOVE_Q,
                ord("e"): AtlasPuppetNode.MOVE_E,
                ord("f"): AtlasPuppetNode.MOVE_F,
                ord("z"): AtlasPuppetNode.MOVE_Z,
            }
            if key in move_keys:
                self.move_components[move_keys[key]] = False
                self.teleop.setMovement(self.move_components)
                return True

        return False


def _create_ground():
    ground = dart.dynamics.Skeleton("ground")
    tf = dart.math.Isometry3()
    thickness = 0.01
    tf.set_translation([0, 0, -thickness / 2.0])

    # The original standalone example builds a WeldJoint::Properties with a
    # custom parent-to-joint transform and passes it to a generic
    # createJointAndBodyNodePair(parent, jointProperties, bodyProperties).
    # Neither the generic method nor a constructible WeldJoint.Properties
    # type is bound in dartpy (only per-type createWeldJointAndBodyNodePair
    # overloads are, and none take a properties object dartpy can construct)
    # -- so this port creates the joint with defaults and sets the transform
    # via the separately-bound setTransformFromParentBodyNode instead.
    joint, bn = ground.createWeldJointAndBodyNodePair(None)
    joint.setTransformFromParentBodyNode(tf)
    bn.setName("ground_body")

    # Likewise, BodyNode.createShapeNodeWith(...) (aspect-bundling) is
    # unbound; only createShapeNode(shape[, name]) is, so aspects are
    # attached individually via the ShapeFrame API instead.
    shape_node = bn.createShapeNode(dart.dynamics.BoxShape([10, 10, thickness]))
    shape_node.createVisualAspect()
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()
    shape_node.getVisualAspect().setColor([0.2, 0.2, 1.0, 1.0])
    return ground


def _create_atlas():
    urdf = dart.utils.DartLoader()
    atlas = urdf.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf")

    scale = 0.25
    box_shape = dart.dynamics.BoxShape([scale * 1.0, scale * 1.0, scale * 0.5])
    tf = dart.math.Isometry3()
    tf.set_translation([0.0, 0.0, 0.1])

    shape_node = atlas.getBodyNode(0).createShapeNode(box_shape)
    shape_node.createVisualAspect()
    shape_node.getVisualAspect().setColor([0.0, 0.0, 0.0, 1.0])
    shape_node.setRelativeTransform(tf)
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


def _setup_ik_targets(atlas):
    """Whole-body IK on four BodyNodes directly (l_hand, r_hand, l_foot,
    r_foot). See the module docstring for why this targets the raw bodies
    instead of EndEffectors (unbound) -- the per-limb contact-offset
    transforms and foot Support polygons the original attached to each
    EndEffector are dropped along with it.
    """

    linear_bounds = np.full(3, np.inf)
    angular_bounds = np.full(3, np.inf)
    rootjoint_weights = 0.01 * np.ones(6)

    targets = []
    for body_name, frame_name in (
        ("l_hand", "lh_target"),
        ("r_hand", "rh_target"),
        ("l_foot", "lf_target"),
        ("r_foot", "rf_target"),
    ):
        body = atlas.getBodyNode(body_name)
        frame = dart.gui.osg.InteractiveFrame(dart.dynamics.Frame.World(), frame_name)

        ik = body.getIK(True)
        ik.setTarget(frame)
        ik.useWholeBody()
        ik.getGradientMethod().setComponentWeights(rootjoint_weights)
        ik.getErrorMethod().setLinearBounds(-linear_bounds, linear_bounds)
        ik.getErrorMethod().setAngularBounds(-angular_bounds, angular_bounds)
        solver = ik.getSolver()
        if hasattr(solver, "setNumMaxIterations"):
            solver.setNumMaxIterations(10)
        frame.setTransform(body.getTransform())

        targets.append((body, ik, frame))

    return targets


def build() -> SceneHandle:
    world = dart.simulation.World()

    atlas = _create_atlas()
    world.addSkeleton(atlas)
    world.addSkeleton(_create_ground())

    _setup_start_configuration(atlas)
    targets = _setup_ik_targets(atlas)

    node = AtlasPuppetNode(world, atlas, targets)
    handler = AtlasPuppetInputHandler(atlas, world, node, targets)

    drag_and_drop = [
        (atlas.getBodyNode(i), False, False) for i in range(atlas.getNumBodyNodes())
    ]
    drag_and_drop.extend(frame for _body, _ik, frame in targets)

    return SceneHandle(
        node=node,
        allow_simulation=False,
        drag_and_drop=drag_and_drop,
        extra_handler=handler,
        camera_home=([5.34, 3.00, 2.41], [0.00, 0.00, 1.00], [-0.20, -0.08, 0.98]),
        instructions=[
            "Alt + Click:   Try to translate a body without changing its orientation\n",
            "Ctrl + Click:  Try to rotate a body without changing its translation\n",
            "Shift + Click: Move a body using only its parent joint\n",
            "F1 -> F4:      Toggle the interactive target of a hand/foot\n",
            "W A S D:       Move the robot around the scene\n",
            "Q E:           Rotate the robot counter-clockwise and clockwise\n",
            "F Z:           Shift the robot's elevation up and down\n",
            "T:             Reset the robot to its relaxed posture\n",
            "V:             Print all dof positions\n",
            "Note: this is purely kinematic. Physical simulation is not allowed.\n",
            "Note: each hand/foot target solves independently (no whole-body\n",
            "posture objective is reachable from Python); see build() notes.\n",
        ],
        notes=(
            "EndEffector is unbound in dartpy (no createEndEffector/support "
            "polygon), so IK targets attach directly to l_hand/r_hand/l_foot/"
            "r_foot BodyNodes instead, and the foot-support/BalanceConstraint "
            "balancing feature is dropped (needs EndEffector-based support "
            "polygons). Skeleton-level WholeBodyIK is also unreachable "
            "(Skeleton.getIK() has no createIfNull overload and there is no "
            "way to construct/attach one from Python), so the RelaxedPosture "
            "objective and 'r'/hold-to-tighten-posture feature are dropped "
            "too; each of the 4 targets solves independently via its own "
            "per-BodyNode InverseKinematics instead of one joint whole-body "
            "solve. SupportPolygonVisual dropped too (unbound; the original "
            "standalone example crashes with AttributeError at that call). "
            "Keys remapped: 1-4 -> F1-F4, p -> v; o/x/c removed with the "
            "features they drove."
        ),
    )
