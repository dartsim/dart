"""Interactively assemble jointed structures from balsa-wood blocks."""

from __future__ import annotations

import dartpy as dart
import numpy as np

BLOCK_LENGTH = 0.5
BLOCK_WIDTH = 0.075
JOINT_RADIUS = 1.5 * BLOCK_WIDTH / 2.0
BLOCK_MASS = 0.16 * 10e3 * BLOCK_LENGTH * BLOCK_WIDTH * BLOCK_WIDTH


def _transform(position=(0, 0, 0), angles=(0, 0, 0)):
    return dart.math.Isometry3(dart.math.eulerXYZToMatrix(np.radians(angles)), position)


class _Tinkertoy:
    def __init__(self):
        self.world = dart.simulation.World()
        self.viewer = None
        self.log = None
        self.picked_node = None
        self.picked_point = np.zeros(3)
        self.force_coeff = 100.0

        coordinates = dart.gui.InteractiveFrame(
            dart.dynamics.Frame.World(), "coordinates", dart.math.Isometry3(), 0.2
        )
        for tool in (
            dart.gui.InteractiveTool.LINEAR,
            dart.gui.InteractiveTool.ANGULAR,
            dart.gui.InteractiveTool.PLANAR,
        ):
            for coordinate in range(3):
                coordinates.getTool(tool, coordinate).setEnabled(False)
        self.world.addSimpleFrame(coordinates)
        self.target = dart.gui.InteractiveFrame(dart.dynamics.Frame.World())
        self.world.addSimpleFrame(self.target)

        self.joint_shapes = {
            "Weld": dart.dynamics.BoxShape(
                [2.0 * JOINT_RADIUS, BLOCK_WIDTH, BLOCK_WIDTH]
            ),
            "Revolute": dart.dynamics.CylinderShape(JOINT_RADIUS, 1.5 * BLOCK_WIDTH),
            "Ball": dart.dynamics.SphereShape(JOINT_RADIUS),
        }
        self.block_shape = dart.dynamics.BoxShape(
            [BLOCK_LENGTH, BLOCK_WIDTH, BLOCK_WIDTH]
        )
        for shape in (*self.joint_shapes.values(), self.block_shape):
            shape.addDataVariance(dart.dynamics.Shape.DYNAMIC_COLOR)
        self.block_offset = _transform((BLOCK_LENGTH / 2.0, 0, 0))
        self._create_initial_toys()

        line_frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
        self.force_line = dart.dynamics.LineSegmentShape(np.zeros(3), np.zeros(3), 3.0)
        self.force_line.addDataVariance(dart.dynamics.Shape.DYNAMIC_VERTICES)
        line_frame.setShape(self.force_line)
        line_frame.createVisualAspect().setColor([1.0, 0.63, 0.0, 1.0])
        self.world.addSimpleFrame(line_frame)

        self.panel = dart.viewer.Panel()
        self.selection_text = self.panel.add_text("Selected: (none)")
        self.force_control = self.panel.add_slider(
            "Force Coeff", self.force_coeff, min=10.0, max=1000.0, decimals=1
        )
        self.force_control.on_change = self._edit_force
        self.gravity_control = self.panel.add_checkbox("Gravity On/Off", True)
        self.gravity_control.on_change = self._edit_gravity
        self.panel.add_button("Reorient Target").on_change = self.reorient_target
        reset = self.panel.add_button("Reset Target")
        reset.same_line = True
        reset.on_change = self.clear_pick
        self.panel.add_text("Add block at target:")
        for index, kind in enumerate(self.joint_shapes):
            control = self.panel.add_button(kind)
            control.same_line = index > 0
            control.on_change = lambda kind=kind: self.add_at_target(kind)
        self.panel.add_button("Delete Block").on_change = self.delete_pick
        self.panel.add_text(
            "Left-click a block to select it (fuchsia/red while "
            "simulating/paused). While paused: 1/2/3 attach a new "
            "Weld/Revolute/Ball-jointed block to the target's pose; Delete "
            "removes the selection and its children. While simulating, "
            "dragging the target pulls the selection with a clamped spring "
            "force (Up/Down or the slider adjust its strength). Backspace "
            "clears the selection, '`' resets the target's rotation, 'g' "
            "toggles gravity.",
            wrap=True,
        )
        self.pre_refresh()

    def is_simulating(self):
        return self.viewer is not None and self.viewer.isSimulating()

    def add_block(self, kind, parent, transform):
        if self.is_simulating():
            if self.log:
                self.log(
                    "Tinkertoy: pause the simulation (Spacebar) before adding a block."
                )
            return None
        if parent is None:
            skeleton = dart.dynamics.Skeleton(
                f"toy_#{self.world.getNumSkeletons() + 1}"
            )
            self.world.addSkeleton(skeleton)
        else:
            skeleton = parent.getSkeleton()
        joint, body = getattr(skeleton, f"create{kind}JointAndBodyNodePair")(parent)
        body.setName(f"block_#{skeleton.getNumBodyNodes()}")
        joint.setName(f"joint_#{skeleton.getNumJoints()}")
        joint.setTransformFromParentBodyNode(transform)
        for i in range(joint.getNumDofs()):
            joint.setDampingCoefficient(i, 0.4)
        for shape in (self.joint_shapes[kind], self.block_shape):
            node = body.createShapeNode(shape)
            node.createVisualAspect()
            node.createCollisionAspect()
            node.createDynamicsAspect()
        node.setRelativeTransform(self.block_offset)
        body.setInertia(
            dart.dynamics.Inertia(
                BLOCK_MASS,
                [BLOCK_LENGTH / 2.0, 0, 0],
                self.block_shape.computeInertia(BLOCK_MASS),
            )
        )
        self.world.getConstraintSolver().getCollisionGroup().addShapeFramesOf(body)
        self.clear_pick()
        if kind == "Revolute":
            joint.setAxis([0, 0, 1])
        return body

    def _create_initial_toys(self):
        body = self.add_block("Ball", None, _transform(angles=(0, 45, 0)))
        body = self.add_block(
            "Revolute", body, _transform((BLOCK_LENGTH, 0, 0), (90, 0, 0))
        )
        body = self.add_block("Weld", body, _transform(angles=(0, 0, 90)))
        self.add_block(
            "Ball", body, _transform((BLOCK_LENGTH / 2.0, 0, BLOCK_WIDTH), (0, 0, -30))
        )

        body = self.add_block("Ball", None, _transform((-1, 0, 0), (0, 90, 0)))
        body = self.add_block(
            "Weld", body, _transform((BLOCK_LENGTH, 0, BLOCK_LENGTH / 2.0), (0, 90, 0))
        )
        self.add_block(
            "Revolute", body, _transform((0, 0, BLOCK_WIDTH / 2.0), (-90, 0, -90))
        )
        body = self.add_block(
            "Revolute",
            body,
            _transform((BLOCK_LENGTH, 0, BLOCK_WIDTH / 2.0), (-90, 0, -90)),
        )
        self.add_block("Ball", body, _transform((BLOCK_LENGTH, 0, 0)))

    def clear_pick(self):
        self.picked_node = None
        self.target.setTransform(dart.math.Isometry3())

    def reorient_target(self):
        transform = self.target.getWorldTransform()
        transform.set_rotation(np.eye(3))
        self.target.setTransform(transform)

    def add_at_target(self, kind):
        transform = (
            self.target.getTransform(self.picked_node)
            if self.picked_node is not None
            else self.target.getWorldTransform()
        )
        self.add_block(kind, self.picked_node, transform)

    def delete_pick(self):
        if self.picked_node is None:
            return
        if self.is_simulating():
            if self.log:
                self.log(
                    "Tinkertoy: pause the simulation (Spacebar) before deleting a block."
                )
            return
        removed = self.picked_node.remove()
        self.world.getConstraintSolver().getCollisionGroup().removeShapeFramesOf(
            removed
        )
        self.clear_pick()

    def _edit_force(self):
        self.force_coeff = float(np.float32(self.force_control.value))

    def change_force(self, increment):
        self.force_coeff = float(
            np.float32(min(1000.0, max(10.0, self.force_coeff + increment)))
        )

    def set_gravity(self, on):
        self.world.setGravity([0, 0, -9.81] if on else np.zeros(3))

    def _edit_gravity(self):
        self.set_gravity(self.gravity_control.value)

    def toggle_gravity(self):
        self.set_gravity(np.linalg.norm(self.world.getGravity()) <= 1e-8)

    def reset_camera(self):
        if self.viewer is not None:
            self.viewer.home()

    def handle_event(self, event):
        if event.getEventType() != dart.gui.GUIEventAdapter.PUSH or not (
            event.getButtonMask() & int(dart.gui.GUIEventAdapter.LEFT_MOUSE_BUTTON)
        ):
            return False
        # Native picking and gizmo DnD must run before relocating the target.
        default = self.viewer.getDefaultEventHandler()
        default.handle(event, dart.gui.GUIActionAdapter(self.viewer))
        picks = default.getButtonPicks(
            int(dart.gui.LEFT_MOUSE), int(dart.gui.BUTTON_PUSH)
        )
        if not picks:
            return True
        pick = picks[0]
        body = pick.frame.getParentFrame()
        if not isinstance(body, dart.dynamics.BodyNode):
            return True
        self.picked_node = body
        self.picked_point = body.getWorldTransform().inverse().multiply(pick.position)
        transform = body.getWorldTransform()
        transform.set_translation(
            pick.position
            + pick.normal / np.linalg.norm(pick.normal) * BLOCK_WIDTH / 2.0
        )
        self.target.setTransform(transform)
        return True

    def on_activate(self, ctx):
        self.viewer = ctx.viewer
        self.log = ctx.log
        ctx.add_event_handler(self.handle_event)
        ctx.enable_drag_and_drop(self.target)

    def pre_step(self):
        if self.picked_node is None:
            return
        force = self.force_coeff * (
            self.target.getWorldTransform().translation()
            - self.picked_node.getWorldTransform().multiply(self.picked_point)
        )
        norm = np.linalg.norm(force)
        if norm > 200.0:
            force = 200.0 * force / norm
        self.picked_node.addExtForce(force, self.picked_point)

    def pre_refresh(self):
        simulating = self.is_simulating()
        color = [0.5, 0.5, 1, 1] if simulating else [238 / 255.0, 201 / 255.0, 0, 1]
        for i in range(self.world.getNumSkeletons()):
            self.world.getSkeleton(i).setColor(color)
        if self.picked_node is not None:
            self.picked_node.setColor([1, 0, 1, 1] if simulating else [1, 0, 0, 1])
            self.force_line.setVertex(
                0, self.picked_node.getWorldTransform().multiply(self.picked_point)
            )
            self.force_line.setVertex(1, self.target.getWorldTransform().translation())
            self.selection_text.label = f"Selected: {self.picked_node.getName()}"
        else:
            self.force_line.setVertex(0, np.zeros(3))
            self.force_line.setVertex(1, np.zeros(3))
            self.selection_text.label = "Selected: (none)"
        self.force_control.value = self.force_coeff
        self.gravity_control.value = bool(
            np.linalg.norm(self.world.getGravity()) > 1e-8
        )


def build() -> dart.viewer.SceneSetup:
    state = _Tinkertoy()
    event = dart.gui.GUIEventAdapter
    actions = [
        (int(event.KEY_Tab), "Reset camera", state.reset_camera),
        ("1", "Add weld-joint block", lambda: state.add_at_target("Weld")),
        ("2", "Add revolute-joint block", lambda: state.add_at_target("Revolute")),
        ("3", "Add ball-joint block", lambda: state.add_at_target("Ball")),
        (int(event.KEY_BackSpace), "Clear selection", state.clear_pick),
        (int(event.KEY_Delete), "Delete picked block", state.delete_pick),
        (int(event.KEY_Up), "Increase force coeff", lambda: state.change_force(10.0)),
        (
            int(event.KEY_Down),
            "Decrease force coeff",
            lambda: state.change_force(-10.0),
        ),
        ("`", "Reorient target", state.reorient_target),
        ("g", "Toggle gravity", state.toggle_gravity),
    ]
    return dart.viewer.SceneSetup(
        state.world,
        pre_step=state.pre_step,
        pre_refresh=state.pre_refresh,
        on_activate=state.on_activate,
        key_actions=[dart.viewer.KeyAction(*action) for action in actions],
        panel=state.panel,
        camera_home=dart.viewer.CameraHome(
            eye=[3, 3, 2], center=[0, 0, 0.3], up=[0, 0, 1]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "tinkertoy",
            title="Tinkertoy",
            category="Constraints & Joints",
            summary="Interactively assemble jointed structures from balsa-wood blocks.",
            factory=build,
        )
    ]
