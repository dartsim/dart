"""Keyboard-selected rigid bodies with force and torque arrow visuals."""

from __future__ import annotations

import dartpy as dart
import numpy as np


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()
    world.setGravity([0, 0, -9.81])
    world.setTimeStep(0.001)

    ground = dart.dynamics.Skeleton("ground")
    joint, body = ground.createWeldJointAndBodyNodePair()
    transform = dart.math.Isometry3()
    transform.set_translation([0, 0, -0.05])
    joint.setTransformFromParentBodyNode(transform)
    body.setName("ground_body")
    node = body.createShapeNode(dart.dynamics.BoxShape([10, 10, 0.1]))
    node.createVisualAspect().setColor([0.3, 0.3, 0.3])
    node.createCollisionAspect()
    node.createDynamicsAspect()
    world.addSkeleton(ground)

    rigid_bodies = []
    for name, position, shape, color in [
        (
            "box1",
            [-1, 0, 2],
            dart.dynamics.BoxShape([0.4, 0.4, 0.4]),
            [0.8, 0.2, 0.2],
        ),
        (
            "box2",
            [0, 0, 3],
            dart.dynamics.BoxShape([0.6, 0.3, 0.3]),
            [0.2, 0.8, 0.2],
        ),
        (
            "sphere1",
            [1, 0, 2.5],
            dart.dynamics.SphereShape(0.3),
            [0.2, 0.2, 0.8],
        ),
        (
            "box3",
            [0.5, 1, 2.2],
            dart.dynamics.BoxShape([0.3, 0.5, 0.4]),
            [0.9, 0.9, 0.2],
        ),
    ]:
        skeleton = dart.dynamics.Skeleton(name)
        joint, body = skeleton.createFreeJointAndBodyNodePair()
        transform = dart.math.Isometry3()
        transform.set_translation(position)
        joint.setTransformFromParentBodyNode(transform)
        body.setName(f"{name}_body")
        node = body.createShapeNode(shape)
        node.createVisualAspect().setColor(color)
        node.createCollisionAspect()
        node.createDynamicsAspect()
        body.setInertia(dart.dynamics.Inertia(1.0, [0, 0, 0], shape.computeInertia(1.0)))
        world.addSkeleton(skeleton)
        rigid_bodies.append(body)

    selected_index = 0
    force_magnitude = 100.0
    torque_magnitude = 50.0
    show_arrows = True
    pending_forces = []
    pending_torques = []
    arrow_frames = []
    arrow_shapes = []
    frame_counter = 0

    def select_body(offset):
        nonlocal selected_index
        selected_index = (selected_index + offset) % len(rigid_bodies)

    def apply_vector(direction, torque):
        body = rigid_bodies[selected_index]
        vector = np.array(direction) * (torque_magnitude if torque else force_magnitude)
        if torque:
            body.addExtTorque(vector)
            pending_torques.append((body, vector))
        else:
            body.addExtForce(vector)
            pending_forces.append((body, vector))

    def hide_arrows(start=0):
        for frame in arrow_frames[start:]:
            frame.getVisualAspect(True).setHidden(True)

    def change_show_arrows():
        nonlocal show_arrows
        show_arrows = show.value
        if not show_arrows:
            hide_arrows()
            pending_forces.clear()
            pending_torques.clear()

    def toggle_arrows():
        show.value = not show_arrows
        change_show_arrows()

    def post_step():
        nonlocal frame_counter
        frame_counter += 1
        if frame_counter % 5 != 0:
            return
        if show_arrows:
            count = len(pending_forces) + len(pending_torques)
            while len(arrow_frames) < count:
                frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
                arrow = dart.dynamics.ArrowShape(
                    [0, 0, 0],
                    [0, 0, 0.01],
                    dart.dynamics.ArrowShapeProperties(0.006, 2.0, 0.15),
                    [0.9, 0.1, 0.1, 1.0],
                )
                frame.setShape(arrow)
                frame.getVisualAspect(True).setHidden(True)
                world.addSimpleFrame(frame)
                arrow_frames.append(frame)
                arrow_shapes.append(arrow)
            index = 0
            for body, vector in pending_forces + pending_torques:
                norm = np.linalg.norm(vector)
                if norm < 1e-6:
                    continue
                com = body.getCOM()
                arrow_frames[index].getVisualAspect(True).setHidden(False)
                arrow_shapes[index].setPositions(com, com + vector / norm * 0.5)
                index += 1
            hide_arrows(index)
        else:
            hide_arrows()
        pending_forces.clear()
        pending_torques.clear()

    panel = dart.viewer.Panel()
    panel.add_text(f"Rigid bodies: {len(rigid_bodies)}")
    selected = panel.add_text("")
    force = panel.add_slider("Force mag", 100.0, min=1.0, max=500.0, decimals=1)
    torque = panel.add_slider("Torque mag", 50.0, min=1.0, max=200.0, decimals=1)
    force_text = panel.add_text("")
    torque_text = panel.add_text("")
    show = panel.add_checkbox("Show force/torque arrows", True)
    show.on_change = change_show_arrows
    panel.add_text(
        "Tab/Backspace select a body; arrow keys and u/d push it; "
        "q/w/e/a/z/c apply torque; +/- scale the magnitude.",
        wrap=True,
    )

    def change_force():
        nonlocal force_magnitude
        force_magnitude = float(np.float32(force.value))

    def change_torque():
        nonlocal torque_magnitude
        torque_magnitude = float(np.float32(torque.value))

    force.on_change = change_force
    torque.on_change = change_torque

    def scale_magnitudes(increase):
        nonlocal force_magnitude, torque_magnitude
        if increase:
            force_magnitude = min(force_magnitude * 1.5, 10000.0)
            torque_magnitude = min(torque_magnitude * 1.5, 5000.0)
        else:
            force_magnitude = max(force_magnitude * 0.67, 0.1)
            torque_magnitude = max(torque_magnitude * 0.67, 0.1)
        # Sliders clamp their display; the scene retains the wider key ranges.
        force.value = force_magnitude
        torque.value = torque_magnitude

    def pre_refresh():
        selected.label = (
            f"Selected: {rigid_bodies[selected_index].getName()} "
            f"({selected_index + 1}/{len(rigid_bodies)})"
        )
        force_text.label = f"Force magnitude: {force_magnitude:.1f} N"
        torque_text.label = f"Torque magnitude: {torque_magnitude:.1f} N m"

    pre_refresh()
    event = dart.gui.GUIEventAdapter
    key_actions = [
        dart.viewer.KeyAction(event.KEY_Tab, "Select next body", lambda: select_body(1)),
        dart.viewer.KeyAction(
            event.KEY_BackSpace, "Select previous body", lambda: select_body(-1)
        ),
    ]
    key_actions.extend(
        dart.viewer.KeyAction(
            key,
            label,
            lambda direction=direction, torque=torque: apply_vector(direction, torque),
        )
        for key, label, direction, torque in [
            (event.KEY_Up, "Force +Y", [0, 1, 0], False),
            (event.KEY_Down, "Force -Y", [0, -1, 0], False),
            (event.KEY_Left, "Force -X", [-1, 0, 0], False),
            (event.KEY_Right, "Force +X", [1, 0, 0], False),
            ("u", "Force +Z (up)", [0, 0, 1], False),
            ("d", "Force -Z (down)", [0, 0, -1], False),
            ("q", "Torque +X", [1, 0, 0], True),
            ("w", "Torque +Y", [0, 1, 0], True),
            ("e", "Torque +Z", [0, 0, 1], True),
            ("a", "Torque -X", [-1, 0, 0], True),
            ("z", "Torque -Y", [0, -1, 0], True),
            ("c", "Torque -Z", [0, 0, -1], True),
        ]
    )
    key_actions.extend(
        [
            dart.viewer.KeyAction("=", "Increase magnitude", lambda: scale_magnitudes(True)),
            dart.viewer.KeyAction("-", "Decrease magnitude", lambda: scale_magnitudes(False)),
            dart.viewer.KeyAction("v", "Toggle force arrows", toggle_arrows),
        ]
    )
    return dart.viewer.SceneSetup(
        world,
        post_step=post_step,
        pre_refresh=pre_refresh,
        key_actions=key_actions,
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[5, 5, 3], center=[0, 0, 1], up=[0, 0, 1]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "simulation_event_handler",
            title="Simulation Events",
            category="Rigid Body",
            summary="Force and torque arrows applied to a keyboard-selected rigid body.",
            factory=build,
        )
    ]
