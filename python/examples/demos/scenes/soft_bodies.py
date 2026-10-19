"""Soft-body simulation with recorded-state playback."""

import dartpy as dart
import numpy as np

from ..models.world_history import WorldHistory
from .common import reorient_z_up


def _build(uri, eye, center):
    world = dart.utils.SkelParser.readWorld(uri)
    if world is None:
        raise RuntimeError(f"failed to load {uri}")
    reorient_z_up(world)

    panel = dart.viewer.Panel()
    panel.add_text("Soft mesh alpha")
    alpha = panel.add_slider("", 0.5, min=0.0, max=1.0, decimals=2)
    embedded = panel.add_checkbox("Embedded visuals", True)
    panel.add_separator()
    playback_text = panel.add_text("Playback frame: 0 / 0")
    recorded_text = panel.add_text("Frames recorded (lifetime): 1")
    panel.add_text(
        "'[' / ']' step one recorded frame back/forward, '{' / '}' step "
        "10, 'r' restarts, '\\' jumps to the latest frame. Scrubbing "
        "pauses the simulation; history is capped at 4000 frames.",
        wrap=True,
    )

    colors = (
        (0.35, 0.48, 1.00),
        (0.95, 0.45, 0.30),
        (0.30, 0.72, 0.48),
        (0.95, 0.76, 0.28),
        (0.70, 0.45, 0.90),
    )

    def style_visuals():
        soft_body_index = 0
        for skeleton_index in range(world.getNumSkeletons()):
            skeleton = world.getSkeleton(skeleton_index)
            for body in skeleton.getBodyNodes():
                if not isinstance(body, dart.dynamics.SoftBodyNode):
                    continue
                rgba = [
                    *colors[soft_body_index % len(colors)],
                    float(np.float32(alpha.value)),
                ]
                soft_body_index += 1
                for shape_node in body.getShapeNodes():
                    visual = shape_node.getVisualAspect(False)
                    if visual is None:
                        continue
                    shape = shape_node.getShape()
                    if (
                        shape is not None
                        and shape.getType() == dart.dynamics.SoftMeshShape.getStaticType()
                    ):
                        visual.setRGBA(rgba)
                        visual.show()
                    elif embedded.value:
                        visual.show()
                    else:
                        visual.hide()

    style_visuals()
    history = WorldHistory(world)
    viewer = None
    world_node = None

    def on_activate(ctx):
        nonlocal viewer, world_node
        viewer = ctx.viewer
        world_node = ctx.world_node

        def teardown():
            nonlocal viewer, world_node
            viewer = None
            world_node = None

        ctx.add_teardown(teardown)

    def display_changed():
        style_visuals()
        if world_node is not None:
            was_simulating = world_node.isSimulating()
            world_node.simulate(False)
            world_node.refresh()
            world_node.simulate(was_simulating)

    alpha.on_change = display_changed
    embedded.on_change = display_changed

    def scrub(move):
        if viewer is not None:
            viewer.simulate(False)
        move()

    def pre_refresh():
        playback_text.label = (
            f"Playback frame: {history.current_index} / {history.size - 1}"
        )
        recorded_text.label = f"Frames recorded (lifetime): {history.frames_recorded}"

    return dart.viewer.SceneSetup(
        world,
        post_step=history.record,
        pre_refresh=pre_refresh,
        on_activate=on_activate,
        key_actions=[
            dart.viewer.KeyAction(
                "[", "Step back 1 frame", lambda: scrub(history.move_backward)
            ),
            dart.viewer.KeyAction(
                "]", "Step forward 1 frame", lambda: scrub(history.move_forward)
            ),
            dart.viewer.KeyAction(
                "{", "Step back 10 frames", lambda: scrub(lambda: history.move_backward(10))
            ),
            dart.viewer.KeyAction(
                "}",
                "Step forward 10 frames",
                lambda: scrub(lambda: history.move_forward(10)),
            ),
            dart.viewer.KeyAction(
                "r", "Restart (frame 0)", lambda: scrub(lambda: history.move_to(0))
            ),
            dart.viewer.KeyAction(
                "\\",
                "Jump to latest frame",
                lambda: scrub(lambda: history.move_to(history.size - 1)),
            ),
        ],
        panel=panel,
        camera_home=dart.viewer.CameraHome(eye=eye, center=center, up=[0.0, 0.0, 1.0]),
    )


def build() -> dart.viewer.SceneSetup:
    return _build(
        "dart://sample/skel/softBodies.skel", [3.0, -3.0, 2.2], [0.0, 0.0, 0.4]
    )


def scenes():
    return [
        dart.viewer.Scene(
            "soft_bodies",
            title="Soft Bodies",
            category="Soft Bodies",
            summary="Soft-body simulation with recorded-state playback.",
            factory=build,
        )
    ]
