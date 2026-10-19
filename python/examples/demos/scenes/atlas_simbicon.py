"""Y-up Atlas balance and push perturbations with a Python SIMBICON controller."""

import math

import dartpy as dart
import numpy as np

from ..models.simbicon import Controller


class _AtlasSimbicon:
    def __init__(self):
        self.world = dart.simulation.World()
        ground = dart.utils.DartLoader().parseSkeleton(
            "dart://sample/sdf/atlas/ground.urdf"
        )
        if ground is None:
            raise RuntimeError("failed to load dart://sample/sdf/atlas/ground.urdf")
        atlas = dart.utils.SdfParser.readSkeleton(
            "dart://sample/sdf/atlas/atlas_v3_no_head.sdf"
        )
        if atlas is None:
            raise RuntimeError(
                "failed to load dart://sample/sdf/atlas/atlas_v3_no_head.sdf"
            )
        self.world.addSkeleton(ground)
        self.world.addSkeleton(atlas)
        atlas.setPosition(0, -0.5 * math.pi)
        self.world.setGravity([0.0, -9.81, 0.0])
        self.pelvis = atlas.getBodyNode("pelvis")
        if self.pelvis is None:
            raise RuntimeError("atlas_v3_no_head.sdf: missing body node 'pelvis'")
        self.controller = Controller(atlas, self.world)
        self.external_force = np.zeros(3)
        self.force_duration = 0
        self.viewer = None
        self.world_node = None
        self.control_mode = 2

        self.panel = dart.viewer.Panel()
        self.gravity = self.panel.add_slider(
            "Gravity Acc.", float(np.float32(9.81)), min=5.0, max=20.0, decimals=2
        )
        self.gravity.on_change = self.set_gravity
        self.gravity_text = self.panel.add_text("")
        self.headlights = self.panel.add_checkbox("Headlights On/Off")
        self.headlights.on_change = self.set_headlights
        self.shadow = self.panel.add_checkbox("Shadow On/Off")
        self.shadow.on_change = self.set_shadow
        self.depth = self.panel.add_checkbox("Depth mode")
        self.depth.on_change = self.set_depth
        self.panel.add_separator()
        self.panel.add_button("Reset Atlas").on_change = self.reset
        self.mode = self.panel.add_choice(
            "", ["No Control", "Short-Stride Walking", "Normal-Stride Walking"], index=2
        )
        self.panel.add_separator()
        self.panel.add_text(
            "a/s push the pelvis forward/backward, d/f push left/right (500 N "
            "for 100 steps); 'r' resets Atlas to its initial configuration.",
            wrap=True,
        )

    def push(self, axis, sign):
        self.external_force[axis] = sign * 500.0
        self.force_duration = 100

    def reset(self):
        self.controller.reset_robot()
        self.external_force.fill(0.0)

    def pre_step(self):
        self.pelvis.addExtForce(self.external_force)
        self.controller.update()
        if self.force_duration > 0:
            self.force_duration -= 1
        else:
            self.external_force.fill(0.0)

    def set_gravity(self):
        self.world.setGravity([0.0, -float(np.float32(self.gravity.value)), 0.0])

    def set_headlights(self):
        if self.viewer is not None:
            self.viewer.switchHeadlights(self.headlights.value)

    def set_shadow(self):
        if self.world_node is not None and self.viewer is not None:
            self.world_node.setShadowsEnabled(self.shadow.value)

    def set_depth(self):
        if self.viewer is not None:
            self.viewer.setCameraMode(
                dart.gui.CameraMode.DEPTH
                if self.depth.value
                else dart.gui.CameraMode.RGBA
            )

    def pre_refresh(self):
        self.gravity_text.label = f"Gravity Acc.: {-self.gravity.value:.2f} m/s^2"
        self.headlights.value = (
            self.viewer is not None and self.viewer.checkHeadlights()
        )
        self.shadow.value = self.world_node is not None and self.world_node.isShadowed()
        if self.mode.value != self.control_mode:
            # Preserve the C++ panel's labels, gait mapping, and initial mismatch.
            self.controller.change_state_machine(
                ("standing", "running", "walking")[self.mode.value],
                self.world.getTime(),
            )
            self.control_mode = self.mode.value

    def on_activate(self, ctx):
        self.world_node = ctx.world_node
        self.viewer = ctx.viewer
        self.world_node.setShadowsEnabled(True)
        viewer = self.viewer
        prior_camera_mode = viewer.getCameraMode()
        prior_headlights = viewer.checkHeadlights()
        self.depth.value = prior_camera_mode == dart.gui.CameraMode.DEPTH

        def teardown():
            viewer.setCameraMode(prior_camera_mode)
            viewer.switchHeadlights(prior_headlights)

        ctx.add_teardown(teardown)


def build() -> dart.viewer.SceneSetup:
    state = _AtlasSimbicon()
    return dart.viewer.SceneSetup(
        state.world,
        pre_step=state.pre_step,
        pre_refresh=state.pre_refresh,
        on_activate=state.on_activate,
        key_actions=[
            dart.viewer.KeyAction("a", "Push forward", lambda: state.push(0, 1)),
            dart.viewer.KeyAction("s", "Push backward", lambda: state.push(0, -1)),
            dart.viewer.KeyAction("d", "Push left", lambda: state.push(2, 1)),
            dart.viewer.KeyAction("f", "Push right", lambda: state.push(2, -1)),
            dart.viewer.KeyAction("r", "Reset Atlas", state.reset),
        ],
        panel=state.panel,
        camera_home=dart.viewer.CameraHome(
            eye=np.array([5.14, 3.28, 6.28]) * 2.0,
            center=[1.0, 0.0, 0.0],
            up=[0.0, 0.1, 0.0],
        ),
        shadows=False,
    )


def scenes():
    return [
        dart.viewer.Scene(
            "atlas_simbicon",
            title="Atlas SIMBICON",
            category="Control & IK",
            summary="Atlas humanoid balanced by a SIMBICON state-machine controller.",
            factory=build,
        )
    ]
