"""Kinematic squat replay solved for contact-consistent joint torques."""

import dartpy as dart

from .common import reorient_z_up


class _ContactInverseDynamics:
    def __init__(self):
        self.world = dart.utils.SkelParser.readWorld(
            "dart://sample/skel/fullbody1.skel"
        )
        if self.world is None:
            raise RuntimeError("failed to load dart://sample/skel/fullbody1.skel")
        reorient_z_up(self.world)
        self.model = dart._demos.ContactInverseDynamicsModel(self.world)

        self.panel = dart.viewer.Panel()
        controls = self.panel.add_group("Controls")
        play = controls.add_checkbox("Play", self.model.playing)
        play.on_change = lambda: setattr(self.model, "playing", play.value)
        speed = controls.add_slider(
            "Playback Speed", self.model.playback_speed, min=0.1, max=2.0, decimals=2
        )
        speed.on_change = lambda: setattr(self.model, "playback_speed", speed.value)
        depth = controls.add_slider(
            "Squat Depth (m)",
            self.model.squat_depth,
            min=0.0,
            max=0.25,
            decimals=3,
            notify_on_release=True,
        )

        def change_depth():
            self.model.squat_depth = depth.value
            self.model.request_regeneration()

        depth.on_change = change_depth
        controls.add_button("Regenerate Keyframes").on_change = (
            self.model.request_regeneration
        )
        friction = controls.add_slider(
            "Friction Coefficient",
            self.model.friction_coeff,
            min=0.0,
            max=1.5,
            decimals=2,
        )
        friction.on_change = lambda: setattr(
            self.model, "friction_coeff", friction.value
        )
        scale = controls.add_slider(
            "Force Arrow Scale",
            self.model.force_scale,
            min=0.0001,
            max=0.01,
            decimals=4,
        )
        scale.on_change = lambda: setattr(self.model, "force_scale", scale.value)
        self.gravity_text = controls.add_text("")

        status = self.panel.add_group("Status")
        self.feasibility_text = status.add_text("")
        self.residual_text = status.add_text("")
        self.force_text = status.add_text("")
        self.weight_text = status.add_text("")
        self.ratio_text = status.add_text("")
        status.add_separator()
        status.add_text("Contact Forces:")
        self.contact_texts = [status.add_text("") for _ in range(4)]

        plots = self.panel.add_group("Plots")
        self.torque_plots = [
            plots.add_plot(name, capacity=240, min=-200.0, max=200.0)
            for name in self.model.torque_names
        ]
        self.force_plot = plots.add_plot(
            "Total Vertical GRF (N)", capacity=240, min=0.0, max=1000.0
        )
        self._refresh_panel()

    def _refresh_panel(self):
        gravity = self.world.getGravity()
        self.gravity_text.label = (
            f"Gravity: ({gravity[0]:.2f}, {gravity[1]:.2f}, {gravity[2]:.2f})"
        )
        feasible = self.model.feasible
        self.feasibility_text.label = "FEASIBLE" if feasible else "INFEASIBLE"
        self.feasibility_text.color = [0, 1, 0, 1] if feasible else [1, 0, 0, 1]
        self.residual_text.label = f"Residual Norm: {self.model.residual_norm:.4f}"
        vertical_force = self.model.total_vertical_force
        self.force_text.label = f"Total Vertical Force: {vertical_force:.2f} N"
        weight = self.model.biped.getMass() * gravity.dot(gravity) ** 0.5
        self.weight_text.label = f"Body Weight: {weight:.2f} N"
        ratio = vertical_force / weight if weight > 1e-6 else 0.0
        self.ratio_text.label = f"Ratio: {ratio:.2f}"
        forces = self.model.contact_forces
        for index, text in enumerate(self.contact_texts):
            text.visible = index < len(forces)
            if text.visible:
                magnitude = forces[index].dot(forces[index]) ** 0.5
                text.label = f"  Contact {index}: {magnitude:.2f} N"
        # Native histories retain the C++ ring's raw order (ImGui offset zero).
        for plot, samples in zip(
            [*self.torque_plots, self.force_plot],
            [*self.model.torque_plots, self.model.plot_samples],
        ):
            plot.clear()
            for sample in samples:
                plot.push(sample)

    def pre_refresh(self):
        self.model.update_frame()
        self._refresh_panel()


def build() -> dart.viewer.SceneSetup:
    state = _ContactInverseDynamics()
    return dart.viewer.SceneSetup(
        state.world,
        pre_refresh=state.pre_refresh,
        panel=state.panel,
        camera_home=dart.viewer.CameraHome(
            eye=[3.0, -3.0, 1.5], center=[0.0, 0.0, 0.0], up=[0.0, 0.0, 1.0]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "contact_inverse_dynamics",
            title="Contact Inverse Dynamics",
            category="Control & IK",
            summary="Kinematic squat replay solved for contact-consistent joint torques.",
            factory=build,
        )
    ]
