"""Contact-local activation markers around the soft-contact model."""

import dartpy as dart
import numpy as np

from ..models import adaptive_soft_contact as adaptive_model


def build() -> dart.viewer.SceneSetup:
    model = adaptive_model.create_model()
    world = model.world
    markers = []
    for i in range(model.soft_body.getNumPointMasses()):
        marker = dart.dynamics.SimpleFrame(
            dart.dynamics.Frame.World(), f"activation_marker_{i}"
        )
        marker.setShape(dart.dynamics.SphereShape(0.018))
        marker.createVisualAspect()
        world.addSimpleFrame(marker)
        markers.append(marker)

    panel = dart.viewer.Panel()
    adaptive = panel.add_checkbox("Adaptive activation", True)
    rings = panel.add_slider(
        "Contact rings", adaptive_model.default_ring_count, min=0, max=4, integer=True
    )
    linger = panel.add_slider(
        "Linger steps",
        adaptive_model.default_linger_steps,
        min=0,
        max=120,
        integer=True,
    )
    panel.add_separator()
    active_text = panel.add_text("")
    inactive_text = panel.add_text("")
    contacts_text = panel.add_text("")
    step_text = panel.add_text("")
    nonfinite_text = panel.add_text("State is non-finite")
    nonfinite_text.color = [1.0, 0.25, 0.2, 1.0]
    nonfinite_text.visible = False
    panel.add_separator()
    panel.add_text(
        "Gold markers are a contact-nearest region estimate sized to the "
        "exact public active count; blue markers are the remainder. The "
        "orange pusher repeatedly makes and breaks side contact.",
        wrap=True,
    )
    finite = True

    def apply_settings():
        adaptive_model.configure(
            model,
            adaptive_enabled=adaptive.value,
            ring_count=rings.value,
            linger_steps=linger.value,
        )

    def update_markers():
        positions = adaptive_model.point_positions(model)
        flags = adaptive_model.active_flags(model)
        for marker, position, active in zip(markers, positions, flags):
            if np.isfinite(position).all():
                marker.setRelativeTranslation(position)
            marker.getVisualAspect().setColor(
                [1.0, 0.78, 0.08, 1.0] if active else [0.08, 0.18, 0.48, 0.45]
            )

    def pre_step():
        apply_settings()
        adaptive_model.prepare_step(model)

    def post_step():
        nonlocal finite
        finite = adaptive_model.is_finite(model)
        update_markers()

    def pre_refresh():
        apply_settings()
        update_markers()
        active = adaptive_model.compute_checksum(model).active
        total = model.soft_body.getNumPointMasses()
        active_text.label = f"Active point masses:   {active}"
        inactive_text.label = f"Inactive point masses: {total - active}"
        contacts_text.label = (
            f"Contacts:              {world.getLastCollisionResult().getNumContacts()}"
        )
        step_text.label = f"Simulation step:       {world.getSimFrames()}"
        nonfinite_text.visible = not finite

    def toggle_adaptive():
        adaptive.value = not adaptive.value

    pre_refresh()
    return dart.viewer.SceneSetup(
        world,
        pre_step=pre_step,
        post_step=post_step,
        pre_refresh=pre_refresh,
        key_actions=[
            dart.viewer.KeyAction("a", "Toggle adaptive activation", toggle_adaptive)
        ],
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[3.1, 2.4, 2.0], center=[0.0, 0.0, 0.42], up=[0.0, 0.0, 1.0]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "adaptive_soft_contact",
            title="Adaptive Soft Contact",
            category="Soft Bodies",
            summary="Visualize contact-local point-mass activation around a soft body.",
            factory=build,
        )
    ]
