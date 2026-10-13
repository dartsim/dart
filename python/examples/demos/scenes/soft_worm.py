"""A native five-link soft worm controlled by a traveling-wave gait."""

import dartpy as dart


def build() -> dart.viewer.SceneSetup:
    native = dart._demos.soft_worm
    world, worm = native.create_world()
    initial_root_x = native.root_x(worm)
    finite = True

    panel = dart.viewer.Panel()
    gait = panel.add_checkbox("Traveling-wave gait", True)
    panel.add_text(f"Links: {native.num_links}")
    panel.add_text(f"Driven joints: {native.num_links - 1}")
    root_text = panel.add_text("")
    displacement_text = panel.add_text("")
    frame_text = panel.add_text("")
    panel.add_text(
        f"Gait: {native.gait_frequency_hz:.2f} Hz   "
        f"amplitude: {native.gait_amplitude:.2f} rad"
    )
    checksum_text = panel.add_text("")
    nonfinite_text = panel.add_text("State is non-finite")
    nonfinite_text.color = [1.0, 0.25, 0.2, 1.0]
    nonfinite_text.visible = False
    panel.add_text(
        "Each articulated link carries 3x3x3 box-shaped soft flesh. The "
        "phase-offset sinusoidal gait can be paused with g.",
        wrap=True,
    )

    def pre_step():
        native.apply_gait(worm, world.getTime(), gait.value and finite)

    def post_step():
        nonlocal finite
        finite = native.is_finite(worm)
        if not finite:
            gait.value = False

    def pre_refresh():
        root_x = native.root_x(worm)
        root_text.label = f"Root x: {root_x:.3f} m"
        displacement_text.label = f"Displacement: {root_x - initial_root_x:.3f} m"
        frame_text.label = (
            f"Frame: {world.getSimFrames()}   "
            f"contacts: {world.getLastCollisionResult().getNumContacts()}"
        )
        checksum_text.label = (
            f"Position checksum: {native.position_checksum(worm):.6e}"
        )
        nonfinite_text.visible = not finite

    def toggle_gait():
        gait.value = not gait.value

    pre_refresh()
    return dart.viewer.SceneSetup(
        world,
        pre_step=pre_step,
        post_step=post_step,
        pre_refresh=pre_refresh,
        key_actions=[
            dart.viewer.KeyAction("g", "Toggle traveling-wave gait", toggle_gait)
        ],
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[6.6, -6.8, 3.6], center=[2.4, 0.0, 0.25], up=[0.0, 0.0, 1.0]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "soft_worm",
            title="Soft Worm",
            category="Soft Bodies",
            summary="A five-link soft-bodied worm driven by a traveling-wave gait.",
            factory=build,
        )
    ]
