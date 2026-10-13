"""dartpy.viewer runs that render, so they need a graphics device."""

import dartpy as dart
import pytest

viewer = getattr(dart, "viewer", None)
pytestmark = pytest.mark.skipif(
    viewer is None,
    reason="a vanilla system ImGui compiles out the demo app host",
)


def box_world():
    world = dart.simulation.World()
    skeleton = dart.dynamics.Skeleton("box")
    body = skeleton.createFreeJointAndBodyNodePair()[1]
    shape = body.createShapeNode(dart.dynamics.BoxShape([0.5, 0.5, 0.5]))
    shape.createVisualAspect()
    world.addSkeleton(skeleton)
    return world


def test_rendered_cycle_switches_scenes():
    scenes = [
        viewer.Scene(
            scene_id,
            title=scene_id.title(),
            category="Tests",
            factory=lambda: viewer.SceneSetup(box_world()),
        )
        for scene_id in ("first", "second")
    ]
    args = ["test", "--cycle-scenes", "--headless", "--frames", "2"]
    assert viewer.run(scenes, args + ["--width", "64", "--height", "48"]) == 0


def test_capture_runs_hooks_and_key_actions(tmp_path):
    calls = []

    def on_activate(ctx):
        # A capture installs the host's key handler before the scene.
        event = dart.gui.GUIEventAdapter(dart.gui.GUIEventAdapter.KEYDOWN, ord("k"))
        assert ctx.viewer.handleEvent(event)

    def build():
        return viewer.SceneSetup(
            box_world(),
            pre_step=lambda: calls.append("step"),
            pre_refresh=lambda: calls.append("refresh"),
            on_activate=on_activate,
            key_actions=[viewer.KeyAction("k", "Key", lambda: calls.append("key"))],
        )

    shot = tmp_path / "capture.png"
    scenes = [viewer.Scene("box", title="Box", category="Tests", factory=build)]
    args = ["test", "--headless", "--steps", "3", "--shot", str(shot)]
    assert viewer.run(scenes, args + ["--width", "64", "--height", "48"]) == 0
    assert shot.stat().st_size > 0
    # The key action and each settle step run once; capture frames refresh.
    assert calls[0] == "key"
    assert calls.count("step") == 3
    assert calls.count("refresh") >= 3


@pytest.mark.parametrize("error", [KeyboardInterrupt, SystemExit])
def test_base_exceptions_from_viewer_overrides_stop_a_rendered_cycle(error):
    refreshes = []

    class Raising(dart.gui.ViewerAttachment):
        def refresh(self):
            refreshes.append(1)
            raise error()

    scene = viewer.Scene(
        "raising",
        title="Raising",
        category="Tests",
        # ctx.add_attachment takes native visuals only; the interim viewer
        # API still installs Python overrides.
        factory=lambda: viewer.SceneSetup(
            box_world(), on_activate=lambda ctx: ctx.viewer.addAttachment(Raising())
        ),
    )
    args = ["test", "--cycle-scenes", "--headless", "--frames", "100"]
    with pytest.raises(error):
        viewer.run([scene], args + ["--width", "64", "--height", "48"])
    assert len(refreshes) == 1


def test_capture_stops_between_warm_up_frames(tmp_path):
    refreshes = []

    class Counting(dart.gui.ViewerAttachment):
        def refresh(self):
            refreshes.append(1)

    def pre_refresh():
        raise KeyboardInterrupt()

    scene = viewer.Scene(
        "stopping",
        title="Stopping",
        category="Tests",
        factory=lambda: viewer.SceneSetup(
            box_world(),
            pre_refresh=pre_refresh,
            on_activate=lambda ctx: ctx.viewer.addAttachment(Counting()),
        ),
    )
    shot = tmp_path / "never.png"
    args = ["test", "--headless", "--steps", "0", "--shot", str(shot)]
    with pytest.raises(KeyboardInterrupt):
        viewer.run([scene], args + ["--width", "64", "--height", "48"])
    # The interrupt in the first warm-up frame stops the capture before the
    # next frame draws.
    assert len(refreshes) == 1
    assert not shot.exists()
