"""Device-free tests of dartpy.viewer, the Python face of the demo app host."""

import gc
import threading
import weakref

import dartpy as dart
import numpy as np
import pytest

from ..bindings._support import run_isolated

viewer = getattr(dart, "viewer", None)
pytestmark = pytest.mark.skipif(
    viewer is None,
    reason="a vanilla system ImGui compiles out the demo app host",
)


def make_scene(scene_id="demo", factory=None, **setup):
    def build():
        return viewer.SceneSetup(dart.simulation.World(), **setup)

    return viewer.Scene(
        scene_id, title=scene_id.title(), category="Tests", factory=factory or build
    )


def cycle(scenes, frames=2):
    return viewer.run(scenes, ["test", "--cycle-scenes", "--frames", str(frames)])


@pytest.mark.parametrize(
    "make, error",
    [
        (
            lambda: viewer.Scene("Bad Id", title="t", category="c", factory=print),
            ValueError,
        ),
        (lambda: viewer.Scene("ok", title="t", category="c", factory=None), TypeError),
        (lambda: viewer.SceneSetup(None), TypeError),
        (lambda: viewer.SceneSetup(dart.simulation.World(), pre_step=3), TypeError),
        (lambda: viewer.SceneSetup(dart.simulation.World(), key_actions=[1]), TypeError),
        (
            lambda: viewer.SceneSetup(
                dart.simulation.World(),
                key_actions=[
                    viewer.KeyAction("a", "first", print),
                    viewer.KeyAction(ord("a"), "second", print),
                ],
            ),
            ValueError,
        ),
        (
            lambda: viewer.SceneSetup(
                dart.simulation.World(), panel=viewer.Panel().add_button("b")
            ),
            TypeError,
        ),
        (lambda: viewer.KeyAction("ab", "x", print), ValueError),
        (lambda: viewer.KeyAction(-1, "x", print), ValueError),
        (lambda: viewer.KeyAction("a", "x", None), TypeError),
        (lambda: viewer.CameraHome(eye=[0, 0, 0], center=[0, 0, 0]), ValueError),
        (lambda: viewer.CameraHome(eye=[0, 0, 1], center=[0, 0, 0]), ValueError),
        (lambda: viewer.CameraHome(eye=[np.nan, 0, 0], center=[0, 0, 0]), ValueError),
    ],
)
def test_types_validate_their_arguments(make, error):
    with pytest.raises(error):
        make()


def test_scene_setup_exposes_its_parts():
    world = dart.simulation.World()
    action = viewer.KeyAction("q", "Quit", print)
    panel = viewer.Panel()
    home = viewer.CameraHome(eye=[3, 0, 1], center=[0, 0, 0])
    setup = viewer.SceneSetup(
        world,
        pre_step=print,
        key_actions=[action],
        panel=panel,
        camera_home=home,
        shadows=False,
        allow_simulation=False,
    )
    assert setup.world is world
    assert setup.pre_step is print and setup.post_step is None
    assert setup.key_actions == (action,)
    assert action.key == ord("q") and action.label == "Quit"
    assert setup.panel is panel
    np.testing.assert_allclose(setup.camera_home.eye, [3, 0, 1])
    np.testing.assert_allclose(setup.camera_home.up, [0, 0, 1])
    assert not setup.shadows and not setup.allow_simulation


def test_panel_controls_validate_values():
    panel = viewer.Panel()
    assert panel.kind == viewer.Control.Kind.GROUP
    slider = panel.add_slider("gain", 5.0, min=0.0, max=2.0)
    assert slider.value == 2.0
    slider.value = -1
    assert slider.value == 0.0
    with pytest.raises(ValueError):
        slider.value = float("nan")
    with pytest.raises(ValueError):
        slider.set_range(1.0, 0.0)
    count = panel.add_input("count", 2.6, integer=True, min=0, max=10)
    assert count.value == 3 and isinstance(count.value, int)
    assert panel.add_input("fast", 1.0, step=0.1, step_fast=1.0).value == 1.0
    with pytest.raises(ValueError):
        panel.add_input("bad", 0.0, step_fast=float("nan"))
    assert panel.add_checkbox("on", True).value is True
    choice = panel.add_choice("mode", ["a", "b"], index=1)
    assert choice.value == 1 and choice.options == ["a", "b"]
    with pytest.raises(ValueError):
        choice.value = 2
    vector = panel.add_vec3("target", [5, -5, 0.5], min=-1, max=1)
    np.testing.assert_allclose(vector.value, [1, -1, 0.5])
    color = panel.add_color("tint", [2, -1, 0.5, 0.5], alpha=False)
    np.testing.assert_allclose(color.value, [1, 0, 0.5, 1])
    plot = panel.add_plot("error", capacity=2, min=-1, max=1)
    for sample in (0.25, 0.5, 0.75):
        plot.push(sample)
    assert plot.samples == [0.5, 0.75]
    with pytest.raises(ValueError):
        plot.push(float("inf"))
    text = panel.add_text("hello")
    with pytest.raises(TypeError):
        text.value
    with pytest.raises(TypeError):
        text.on_change = print
    with pytest.raises(RuntimeError):
        text.add_button("child")
    assert panel.add_separator() is None


def test_run_lists_and_cycles_scenes(capfd):
    calls = []

    def build():
        def on_activate(ctx):
            assert isinstance(ctx.viewer, dart.gui.Viewer)
            assert isinstance(ctx.world_node, dart.gui.RealTimeWorldNode)
            calls.append("activate")
            ctx.add_teardown(lambda: calls.append("teardown"))

        return viewer.SceneSetup(
            dart.simulation.World(),
            pre_step=lambda: calls.append("pre_step"),
            pre_refresh=lambda: calls.append("pre_refresh"),
            on_activate=on_activate,
        )

    scenes = [
        viewer.Scene(
            "first", title="First", category="Tests", summary="One", factory=build
        )
    ]
    assert viewer.run(scenes, ["demo", "--list-scenes"]) == 0
    assert "first  -  First: One" in capfd.readouterr().out
    assert cycle(scenes) == 0
    once = ["activate", "pre_refresh", "pre_step", "pre_refresh", "pre_step"]
    assert calls == (once + ["teardown"]) * 2


def test_failing_hook_is_disabled_and_fails_the_cycle(capfd):
    calls = []

    def pre_step():
        calls.append("pre_step")
        raise ZeroDivisionError("boom")

    scene = make_scene(pre_step=pre_step, post_step=lambda: calls.append("post"))
    assert cycle([scene], frames=3) == 1
    errors = capfd.readouterr().err
    assert "Traceback" in errors and "ZeroDivisionError: boom" in errors
    # Each activation's pre_step fails once; the other hook keeps running.
    assert calls.count("pre_step") == 2 and calls.count("post") == 6


def test_factory_failures(capfd):
    def missing():
        raise viewer.AssetsNotPrefetchedError("fetch the robot models first")

    def broken():
        raise ValueError("broken scene")

    assert cycle([make_scene("missing", factory=missing)]) == 0
    assert "fetch the robot models first" in capfd.readouterr().out
    assert cycle([make_scene("broken", factory=broken)]) == 1
    assert "broken scene" in capfd.readouterr().err
    assert cycle([make_scene("wrong", factory=lambda: 42)]) == 1
    assert "returned int, not a dartpy.viewer.SceneSetup" in capfd.readouterr().err


@pytest.mark.parametrize("error", [KeyboardInterrupt, SystemExit])
def test_base_exceptions_stop_the_run_after_teardown(error):
    calls = []
    worlds = []

    def pre_step():
        calls.append("pre_step")
        raise error()

    def on_activate(ctx):
        ctx.add_teardown(lambda: calls.append("teardown"))

    def build():
        worlds.append(dart.simulation.World())
        return viewer.SceneSetup(worlds[-1], pre_step=pre_step, on_activate=on_activate)

    scene = viewer.Scene("demo", title="Demo", category="Tests", factory=build)
    with pytest.raises(error):
        cycle([scene], frames=100)
    assert calls == ["pre_step", "teardown"]
    # The interrupted step never ran.
    assert worlds[0].getSimFrames() == 0


def test_signal_interrupts_the_native_loop():
    run_isolated(
        """
        import _thread
        import threading
        import time
        v = dart.viewer
        scene = v.Scene(
            "busy",
            title="Busy",
            category="Tests",
            factory=lambda: v.SceneSetup(dart.simulation.World()),
        )
        fired = []
        def interrupt():
            fired.append(time.monotonic())
            _thread.interrupt_main()
        # The scene runs no Python until its second pass, a million frames in,
        # so only the host's own signal polling stops it promptly.
        threading.Timer(0.2, interrupt).start()
        try:
            v.run([scene], ["test", "--cycle-scenes", "--frames", "1000000"])
        except KeyboardInterrupt:
            delay = time.monotonic() - fired[0]
        else:
            raise AssertionError("the run ignored the interrupt")
        assert delay < 0.5, f"the interrupt surfaced {delay:.2f} s late"
        """
    )


def test_abandoned_panel_callback_cycles_are_collected():
    panel = viewer.Panel()
    button = panel.add_button("go")
    button.on_change = lambda: panel
    ref = weakref.ref(panel)
    del panel, button
    gc.collect()
    assert ref() is None


def test_scene_closure_cycles_are_collected_after_a_run():
    refs = []

    def build():
        panel = viewer.Panel()
        setup = viewer.SceneSetup(
            dart.simulation.World(), panel=panel, pre_step=lambda: setup
        )
        panel.add_button("b").on_change = lambda: setup
        refs.append(weakref.ref(setup))
        return setup

    scene = viewer.Scene("cyclic", title="Cyclic", category="Tests", factory=build)
    scene_ref = weakref.ref(scene)
    assert cycle([scene]) == 0
    del scene, build
    gc.collect()
    assert scene_ref() is None
    assert len(refs) == 2 and all(ref() is None for ref in refs)


def test_camera_home_does_not_pin_its_setup():
    refs = []

    def build():
        held = []
        setup = viewer.SceneSetup(
            dart.simulation.World(),
            camera_home=viewer.CameraHome(eye=[3, 0, 1], center=[0, 0, 0]),
            pre_step=lambda: held,
        )
        held.append(setup.camera_home)
        refs.append(weakref.ref(setup))
        return setup

    assert cycle([make_scene(factory=build)]) == 0
    gc.collect()
    assert len(refs) == 2 and all(ref() is None for ref in refs)


def test_run_requires_the_main_thread():
    errors = []

    def target():
        try:
            viewer.run([], ["test", "--list-scenes"])
        except RuntimeError as error:
            errors.append(str(error))

    thread = threading.Thread(target=target)
    thread.start()
    thread.join()
    assert errors and "main thread" in errors[0]


def test_run_rejects_a_nested_run():
    errors = []

    def on_activate(ctx):
        try:
            viewer.run([], ["test", "--list-scenes"])
        except RuntimeError as error:
            errors.append(str(error))

    assert cycle([make_scene(on_activate=on_activate)]) == 0
    assert errors and "already running" in errors[0]


def test_scene_context_expires_after_teardown():
    contexts = []
    assert cycle([make_scene(on_activate=contexts.append)]) == 0
    assert len(contexts) == 2
    for context in contexts:
        with pytest.raises(RuntimeError):
            context.viewer
        with pytest.raises(RuntimeError):
            context.log("too late")


def test_scene_context_event_handlers_and_drag_and_drop(capfd):
    keys = []

    def failing(event):
        raise RuntimeError("handler failure")

    def on_activate(ctx):
        frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "target")
        assert ctx.enable_drag_and_drop(frame) is not None
        # The failing handler runs first, is disabled, and lets the event on.
        ctx.add_event_handler(failing)
        ctx.add_event_handler(lambda event: keys.append(event.getKey()) or True)
        event = dart.gui.GUIEventAdapter(dart.gui.GUIEventAdapter.KEYDOWN, ord("k"))
        assert ctx.viewer.handleEvent(event)

    # The failing handler counts like any failed callback.
    assert cycle([make_scene(on_activate=on_activate)]) == 1
    assert keys == [ord("k"), ord("k")]
    errors = capfd.readouterr().err
    assert "event handler failed" in errors and "handler failure" in errors


def test_edit_drives_panel_callbacks_like_a_user():
    panel = viewer.Panel()
    calls = []
    button = panel.add_button("go")
    button.on_change = lambda: calls.append("click")
    slider = panel.add_slider("gain", 0.5, min=0.0, max=1.0)
    slider.on_change = lambda: calls.append(slider.value)
    with pytest.raises(TypeError):
        button.edit(1)
    with pytest.raises(TypeError):
        panel.add_swatch([1, 0, 0, 1]).edit([0, 1, 0, 1])
    with pytest.raises(TypeError):
        panel.add_text("label").edit()

    def on_activate(ctx):
        button.edit()
        slider.edit(5.0)  # Clamped, as a drag past the end would be.
        slider.edit(float("nan"))  # Ignored, as typed garbage would be.

    scene = make_scene(panel=panel, on_activate=on_activate)
    assert cycle([scene], frames=1) == 0
    # A factory may return the same panel each time: every activation of the
    # cycle's two passes reconnects its callbacks.
    assert calls == ["click", 1.0, "click"]


def test_scene_setup_keeps_a_world_subclass():
    class TaggedWorld(dart.simulation.World):
        pass

    def make():
        world = TaggedWorld()
        world.tag = "kept"
        return viewer.SceneSetup(world)

    setup = make()
    gc.collect()
    assert type(setup.world) is TaggedWorld and setup.world.tag == "kept"


def test_add_attachment_takes_native_visuals_only():
    class Drawing(dart.gui.ViewerAttachment):
        def refresh(self):
            pass

    grids = []

    def on_activate(ctx):
        with pytest.raises(TypeError, match="native"):
            ctx.add_attachment(Drawing())
        grids.append(dart.gui.GridVisual())
        ctx.add_attachment(grids[-1])
        assert ctx.viewer.getAttachments()[-1] is grids[-1]

    viewers = []

    def keep_viewer(ctx):
        on_activate(ctx)
        viewers.append(ctx.viewer)

    assert cycle([make_scene(on_activate=keep_viewer)]) == 0
    # Each activation's grid was removed at its teardown.
    assert len(grids) == 2
    assert all(grid not in viewers[-1].getAttachments() for grid in grids)


def test_viewer_objects_survive_interpreter_shutdown():
    run_isolated(
        """
        v = dart.viewer
        panel = v.Panel()
        panel.add_button("b").on_change = lambda: panel
        setups = []
        def build():
            setup = v.SceneSetup(dart.simulation.World(), panel=panel)
            setups.append(setup)
            return setup
        scene = v.Scene("kept", title="Kept", category="Tests", factory=build)
        assert v.run([scene], ["test", "--cycle-scenes", "--frames", "1"]) == 0
        # run_isolated drops function locals before exit; keep these to the end.
        globals().update(kept_panel=panel, kept_scene=scene, kept_setups=setups)
        """
    )
