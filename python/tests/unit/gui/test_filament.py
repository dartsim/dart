"""Neutral GUI callbacks, ownership, and real display-free capture."""

import gc
import os
import subprocess
import sys
import textwrap
import weakref
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "scripts"))
from _image_tools import read_image


def _run_gui_subprocess(code):
    environment = os.environ.copy()
    environment.pop("DISPLAY", None)
    environment.pop("WAYLAND_DISPLAY", None)
    result = subprocess.run(
        [
            sys.executable,
            "-I",
            "-c",
            "import gc, sys\n"
            "sys.path.insert(0, sys.argv[1])\n"
            "import dartpy as dart\n" + textwrap.dedent(code),
            str(Path(dart.__file__).resolve().parent),
        ],
        env=environment,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout == "ready\n"


def test_world_node_callbacks_when_paused_and_stepping():
    events = []

    class Node(dart.gui.WorldNode):
        def customPreRefresh(self):
            events.append("preRefresh")

        def customPostRefresh(self):
            events.append("postRefresh")

        def customPreStep(self):
            events.append("preStep")

        def customPostStep(self):
            events.append("postStep")

    world = dart.simulation.World()
    node = Node(world)
    node.setNumStepsPerCycle(3)
    node.refresh()
    assert events == ["preRefresh", "postRefresh"]
    assert world.getSimFrames() == 0
    events.clear()
    node.simulate(True)
    node.refresh()
    assert events == ["preRefresh"] + ["preStep", "postStep"] * 3 + ["postRefresh"]
    assert world.getSimFrames() == 3


def test_viewer_retains_python_callbacks_until_node_removed(monkeypatch):
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    events = []

    class Node(dart.gui.WorldNode):
        def setupViewer(self):
            assert self.getViewer() is viewer
            events.append("setup")

        def customPreRefresh(self):
            events.append("refresh")

    world = dart.simulation.World()
    viewer = dart.gui.Viewer()
    node = Node(world)
    reference = weakref.ref(node)
    viewer.addWorldNode(node)
    del node
    gc.collect()
    assert reference() is not None
    assert viewer.getNumWorldNodes() == 1
    viewer.getWorldNode(world).refresh()
    assert events == ["setup", "refresh"]
    viewer.removeWorldNode(world)
    gc.collect()
    assert viewer.getNumWorldNodes() == 0
    assert reference() is None


def test_attachment_callback_and_release():
    attached = []

    class Attachment(dart.gui.ViewerAttachment):
        def customAttach(self, viewer):
            attached.append(viewer is self.getViewer())

        def refresh(self):
            pass

    viewer = dart.gui.Viewer()
    attachment = Attachment()
    viewer.addAttachment(attachment)
    assert viewer.getAttachments() == [attachment]
    assert attached == [True]
    assert attachment.getViewer() is viewer
    viewer.removeAttachment(attachment)
    assert viewer.getAttachments() == []
    assert attachment.getViewer() is None


def test_world_activation_accepts_world_and_node():
    world = dart.simulation.World()
    node = dart.gui.WorldNode(world)
    node.setNumStepsPerCycle(1)
    viewer = dart.gui.Viewer()
    viewer.addWorldNode(node)
    viewer.setUpViewForCapture(320, 240)
    viewer.simulate(True)
    viewer.setWorldNodeActive(world, False)
    viewer.frame()
    assert world.getSimFrames() == 0
    viewer.setWorldNodeActive(node, True)
    viewer.frame()
    assert world.getSimFrames() == 1


def test_real_time_node_subclass_callbacks():
    events = []

    class Node(dart.gui.RealTimeWorldNode):
        def customPreRefresh(self):
            events.append("pre")

        def customPostRefresh(self):
            events.append("post")

    node = Node(dart.simulation.World(), 120.0, 0.5)
    assert node.getTargetFrequency() == pytest.approx(120.0)
    assert node.getTargetRealTimeFactor() == pytest.approx(0.5)
    node.refresh()
    assert events == ["pre", "post"]


def test_widget_callback_survives_shared_ownership():
    rendered = []

    class Widget(dart.gui.ImGuiWidget):
        def render(self):
            rendered.append(True)

    viewer = dart.gui.Viewer()
    widget = Widget()
    reference = weakref.ref(widget)
    viewer.addWidget(widget)
    del widget
    gc.collect()
    assert reference() is not None
    reference().render()
    assert rendered == [True]
    viewer.removeWidget(reference())
    gc.collect()
    assert reference() is None


def test_event_handler_retained_until_removed_without_graphics():
    received = []
    adapters = []
    viewer = dart.gui.Viewer()

    class Handler(dart.gui.GUIEventHandler):
        def handle(self, event, action):
            assert action.getViewer() is viewer
            received.append((event.getEventType(), event.getKey()))
            adapters.append((event, action))
            return True

    handler = Handler()
    reference = weakref.ref(handler)
    viewer.addEventHandler(handler)
    del handler
    gc.collect()
    event = dart.gui.GUIEventAdapter(dart.gui.GUIEventAdapter.KEYDOWN, ord("a"))
    assert viewer.handleEvent(event)
    assert received == [(dart.gui.GUIEventAdapter.KEYDOWN, ord("a"))]
    viewer.removeEventHandler(reference())
    gc.collect()
    assert reference() is None
    del event
    viewer_reference = weakref.ref(viewer)
    del viewer
    gc.collect()
    assert adapters[0][0].getKey() == ord("a")
    assert adapters[0][1].getViewer() is viewer_reference()
    adapters.clear()
    gc.collect()
    assert viewer_reference() is None


def test_interactive_visual_children_keep_parent_alive():
    frame = dart.gui.InteractiveFrame(dart.dynamics.Frame.World())
    reference = weakref.ref(frame)
    shapes = frame.getShapeFrames()
    tool = frame.getTool(dart.gui.InteractiveTool.LINEAR, 0)
    assert shapes
    del frame
    gc.collect()
    assert reference() is not None
    assert all(shape.getName() for shape in shapes)
    assert tool.getInteractiveFrame() is reference()
    del shapes, tool
    gc.collect()
    assert reference() is None


def test_interactive_children_survive_visual_removal_and_resize():
    frame = dart.gui.InteractiveFrame(dart.dynamics.Frame.World())
    child = frame.addShapeFrame(dart.dynamics.SphereShape(0.1))
    original_tool = frame.getTool(dart.gui.InteractiveTool.LINEAR, 0)
    tool_child = original_tool.addShapeFrame(dart.dynamics.SphereShape(0.2))
    frame.removeAllShapeFrames()
    original_tool.removeAllShapeFrames()
    assert child.getParentFrame().isWorld()
    assert tool_child.getParentFrame().isWorld()
    frame.resizeStandardVisuals(0.3, 1.0)
    assert original_tool.getInteractiveFrame() is frame
    assert child.getShape().getRadius() == pytest.approx(0.1)
    assert tool_child.getShape().getRadius() == pytest.approx(0.2)
    del frame, original_tool
    gc.collect()
    assert np.isfinite(child.getWorldTransform().matrix()).all()
    assert np.isfinite(tool_child.getWorldTransform().matrix()).all()


def test_drag_reference_remains_safe_after_disable_and_viewer_deletion():
    frame = dart.gui.InteractiveFrame(dart.dynamics.Frame.World())
    viewer = dart.gui.Viewer()
    drag = viewer.enableDragAndDrop(frame)
    assert isinstance(drag, dart.gui.InteractiveFrameDnD)
    assert drag.getFrame() is frame
    assert viewer.disableDragAndDrop(drag)
    assert drag.getEntity() is None
    assert drag.getFrame() is None
    del viewer, frame
    gc.collect()
    drag.update()
    drag.saveState()
    drag.move()


def test_shape_drag_reference_remains_safe_after_shape_replacement():
    frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
    frame.setShape(dart.dynamics.SphereShape(0.5))
    viewer = dart.gui.Viewer()
    drag = viewer.enableDragAndDrop(frame, frame.getShape())
    assert isinstance(drag, dart.gui.SimpleFrameShapeDnD)
    frame.setShape(dart.dynamics.SphereShape(1.0))
    gc.collect()
    assert drag.getShape() is None
    assert drag.getEntity() is None
    drag.update()
    assert not viewer.disableDragAndDrop(drag)


@pytest.mark.parametrize(
    "constructor,argument",
    [
        ("SimpleFrameDnD", 0),
        ("SimpleFrameDnD", 1),
        ("SimpleFrameShapeDnD", 0),
        ("SimpleFrameShapeDnD", 1),
        ("SimpleFrameShapeDnD", 2),
        ("InteractiveFrameDnD", 0),
        ("InteractiveFrameDnD", 1),
        ("BodyNodeDnD", 0),
        ("BodyNodeDnD", 1),
    ],
)
def test_drag_constructors_reject_null_arguments(constructor, argument):
    _run_gui_subprocess(
        f"""
        viewer = dart.gui.Viewer()
        frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
        interactive = dart.gui.InteractiveFrame(dart.dynamics.Frame.World())
        shape = dart.dynamics.SphereShape(0.5)
        skeleton = dart.dynamics.Skeleton()
        body = skeleton.createWeldJointAndBodyNodePair()[1]
        arguments = {{
            "SimpleFrameDnD": [viewer, frame],
            "SimpleFrameShapeDnD": [viewer, frame, shape],
            "InteractiveFrameDnD": [viewer, interactive],
            "BodyNodeDnD": [viewer, body],
        }}["{constructor}"]
        arguments[{argument}] = None
        try:
            drag = getattr(dart.gui, "{constructor}")(*arguments)
            drag.update()
        except TypeError:
            pass
        else:
            raise AssertionError("null drag argument was accepted")
        print("ready")
        """
    )


def test_drag_factory_rejects_null_arguments():
    _run_gui_subprocess(
        """
        viewer = dart.gui.Viewer()
        frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
        shape = dart.dynamics.SphereShape(0.5)
        for arguments in ((None,), (frame, None), (None, shape)):
            try:
                viewer.enableDragAndDrop(*arguments)
            except TypeError:
                pass
            else:
                raise AssertionError("null drag argument was accepted")
        print("ready")
        """
    )


@pytest.mark.parametrize(
    "entity_type,getter",
    [
        ("simple", "getEntity"),
        ("simple", "getSimpleFrame"),
        ("interactive", "getEntity"),
        ("interactive", "getFrame"),
        ("body", "getEntity"),
        ("body", "getBodyNode"),
        ("soft_body", "getEntity"),
        ("soft_body", "getBodyNode"),
        ("shape", "getShape"),
    ],
)
@pytest.mark.parametrize("creation", ["factory", "direct"])
def test_drag_getters_retain_exact_owner_after_detach(entity_type, getter, creation):
    _run_gui_subprocess(
        f"""
        world = dart.simulation.World()
        viewer = dart.gui.Viewer()
        if "{entity_type}" in ("body", "soft_body"):
            skeleton = dart.dynamics.Skeleton()
            if "{entity_type}" == "soft_body":
                properties = dart.dynamics.SoftBodyNodeHelper.makeEllipsoidProperties(
                    [0.2, 0.2, 0.2], 6, 6, 0.5)
                body_properties = dart.dynamics.BodyNodeProperties(
                    dart.dynamics.BodyNodeAspectProperties("retained"))
                frame = skeleton.createFreeJointAndSoftBodyNodePair(
                    None, dart.dynamics.FreeJointProperties(),
                    dart.dynamics.SoftBodyNodeProperties(
                        body_properties, properties))[1]
            else:
                frame = skeleton.createWeldJointAndBodyNodePair()[1]
            frame.setName("retained")
            world.addSkeleton(skeleton)
            drag = (dart.gui.BodyNodeDnD(viewer, frame)
                    if "{creation}" == "direct"
                    else viewer.enableDragAndDrop(frame))
            del skeleton
        else:
            frame_type = (dart.gui.InteractiveFrame
                          if "{entity_type}" == "interactive"
                          else dart.dynamics.SimpleFrame)
            frame = frame_type(dart.dynamics.Frame.World(), "retained")
            frame.setShape(dart.dynamics.SphereShape(0.5))
            world.addSimpleFrame(frame)
            if "{creation}" == "direct":
                if "{entity_type}" == "shape":
                    drag = dart.gui.SimpleFrameShapeDnD(viewer, frame, frame.getShape())
                elif "{entity_type}" == "interactive":
                    drag = dart.gui.InteractiveFrameDnD(viewer, frame)
                else:
                    drag = dart.gui.SimpleFrameDnD(viewer, frame)
            else:
                drag = (viewer.enableDragAndDrop(frame, frame.getShape())
                        if "{entity_type}" == "shape"
                        else viewer.enableDragAndDrop(frame))
        del frame
        gc.collect()
        saved = getattr(drag, "{getter}")()
        assert saved is not None
        if "{creation}" == "factory":
            assert viewer.disableDragAndDrop(drag)
            assert drag.getEntity() is None
        if "{entity_type}" in ("body", "soft_body"):
            saved.remove()
            world.removeAllSkeletons()
        elif "{entity_type}" == "shape":
            world.removeAllSimpleFrames()
        else:
            # The wrapper's holder must address the frame itself.
            world.removeSimpleFrame(saved)
            assert world.getNumSimpleFrames() == 0
        del world, viewer, drag
        gc.collect()
        if "{entity_type}" == "shape":
            assert saved.getRadius() == 0.5
        else:
            assert saved.getName() == "retained"
            saved.getWorldTransform()
        if "{entity_type}" == "soft_body":
            assert isinstance(saved, dart.dynamics.SoftBodyNode)
            assert saved.getNumPointMasses() == 32
        print("ready")
        """
    )


@pytest.mark.parametrize("frame_kind", ["simple", "interactive"])
def test_pick_frame_owner_is_usable_after_wrapper_collection(frame_kind):
    _run_gui_subprocess(
        f"""
        world = dart.simulation.World()
        frame_type = (dart.gui.InteractiveFrame if "{frame_kind}" == "interactive"
                      else dart.dynamics.SimpleFrame)
        frame = frame_type(dart.dynamics.Frame.World(), "picked")
        world.addSimpleFrame(frame)
        pick = dart.gui.PickInfo()
        pick.frame = frame
        del frame
        gc.collect()
        world.removeSimpleFrame(pick.frame)
        assert world.getNumSimpleFrames() == 0
        print("ready")
        """
    )


def test_drag_getter_owner_releases_with_returned_wrapper():
    world = dart.simulation.World()
    frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
    reference = weakref.ref(frame)
    world.addSimpleFrame(frame)
    viewer = dart.gui.Viewer()
    drag = viewer.enableDragAndDrop(frame)
    saved = drag.getEntity()
    world.removeAllSimpleFrames()
    del frame
    gc.collect()
    assert reference() is saved
    del saved
    gc.collect()
    assert reference() is None
    assert drag.getEntity() is None


@pytest.mark.parametrize("frame_kind", ["simple", "interactive"])
@pytest.mark.parametrize("comparison", ["unhashable", "equal", "hostile"])
@pytest.mark.parametrize("creation", ["factory", "direct_subclass"])
def test_owner_caches_use_wrapper_identity(frame_kind, comparison, creation):
    _run_gui_subprocess(
        f"""
        import weakref
        frame_base = (dart.gui.InteractiveFrame if "{frame_kind}" == "interactive"
                      else dart.dynamics.SimpleFrame)
        drag_base = (dart.gui.InteractiveFrameDnD if "{frame_kind}" == "interactive"
                     else dart.gui.SimpleFrameDnD)

        class Frame(frame_base):
            calls = 0
            def __eq__(self, other):
                Frame.calls += 1
                if "{comparison}" == "hostile":
                    raise AssertionError("frame equality was invoked")
                return isinstance(other, Frame)
            if "{comparison}" == "unhashable":
                __hash__ = None
            else:
                def __hash__(self):
                    Frame.calls += 1
                    if "{comparison}" == "hostile":
                        raise AssertionError("frame hashing was invoked")
                    return 1

        class Drag(drag_base):
            calls = 0
            def __eq__(self, other):
                Drag.calls += 1
                if "{comparison}" == "hostile":
                    raise AssertionError("drag equality was invoked")
                return isinstance(other, Drag)
            if "{comparison}" == "unhashable":
                __hash__ = None
            else:
                def __hash__(self):
                    Drag.calls += 1
                    if "{comparison}" == "hostile":
                        raise AssertionError("drag hashing was invoked")
                    return 1

        world = dart.simulation.World()
        viewer = dart.gui.Viewer()
        frames = [Frame(dart.dynamics.Frame.World(), name) for name in ("A", "B")]
        references = [weakref.ref(frame) for frame in frames]
        for frame in frames:
            world.addSimpleFrame(frame)
        del frame
        drags = [(Drag(viewer, frame) if "{creation}" == "direct_subclass"
                  else viewer.enableDragAndDrop(frame)) for frame in frames]
        saved = [drag.getEntity() for drag in drags]
        assert all(left is right for left, right in zip(saved, frames))
        specific_getter = ("getFrame" if "{frame_kind}" == "interactive"
                           else "getSimpleFrame")
        assert all(getattr(drag, specific_getter)() is frame
                   for drag, frame in zip(drags, frames))
        picks = [dart.gui.PickInfo(), dart.gui.PickInfo()]
        for pick, frame in zip(picks, frames):
            pick.frame = frame
            assert pick.frame is frame
            pick.frame = None
        del pick, frame, picks
        if "{frame_kind}" == "interactive":
            for frame in frames:
                children = frame.getShapeFrames()
                tool = frame.getTool(dart.gui.InteractiveTool.LINEAR, 0)
                assert tool.getInteractiveFrame() is frame
                del children, tool
            del frame
        if "{creation}" == "factory":
            for drag in drags:
                assert viewer.disableDragAndDrop(drag)
            del drag
        for frame in frames:
            world.removeSimpleFrame(frame)
        del frame
        del frames, drags, world, viewer
        gc.collect()
        assert [frame.getName() for frame in saved] == ["A", "B"]
        saved[1].setName("changed B")
        assert saved[0].getName() == "A"
        assert saved[1].getName() == "changed B"
        assert all(reference() is frame for reference, frame in zip(references, saved))
        assert Frame.calls == Drag.calls == 0
        del saved
        gc.collect()
        assert all(reference() is None for reference in references)
        print("ready")
        """
    )


def test_drag_getter_reuses_exact_owner_of_borrowed_pick_wrapper():
    _run_gui_subprocess(
        """
        world = dart.simulation.World()
        parent = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "parent")
        frame = dart.dynamics.SimpleFrame(parent, "borrowed")
        frame.setShape(dart.dynamics.BoxShape([1, 1, 1]))
        frame.getVisualAspect(True)
        world.addSimpleFrame(parent)
        world.addSimpleFrame(frame)
        del frame
        gc.collect()
        borrowed = next(child for child in parent.getChildFrames()
                        if child.getName() == "borrowed")
        viewer = dart.gui.Viewer()
        viewer.addWorldNode(dart.gui.WorldNode(world))
        viewer.setCameraHome([0, 0, 4], [0, 0, 0], [0, 1, 0])
        picks = viewer.computeIntersections(dart.gui.GUIEventAdapter(
            dart.gui.GUIEventAdapter.MOVE, x=800, y=500))
        assert picks[0].frame is borrowed
        drag = viewer.enableDragAndDrop(borrowed)
        saved = drag.getEntity()
        assert saved is borrowed
        assert viewer.disableDragAndDrop(drag)
        world.removeAllSimpleFrames()
        del picks, borrowed, drag, world, viewer, parent
        gc.collect()
        assert saved.getName() == "borrowed"
        saved.getWorldTransform()
        print("ready")
        """
    )


def test_event_handler_rejects_out_of_range_mouse_enums():
    _run_gui_subprocess(
        """
        viewer = dart.gui.Viewer()
        handler = viewer.getDefaultEventHandler()
        button = dart.gui.MouseButton(99)
        event = dart.gui.MouseButtonEvent(-1)
        for call in (
            lambda: handler.getButtonEvent(button),
            lambda: handler.getButtonPicks(button, dart.gui.BUTTON_PUSH),
            lambda: handler.getButtonPicks(dart.gui.LEFT_MOUSE, event),
            lambda: handler.suppressButtonPicks(button, dart.gui.BUTTON_PUSH),
            lambda: handler.suppressButtonPicks(dart.gui.LEFT_MOUSE, event),
            lambda: handler.activateButtonPicks(button, dart.gui.BUTTON_PUSH),
            lambda: handler.activateButtonPicks(dart.gui.LEFT_MOUSE, event),
        ):
            try:
                call()
            except (IndexError, ValueError):
                pass
            else:
                raise AssertionError("invalid mouse enum was accepted")
        print("ready")
        """
    )


def test_viewers_can_share_world():
    world = dart.simulation.World()
    first = dart.gui.Viewer()
    second = dart.gui.Viewer()
    first_node = dart.gui.WorldNode(world)
    second_node = dart.gui.WorldNode(world)
    first.addWorldNode(first_node)
    second.addWorldNode(second_node)
    assert first_node.getWorld() is second_node.getWorld()
    assert first.getWorldNode(world) is first_node
    assert second.getWorldNode(world) is second_node


def test_default_event_handler_is_owned_by_its_viewer():
    # A standalone handler could outlive the viewer it points to.
    with pytest.raises(TypeError):
        dart.gui.DefaultEventHandler(dart.gui.Viewer())
    viewer = dart.gui.Viewer()
    assert isinstance(viewer.getDefaultEventHandler(), dart.gui.DefaultEventHandler)


def _center_picks(world):
    viewer = dart.gui.Viewer()
    viewer.addWorldNode(dart.gui.WorldNode(world))
    viewer.setCameraHome([0, 0, 4], [0, 0, 0], [0, 1, 0])
    return viewer.computeIntersections(
        dart.gui.GUIEventAdapter(dart.gui.GUIEventAdapter.MOVE, x=800, y=500)
    )


def test_picked_frame_survives_world_removal_and_pick_deletion():
    world, box = _box_world()
    picks = _center_picks(world)
    assert picks
    retained = picks[0].frame
    world.removeSimpleFrame(box)
    del box, picks, world
    gc.collect()
    assert retained.getName() == "colored box"
    assert np.isfinite(retained.getWorldTransform().matrix()).all()


def test_picked_shape_node_survives_explicit_removal():
    world = dart.simulation.World()
    skeleton = dart.dynamics.Skeleton()
    body = skeleton.createWeldJointAndBodyNodePair()[1]
    shape = body.createShapeNode(dart.dynamics.BoxShape([1, 1, 1]))
    shape.getVisualAspect(True)
    world.addSkeleton(skeleton)
    picks = _center_picks(world)
    assert picks
    retained = picks[0].frame
    assert retained is shape  # Reuse the pre-existing borrowed core wrapper.
    picks[0].frame = None
    body.removeAllShapeNodes()
    world.removeSkeleton(skeleton)
    del shape, body, skeleton, world, picks
    gc.collect()
    assert np.isfinite(retained.getWorldTransform().matrix()).all()
    assert retained.getShape().getSize().tolist() == [1, 1, 1]


def test_externally_owned_picked_child_rejects_unsafe_borrowing():
    world = dart.simulation.World()
    parent = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
    child = dart.dynamics.SimpleFrame(parent)
    child.setShape(dart.dynamics.BoxShape([1, 1, 1]))
    child.getVisualAspect(True)
    world.addSimpleFrame(parent)
    picks = _center_picks(world)
    assert picks
    with pytest.raises(RuntimeError, match="no shared owner"):
        _ = picks[0].frame
    del child
    gc.collect()
    assert picks[0].frame is None


def _box_world():
    world = dart.simulation.World()
    box = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "colored box")
    box.setShape(dart.dynamics.BoxShape([1.0, 1.0, 1.0]))
    box.getVisualAspect(True).setColor([0.9, 0.08, 0.08])
    world.addSimpleFrame(box)
    return world, box


def _capture(viewer, filename):
    viewer.captureScreen(str(filename))
    viewer.frame()
    assert filename.is_file()
    image = read_image(filename)
    assert (image.width, image.height) == (320, 240)
    return (
        np.frombuffer(image.pixels, dtype=np.uint8)
        .reshape(240, 320, 3)
        .astype(np.int16)
    )


def test_display_free_capture_updates_live_colors(tmp_path, monkeypatch):
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    world, box = _box_world()
    viewer = dart.gui.Viewer()
    rendered = []
    frame_events = []
    viewer_reference = weakref.ref(viewer)

    class Handler(dart.gui.GUIEventHandler):
        def handle(self, event, action):
            if event.getEventType() == dart.gui.GUIEventAdapter.FRAME:
                assert action.getViewer() is viewer_reference()
                frame_events.append(True)
            return False

    class Widget(dart.gui.ImGuiWidget):
        def render(self):
            with pytest.raises(RuntimeError, match="recursively render"):
                viewer_reference().frame()
            rendered.append(True)

    widget = Widget()
    viewer.addWidget(widget)
    viewer.addEventHandler(Handler())
    viewer.addWorldNode(dart.gui.WorldNode(world))
    viewer.setCameraHome([2.0, -3.0, 2.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0])
    viewer.setUpViewForCapture(320, 240)
    for _ in range(5):
        viewer.frame()
    assert len(rendered) == 5
    assert len(frame_events) == 5
    assert viewer.getRenderBackend() in {"opengl", "vulkan", "metal"}
    assert viewer.getRenderDevice()
    red = _capture(viewer, tmp_path / "red.png")
    red_pixels = (red[:, :, 0] > red[:, :, 1] + 30) & (red[:, :, 0] > red[:, :, 2] + 30)
    assert np.count_nonzero(red_pixels) > 100

    box.getVisualAspect().setColor([0.08, 0.08, 0.9])
    blue = _capture(viewer, tmp_path / "blue.png")
    blue_pixels = (blue[:, :, 2] > blue[:, :, 0] + 30) & (
        blue[:, :, 2] > blue[:, :, 1] + 30
    )
    assert np.count_nonzero(blue_pixels) > 100
    assert np.count_nonzero(np.any(red != blue, axis=2)) > 100


def test_capture_rejects_invalid_size():
    viewer = dart.gui.Viewer()
    with pytest.raises(ValueError):
        viewer.setUpViewForCapture(0, 240)
    with pytest.raises(ValueError):
        viewer.setUpViewForCapture(320, -1)


def test_capture_preserves_two_pixel_overlay_line(tmp_path):
    viewer = dart.gui.Viewer()
    viewer.setCameraHome([0, -4, 0], [0, 0, 0], [0, 0, 1])
    overlay = dart.gui.DebugOverlay()
    overlay.setLineWidth(2.0)
    color = np.array([17, 101, 199])
    overlay.addLine([-0.2, 0, -0.5], [-0.2, 0, 0.5], [*(color / 255), 1.0])
    viewer.addAttachment(overlay)
    viewer.setUpViewForCapture(320, 240)
    image = _capture(viewer, tmp_path / "overlay-width.png")
    exact_color = np.all(image == color, axis=2)
    assert np.all(np.count_nonzero(exact_color[110:130], axis=1) == 2)


def test_capture_reports_write_errors(tmp_path):
    world, _ = _box_world()
    viewer = dart.gui.Viewer()
    viewer.addWorldNode(dart.gui.WorldNode(world))
    viewer.setUpViewForCapture(320, 240)
    viewer.captureScreen(str(tmp_path / "missing" / "capture.png"))
    with pytest.raises(RuntimeError):
        viewer.frame()
    # The failed capture is reported once; later frames render normally.
    viewer.frame()
    viewer.captureScreen(str(tmp_path / "capture.png"))
    viewer.frame()
    assert (tmp_path / "capture.png").stat().st_size > 0
