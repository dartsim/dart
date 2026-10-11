"""Production GUI ownership regressions, for nanobind."""

import gc
import weakref

import dartpy as dart
import pytest

from ._gui_probe import probe

gui = dart.gui


def test_factory_subclass_custom_initializer():
    class CustomViewer(gui.Viewer):
        def __init__(self, world, extra):
            super().__init__()
            self.extra = extra
            self.node = gui.WorldNode(world)
            self.addWorldNode(self.node)

    world = dart.simulation.World()
    viewer = CustomViewer(world, "subclass state")
    assert viewer.extra == "subclass state"
    assert viewer.node.getWorld() is world


def test_viewer_primary_and_secondary_bases():
    viewer = gui.Viewer()
    assert probe.base_subject(viewer) is viewer
    assert isinstance(viewer, dart.common.Subject) is False
    assert (dart.common.Subject in type(viewer).__mro__) is False
    assert gui.GUIActionAdapter(viewer).getViewer() is viewer


def test_drag_and_drop_detachment():
    viewer = gui.Viewer()
    frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
    dnd = viewer.enableDragAndDrop(frame)
    assert probe.base_subject(dnd) is dnd
    assert probe.base_observer(dnd) is dnd
    assert isinstance(dnd, dart.common.Subject) is False
    assert isinstance(dnd, dart.common.Observer) is False
    dnd.setObstructable(False)
    assert dnd.isObstructable() is False
    assert viewer.disableDragAndDrop(dnd) is True
    assert dnd.getEntity() is None
    assert dnd.getSimpleFrame() is None
    dnd.update()
    dnd.move()
    dnd.saveState()


def test_raw_default_handler_getter_identity_and_ownership():
    viewer = gui.Viewer()
    handler = viewer.getDefaultEventHandler()
    assert viewer.getDefaultEventHandler() is handler
    reference = weakref.ref(viewer)
    del viewer
    gc.collect()
    assert reference() is not None
    assert handler.getModKeyMask() == 0
    del handler
    gc.collect()
    assert reference() is None


def test_attachment_is_released_with_viewer():
    viewer = gui.Viewer()
    overlay = gui.DebugOverlay()
    reference = weakref.ref(overlay)
    viewer.addAttachment(overlay)
    del overlay
    gc.collect()
    del viewer
    gc.collect()
    assert reference() is None


def test_widget_remains_abstract():
    with pytest.raises(RuntimeError, match="pure virtual"):
        gui.ImGuiWidget().render()


def test_combined_modifier_mask_roundtrip():
    cls = gui.GUIEventAdapter.ModKeyMask
    value = cls(int(cls.MODKEY_CTRL) | int(cls.MODKEY_SHIFT))
    assert int(value) == 15
    viewer = gui.Viewer()
    frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
    dnd = viewer.enableDragAndDrop(frame)
    dnd.setRotationModKey(value)
    assert int(dnd.getRotationModKey()) == 15
    viewer.disableDragAndDrop(dnd)


@pytest.mark.parametrize("kind", ["SimpleFrameDnD", "InteractiveFrameDnD"])
def test_direct_drag_and_drop_constructor_handles_viewer_deletion(kind):
    viewer = gui.Viewer()
    frame = (
        gui.InteractiveFrame(dart.dynamics.Frame.World())
        if kind == "InteractiveFrameDnD"
        else dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
    )
    dnd = getattr(gui, kind)(viewer, frame)
    assert dnd.isMoving() is False
    del viewer
    gc.collect()
    # Direct interactions keep their viewer alive until they are released.
    assert dnd.isMoving() is False
    assert dnd.getEntity() is frame
    del dnd
    gc.collect()


def test_drag_and_drop_reenable_preserves_new_wrapper_identity():
    viewer = gui.Viewer()
    frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
    retired = []
    for _ in range(100):
        dnd = viewer.enableDragAndDrop(frame)
        assert dnd.isMoving() is False
        assert viewer.enableDragAndDrop(frame) is dnd
        assert viewer.disableDragAndDrop(dnd)
        retired.append(dnd)
        assert dnd.getEntity() is None
        assert dnd.getSimpleFrame() is None
        assert dnd.isMoving() is False
        del dnd


@pytest.mark.parametrize(
    "name,secondary",
    [
        ("Viewer", dart.common.Subject),
        ("DragAndDrop", dart.common.Subject),
        ("SimpleFrameDnD", dart.common.Subject),
        ("SimpleFrameShapeDnD", dart.common.Subject),
        ("BodyNodeDnD", dart.common.Subject),
        ("InteractiveFrameDnD", dart.common.Subject),
        ("InteractiveTool", dart.dynamics.Detachable),
        ("InteractiveFrame", dart.dynamics.Detachable),
    ],
)
def test_secondary_ancestry_is_explicit(name, secondary):
    cls = getattr(gui, name)
    assert issubclass(cls, secondary) is False
    assert (secondary in cls.__mro__) is False


def test_attachment_viewer_cycle_is_collected():
    class Overlay(gui.DebugOverlay):
        pass

    viewer = gui.Viewer()
    overlay = Overlay()
    overlay.owner = viewer
    viewer.addAttachment(overlay)
    references = weakref.ref(viewer), weakref.ref(overlay)
    del viewer, overlay
    gc.collect()
    assert all(reference() is None for reference in references)


def test_view_handler_cycle_is_collected():
    class Handler(gui.GUIEventHandler):
        pass

    viewer = gui.Viewer()
    handler = Handler()
    handler.owner = viewer
    viewer.addEventHandler(handler)
    references = weakref.ref(viewer), weakref.ref(handler)
    del viewer, handler
    gc.collect()
    assert all(reference() is None for reference in references)
