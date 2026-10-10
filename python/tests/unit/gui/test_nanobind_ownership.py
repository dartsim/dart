"""Production GUI ownership regressions, shared by both binders."""

import gc
import weakref

import dartpy as dart
import pytest


if not hasattr(dart.gui, "osg"):
    pytest.skip("DART_BUILD_GUI_OSG is disabled", allow_module_level=True)

from ._gui_probe import probe

osg = dart.gui.osg
NANOBIND = getattr(dart, "_binder", None) == "nanobind"


@pytest.mark.parametrize("base", [osg.Viewer, osg.ImGuiViewer])
def test_factory_subclass_custom_initializer(base):
    class CustomViewer(base):
        def __init__(self, world, extra):
            super().__init__()
            self.extra = extra
            self.node = osg.WorldNode(world)
            self.addWorldNode(self.node)

    world = dart.simulation.World()
    viewer = CustomViewer(world, "subclass state")
    assert viewer.extra == "subclass state"
    assert viewer.node.getWorld() is world


def test_viewer_primary_and_secondary_bases():
    viewer = osg.Viewer()
    if NANOBIND:
        assert probe.accept_action_adapter(viewer)
    else:
        with pytest.raises(TypeError):
            probe.accept_action_adapter(viewer)
    assert probe.base_view(viewer) is viewer
    assert probe.base_subject(viewer) is viewer
    assert isinstance(viewer, osg.osgViewer)
    assert isinstance(viewer, dart.common.Subject) is (not NANOBIND)
    assert osg.osgViewer in type(viewer).__mro__
    assert (dart.common.Subject in type(viewer).__mro__) is (not NANOBIND)


def test_drag_and_drop_invalidation():
    viewer = osg.Viewer()
    frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
    dnd = viewer.enableDragAndDrop(frame)
    dnd.setObstructable(False)
    assert dnd.isObstructable() is False
    assert viewer.disableDragAndDrop(dnd) is True
    if NANOBIND:
        with pytest.raises((TypeError, RuntimeError)):
            dnd.isObstructable()
    # pybind11 use-after-disable is unsupported; do not dereference freed memory.


def test_raw_imgui_getter_identity_and_ownership():
    viewer = osg.ImGuiViewer()
    handler = viewer.getImGuiHandler()
    assert viewer.getImGuiHandler() is handler
    del viewer
    gc.collect()
    handler.removeAllWidget()


def test_attachment_is_released_with_viewer():
    viewer = osg.Viewer()
    overlay = osg.DebugOverlay()
    reference = weakref.ref(overlay)
    viewer.addAttachment(overlay)
    del overlay
    gc.collect()
    del viewer
    gc.collect()
    assert reference() is None


def test_widget_remains_abstract():
    with pytest.raises(TypeError):
        osg.ImGuiWidget()


def test_combined_modifier_mask_roundtrip():
    cls = osg.GUIEventAdapter.ModKeyMask
    value = cls(int(cls.MODKEY_CTRL) | int(cls.MODKEY_SHIFT))
    assert int(value) == 15
    assert value.name == "???"
    assert str(value) == "ModKeyMask.???"
    viewer = osg.Viewer()
    frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
    dnd = viewer.enableDragAndDrop(frame)
    dnd.setRotationModKey(value)
    assert int(dnd.getRotationModKey()) == 15
    viewer.disableDragAndDrop(dnd)


@pytest.mark.parametrize("kind", ["SimpleFrameDnD", "InteractiveFrameDnD"])
def test_direct_drag_and_drop_constructor_handles_viewer_deletion(kind):
    viewer = osg.Viewer()
    frame = (osg.InteractiveFrame(dart.dynamics.Frame.World())
             if kind == "InteractiveFrameDnD"
             else dart.dynamics.SimpleFrame(dart.dynamics.Frame.World()))
    dnd = getattr(osg, kind)(viewer, frame)
    assert dnd.isMoving() is False
    if NANOBIND:
        del viewer
        gc.collect()
        with pytest.raises((TypeError, RuntimeError)):
            dnd.isMoving()
        del dnd
    else:
        # Direct pybind11 constructors must release their holder before native deletion.
        del dnd, viewer
    gc.collect()


def test_drag_and_drop_reenable_preserves_new_wrapper_identity():
    viewer = osg.Viewer()
    frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
    retired = []
    for _ in range(100):
        dnd = viewer.enableDragAndDrop(frame)
        assert dnd.isMoving() is False
        assert viewer.enableDragAndDrop(frame) is dnd
        assert viewer.disableDragAndDrop(dnd)
        if NANOBIND:
            retired.append(dnd)
            with pytest.raises((TypeError, RuntimeError)):
                dnd.isMoving()
        del dnd


def test_exported_key_aliases_preserve_canonical_names():
    event = osg.GUIEventAdapter
    for alias, canonical in [
        ("KEY_Page_Up", "KEY_Prior"),
        ("KEY_Page_Down", "KEY_Next"),
        ("KEY_KP_Page_Up", "KEY_KP_Prior"),
        ("KEY_KP_Page_Down", "KEY_KP_Next"),
        ("KEY_Script_switch", "KEY_Mode_switch"),
    ]:
        value = getattr(event, alias)
        assert value == getattr(event, canonical)
        assert value.name == canonical
        assert str(value) == "KeySymbol." + canonical


@pytest.mark.parametrize("name,secondary", [
    ("Viewer", dart.common.Subject),
    ("ImGuiViewer", dart.common.Subject),
    ("DragAndDrop", dart.common.Subject),
    ("SimpleFrameDnD", dart.common.Subject),
    ("SimpleFrameShapeDnD", dart.common.Subject),
    ("BodyNodeDnD", dart.common.Subject),
    ("InteractiveFrameDnD", dart.common.Subject),
    ("InteractiveTool", dart.dynamics.Detachable),
    ("InteractiveFrame", dart.dynamics.Detachable),
])
def test_secondary_ancestry_is_explicit(name, secondary):
    cls = getattr(osg, name)
    assert issubclass(cls, secondary) is (not NANOBIND)
    assert (secondary in cls.__mro__) is (not NANOBIND)


def test_attachment_viewer_cycle_is_collected():
    class Overlay(osg.DebugOverlay):
        pass

    viewer = osg.Viewer()
    overlay = Overlay()
    overlay.owner = viewer
    viewer.addAttachment(overlay)
    references = weakref.ref(viewer), weakref.ref(overlay)
    del viewer, overlay
    gc.collect()
    assert all(reference() is None for reference in references)


def test_view_handler_cycle_is_collected():
    class Handler(osg.GUIEventHandler):
        pass

    viewer = osg.osgViewer()
    handler = Handler()
    handler.owner = viewer
    viewer.addEventHandler(handler)
    references = weakref.ref(viewer), weakref.ref(handler)
    del viewer, handler
    gc.collect()
    assert all(reference() is None for reference in references)
