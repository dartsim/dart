"""Run unchanged with either dartpy extension selected through PYTHONPATH."""

import gc
import os
import subprocess
import sys
from pathlib import Path

import dartpy as dart
import pytest

if not hasattr(dart.gui, "osg"):
    pytest.skip("DART_BUILD_GUI_OSG is disabled", allow_module_level=True)

from ._gui_probe import probe
from ._gui_cases import NODE_TYPES, dispatch, recording_handler, recording_node

osg = dart.gui.osg


def test_minimal_world_and_skeleton_shared_ownership():
    world = dart.simulation.World()
    skeleton = dart.dynamics.Skeleton()
    name = world.addSkeleton(skeleton=skeleton)
    assert isinstance(name, str) and name
    del skeleton
    gc.collect()
    assert world.getTime() == 0.0
    world.step()
    assert world.getTime() > 0.0


def run_child(case, kind=None):
    command = [sys.executable, str(Path(__file__).with_name("_gui_cases.py")), case]
    if kind is not None:
        command.append(kind)
    environment = os.environ.copy()
    environment["DISPLAY"] = ""
    # Preserve the caller's chosen dartpy/probe extension, including a guarded
    # runner's sys.path additions that are absent from its ambient PYTHONPATH.
    environment["PYTHONPATH"] = os.pathsep.join(str(path) for path in sys.path if path)
    result = subprocess.run(
        command,
        env=environment,
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    output = f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    assert result.returncode == 0, output
    assert "nanobind: leaked" not in result.stderr, output
    assert "ERROR: AddressSanitizer" not in result.stderr, output
    assert "AddressSanitizer:DEADLYSIGNAL" not in result.stderr, output
    assert "LeakSanitizer" not in result.stderr, output
    assert result.stdout.strip(), "subprocess did not finish its oracle"


@pytest.mark.parametrize("kind", NODE_TYPES)
def test_all_five_virtuals_and_custom_constructor(kind):
    dispatch(kind)


@pytest.mark.parametrize("kind", NODE_TYPES)
def test_subclass_without_overrides_uses_cpp_fallback(kind):
    base = getattr(osg, kind)

    class Plain(base):
        def __init__(self, world, extra):
            super().__init__(world)
            self.extra = extra

    world = dart.simulation.World()
    node = Plain(world, "fallback")
    assert node.extra == "fallback"
    assert node.getWorld() is world
    probe.refresh(node)
    node.customPreRefresh()
    node.customPostRefresh()
    node.customPreStep()
    node.customPostStep()
    assert world.getTime() == 0.0


def test_handler_virtual_dispatch_custom_constructor_and_fallback():
    events = []
    viewer = osg.Viewer()
    handler = recording_handler(events, expected_viewer=viewer)
    assert handler.extra == "custom constructor argument"
    key = int(osg.GUIEventAdapter.KEY_A)
    assert probe.handle(handler, viewer, key=key) is True
    assert probe.handle(handler, viewer, key=key + 1) is False
    assert events == [
        (key, int(osg.GUIEventAdapter.KEYDOWN)),
        (key + 1, int(osg.GUIEventAdapter.KEYDOWN)),
    ]

    class Plain(osg.GUIEventHandler):
        pass

    assert probe.handle(Plain(), viewer, key=key) is True
    assert probe.handle(osg.GUIEventHandler(), viewer, key=key) is True


def test_callback_event_survives_native_temporary_owner():
    class RememberEvent(osg.GUIEventHandler):
        def handle(self, ea, aa):
            self.event = ea
            return True

    handler = RememberEvent()
    viewer = osg.Viewer()
    key = int(osg.GUIEventAdapter.KEY_A)
    assert probe.handle(handler, viewer, key=key) is True
    gc.collect()
    assert handler.event.getKey() == key
    assert handler.event.getEventType() == osg.GUIEventAdapter.KEYDOWN


@pytest.mark.parametrize("name", ("KEY_A", "KEYDOWN"))
def test_event_enums_equal_and_hash_like_int(name):
    value = getattr(osg.GUIEventAdapter, name)
    integer = int(value)
    assert value == integer
    assert integer == value
    assert hash(value) == hash(integer)
    assert {value: "enum"}[integer] == "enum"
    assert {integer: "integer"}[value] == "integer"


@pytest.mark.parametrize("kind", NODE_TYPES)
@pytest.mark.parametrize("with_world", (False, True))
def test_base_constructors_and_world_replacement(kind, with_world):
    base = getattr(osg, kind)
    world = dart.simulation.World()
    node = base(world=world) if with_world else base()
    assert node.getWorld() is (world if with_world else None)
    replacement = dart.simulation.World()
    node.setWorld(newWorld=replacement)
    assert node.getWorld() is replacement
    probe.refresh(node)


def test_viewer_keyword_arguments_and_world_removal_overload():
    world = dart.simulation.World()
    events = []
    node = recording_node("WorldNode", world, events)
    viewer = osg.Viewer()
    expected = ["refresh", "pre_refresh", "post_refresh"]
    viewer.addWorldNode(newWorldNode=node)
    probe.refresh_viewer(viewer)
    assert events == expected, events
    events.clear()
    viewer.removeWorldNode(oldWorldNode=node)
    probe.refresh_viewer(viewer)
    assert events == [], events
    viewer.addWorldNode(newWorldNode=node, active=True)
    probe.refresh_viewer(viewer)
    assert events == expected, events
    events.clear()
    viewer.removeWorldNode(oldWorld=world)
    probe.refresh_viewer(viewer)
    assert events == [], events


def test_ref_ptr_return_and_parameter_survive_original_owner():
    world = dart.simulation.World()
    viewer = osg.Viewer()
    technique = osg.WorldNode.createDefaultShadowTechnique(viewer)
    assert isinstance(technique, osg.ShadowTechnique)
    node = osg.WorldNode(world=world, shadowTechnique=technique)
    assert node.getShadowTechnique() is technique
    other = osg.RealTimeWorldNode(
        world=world,
        shadower=technique,
        targetFrequency=120.0,
        targetRealTimeFactor=0.5,
    )
    assert other.getShadowTechnique() is technique
    assert other.getTargetFrequency() == pytest.approx(120.0)
    assert other.getTargetRealTimeFactor() == pytest.approx(0.5)
    del node, other, viewer
    gc.collect()
    # This reuses a C++-created shadow object after its C++ owners have gone.
    survivor = osg.WorldNode(world, technique)
    assert survivor.getShadowTechnique() is technique
    survivor.setShadowTechnique()
    assert survivor.getShadowTechnique() is None
    survivor.setShadowTechnique(shadowTechnique=technique)
    assert survivor.getShadowTechnique() is technique


def test_repeated_ref_ptr_returns_do_not_accumulate_native_references():
    viewer = osg.Viewer()
    technique = osg.WorldNode.createDefaultShadowTechnique(viewer)
    node = osg.WorldNode(dart.simulation.World(), technique)
    assert node.getShadowTechnique() is technique
    before = probe.shadow_ref_count(technique)
    for _ in range(1000):
        assert node.getShadowTechnique() is technique
    gc.collect()
    assert probe.shadow_ref_count(technique) == before


@pytest.mark.parametrize("kind", NODE_TYPES)
@pytest.mark.parametrize(
    "case",
    (
        pytest.param("drop_node", marks=pytest.mark.xfail(
            getattr(dart, "_binder", None) != "nanobind", strict=True,
            reason="pybind11 loses node overrides retained only by OSG")),
        "delete_viewer",
        "remove_readd",
        "distinct_node_cycles",
        "remove_drop",
        pytest.param("two_viewers", marks=pytest.mark.xfail(
            getattr(dart, "_binder", None) != "nanobind", strict=True,
            reason="pybind11 loses node overrides shared by native viewers")),
        "two_viewers_remove",
        pytest.param("node_self_remove", marks=pytest.mark.xfail(
            getattr(dart, "_binder", None) != "nanobind", strict=True,
            reason="pybind11 loses overrides during self-removing native callbacks")),
        "exit_live",
        "direct_base",
        "node_viewer_cycle",
    ),
)
def test_node_lifetime_subprocess(case, kind):
    run_child(case, kind)


@pytest.mark.parametrize(
    "case",
    (
        pytest.param("handler_drop", marks=pytest.mark.xfail(
            getattr(dart, "_binder", None) != "nanobind", strict=True,
            reason="pybind11 loses handler overrides retained only by native callbacks")),
        "handler_delete_viewer",
        "handler_viewer_cycle",
        pytest.param("handler_self_remove", marks=pytest.mark.xfail(
            getattr(dart, "_binder", None) != "nanobind", strict=True,
            reason="pybind11 loses handler overrides retained only by native callbacks")),
    ),
)
def test_handler_lifetime_subprocess(case):
    run_child(case)


@pytest.mark.parametrize("platform", ["win32", "darwin"])
def test_rss_oracle_is_optional_outside_linux(platform, monkeypatch):
    from types import SimpleNamespace
    from . import _gui_cases

    monkeypatch.setattr(_gui_cases, "sys", SimpleNamespace(platform=platform))
    assert _gui_cases.rss_bytes() is None
