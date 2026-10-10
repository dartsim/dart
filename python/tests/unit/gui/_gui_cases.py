"""Shared ownership oracles. None of these paths creates a graphics context."""

import gc
import json
import math
import os
import sys
import time
import weakref
from pathlib import Path

if __package__:
    from ._gui_probe import probe
else:
    from _gui_probe import probe
import dartpy as dart

osg = dart.gui.osg
NODE_TYPES = ("WorldNode", "RealTimeWorldNode")
EXIT_ROOTS = []


def recording_node(kind, world, events):
    base = getattr(osg, kind)

    class Node(base):
        def __init__(self, world, extra):
            super().__init__(world)
            self.extra = extra

        def refresh(self):
            events.append("refresh")
            super().refresh()

        def customPreRefresh(self):
            events.append("pre_refresh")

        def customPostRefresh(self):
            events.append("post_refresh")

        def customPreStep(self):
            events.append("pre_step")

        def customPostStep(self):
            events.append("post_step")

    return Node(world, "custom constructor argument")


def recording_handler(events, expected_viewer=None):
    class Handler(osg.GUIEventHandler):
        def __init__(self, extra):
            super().__init__()
            self.extra = extra

        def handle(self, ea, aa):
            if expected_viewer is not None:
                assert aa is expected_viewer, (aa, expected_viewer)
            events.append((ea.getKey(), int(ea.getEventType())))
            return ea.getKey() == int(osg.GUIEventAdapter.KEY_A)

    return Handler("custom constructor argument")


def rss_bytes():
    if sys.platform != "linux":
        return None
    # Current resident pages avoid ru_maxrss's monotonic high-water mark.
    return int(Path("/proc/self/statm").read_text().split()[1]) * os.sysconf(
        "SC_PAGE_SIZE"
    )


def dispatch(kind):
    events = []
    world = dart.simulation.World()
    node = recording_node(kind, world, events)
    assert node.extra == "custom constructor argument"
    assert node.getWorld() is world
    probe.refresh(node)
    assert events == ["refresh", "pre_refresh", "post_refresh"], events
    events.clear()
    node.simulate(True)
    if kind == "WorldNode":
        node.setNumStepsPerCycle(3)
        assert node.getNumStepsPerCycle() == 3
        probe.refresh(node)
        assert events == [
            "refresh",
            "pre_refresh",
            "pre_step",
            "post_step",
            "pre_step",
            "post_step",
            "pre_step",
            "post_step",
            "post_refresh",
        ], events
        steps = 3
    else:
        # RealTimeWorldNode budgets steps by wall time, so compare callbacks
        # with the actual simulation advance rather than a guessed step count.
        node.setTargetFrequency(100.0)
        node.setTargetRealTimeFactor(1.0)
        assert math.isclose(node.getTargetFrequency(), 100.0)
        assert math.isclose(node.getTargetRealTimeFactor(), 1.0)
        for _ in range(6):
            time.sleep(0.005)
            probe.refresh(node)
        assert events.count("refresh") == 6, events
        assert events.count("pre_refresh") == 6, events
        assert events.count("post_refresh") == 6, events
        steps = events.count("pre_step")
        assert steps > 0, events
        assert events.count("post_step") == steps, events
        for index, event in enumerate(events):
            if event == "pre_step":
                assert events[index + 1] == "post_step", events
    calibration = dart.simulation.World()
    calibration.step()
    assert math.isclose(
        world.getTime(), steps * calibration.getTime(), rel_tol=1e-8, abs_tol=1e-12
    ), (world.getTime(), steps)
    return {"steps": steps, "callback_count": len(events)}


def lifetime(case, kind):
    world = dart.simulation.World()
    events = []
    node = recording_node(kind, world, events)
    viewer = osg.Viewer()
    viewer.addWorldNode(node)
    expected = ["refresh", "pre_refresh", "post_refresh"]

    if case == "drop_node":
        reference = weakref.ref(node)
        del node
        gc.collect()
        probe.refresh_viewer(viewer)
        print(json.dumps({"node_alive": reference() is not None, "events": events}))
        assert events == expected, events
        del viewer
        gc.collect()
        assert reference() is None
    elif case == "delete_viewer":
        del viewer
        gc.collect()
        probe.refresh(node)
        assert events == expected, events
        assert node.getWorld() is world
    elif case == "remove_readd":
        for _ in range(20):
            viewer.removeWorldNode(node)
            viewer.addWorldNode(node)
        viewer.removeWorldNode(node)
        gc.collect()
        before_rss = rss_bytes()
        before_refs = sys.getrefcount(node)
        for _ in range(1000):
            viewer.addWorldNode(node)
            viewer.removeWorldNode(node)
        gc.collect()
        growth = None if before_rss is None else rss_bytes() - before_rss
        assert sys.getrefcount(node) == before_refs
        assert growth is None or growth < 8 * 1024 * 1024, growth
        reference = weakref.ref(node)
        del node, viewer
        gc.collect()
        assert reference() is None, "destroyed viewer retained the Python node"
        return {"rss_growth_bytes": growth, "iterations": 1000}
    elif case == "distinct_node_cycles":
        viewer.removeWorldNode(node)
        del node
        for _ in range(20):
            node = recording_node(kind, world, events)
            viewer.addWorldNode(node)
            viewer.removeWorldNode(node)
            del node
        gc.collect()
        before_rss = rss_bytes()
        references = []
        for _ in range(1000):
            node = recording_node(kind, world, events)
            references.append(weakref.ref(node))
            viewer.addWorldNode(node)
            viewer.removeWorldNode(node)
            del node
        gc.collect()
        growth = None if before_rss is None else rss_bytes() - before_rss
        retained = sum(ref() is not None for ref in references)
        # A camera can retain its latest/pending node after scene removal.
        assert retained <= 2, retained
        del viewer
        gc.collect()
        assert all(ref() is None for ref in references)
        return {"rss_growth_bytes": growth, "iterations": 1000, "retained": retained}
    elif case == "remove_drop":
        viewer.removeWorldNode(node)
        reference = weakref.ref(node)
        del node
        gc.collect()
        del viewer
        gc.collect()
        assert reference() is None
    elif case == "two_viewers":
        other = osg.Viewer()
        other.addWorldNode(node)
        reference = weakref.ref(node)
        del node
        gc.collect()
        probe.refresh_viewer(viewer)
        del viewer
        gc.collect()
        probe.refresh_viewer(other)
        assert events == expected * 2, events
        del other
        gc.collect()
        assert reference() is None
    elif case == "two_viewers_remove":
        viewer.removeWorldNode(node)
        del node
        other = osg.Viewer()
        nodes = [recording_node(kind, world, events) for _ in range(10)]
        references = [weakref.ref(node) for node in nodes]
        for node in nodes:
            viewer.addWorldNode(node)
            other.addWorldNode(node)
        for node in nodes:
            viewer.removeWorldNode(node)
        del node, nodes
        gc.collect()
        del other
        gc.collect()
        retained = sum(ref() is not None for ref in references)
        print(json.dumps({"retained_after_other_viewer_destruction": retained}))
        assert retained <= 1, retained
        del viewer
        gc.collect()
        assert all(ref() is None for ref in references)
    elif case == "node_self_remove":
        viewer.removeWorldNode(node)
        del node
        viewer_reference = weakref.ref(viewer)
        base = getattr(osg, kind)

        class SelfRemovingNode(base):
            def __init__(self, world, extra):
                super().__init__(world)
                self.extra = extra

            def customPreRefresh(self):
                events.append("pre_refresh")
                current_viewer = viewer_reference()
                current_viewer.removeWorldNode(self)
                current_viewer.addWorldNode(base(world))

            def customPostRefresh(self):
                events.append("post_refresh")

        node = SelfRemovingNode(world, "self removal")
        reference = weakref.ref(node)
        viewer.addWorldNode(node)
        del node
        gc.collect()
        probe.refresh_viewer(viewer)
        assert events == ["pre_refresh", "post_refresh"], events
        del viewer
        gc.collect()
        assert reference() is None
    elif case == "exit_live":
        handler = recording_handler([])
        viewer.addEventHandler(handler)
        EXIT_ROOTS.append((world, viewer, node, handler))
        probe.refresh_viewer(viewer)
        assert events == expected, events
    elif case == "direct_base":
        viewer.removeWorldNode(node)
        del node
        node = getattr(osg, kind)(world)
        viewer.addWorldNode(node)
        del node
        gc.collect()
        probe.refresh_viewer(viewer)
        del viewer
        gc.collect()
        node = getattr(osg, kind)(world)
        viewer = osg.Viewer()
        viewer.addWorldNode(node)
        del viewer
        gc.collect()
        probe.refresh(node)
    elif case == "node_viewer_cycle":
        node.viewer = viewer
        references = [weakref.ref(node), weakref.ref(viewer)]
        del node, viewer
        gc.collect()
        assert all(ref() is None for ref in references), "node/viewer cycle leaked"
    else:
        raise AssertionError(case)
    return {"case": case, "node_type": kind}


def handler_lifetime(case):
    events = []
    if case == "handler_self_remove":

        class SelfRemovingHandler(osg.GUIEventHandler):
            def handle(self, ea, aa):
                events.append((ea.getKey(), int(ea.getEventType())))
                probe.remove_handler(aa, self)
                return True

        handler = SelfRemovingHandler()
    else:
        handler = recording_handler(events)
    viewer = osg.Viewer()
    viewer.addEventHandler(handler)
    expected = [(int(osg.GUIEventAdapter.KEY_A), int(osg.GUIEventAdapter.KEYDOWN))]
    if case == "handler_drop":
        reference = weakref.ref(handler)
        del handler
        gc.collect()
        probe.dispatch_viewer_handlers(viewer, key=int(osg.GUIEventAdapter.KEY_A))
        print(json.dumps({"handler_alive": reference() is not None, "events": events}))
        assert events == expected, events
        del viewer
        gc.collect()
        assert reference() is None
    elif case == "handler_delete_viewer":
        del viewer
        gc.collect()
        viewer = osg.Viewer()
        assert probe.handle(handler, viewer, key=int(osg.GUIEventAdapter.KEY_A))
        assert events == expected, events
    elif case == "handler_viewer_cycle":
        handler.viewer = viewer
        references = [weakref.ref(handler), weakref.ref(viewer)]
        del handler, viewer
        gc.collect()
        assert all(ref() is None for ref in references), "handler/viewer cycle leaked"
    elif case == "handler_self_remove":
        reference = weakref.ref(handler)
        del handler
        gc.collect()
        probe.dispatch_viewer_handlers(viewer, key=int(osg.GUIEventAdapter.KEY_A))
        assert events == expected, events
        gc.collect()
        probe.dispatch_viewer_handlers(viewer, key=int(osg.GUIEventAdapter.KEY_A))
        assert events == expected, events
        del viewer
        gc.collect()
        assert reference() is None
    else:
        raise AssertionError(case)
    return {"case": case}


if __name__ == "__main__":
    if os.name != "nt":
        import resource

        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    os.environ["DISPLAY"] = ""
    case = sys.argv[1]
    if case.startswith("handler_"):
        result = handler_lifetime(case)
    elif case == "dispatch":
        result = dispatch(sys.argv[2])
    else:
        result = lifetime(case, sys.argv[2])
    print(json.dumps(result), flush=True)
