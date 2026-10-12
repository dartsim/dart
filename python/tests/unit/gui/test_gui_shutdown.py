"""Shutdown regressions for the interim dartpy.gui bindings."""

import importlib.machinery
import sys

import dartpy as dart
import pytest

from ..bindings._support import run_isolated


def test_python_gui_handler_pinned_until_shutdown():
    run_isolated(
        """
        import weakref
        class Handler(dart.gui.GUIEventHandler):
            def handle(self, event, action):
                return True
        owner = dart.gui.Viewer()
        child = Handler()
        owner.addEventHandler(child)
        ref = weakref.ref(child)
        # run_isolated drops function locals before exit; retain the native root.
        globals()["shutdown_owner"] = owner
        del child, owner
        gc.collect()
        assert ref() is not None, "native ownership did not pin the Python handler"
        event = dart.gui.GUIEventAdapter(dart.gui.GUIEventAdapter.KEYDOWN, ord("a"))
        assert globals()["shutdown_owner"].handleEvent(event)
        """
    )


def test_gui_ownership_release_without_thread_state():
    run_isolated(
        """
        import weakref
        class Handler(dart.gui.GUIEventHandler):
            pass
        owner = dart.gui.Viewer()
        child = Handler()
        owner.addEventHandler(child)
        ref = weakref.ref(child)
        del child
        gc.collect()
        assert ref() is not None
        dart._probe.release_gui_ownership_without_thread_state(owner)
        gc.collect()
        assert ref() is None
        """
    )


@pytest.mark.skipif(
    sys.version_info < (3, 15)
    # Limited-API builds (".abi3.so", or a plain ".pyd" on Windows) cannot see
    # refused attachment; version-specific builds use the first suffix.
    or not dart.__file__.endswith(importlib.machinery.EXTENSION_SUFFIXES[0]),
    reason="requires a Python 3.15+ build without the limited API",
)
def test_gui_ownership_release_without_thread_state_during_finalization():
    result = run_isolated(
        """
        class Handler(dart.gui.GUIEventHandler):
            pass
        owner = dart.gui.Viewer()
        owner.addEventHandler(Handler())
        class ReleaseOnExit:
            def __del__(self):
                dart._probe.release_gui_ownership_without_thread_state(owner)
                print("released")
        # Retain the finalizer after run_isolated's main() returns.
        globals()["keeper"] = ReleaseOnExit()
        """,
        allow_leaks=True,
    )
    assert "released" in result.stdout
    assert any(
        "leaked instance " in line and 'Handler"' in line
        for line in result.stderr.splitlines()
    ), result.stderr[-4000:]
