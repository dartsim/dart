"""Native owners retain Python objects until interpreter finalization.

Existing pinning tests are shutdown smoke tests: native owners are released
during module teardown on the main thread.
"""

import importlib.machinery
import sys

import dartpy as dart
import pytest

from ._support import run_isolated


def _run_at_shutdown(script):
    run_isolated(
        script
        + """
        import weakref
        ref = weakref.ref(child)
        # run_isolated drops function locals before exit; retain the native root.
        globals()["shutdown_owner"] = owner
        del child, owner
        gc.collect()
        assert ref() is not None, "native ownership did not pin the Python object"
        """
    )


@pytest.mark.parametrize("owner_kind", ["skeleton", "world"])
def test_python_shape_pinned_until_shutdown(owner_kind):
    _run_at_shutdown(
        f"""
        class Shape(dart.dynamics.BoxShape):
            pass
        child = Shape([1, 2, 3])
        skeleton = dart.dynamics.Skeleton()
        skeleton.createFreeJointAndBodyNodePair()[1].createShapeNode(child)
        if {owner_kind!r} == "world":
            owner = dart.simulation.World()
            owner.addSkeleton(skeleton)
        else:
            owner = skeleton
        del skeleton
        """
    )


@pytest.mark.parametrize("assignment", ["constructor", "property"])
def test_python_collision_filter_pinned_until_shutdown(assignment):
    _run_at_shutdown(
        f"""
        class Filter(dart.collision.BodyNodeCollisionFilter):
            def ignoresCollision(self, object1, object2):
                return False
        child = Filter()
        if {assignment!r} == "constructor":
            owner = dart.collision.CollisionOption(True, 1000, child)
        else:
            owner = dart.collision.CollisionOption()
            owner.collisionFilter = child
        """
    )


def test_python_analytical_ik_callable_pinned_until_shutdown():
    _run_at_shutdown(
        """
        # Separate globals prevent the callable from retaining shutdown_owner.
        child = eval("lambda target: []", {})
        skeleton = dart.dynamics.Skeleton()
        joint, body = skeleton.createFreeJointAndBodyNodePair()
        ik = body.getOrCreateIK()
        ik.setDofs(list(range(skeleton.getNumDofs())))
        ik.setPythonAnalytical(child)
        owner = skeleton
        del skeleton, joint, body, ik
        """
    )


def test_python_optimizer_function_pinned_until_shutdown():
    _run_at_shutdown(
        """
        class Function(dart.optimizer.Function):
            def eval(self, x):
                return float(x[0] ** 2)
        child = Function()
        owner = dart.optimizer.Problem(1)
        owner.setObjective(child)
        """
    )


@pytest.mark.skipif(
    sys.version_info < (3, 15)
    # Limited-API builds (".abi3.so", or a plain ".pyd" on Windows) cannot see
    # refused attachment; version-specific builds use the first suffix.
    or not dart._dartpy.__file__.endswith(importlib.machinery.EXTENSION_SUFFIXES[0]),
    reason="requires a Python 3.15+ build without the limited API",
)
def test_pinned_release_without_thread_state_during_finalization():
    result = run_isolated(
        """
        class Retriever(dart.common.LocalResourceRetriever):
            pass
        shape = dart.dynamics.MeshShape([1.0, 1.0, 1.0], None)
        shape.setMesh(None, dart.common.Uri(), Retriever())
        class ReleaseOnExit:
            def __del__(self):
                shape.setMesh(None, dart.common.Uri(), None)
        # Retain the finalizer after run_isolated's main() returns.
        globals()["keeper"] = ReleaseOnExit()
        """,
        allow_leaks=True,
    )
    assert any(
        "leaked instance " in line and 'Retriever"' in line
        for line in result.stderr.splitlines()
    ), result.stderr[-4000:]
