"""Strict collection and value-input regressions shared with the pybind11 oracle."""

import gc
import weakref
from pathlib import Path

import dartpy as dart
import pytest

from ._support import IS_NANOBIND, run_isolated


def test_all_eight_native_ownership_cycles():
    tools = Path(__file__).resolve().parents[4] / "scripts/nanobind"
    run_isolated(
        f"""
        import sys
        sys.path.insert(0, {str(tools)!r})
        from probe_gc import probe
        assert len(probe()) == 8
    """
    )


@pytest.mark.skipif(
    not IS_NANOBIND,
    reason="native-owner tp_clear slots are implemented by the nanobind binder",
)
def test_native_owner_clear_slots_are_order_independent_and_idempotent():
    tools = Path(__file__).resolve().parents[4] / "scripts/nanobind"
    run_isolated(
        f"""
        import runpy, sys
        sys.path.insert(0, {str(tools)!r})
        runpy.run_path({str(tools / 'probe_gc_clear.py')!r}, run_name="__main__")
        """
    )


def test_secondary_value_base_inputs_preserve_fields():
    base = dart.optimizer.SolverProperties()
    derived = dart.optimizer.GradientDescentSolverProperties()
    derived.mStepSize = 0.375
    copied = dart.optimizer.GradientDescentSolverProperties(base, derived)
    assert copied.mStepSize == 0.375
    error = dart.dynamics.InverseKinematicsErrorMethodProperties()
    region = dart.dynamics.InverseKinematicsTaskSpaceRegionProperties()
    region.mComputeErrorFromCenter = False
    copied = dart.dynamics.InverseKinematicsTaskSpaceRegionProperties(error, region)
    assert copied.mComputeErrorFromCenter is False


def test_live_cpp_alias_keeps_python_problem_usable():
    class Problem(dart.optimizer.Problem):
        pass

    child = Problem(1)
    properties = dart.optimizer.GradientDescentSolverProperties()
    properties.mProblem = child
    solver = dart.optimizer.GradientDescentSolver(properties)
    child.owner = solver
    ref = weakref.ref(child)
    del child, solver
    gc.collect()
    assert properties.mProblem.getDimension() == 1
    if ref() is not None:
        ref().owner = None
    properties.mProblem = None
    gc.collect()
    assert ref() is None


def test_multiple_problem_function_fields_collect():
    tools = Path(__file__).resolve().parents[4] / "scripts/nanobind"
    run_isolated(
        f"""
        import sys, weakref
        sys.path.insert(0, {str(tools)!r})
        from probe_gc import case
        for _ in range(20):
            child, owner = case('Function->Problem')
            owner.addEqConstraint(child)
            owner.addIneqConstraint(child)
            refs = weakref.ref(child), weakref.ref(owner)
            del child, owner
            gc.collect()
            assert all(ref() is None for ref in refs)
    """
    )


def test_gc_during_unready_owner_construction():
    run_isolated(
        """
        class Problem(dart.optimizer.Problem):
            def __init__(self):
                gc.collect()
                super().__init__(1)
        for _ in range(20):
            assert Problem().getDimension() == 1
    """
    )


def test_shared_skeleton_in_live_world_preserves_python_shape():
    class Shape(dart.dynamics.BoxShape):
        pass

    live = dart.simulation.World()
    owner = dart.simulation.World()
    skeleton = dart.dynamics.Skeleton()
    shape = Shape([1, 2, 3])
    shape.owner = owner
    skeleton.createFreeJointAndBodyNodePair()[1].createShapeNode(shape)
    live.addSkeleton(skeleton)
    owner.addSkeleton(skeleton)
    ref = weakref.ref(shape)
    del shape, skeleton, owner
    gc.collect()
    fetched = live.getSkeleton(0).getBodyNode(0).getShapeNode(0).getShape()
    assert fetched.getVolume() == 6
    if ref() is not None:
        assert fetched is ref()
        ref().owner = None
    del fetched, live
    gc.collect()
    assert ref() is None


def test_multiple_cpp_owners_preserve_a_shared_python_pin_until_cleanup():
    """Nanobind conservatively retains this cycle instead of clearing live owners."""
    run_isolated(
        f"""
        import weakref
        class Problem(dart.optimizer.Problem):
            pass
        for _ in range(20):
            child = Problem(1)
            properties = dart.optimizer.GradientDescentSolverProperties()
            properties.mProblem = child
            solver = dart.optimizer.GradientDescentSolver(properties)
            child.owner = (solver, properties)
            ref = weakref.ref(child)
            del child, solver, properties
            gc.collect()
            assert (ref() is not None) is {IS_NANOBIND!r}
            if ref() is not None:
                assert ref().getDimension() == 1
                ref().owner = None
            gc.collect()
            assert ref() is None
        """
    )
