"""Strict collection and value-input regressions shared with the pybind11 oracle."""

import gc
import weakref
from pathlib import Path

import dartpy as dart
import pytest

from ._support import IS_NANOBIND, run_isolated


def _task_space_owner(kind, frame):
    if kind in {"unique_properties", "properties"}:
        name = (
            "InverseKinematicsTaskSpaceRegionUniqueProperties"
            if kind == "unique_properties"
            else "InverseKinematicsTaskSpaceRegionProperties"
        )
        owner = getattr(dart.dynamics, name)()
        owner.mReferenceFrame = frame
        return owner, ()
    skeleton = dart.dynamics.Skeleton()
    joint, body = skeleton.createFreeJointAndBodyNodePair()
    ik = body.getOrCreateIK()
    if kind == "ik":
        owner = ik
        owner.getErrorMethod().setReferenceFrame(frame)
        return owner, (skeleton, joint, body)
    owner = dart.dynamics.InverseKinematicsTaskSpaceRegion(ik)
    owner.setReferenceFrame(frame)
    return owner, (skeleton, joint, body, ik)


def _reference_frame(owner, kind):
    if kind == "ik":
        return owner.getErrorMethod().getReferenceFrame()
    if kind == "region":
        return owner.getReferenceFrame()
    return owner.mReferenceFrame


def _parser_owner(kind, retriever):
    classes = {
        "dart_options": dart.utils.DartLoaderOptions,
        "sdf_options": dart.utils.SdfParser.Options,
        "mjcf_options": dart.utils.MjcfParser.Options,
        "loader": dart.utils.DartLoaderOptions,
    }
    options = classes[kind]()
    setattr(
        options,
        "mRetriever" if kind == "mjcf_options" else "mResourceRetriever",
        retriever,
    )
    if kind != "loader":
        return options
    loader = dart.utils.DartLoader()
    loader.setOptions(options)
    # Only the loader's native copy remains after this helper returns.
    return loader


def _retriever(owner, kind):
    if kind == "loader":
        return owner.getOptions().mResourceRetriever
    return getattr(
        owner, "mRetriever" if kind == "mjcf_options" else "mResourceRetriever"
    )


@pytest.mark.parametrize("kind", ["ik", "region", "unique_properties", "properties"])
def test_task_space_reference_frame_cycle_collects(kind):
    run_isolated(
        f"""
        import weakref
        from tests.unit.bindings.test_gc import _task_space_owner, _reference_frame
        class Frame(dart.dynamics.SimpleFrame):
            pass
        for _ in range(10):
            frame = Frame()
            frame.setName('cycle_frame')
            owner, native_aliases = _task_space_owner({kind!r}, frame)
            frame.owner = owner
            refs = weakref.ref(frame), weakref.ref(owner)
            del frame
            gc.collect()
            # Nanobind pins Python state; pybind11 retains only the native frame.
            assert (refs[0]() is not None) is {IS_NANOBIND!r}
            assert _reference_frame(owner, {kind!r}).getName() == 'cycle_frame'
            del owner, native_aliases
            gc.collect()
            collected = all(ref() is None for ref in refs)
            # Break the Python backedge even on a failed regression, so shutdown
            # does not leave a second leak or hide the collection assertion.
            if refs[0]() is not None:
                refs[0]().owner = None
            gc.collect()
            assert all(ref() is None for ref in refs)
            assert collected, 'task-space reference-frame cycle was retained'
        """
    )


def test_direct_ik_factory_cycle_requires_explicit_cleanup():
    """The factory's affiliation holder prevents proving exclusive ownership."""
    run_isolated(
        f"""
        import weakref
        class Frame(dart.dynamics.SimpleFrame):
            pass
        skeleton = dart.dynamics.Skeleton()
        joint, body = skeleton.createFreeJointAndBodyNodePair()
        frame = Frame()
        frame.setName('factory_cycle_frame')
        owner = dart.dynamics.InverseKinematics(body)
        owner.getErrorMethod().setReferenceFrame(frame)
        frame.owner = owner
        refs = weakref.ref(frame), weakref.ref(owner)
        del frame, owner, skeleton, joint, body
        gc.collect()
        # Nanobind keeps both native factory ownership and node affiliation.
        # Their shared references cannot prove that clearing native edges is safe.
        assert tuple(ref() is not None for ref in refs) == ({IS_NANOBIND!r}, {IS_NANOBIND!r})
        if {IS_NANOBIND!r}:
            assert refs[1]().getErrorMethod().getReferenceFrame() is refs[0]()
            assert refs[0]().getName() == 'factory_cycle_frame'
            refs[0]().owner = None
        gc.collect()
        assert all(ref() is None for ref in refs)
        """
    )


@pytest.mark.parametrize("kind", ["ik", "region", "unique_properties", "properties"])
def test_task_space_live_native_alias_preserves_reference_frame(kind):
    run_isolated(
        f"""
        import weakref
        from tests.unit.bindings.test_gc import _task_space_owner
        class Frame(dart.dynamics.SimpleFrame):
            pass
        frame = Frame()
        frame.setName('retained_frame')
        owner, native_aliases = _task_space_owner({kind!r}, frame)
        frame.owner = owner
        if {kind!r} in ('ik', 'region'):
            method = owner.getErrorMethod() if {kind!r} == 'ik' else owner
            live = method.getTaskSpaceRegionProperties()
            del method
        else:
            live = type(owner)()
            live.mReferenceFrame = frame
        refs = weakref.ref(frame), weakref.ref(owner)
        del frame, owner, native_aliases
        gc.collect()
        assert tuple(ref() is not None for ref in refs) == ({IS_NANOBIND!r}, {IS_NANOBIND!r})
        fetched = live.mReferenceFrame
        assert fetched.getName() == 'retained_frame'
        if {IS_NANOBIND!r}:
            assert fetched is refs[0]()
            assert fetched.owner is refs[1]()
            # A live external native alias must keep the shared Python pin.
            fetched.owner = None
        del fetched, live
        gc.collect()
        assert all(ref() is None for ref in refs)
        """
    )


@pytest.mark.parametrize(
    "kind", ["dart_options", "sdf_options", "mjcf_options", "loader"]
)
def test_parser_retriever_cycle_collects(kind):
    run_isolated(
        f"""
        import weakref
        from tests.unit.bindings.test_gc import _parser_owner, _retriever
        class Retriever(dart.utils.DartResourceRetriever):
            pass
        for _ in range(10):
            child = Retriever()
            owner = _parser_owner({kind!r}, child)
            child.owner = owner
            refs = weakref.ref(child), weakref.ref(owner)
            del child
            gc.collect()
            # Pybind11 preserves the C++ retriever without pinning its subclass.
            assert (refs[0]() is not None) is {IS_NANOBIND!r}
            assert _retriever(owner, {kind!r}).exists(
                dart.common.Uri('unsupported://gc-regression')) is False
            del owner
            gc.collect()
            collected = all(ref() is None for ref in refs)
            if refs[0]() is not None:
                refs[0]().owner = None
            gc.collect()
            assert all(ref() is None for ref in refs)
            assert collected, 'parser retriever cycle was retained'
        """
    )


@pytest.mark.parametrize(
    "kind", ["dart_options", "sdf_options", "mjcf_options", "loader"]
)
def test_parser_live_native_alias_preserves_retriever(kind):
    run_isolated(
        f"""
        import weakref
        from tests.unit.bindings.test_gc import _parser_owner, _retriever
        class Retriever(dart.utils.DartResourceRetriever):
            pass
        child = Retriever()
        owner = _parser_owner({kind!r}, child)
        live = _parser_owner({kind!r}, child)
        child.owner = owner
        refs = weakref.ref(child), weakref.ref(owner)
        del child, owner
        gc.collect()
        assert tuple(ref() is not None for ref in refs) == ({IS_NANOBIND!r}, {IS_NANOBIND!r})
        fetched = _retriever(live, {kind!r})
        assert fetched.exists(dart.common.Uri('unsupported://gc-regression')) is False
        if {IS_NANOBIND!r}:
            assert fetched is refs[0]()
            assert fetched.owner is refs[1]()
            fetched.owner = None
        del fetched, live
        gc.collect()
        assert all(ref() is None for ref in refs)
        """
    )


def test_private_package_retriever_cycle_requires_explicit_cleanup():
    """The private local-retriever holder has no public traverse/clear API."""
    run_isolated(
        f"""
        import weakref
        class Retriever(dart.utils.DartResourceRetriever):
            pass
        child = Retriever()
        owner = dart.utils.PackageResourceRetriever(child)
        child.owner = owner
        refs = weakref.ref(child), weakref.ref(owner)
        del child, owner
        gc.collect()
        assert tuple(ref() is not None for ref in refs) == ({IS_NANOBIND!r}, {IS_NANOBIND!r})
        if refs[0]() is not None:
            # Nanobind conservatively retains the private native owner.
            refs[0]().owner = None
        gc.collect()
        assert all(ref() is None for ref in refs)
        """
    )


def test_mesh_retriever_cycle_collects():
    run_isolated(
        f"""
        import weakref
        class Retriever(dart.common.LocalResourceRetriever):
            pass
        child = Retriever()
        if not {IS_NANOBIND!r}:
            # Pybind11's unbound legacy aiScene argument rejects None.
            try:
                dart.dynamics.MeshShape([1, 1, 1], None, dart.common.Uri(), child)
            except TypeError:
                return
            raise AssertionError('legacy aiScene constructor unexpectedly accepted None')
        owner = dart.dynamics.MeshShape([1, 1, 1], None, dart.common.Uri(), child)
        child.owner = owner
        refs = weakref.ref(child), weakref.ref(owner)
        del child, owner
        gc.collect()
        collected = all(ref() is None for ref in refs)
        if refs[0]() is not None:
            refs[0]().owner = None
        gc.collect()
        assert all(ref() is None for ref in refs)
        assert collected, 'mesh retriever cycle was retained'
        """
    )


def test_mesh_live_native_shape_alias_preserves_retriever():
    run_isolated(
        f"""
        import weakref
        class Retriever(dart.common.LocalResourceRetriever):
            pass
        child = Retriever()
        if not {IS_NANOBIND!r}:
            try:
                dart.dynamics.MeshShape([1, 1, 1], None, dart.common.Uri(), child)
            except TypeError:
                return
            raise AssertionError('legacy aiScene constructor unexpectedly accepted None')
        owner = dart.dynamics.MeshShape([1, 1, 1], None, dart.common.Uri(), child)
        child.owner = owner
        live = dart.dynamics.SimpleFrame()
        live.setShape(owner)
        refs = weakref.ref(child), weakref.ref(owner)
        del child, owner
        gc.collect()
        assert all(ref() is not None for ref in refs)
        fetched = live.getShape()
        assert fetched is refs[1]()
        assert fetched.getResourceRetriever() is refs[0]()
        np.testing.assert_array_equal(fetched.getScale(), [1, 1, 1])
        del fetched, live
        gc.collect()
        collected = all(ref() is None for ref in refs)
        if refs[0]() is not None:
            refs[0]().owner = None
        gc.collect()
        assert all(ref() is None for ref in refs)
        assert collected, 'mesh cycle remained after releasing its native shape alias'
        """
    )


@pytest.mark.parametrize("native_alias", [False, True])
def test_contact_inverse_dynamics_native_skeleton_shape_cycle(native_alias):
    run_isolated(
        f"""
        import weakref
        class Shape(dart.dynamics.BoxShape):
            pass
        skeleton = dart.dynamics.Skeleton()
        skeleton.createFreeJointAndBodyNodePair()[1].createShapeNode(
            shape := Shape([1, 2, 3]))
        owner = dart.dynamics.ContactInverseDynamics(skeleton)
        shape.owner = owner
        refs = weakref.ref(shape), weakref.ref(owner)
        if {native_alias!r}:
            live = dart.simulation.World()
            live.addSkeleton(skeleton)
        del skeleton, shape, owner
        gc.collect()
        if {native_alias!r}:
            assert tuple(ref() is not None for ref in refs) == ({IS_NANOBIND!r}, {IS_NANOBIND!r})
            fetched = live.getSkeleton(0).getBodyNode(0).getShapeNode(0).getShape()
            assert fetched.getVolume() == 6
            if {IS_NANOBIND!r}:
                assert fetched is refs[0]()
                fetched.owner = None
            del fetched, live
            gc.collect()
        collected = all(ref() is None for ref in refs)
        if refs[0]() is not None:
            refs[0]().owner = None
        gc.collect()
        assert all(ref() is None for ref in refs)
        assert collected, 'ContactInverseDynamics native-skeleton shape cycle was retained'
        """
    )


def _boxed_owner(kind, child):
    owner = dart.constraint.BoxedLcpConstraintSolver()
    if kind in {"primary", "both"}:
        owner.setBoxedLcpSolver(child)
    if kind in {"secondary", "both"}:
        owner.setSecondaryBoxedLcpSolver(child)
    return owner


@pytest.mark.parametrize("kind", ["primary", "secondary", "both"])
@pytest.mark.parametrize("native_alias", [False, True])
def test_boxed_lcp_python_solver_cycle_and_native_alias(kind, native_alias):
    run_isolated(
        f"""
        import weakref
        from tests.unit.bindings.test_gc import _boxed_owner
        class Solver(dart.constraint.NsgsFrictionSolver):
            pass
        child = Solver()
        owner = _boxed_owner({kind!r}, child)
        child.owner = owner
        refs = weakref.ref(child), weakref.ref(owner)
        if {native_alias!r}:
            live = _boxed_owner({kind!r}, child)
        del child, owner
        gc.collect()
        if {native_alias!r}:
            assert tuple(ref() is not None for ref in refs) == ({IS_NANOBIND!r}, {IS_NANOBIND!r})
            fetched = (live.getBoxedLcpSolver() if {kind!r} != 'secondary'
                       else live.getSecondaryBoxedLcpSolver())
            assert fetched.getStats().numSolves == 0
            if {IS_NANOBIND!r}:
                assert fetched is refs[0]()
                fetched.owner = None
            del fetched, live
            gc.collect()
        collected = all(ref() is None for ref in refs)
        if refs[0]() is not None:
            refs[0]().owner = None
        gc.collect()
        assert all(ref() is None for ref in refs)
        assert collected, 'BoxedLcpConstraintSolver Python solver cycle was retained'
        """
    )


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


def test_body_owned_ik_solver_cycle_requires_explicit_cleanup():
    """The retained node graph owns the IK in addition to its Python wrapper."""
    tools = Path(__file__).resolve().parents[4] / "scripts/nanobind"
    run_isolated(
        f"""
        import sys, weakref
        sys.path.insert(0, {str(tools)!r})
        from probe_cycles import make_case
        for _ in range(20):
            child, owner = make_case('Solver->InverseKinematics')
            refs = weakref.ref(child), weakref.ref(owner)
            del child, owner
            gc.collect()
            assert tuple(ref() is not None for ref in refs) == ({IS_NANOBIND!r}, {IS_NANOBIND!r})
            if {IS_NANOBIND!r}:
                assert refs[1]().getSolver() is refs[0]()
                assert refs[0]().owner is refs[1]()
                assert refs[0]().skeleton.getBodyNode(0).getIK(False) is refs[1]()
                # Release the native graph's extra IK owner, then disconnect
                # the Python backedge before destroying the affiliated node.
                refs[0]().skeleton.getBodyNode(0).clearIK()
                refs[0]().owner = None
            gc.collect()
            assert all(ref() is None for ref in refs)
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


@pytest.mark.parametrize(
    "owner_kind", ["skeleton", "body", "shape_frame", "simple_frame"]
)
def test_live_world_preserves_shape_with_direct_graph_wrapper_backref(owner_kind):
    run_isolated(
        f"""
        import weakref
        class Shape(dart.dynamics.BoxShape):
            def getVolume(self):
                return super().getVolume() + self.offset
        shape = Shape([1, 2, 3])
        shape.offset = 17
        shape.label = 'retained_python_state'
        live = dart.simulation.World()
        if {owner_kind!r} == 'simple_frame':
            frame = dart.dynamics.SimpleFrame()
            frame.setShape(shape)
            shape.owner = frame
            owner_ref = weakref.ref(frame)
            live.addSimpleFrame(frame)
            del frame
        else:
            skeleton = dart.dynamics.Skeleton()
            joint, body = skeleton.createFreeJointAndBodyNodePair()
            frame = body.createShapeNode(shape)
            owner = {{'skeleton': skeleton, 'body': body, 'shape_frame': frame}}[{owner_kind!r}]
            shape.owner = owner
            owner_ref = weakref.ref(owner)
            live.addSkeleton(skeleton)
            del owner, frame, joint, body, skeleton
        shape_ref = weakref.ref(shape)
        del shape
        # The World owns native graph objects without keeping their Python
        # wrappers as external roots. GC must preserve that live native state.
        for _ in range(3):
            gc.collect()
        if {owner_kind!r} == 'simple_frame':
            fetched = live.getSimpleFrame(0).getShape()
        else:
            fetched = live.getSkeleton(0).getBodyNode(0).getShapeNode(0).getShape()
        assert fetched is not None, 'GC cleared a shape still owned by a live World'
        assert dart.dynamics.Shape.getVolume(fetched) == 6
        assert tuple(ref() is not None for ref in (shape_ref, owner_ref)) == ({IS_NANOBIND!r}, {IS_NANOBIND!r})
        if {IS_NANOBIND!r}:
            assert fetched is shape_ref()
            assert fetched.owner is owner_ref()
            assert fetched.label == 'retained_python_state'
            assert fetched.getVolume() == 23
            # Shared native graph roots may retain the Python backedge
            # conservatively. Disconnect it before destroying the live graph.
            fetched.owner = None
        else:
            assert fetched.getVolume() == 6
        del fetched, live
        gc.collect()
        assert shape_ref() is None and owner_ref() is None
        """
    )


def test_live_world_preserves_constraint_with_borrowed_solver_backref():
    run_isolated(
        f"""
        import weakref
        class Constraint(dart.constraint.BallJointConstraint):
            pass
        live = dart.simulation.World()
        skeleton = dart.dynamics.Skeleton()
        _, first = skeleton.createFreeJointAndBodyNodePair()
        _, second = skeleton.createFreeJointAndBodyNodePair()
        child = Constraint(first, second, [0, 0, 0])
        child.skeleton = skeleton
        child.label = 'retained_constraint_state'
        owner = live.getConstraintSolver()
        owner.addConstraint(child)
        child.owner = owner
        live.addSkeleton(skeleton)
        refs = weakref.ref(child), weakref.ref(owner)
        del child, owner, first, second, _, skeleton
        for _ in range(3):
            gc.collect()
        assert live.getConstraintSolver().getNumConstraints() == 1
        fetched = live.getConstraintSolver().getConstraint(0)
        assert fetched.getDimension() == 3
        assert tuple(ref() is not None for ref in refs) == ({IS_NANOBIND!r}, {IS_NANOBIND!r})
        if {IS_NANOBIND!r}:
            assert fetched is refs[0]()
            assert fetched.owner is refs[1]()
            assert fetched.label == 'retained_constraint_state'
            fetched.owner = None
        del fetched, live
        gc.collect()
        assert all(ref() is None for ref in refs)
        """
    )


def test_live_solver_preserves_objective_with_native_problem_backref():
    run_isolated(
        f"""
        import weakref
        class Function(dart.optimizer.Function):
            def eval(self, x):
                return float(x[0] ** 2 + self.offset)
        owner = dart.optimizer.Problem(1)
        child = Function()
        child.offset = 5
        child.owner = owner
        owner.setObjective(child)
        live = dart.optimizer.GradientDescentSolver(owner)
        refs = weakref.ref(child), weakref.ref(owner)
        del child, owner
        for _ in range(3):
            gc.collect()
        assert live.getProblem().getDimension() == 1
        fetched = live.getProblem().getObjective()
        assert fetched is not None, 'GC cleared the live solver objective'
        assert tuple(ref() is not None for ref in refs) == ({IS_NANOBIND!r}, {IS_NANOBIND!r})
        if {IS_NANOBIND!r}:
            assert fetched is refs[0]()
            assert fetched.owner is refs[1]()
            assert fetched.eval([2.0]) == 9
            fetched.owner = None
        del fetched, live
        gc.collect()
        assert all(ref() is None for ref in refs)
        """
    )


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
