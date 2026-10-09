"""Constructor and wrapper-identity checks shared by both dartpy backends."""

import inspect
import math
import weakref

import dartpy as dart
import numpy as np
import pytest

from ._support import run_isolated

D = dart.dynamics


@pytest.mark.parametrize(
    ("kind", "argument", "volume"),
    [
        ("BoxShape", [1, 2, 3], 6.0),
        ("EllipsoidShape", [2, 4, 6], 8 * math.pi),
        ("SphereShape", 0.5, math.pi / 6),
    ],
)
def test_shape_subclass_translated_init_and_identity(kind, argument, volume):
    class PythonShape(getattr(D, kind)):
        def __init__(self, *, dimension, label):
            super().__init__(dimension)
            self.label = label

    shape = PythonShape(dimension=argument, label="translated")
    assert shape.label == "translated"
    assert D.Shape.getVolume(shape) == pytest.approx(volume)
    skeleton = D.Skeleton()
    _, body = skeleton.createFreeJointAndBodyNodePair()
    node = body.createShapeNode(shape)
    assert node.getShape() is shape
    assert D.ShapeFrame.getShape(node) is shape
    assert shape.getType() == kind


def test_simple_frame_subclass_translated_init_and_parent_identity():
    class PythonFrame(D.SimpleFrame):
        def __init__(self, label, offset, *, parent):
            transform = dart.math.Isometry3()
            transform.set_translation(offset)
            super().__init__(
                refFrame=parent, name=f"python_{label}", relativeTransform=transform
            )

    parent = PythonFrame("parent", [1, 2, 3], parent=D.Frame.World())
    child = D.SimpleFrame(refFrame=parent, name="child")
    assert child.getParentFrame() is parent
    assert D.Entity.getParentFrame(child) is parent
    assert parent.getName() == "python_parent"
    np.testing.assert_allclose(parent.getWorldTransform().translation(), [1, 2, 3])


def test_skeleton_subclass_translated_init_and_world_identity():
    class PythonSkeleton(D.Skeleton):
        def __init__(self, label, *, prefix):
            super().__init__(name=f"{prefix}_{label}")

    skeleton = PythonSkeleton("translated", prefix="python")
    skeleton.createFreeJointAndBodyNodePair()
    assert skeleton.getName() == "python_translated"
    world = dart.simulation.World()
    assert world.addSkeleton(skeleton) == "python_translated"
    assert world.getSkeleton(0) is skeleton
    assert world.getSkeleton(0).getNumDofs() == 6
    world.step()


@pytest.mark.parametrize("kind", ["Skeleton", "World"])
def test_factory_subclass_preserves_default_and_translated_names(kind):
    base = D.Skeleton if kind == "Skeleton" else dart.simulation.World

    class Default(base):
        pass

    class Translated(base):
        def __init__(self, label, *, prefix):
            super().__init__(name=f"{prefix}_{label}")
            self.label = label

    assert Default().getName() == base().getName()
    value = Translated("named", prefix="python")
    assert value.getName() == "python_named"
    assert value.label == "named"
    if kind == "Skeleton":
        assert value.getPtr() is value
        _, body = value.createFreeJointAndBodyNodePair()
        assert body.getSkeleton() is value
    else:
        skeleton = D.Skeleton()
        value.addSkeleton(skeleton)
        assert value.getSkeleton(0) is skeleton


def test_chain_factory_subclass_defers_native_arguments_and_preserves_new_identity():
    run_isolated(
        """
        import weakref
        skeleton = dart.dynamics.Skeleton()
        _, start = skeleton.createFreeJointAndBodyNodePair()
        _, target = skeleton.createRevoluteJointAndBodyNodePair(start)
        class Parent(dart.dynamics.Chain):
            def __init__(self, *, include_parent, native_name):
                super().__init__(start, target, include_parent, native_name)
        class Chain(Parent):
            def __new__(cls, label):
                value = super().__new__(cls)
                value.allocated_label = label
                cls.pending = weakref.ref(value)
                return value
            def __init__(self, label):
                assert self is self.pending()
                assert self.allocated_label == label
                super().__init__(include_parent=True, native_name='python_' + label)
                self.label = label
        value = Chain('translated')
        assert value is Chain.pending()
        assert value.label == 'translated'
        assert value.getName() == 'python_translated'
        assert value.getNumDofs() == 7
        assert value.getBodyNode(0) is start
        assert value.getBodyNode(1) is target
        clone = value.cloneChain('cloned')
        assert clone.getName() == 'cloned' and clone.getNumDofs() == 7
        del clone
        wrappers = [item for item in gc.get_objects()
                    if type(item) in (dart.dynamics.Chain, Chain)]
        assert wrappers == [value], [type(item).__name__ for item in wrappers]
        del wrappers, value
        gc.collect()
        assert Chain.pending() is None
        """
    )


def test_chain_factory_uses_forwarded_arguments_even_when_outer_signature_matches():
    skeleton = D.Skeleton()
    _, start = skeleton.createFreeJointAndBodyNodePair()
    _, target = skeleton.createRevoluteJointAndBodyNodePair(start)

    class Chain(D.Chain):
        def __init__(self, start, target, name):
            super().__init__(start, target, True, f"python_{name}")

    value = Chain(start, target, "translated")
    assert value.getName() == "python_translated"
    assert value.getNumDofs() == 7
    D.Chain.__init__(value, start, target, "replacement")
    assert value.getName() == "python_translated"
    assert value.getNumDofs() == 7


def test_linkage_factory_subclass_translates_criteria_and_name():
    skeleton = D.Skeleton()
    _, start = skeleton.createFreeJointAndBodyNodePair()
    _, target = skeleton.createRevoluteJointAndBodyNodePair(start)

    class Linkage(D.Linkage):
        def __init__(self, label):
            criteria = D.ChainCriteria(start, target, True).convert()
            super().__init__(criteria, f"python_{label}")
            self.label = label

    value = Linkage("translated")
    assert value.label == "translated"
    assert value.getName() == "python_translated"
    assert value.getNumDofs() == 7
    assert value.getBodyNode(0) is start
    assert value.getBodyNode(1) is target
    assert value.cloneLinkage().getNumDofs() == 7


def test_inverse_kinematics_factory_subclass_preserves_native_backreferences():
    skeleton = D.Skeleton()
    _, body = skeleton.createFreeJointAndBodyNodePair()

    class InverseKinematics(D.InverseKinematics):
        def __init__(self, *, label):
            super().__init__(body)
            self.label = label

    value = InverseKinematics(label="translated")
    assert value.label == "translated"
    assert value.getDofs() == list(range(6))
    assert value.getGradientMethod().getIK() is value


def test_inverse_kinematics_factory_keeps_affiliated_node_alive():
    run_isolated(
        """
        skeleton = dart.dynamics.Skeleton()
        _, body = skeleton.createFreeJointAndBodyNodePair()
        class InverseKinematics(dart.dynamics.InverseKinematics):
            def __init__(self, label, body):
                super().__init__(body)
                self.label = label
        value = InverseKinematics('translated', body)
        del _, body, skeleton
        gc.collect()
        assert value.label == 'translated'
        assert value.getDofs() == list(range(6))
        assert value.getGradientMethod().getIK() is value
        """
    )


@pytest.mark.parametrize(
    "kind",
    [
        "DARTCollisionDetector",
        "FCLCollisionDetector",
        "BulletCollisionDetector",
        "OdeCollisionDetector",
    ],
)
def test_collision_factory_subclass_preserves_native_shared_owner(kind):
    base = getattr(dart.collision, kind, None)
    if base is None:
        pytest.skip(f"{kind} requires its optional collision component")

    class Detector(base):
        def __init__(self, label):
            super().__init__()
            self.label = label

    value = Detector("translated")
    assert value.label == "translated"
    assert value.createCollisionGroup().getCollisionDetector() is value


@pytest.mark.parametrize("failure", ["before_base", "after_base", "native_arguments"])
def test_factory_subclass_constructor_failure_releases_pending_wrapper(failure):
    run_isolated(
        f"""
        import weakref
        skeleton = dart.dynamics.Skeleton()
        _, start = skeleton.createFreeJointAndBodyNodePair()
        _, target = skeleton.createRevoluteJointAndBodyNodePair(start)
        class Chain(dart.dynamics.Chain):
            def __new__(cls, label):
                value = super().__new__(cls)
                cls.pending = weakref.ref(value)
                return value
            def __init__(self, label):
                self.label = label
                if {failure!r} == 'before_base':
                    raise ValueError('before native construction')
                if {failure!r} == 'native_arguments':
                    super().__init__('invalid criteria')
                else:
                    super().__init__(start, target, label)
                    raise ValueError('after native construction')
        try:
            Chain('failing')
        except (TypeError, ValueError):
            pass
        else:
            raise AssertionError('constructor failure was ignored')
        gc.collect()
        assert Chain.pending() is None
        """
    )


@pytest.mark.parametrize("arity", [0, 1, 2, 3])
def test_simple_frame_constructor_overloads_initialize_state(arity):
    parent = D.SimpleFrame()
    transform = dart.math.Isometry3()
    transform.set_translation([1, 2, 3])
    arguments = (parent, "named", transform)[:arity]
    frame = D.SimpleFrame(*arguments)
    if arity:
        assert frame.getParentFrame() is parent
    else:
        assert frame.getParentFrame().isWorld()
        assert frame.getParentFrame().getName() == "World"
        assert frame.getParentFrame() is not D.Frame.World()
    if arity >= 2:
        assert frame.getName() == "named"
    np.testing.assert_allclose(
        frame.getWorldTransform().translation(), [1, 2, 3] if arity == 3 else [0, 0, 0]
    )
    assert frame.getShape() is None


@pytest.mark.parametrize(
    "keywords", ["name", "relativeTransform", "refFrame+relativeTransform"]
)
def test_simple_frame_constructor_rejects_incomplete_keyword_overloads(keywords):
    arguments = {}
    if "name" in keywords:
        arguments["name"] = "incomplete"
    if "relativeTransform" in keywords:
        arguments["relativeTransform"] = dart.math.Isometry3()
    if "refFrame" in keywords:
        arguments["refFrame"] = D.Frame.World()
    with pytest.raises(TypeError):
        D.SimpleFrame(**arguments)


@pytest.mark.parametrize(
    "kind",
    ["Shape", "BoxShape", "SphereShape", "EllipsoidShape", "SimpleFrame", "Skeleton"],
)
def test_python_subclass_must_call_base_constructor(kind):
    run_isolated(
        f"""
        class Uninitialized(dart.dynamics.{kind}):
            def __init__(self):
                self.python_only = True
        try:
            Uninitialized()
        except Exception:
            return
        raise AssertionError("missing {kind} base constructor accepted")
        """
    )


@pytest.mark.parametrize(
    ("kind", "arguments"),
    [
        ("BoxShape", "np.ones(3)"),
        ("EllipsoidShape", "np.ones(3)"),
        ("SphereShape", "0.5"),
        ("SimpleFrame", ""),
        ("Skeleton", ""),
    ],
)
def test_constructor_creates_no_hidden_second_wrapper(kind, arguments):
    run_isolated(
        f"""
        import weakref
        base = dart.dynamics.{kind}
        class PythonSubclass(base):
            pass
        value = PythonSubclass({arguments})
        wrapper = weakref.ref(value)
        # A factory subtype-fixup wrapper is tracked even when the native type is not.
        wrappers = [item for item in gc.get_objects() if type(item) in (base, PythonSubclass)]
        assert wrappers == [value], [(type(item).__name__) for item in wrappers]
        del wrappers, value
        gc.collect()
        assert wrapper() is None
        """
    )


@pytest.mark.parametrize("kind", ["BoxShape", "SimpleFrame", "Skeleton"])
def test_subclass_world_cycle_collection_and_cleanup(kind):
    """Collect native-owner/Python-attribute cycles and verify explicit cleanup."""
    run_isolated(
        f"""
        import weakref
        class PythonOwned(dart.dynamics.{kind}):
            def __init__(self, world):
                super().__init__(*([np.ones(3)] if {kind!r} == "BoxShape" else []))
                self.world = world
        retained = 0
        for _ in range(20):
            world = dart.simulation.World()
            value = PythonOwned(world)
            if {kind!r} == "BoxShape":
                skeleton = dart.dynamics.Skeleton()
                skeleton.createFreeJointAndBodyNodePair()[1].createShapeNode(value)
                world.addSkeleton(skeleton)
                del skeleton
            elif {kind!r} == "SimpleFrame":
                world.addSimpleFrame(value)
            else:
                value.createFreeJointAndBodyNodePair()
                world.addSkeleton(value)
            world_wrapper, value_wrapper = weakref.ref(world), weakref.ref(value)
            del value, world
            gc.collect()
            if value_wrapper() is not None:
                retained += 1
                value_wrapper().world = None
            gc.collect()
            assert value_wrapper() is None
            assert world_wrapper() is None
        assert retained == 0, retained
        """
    )


@pytest.mark.parametrize(
    "kind", ["BoxShape", "EllipsoidShape", "SphereShape", "SimpleFrame", "Skeleton"]
)
def test_repeated_base_init_preserves_initialized_state(kind):
    base = getattr(D, kind)
    if kind in ("BoxShape", "EllipsoidShape"):
        value = base([1, 2, 3])
        volume = value.getVolume()
        base.__init__(value, [4, 5, 6])
        assert value.getVolume() == pytest.approx(volume)
    elif kind == "SphereShape":
        value = base(0.5)
        base.__init__(value, 2.0)
        assert value.getVolume() == pytest.approx(math.pi / 6)
    elif kind == "SimpleFrame":
        parent = D.SimpleFrame()
        value = base(parent, "original")
        base.__init__(value, D.Frame.World(), "replacement")
        assert value.getName() == "original"
        assert value.getParentFrame() is parent
    else:
        value = base("original")
        value.createFreeJointAndBodyNodePair()
        base.__init__(value, "replacement")
        assert value.getName() == "original"
        assert value.getNumDofs() == 6


def test_constructor_guard_survives_init_subclass_without_super():
    class Parent(D.SimpleFrame):
        def __init_subclass__(cls):
            cls.parent_hook_ran = True

    class Initialized(Parent):
        def __init__(self, label):
            super().__init__(D.Frame.World(), label)

    class Uninitialized(Parent):
        def __init__(self):
            pass

    assert Initialized.parent_hook_ran
    assert Initialized("translated").getName() == "translated"
    with pytest.raises(Exception):
        Uninitialized()


def test_staticmethod_subclass_initializer():
    class Initialized(D.SimpleFrame):
        def __new__(cls, *, label):
            value = super().__new__(cls)
            cls.pending = weakref.ref(value)
            return value

        @staticmethod
        def __init__(*, label):
            D.SimpleFrame.__init__(Initialized.pending(), D.Frame.World(), label)

    class Uninitialized(D.SimpleFrame):
        @staticmethod
        def __init__():
            pass

    value = Initialized(label="static_initializer")
    assert value.getName() == "static_initializer"
    with pytest.raises(Exception):
        Uninitialized()


@pytest.mark.parametrize(
    ("kind", "argument"),
    [("BoxShape", [1, 2, 3]), ("EllipsoidShape", [1, 2, 3]), ("SphereShape", 0.5)],
)
@pytest.mark.parametrize("subclass", [False, True])
def test_shape_constructor_preserves_initial_version(kind, argument, subclass):
    base = getattr(D, kind)
    if subclass:

        class PythonShape(base):
            pass

        base = PythonShape
    shape = base(argument)
    assert shape.incrementVersion() == 3


@pytest.mark.parametrize(
    ("kind", "arguments"),
    [("BoxShape", ([1, 2, 3],)), ("SimpleFrame", ()), ("Skeleton", ())],
)
@pytest.mark.parametrize("invalid_self", [None, object(), "wrong instance"])
def test_direct_base_initializer_rejects_non_instance_self(
    kind, arguments, invalid_self
):
    with pytest.raises(TypeError):
        getattr(D, kind).__init__(invalid_self, *arguments)


def test_python_subclass_initializer_introspection_survives_construction():
    class AnnotatedFrame(D.SimpleFrame):
        def __init__(self, parent: D.Frame, label: str, *, suffix: str = "") -> None:
            """Initialize a frame from its annotated Python signature."""
            super().__init__(refFrame=parent, name=f"{label}{suffix}")

    def metadata():
        initializer = AnnotatedFrame.__init__
        return (
            inspect.signature(initializer),
            initializer.__name__,
            initializer.__doc__,
            dict(initializer.__annotations__),
        )

    before = metadata()
    frame = AnnotatedFrame(D.Frame.World(), "annotated", suffix="_frame")
    assert frame.getName() == "annotated_frame"
    assert metadata() == before
