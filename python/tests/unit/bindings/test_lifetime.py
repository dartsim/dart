"""Lifetime regressions run to interpreter exit, including nanobind's leak check."""

import pytest

from ._support import IS_NANOBIND, run_isolated


def test_cpp_owner_retains_python_overrides_with_nanobind():
    run_isolated(
        f"""
        import weakref
        calls = []
        class Function(dart.optimizer.Function):
            def eval(self, x):
                calls.append(float(x[0]))
                return float(x[0] ** 2)
            def evalGradient(self, x, gradient):
                gradient[:] = 2 * x
        problem = dart.optimizer.Problem(1)
        problem.setInitialGuess([0.0])
        objective = Function()
        problem.setObjective(objective)
        ref = weakref.ref(objective)
        del objective
        gc.collect()
        assert (ref() is not None) is {IS_NANOBIND!r}
        if {IS_NANOBIND!r}:
            solver = dart.optimizer.GradientDescentSolver(problem)
            assert solver.solve()
            assert calls
            del solver
        del problem
        gc.collect()
        assert ref() is None
        """
    )


@pytest.mark.parametrize("return_path", ["direct", "frame", "node", "children"])
def test_body_holder_retains_skeleton_from_every_return_path(return_path):
    run_isolated(
        f"""
        skel = dart.dynamics.Skeleton(name="owner")
        skel.createFreeJointAndBodyNodePair()[1].setName("retained")
        if {return_path!r} == "direct":
            body = skel.getBodyNode(0)
        elif {return_path!r} == "frame":
            shape = skel.getBodyNode(0).createShapeNode(dart.dynamics.BoxShape(np.ones(3)))
            body = dart.dynamics.Entity.getParentFrame(shape)
            del shape
        elif {return_path!r} == "node":
            body = dart.dynamics.Node.getBodyNodePtr(skel.getBodyNode(0))
        else:
            parent = skel.getBodyNode(0)
            skel.createRevoluteJointAndBodyNodePair(parent)[1].setName("retained_child")
            body = next(x for x in parent.getChildFrames() if isinstance(x, dart.dynamics.BodyNode))
            del parent
        del skel
        gc.collect()
        assert body.getName().startswith("retained")
        assert body.getSkeleton().getNumBodyNodes() == (2 if {return_path!r} == "children" else 1)
        assert body.getParentJoint().getNumDofs() in (1, 6)
        """
    )


def test_new_body_wrapper_from_frame_pointer_retains_skeleton():
    run_isolated(
        """
        import weakref
        skel = dart.dynamics.Skeleton(name="frame_owner")
        skel.createFreeJointAndBodyNodePair()
        gc.collect()
        original = skel.getBodyNode(0)
        original.setName("via_frame")
        # SimpleFrame's raw parent permits the original wrapper to disappear.
        frame = dart.dynamics.SimpleFrame(refFrame=original, name="observer")
        original_wrapper = weakref.ref(original)
        del original
        gc.collect()
        assert original_wrapper() is None
        body = dart.dynamics.Entity.getParentFrame(frame)
        assert type(body) is dart.dynamics.BodyNode
        del frame, skel
        gc.collect()
        assert body.getName() == "via_frame"
        assert body.getSkeleton().getName() == "frame_owner"
        assert body.getSkeleton().getNumBodyNodes() == 1
        """
    )


@pytest.mark.parametrize("drop_new_owner", [False, True])
def test_body_holder_follows_move_to_another_skeleton(drop_new_owner):
    run_isolated(
        f"""
        import weakref
        old = dart.dynamics.Skeleton(name="old")
        new = dart.dynamics.Skeleton(name="new")
        old.createFreeJointAndBodyNodePair()
        gc.collect()
        body = old.getBodyNode(0)
        body.setName("moved")
        old_wrapper = weakref.ref(old)
        assert body.moveTo(newSkeleton=new, newParent=None)
        assert old.getNumBodyNodes() == 0
        assert new.getNumBodyNodes() == 1
        del old
        if {drop_new_owner!r}:
            del new
        gc.collect()
        assert old_wrapper() is None
        assert body.getName() == "moved"
        assert body.getSkeleton().getName() == "new"
        assert body.getSkeleton().getNumBodyNodes() == 1
        world = dart.simulation.World()
        world.addSkeleton(body.getSkeleton())
        world.step()
        """
    )


def test_returned_graph_lists_never_take_ownership():
    run_isolated(
        """
        skel = dart.dynamics.Skeleton()
        joint, body = skel.createFreeJointAndBodyNodePair()
        skel.createRevoluteJointAndBodyNodePair(body)
        shape = body.createShapeNode(dart.dynamics.BoxShape(np.ones(3)))
        shape_name = shape.getName()
        del joint, shape
        for get in (
            skel.getJoints,
            lambda: dart.dynamics.MetaSkeleton.getJoints(skel),
            skel.getDofs,
            lambda: dart.dynamics.MetaSkeleton.getDofs(skel),
            body.getChildFrames,
            body.getChildEntities,
            lambda: dart.dynamics.Frame.getChildFrames(body),
            lambda: dart.dynamics.Frame.getChildEntities(body),
        ):
            values = get()
            assert values
            del values
            gc.collect()
            assert skel.getJoint(0).getNumDofs() == 6
            assert body.getShapeNode(0).getName() == shape_name
            assert skel.getDofs()[0].getPosition() == 0
        world = dart.simulation.World()
        world.addSkeleton(skel)
        world.step()
        """
    )


def test_world_retains_native_skeleton_without_pinning_python_wrapper():
    run_isolated(
        """
        import weakref
        world = dart.simulation.World()
        skeleton = dart.dynamics.Skeleton(name="world_owner")
        skeleton.createFreeJointAndBodyNodePair()
        assert world.addSkeleton(skeleton=skeleton) == "world_owner"
        wrapper = weakref.ref(skeleton)
        del skeleton
        gc.collect()
        assert wrapper() is None
        assert world.getNumSkeletons() == 1
        recovered = world.getSkeleton(0)
        assert world.getSkeleton(0) is recovered
        assert recovered.getName() == "world_owner"
        assert recovered.getNumDofs() == 6
        assert recovered.getBodyNode(0).getSkeleton() is recovered
        world.step()
        """
    )


@pytest.mark.parametrize(
    ("kind", "arguments", "volume"),
    [
        ("BoxShape", "np.array([1.0, 2.0, 3.0])", "6.0"),
        ("EllipsoidShape", "np.array([2.0, 4.0, 6.0])", "8 * np.pi"),
        ("SphereShape", "0.5", "np.pi / 6"),
    ],
)
def test_frame_retains_native_shape_without_pinning_python_wrapper(
    kind, arguments, volume
):
    run_isolated(
        f"""
        import weakref
        frame = dart.dynamics.SimpleFrame()
        shape = dart.dynamics.{kind}({arguments})
        frame.setShape(shape)
        wrapper = weakref.ref(shape)
        del shape
        gc.collect()
        assert wrapper() is None
        recovered = frame.getShape()
        assert frame.getShape() is recovered
        assert type(recovered) is dart.dynamics.{kind}
        assert np.isclose(recovered.getVolume(), {volume})
        del frame
        gc.collect()
        assert np.isclose(recovered.getVolume(), {volume})
        """
    )


def test_world_retains_native_simple_frame_without_pinning_python_wrapper():
    run_isolated(
        """
        import weakref
        world = dart.simulation.World()
        frame = dart.dynamics.SimpleFrame()
        frame.setName("world_frame")
        observer = dart.dynamics.SimpleFrame(refFrame=frame, name="observer")
        assert world.addSimpleFrame(frame=frame) == "world_frame"
        wrapper = weakref.ref(frame)
        del frame
        gc.collect()
        assert wrapper() is None
        assert observer.getParentFrame().getName() == "world_frame"
        recovered = world.getSimpleFrame(0)
        assert recovered is observer.getParentFrame()
        assert recovered.getName() == "world_frame"
        world.step()
        del observer
        """
    )


def test_reentrant_wrapper_replacement_retains_native_owner():
    run_isolated(
        """
        import weakref
        world = dart.simulation.World()
        frame = dart.dynamics.SimpleFrame()
        frame.setName("reentrant")
        world.addSimpleFrame(frame)
        replacements = []
        old = weakref.ref(frame, lambda dead: replacements.append(world.getSimpleFrame(0)))
        del frame
        gc.collect()
        assert old() is None
        assert len(replacements) == 1
        replacement = replacements.pop()
        assert world.getSimpleFrame(0) is replacement
        other = dart.simulation.World()
        other.addSimpleFrame(replacement)
        new = weakref.ref(replacement)
        del replacement
        gc.collect()
        assert new() is None
        assert world.getSimpleFrame(0).getName() == "reentrant"
        assert other.getSimpleFrame(0).getName() == "reentrant"
        """
    )


def test_raw_parent_wrapper_cannot_manufacture_shared_ownership():
    run_isolated(
        """
        import weakref
        world = dart.simulation.World()
        frame = dart.dynamics.SimpleFrame()
        observer = dart.dynamics.SimpleFrame(refFrame=frame)
        world.addSimpleFrame(frame)
        original = weakref.ref(frame)
        del frame
        gc.collect()
        assert original() is None
        raw = observer.getParentFrame()
        other = dart.simulation.World()
        try:
            other.addSimpleFrame(raw)
        except (TypeError, RuntimeError):
            rejected = True
        else:
            rejected = False
        # The observer's raw parent must remain alive until observer cleanup.
        del raw, observer
        gc.collect()
        del other
        gc.collect()
        del world
        gc.collect()
        assert rejected
        """
    )


@pytest.mark.parametrize("kind", ["BodyNode", "ShapeNode", "Joint", "DegreeOfFreedom"])
def test_each_new_graph_wrapper_retains_native_skeleton(kind):
    run_isolated(
        f"""
        skel = dart.dynamics.Skeleton(name="graph_owner")
        skel.createFreeJointAndBodyNodePair()[1].createShapeNode(dart.dynamics.BoxShape(np.ones(3)))
        gc.collect()
        if {kind!r} == "BodyNode":
            value = skel.getBodyNode(0)
        elif {kind!r} == "ShapeNode":
            value = skel.getBodyNode(0).getShapeNode(0)
        elif {kind!r} == "Joint":
            value = skel.getJoint(0)
        else:
            value = skel.getDofs()[0]
        del skel
        gc.collect()
        assert value.getName()
        assert value.getSkeleton().getName() == "graph_owner"
        assert value.getSkeleton().getNumDofs() == 6
        if {kind!r} == "DegreeOfFreedom":
            value.setPosition(0.25)
            assert value.getPosition() == 0.25
        else:
            body = value if {kind!r} == "BodyNode" else (value.getBodyNodePtr() if {kind!r} == "ShapeNode" else value.getChildBodyNode())
            assert body.getWorldTransform().matrix().shape == (4, 4)
        """
    )


@pytest.mark.parametrize(
    "getter", ["getJoints", "getDofs", "getChildFrames", "getChildEntities"]
)
def test_graph_objects_in_returned_lists_retain_native_skeleton(getter):
    run_isolated(
        f"""
        skel = dart.dynamics.Skeleton(name="list_owner")
        skel.createFreeJointAndBodyNodePair()[1].createShapeNode(dart.dynamics.SphereShape(0.5))
        skel.createRevoluteJointAndBodyNodePair(skel.getBodyNode(0))
        gc.collect()
        if {getter!r} in ("getJoints", "getDofs"):
            values = getattr(skel, {getter!r})()
        else:
            values = getattr(skel.getBodyNode(0), {getter!r})()
        assert len(values) >= 2
        del skel
        gc.collect()
        for value in values:
            assert value.getName()
            assert value.getSkeleton().getName() == "list_owner"
            assert value.getSkeleton().getNumBodyNodes() == 2
        """
    )
