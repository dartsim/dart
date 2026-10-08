import subprocess
import sys
import textwrap


def run_isolated(script):
    # A use-after-free crashes the interpreter, so run each case in a
    # subprocess and require a clean exit.
    result = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(script)],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, (
        f"subprocess exited with {result.returncode}\n"
        f"stdout: {result.stdout[-2000:]}\nstderr: {result.stderr[-2000:]}"
    )
    assert "done" in result.stdout


def test_getjoints_does_not_take_ownership():
    # Regression: like getDofs before (test_degree_of_freedom_ownership.py),
    # the getJoints bindings of Skeleton and MetaSkeleton had no return-value
    # policy, so pybind11 took ownership of the body-owned Joint pointers and
    # deleted them with the returned list.
    run_isolated(
        """
        import gc
        import dartpy as dart

        skel = dart.dynamics.Skeleton()
        body = skel.createRevoluteJointAndBodyNodePair()[1]
        skel.createRevoluteJointAndBodyNodePair(body)
        name = skel.getJoint(1).getName()
        gc.collect()
        for get in (
            lambda: skel.getJoints(),
            lambda: skel.getJoints(name),
            lambda: dart.dynamics.MetaSkeleton.getJoints(skel),
            lambda: dart.dynamics.MetaSkeleton.getJoints(skel, name),
        ):
            joints = get()
            assert joints
            del joints
            gc.collect()
            assert skel.getJoint(1).getName() == name
        world = dart.simulation.World()
        world.addSkeleton(skel)
        world.step()
        print("done")
        """
    )


def test_getchildframes_does_not_take_ownership():
    # Regression: Frame.getChildFrames and getChildEntities took ownership of
    # child frames that DART owns, such as a BodyNode's ShapeNodes, and
    # deleted them with the returned set.
    run_isolated(
        """
        import gc
        import dartpy as dart

        skel = dart.dynamics.Skeleton()
        body = skel.createFreeJointAndBodyNodePair()[1]
        body.createShapeNode(dart.dynamics.BoxShape([1, 1, 1]))
        name = body.getShapeNode(0).getName()
        gc.collect()
        for get in (body.getChildFrames, body.getChildEntities):
            children = get()
            assert children
            del children
            gc.collect()
            assert body.getShapeNode(0).getName() == name
        world = dart.simulation.World()
        world.addSkeleton(skel)
        world.step()
        print("done")
        """
    )
