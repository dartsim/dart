import subprocess
import sys
import textwrap

import pytest


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


def test_soft_body_pair_keeps_native_owners_alive():
    run_isolated(
        """
        import gc
        import dartpy as dart

        skeleton = dart.dynamics.Skeleton()
        props = dart.dynamics.SoftBodyNodeHelper.makeEllipsoidProperties(
            [0.2, 0.2, 0.2], 6, 6, 0.5)
        joint, body = skeleton.createFreeJointAndSoftBodyNodePair(
            None, dart.dynamics.FreeJointProperties(),
            dart.dynamics.SoftBodyNodeProperties(
                dart.dynamics.BodyNodeProperties(), props))
        del skeleton
        gc.collect()
        assert body.getNumPointMasses() == 32
        assert joint.getNumDofs() == 6
        del body
        gc.collect()
        assert joint.getNumDofs() == 6
        print("done")
        """
    )


def test_euler_move_joint_keeps_destination_alive():
    run_isolated(
        """
        import gc
        import dartpy as dart

        source = dart.dynamics.Skeleton("source")
        body = source.createFreeJointAndBodyNodePair()[1]
        destination = dart.dynamics.Skeleton("destination")
        parent = destination.createWeldJointAndBodyNodePair()[1]
        joint = body.moveToEulerJoint(parent, dart.dynamics.EulerJointProperties())
        assert isinstance(joint, dart.dynamics.EulerJoint)
        assert source.getNumBodyNodes() == 0
        assert destination.getNumBodyNodes() == 2
        joint.setName("moved")
        del body, parent, source, destination
        gc.collect()
        replacements = []
        for _ in range(200):
            other = dart.dynamics.Skeleton()
            other.createFreeJointAndBodyNodePair()[0].setName("replacement")
            replacements.append(other)
        assert joint.getName() == "moved"
        joint.setPosition(0, 0.2)
        assert joint.getPosition(0) == 0.2
        print("done")
        """
    )


def test_recording_keeps_world_alive():
    run_isolated(
        """
        import gc
        import dartpy as dart

        world = dart.simulation.World()
        skeleton = dart.dynamics.Skeleton()
        skeleton.createFreeJointAndBodyNodePair()
        world.addSkeleton(skeleton)
        world.bake()
        recording = world.getRecording()
        del world, skeleton
        gc.collect()
        assert recording.getNumFrames() == 1
        assert len(recording.getConfig(0, 0)) == 6
        recording.clear()
        assert recording.getNumFrames() == 0
        print("done")
        """
    )


@pytest.mark.parametrize("overload", range(4))
def test_copyto_does_not_take_ownership(overload):
    run_isolated(
        f"""
        import gc
        import dartpy as dart

        src = dart.dynamics.Skeleton("src")
        joint, body = src.createRevoluteJointAndBodyNodePair()
        del joint
        dst = dart.dynamics.Skeleton("dst")
        parent = dst.createRevoluteJointAndBodyNodePair()[1]
        args = ((parent,), (parent, False), (dst, None), (dst, None, False))
        pair = body.copyTo(*args[{overload}])
        name = pair[0].getName()
        del pair
        gc.collect()
        assert dst.getJoint(1).getName() == name
        world = dart.simulation.World()
        world.addSkeleton(dst)
        world.step()
        print("done")
        """
    )


@pytest.mark.parametrize("overload", range(4))
def test_copyto_joint_keeps_destination_alive(overload):
    run_isolated(
        f"""
        import gc
        import dartpy as dart

        src = dart.dynamics.Skeleton("src")
        body = src.createRevoluteJointAndBodyNodePair()[1]
        dst = dart.dynamics.Skeleton("dst")
        parent = dst.createRevoluteJointAndBodyNodePair()[1]
        args = ((parent,), (parent, False), (dst, None), (dst, None, False))
        pair = body.copyTo(*args[{overload}])
        joint = pair[0]
        name = joint.getName()
        del pair, dst, parent, args
        gc.collect()
        replacements = []
        for _ in range(200):
            other = dart.dynamics.Skeleton()
            other.createRevoluteJointAndBodyNodePair()[0].setName("replacement")
            replacements.append(other)
        assert joint.getName() == name
        print("done")
        """
    )
