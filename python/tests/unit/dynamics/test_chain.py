import subprocess
import sys
import textwrap

import dartpy as dart
import pytest


@pytest.mark.parametrize(
    "getter, count", [("getBodyNodes", 1), ("getJoints", 1), ("getDofs", 2)]
)
def test_chain_getters_return_lists(getter, count):
    skel = dart.dynamics.Skeleton()
    _, body1 = skel.createFreeJointAndBodyNodePair()
    _, body2 = skel.createUniversalJointAndBodyNodePair(body1)
    chain = dart.dynamics.Chain(body1, body2, "chain")

    result = getattr(chain, getter)()
    assert isinstance(result, list)
    assert len(result) == count


def test_chain_joints_do_not_take_ownership():
    script = textwrap.dedent(
        """
        import gc
        import dartpy as dart

        skel = dart.dynamics.Skeleton()
        body1 = skel.createFreeJointAndBodyNodePair()[1]
        body2 = skel.createUniversalJointAndBodyNodePair(body1)[1]
        chain = dart.dynamics.Chain(body1, body2, "chain")
        joints = chain.getJoints()
        assert len(joints) == 1
        del joints
        gc.collect()
        assert skel.getJoint(1).getNumDofs() == 2
        """
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
