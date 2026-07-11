import gc
import subprocess
import sys
import textwrap

import dartpy as dart


def test_getdofs_does_not_take_ownership():
    # Regression: Skeleton.getDofs (and the getChainDofs variants) used to be
    # bound without a return-value policy, so pybind11 took ownership of the
    # joint-owned DegreeOfFreedom pointers and deleted them with the Python
    # wrappers, corrupting the heap.
    world = dart.utils.MjcfParser.readWorld("dart://sample/mjcf/openai/reacher.xml")
    assert world is not None

    skel = world.getSkeleton(0)
    num_dofs = skel.getNumDofs()
    names_before = [skel.getDof(i).getName() for i in range(num_dofs)]

    dofs = skel.getDofs()
    assert len(dofs) == num_dofs
    del dofs
    gc.collect()

    assert skel.getNumDofs() == num_dofs
    assert [skel.getDof(i).getName() for i in range(num_dofs)] == names_before

    for _ in range(10):
        world.step()
    assert all(abs(v) < 1e6 for v in skel.getPositions())


def test_getdofs_survives_simulation_and_interpreter_exit():
    # The heap corruption typically surfaced as a SIGSEGV when the objects
    # were finally destroyed, so run the full reproducer in a subprocess and
    # require a clean exit code.
    script = textwrap.dedent(
        """
        import dartpy as dart

        world = dart.utils.MjcfParser.readWorld(
            "dart://sample/mjcf/openai/reacher.xml")
        dofs = {}
        for i in range(world.getNumSkeletons()):
            for dof in world.getSkeleton(i).getDofs():
                dofs[dof.getName()] = dof
        for _ in range(2000):
            for dof in dofs.values():
                dof.setForce(0.1)
            world.step()
        print("done")
        """
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        timeout=600,
    )
    assert result.returncode == 0, (
        f"subprocess exited with {result.returncode}\n"
        f"stdout: {result.stdout[-2000:]}\nstderr: {result.stderr[-2000:]}"
    )
    assert "done" in result.stdout
