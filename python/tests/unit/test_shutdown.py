"""Exercise dartpy's Python-owning objects while the interpreter finalizes."""

import subprocess
import sys
import textwrap
from pathlib import Path

import dartpy as dart


def _run_at_shutdown(setup, action):
    # The guarded pytest runner sets sys.path, rather than ambient PYTHONPATH.
    script = f"import sys\nsys.path.insert(0, {str(Path(dart.__file__).parent)!r})\n"
    probe = """
import os
import sys
import weakref
import numpy
import dartpy as dart

class ShutdownProbe:
    def __init__(self, action):
        self.action = action

    def __del__(self, is_finalizing=sys.is_finalizing, write=os.write):
        assert is_finalizing(), "probe ran before interpreter finalization"
        self.action()
        self.action = None
        write(1, b"dartpy finalization probe passed\\n")

def prepare():
"""
    probe += textwrap.indent(textwrap.dedent(setup), "    ")
    probe += "\n    def check():\n"
    probe += textwrap.indent(textwrap.dedent(action), "        ")
    probe += "\n    return ShutdownProbe(check)\n"
    # Native callbacks must not retain the namespace containing the keeper.
    script += f"namespace = {{}}\nexec({probe!r}, namespace)\n"
    script += "keeper = namespace['prepare']()\ndel namespace\n"
    result = subprocess.run(
        [sys.executable, "-I", "-X", "faulthandler", "-c", script],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert (
        result.returncode == 0
    ), f"subprocess exited with {result.returncode}\n{result.stdout}{result.stderr}"
    assert "dartpy finalization probe passed" in result.stdout, result.stderr
    assert "Exception ignored" not in result.stderr, result.stderr


def test_python_analytical_ik_during_finalization():
    _run_at_shutdown(
        """
        skeleton = dart.dynamics.Skeleton()
        joint, body = skeleton.createFreeJointAndBodyNodePair()
        ik = body.getIK(True)
        callback = eval("lambda target: []", {})
        ref = weakref.ref(callback)
        # shortcut: use zero analytical DOFs until Skeleton.clone fixes IK indexing.
        analytical = ik.setPythonAnalytical(callback, [])
        owners = [skeleton, ik, analytical]
        transform = dart.math.Isometry3()
        del skeleton, joint, body, ik, analytical, callback
        assert ref() is not None
        """,
        """
        assert not owners[2].getSolutions(transform)
        clone = owners[0].clone()
        assert clone.getBodyNode(0).getIK().getAnalytical() is not None
        del clone
        owners.clear()
        assert ref() is None, "analytical IK retained its callable"
        """,
    )


def test_optimizer_callbacks_during_finalization():
    _run_at_shutdown(
        """
        calls = []
        def cost(x):
            calls.append("cost")
            return float(x[0] ** 2)
        def gradient(x, grad):
            calls.append("gradient")
            grad[0] = 2 * x[0]
        def hessian(x, hess):
            hess[0, 0] = 2
        refs = [weakref.ref(callback) for callback in (cost, gradient, hessian)]
        function = dart.optimizer.ModularFunction()
        function.setCostFunction(cost)
        function.setGradientFunction(gradient)
        function.setHessianFunction(hessian)
        owners = [function]
        del function, cost, gradient, hessian
        assert all(ref() is not None for ref in refs)
        """,
        """
        function = owners[0]
        assert function.eval([3]) == 9
        # Replacement also exercises the binder's std::function copy/deleter.
        function.setCostFunction(refs[0]())
        problem = dart.optimizer.Problem(1)
        problem.setInitialGuess([3])
        problem.setObjective(function)
        solver = dart.optimizer.GradientDescentSolver(problem)
        solver.setNumMaxIterations(1)
        solver.solve()
        assert "cost" in calls and "gradient" in calls
        del solver, problem, function
        owners.clear()
        assert all(ref() is None for ref in refs), "optimizer retained a callable"
        """,
    )


def test_raycast_filter_during_finalization():
    _run_at_shutdown(
        """
        callback = eval("lambda object: False", {})
        ref = weakref.ref(callback)
        owners = [dart.collision.RaycastOption(True, True, callback),
                  dart.collision.RaycastOption()]
        owners[1].mFilter = callback
        del callback
        assert ref() is not None
        """,
        """
        assert owners[0].mFilter(None) is False
        assert owners[1].mFilter(None) is False
        owners[1].mFilter = ref()
        owners.clear()
        assert ref() is None, "raycast option retained its callable"
        """,
    )


def test_virtual_callbacks_during_finalization():
    _run_at_shutdown(
        """
        class Solver(dart.optimizer.Solver):
            def solve(self):
                return False
            def getType(self):
                return "shutdown solver"
            def clone(self):
                return type(self)()
        class Filter(dart.collision.BodyNodeCollisionFilter):
            def ignoresCollision(self, object1, object2):
                return True
        function_calls = []
        class Function(dart.optimizer.Function):
            def eval(self, x):
                function_calls.append("eval")
                return float(x[0] ** 2)
            def evalGradient(self, x, grad):
                function_calls.append("gradient")
                grad[0] = 2 * x[0]
        solver = Solver()
        filter = Filter()
        function = Function()
        problem = dart.optimizer.Problem(1)
        problem.setInitialGuess([3])
        problem.setObjective(function)
        native_solver = dart.optimizer.GradientDescentSolver(problem)
        native_solver.setNumMaxIterations(1)
        solver_base = dart.optimizer.Solver
        filter_base = dart.collision.BodyNodeCollisionFilter
        osg = dart.gui.osg
        methods = ("refresh", "customPreRefresh", "customPostRefresh",
                   "customPreStep", "customPostStep")
        calls = []
        def callback(self):
            calls.append(self)
        nodes = [type("Node", (base,), dict.fromkeys(methods, callback))()
                 for base in (osg.WorldNode, osg.RealTimeWorldNode)]
        """,
        """
        assert not solver_base.solve(solver)
        assert solver_base.getType(solver) == "shutdown solver"
        assert solver_base.clone(solver) is not None
        assert filter_base.ignoresCollision(filter, None, None)
        assert problem.getObjective() is function
        native_solver.solve()
        assert "eval" in function_calls and "gradient" in function_calls
        for node in nodes:
            for method in methods:
                getattr(osg.WorldNode, method)(node)
        assert len(calls) == 10
        """,
    )


if __name__ == "__main__":
    test_python_analytical_ik_during_finalization()
    test_optimizer_callbacks_during_finalization()
    test_raycast_filter_during_finalization()
    test_virtual_callbacks_during_finalization()
    print("All dartpy finalization subprocess tests passed")
