import dartpy as dart
import pytest
import warnings

class CustomSolver(dart.Solver):
    def __init__(self):
        super(CustomSolver, self).__init__()
        self.solve_called = False

    def solve(self):
        self.solve_called = True
        return True

    def getType(self):
        return "CustomSolverType"

    def clone(self):
        return CustomSolver()

def test_solver_trampoline_getType():
    solver = CustomSolver()
    
    # This calls the Python implementation directly
    assert solver.getType() == "CustomSolverType"
    
    # This calls the C++ helper, which calls C++ Solver::getType(), 
    # which should go through the trampoline and call Python's getType
    # This exercises the caching logic in the trampoline.
    get_solver_type = getattr(dart, "_getSolverType", None)
    if get_solver_type is None:
         # Try dart.optimizer
         with warnings.catch_warnings():
             warnings.simplefilter("ignore", DeprecationWarning)
             optimizer = getattr(dart, "optimizer", None)
             if optimizer:
                 get_solver_type = getattr(optimizer, "_getSolverType", None)

    if get_solver_type:
        assert get_solver_type(solver) == "CustomSolverType"
        
        # Call it again to check caching logic (mTypeCacheInitialized)
        assert get_solver_type(solver) == "CustomSolverType"
    else:
        pytest.skip("dart._getSolverType helper not available")

if __name__ == "__main__":
    pytest.main()
