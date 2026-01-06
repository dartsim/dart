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
    # The helper is exposed as 'get_solver_type_wrapper' and promoted to dartpy.
    get_solver_type = getattr(dart, "get_solver_type_wrapper", None)

    if get_solver_type:
        assert get_solver_type(solver) == "CustomSolverType"
        
        # Call it again to check caching logic (mTypeCacheInitialized)
        assert get_solver_type(solver) == "CustomSolverType"
    else:
        pytest.skip("dart.get_solver_type_wrapper helper not available")

if __name__ == "__main__":
    pytest.main()
