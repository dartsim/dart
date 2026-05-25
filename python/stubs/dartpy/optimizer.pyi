from __future__ import annotations

__all__: list[str] = [
    "Function",
    "GradientDescentSolver",
    "GradientDescentSolverProperties",
    "GradientDescentSolverUniqueProperties",
    "NullFunction",
    "Problem",
    "Solver",
    "SolverProperties",
    "get_solver_type_wrapper",
]


from typing import Any, overload


class Function:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, name: str) -> None: ...

    def setName(*args, **kwargs) -> Any: ...

    def getName(*args, **kwargs) -> Any: ...

    get_name = getName

    set_name = setName

class NullFunction(Function):
    def __init__(self, name: str) -> None: ...

    def eval(self, x: object) -> float: ...

class Problem:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, dim: int) -> None: ...

    def setDimension(*args, **kwargs) -> Any: ...

    def getDimension(*args, **kwargs) -> Any: ...

    def setInitialGuess(*args, **kwargs) -> Any: ...

    def getInitialGuess(*args, **kwargs) -> Any: ...

    def addSeed(*args, **kwargs) -> Any: ...

    def clearAllSeeds(*args, **kwargs) -> Any: ...

    def setLowerBounds(*args, **kwargs) -> Any: ...

    def setUpperBounds(*args, **kwargs) -> Any: ...

    def setObjective(*args, **kwargs) -> Any: ...

    def getObjective(*args, **kwargs) -> Any: ...

    def addEqConstraint(*args, **kwargs) -> Any: ...

    def addIneqConstraint(*args, **kwargs) -> Any: ...

    def getNumEqConstraints(*args, **kwargs) -> Any: ...

    def getNumIneqConstraints(*args, **kwargs) -> Any: ...

    def getEqConstraint(*args, **kwargs) -> Any: ...

    def getIneqConstraint(*args, **kwargs) -> Any: ...

    def removeEqConstraint(*args, **kwargs) -> Any: ...

    def removeIneqConstraint(*args, **kwargs) -> Any: ...

    def removeAllEqConstraints(*args, **kwargs) -> Any: ...

    def removeAllIneqConstraints(*args, **kwargs) -> Any: ...

    def setOptimumValue(*args, **kwargs) -> Any: ...

    def getOptimumValue(*args, **kwargs) -> Any: ...

    def setOptimalSolution(*args, **kwargs) -> Any: ...

    def getOptimalSolution(*args, **kwargs) -> Any: ...

    add_eq_constraint = addEqConstraint

    add_ineq_constraint = addIneqConstraint

    add_seed = addSeed

    clear_all_seeds = clearAllSeeds

    get_dimension = getDimension

    get_eq_constraint = getEqConstraint

    get_ineq_constraint = getIneqConstraint

    get_initial_guess = getInitialGuess

    get_num_eq_constraints = getNumEqConstraints

    get_num_ineq_constraints = getNumIneqConstraints

    get_objective = getObjective

    get_optimal_solution = getOptimalSolution

    get_optimum_value = getOptimumValue

    remove_all_eq_constraints = removeAllEqConstraints

    remove_all_ineq_constraints = removeAllIneqConstraints

    remove_eq_constraint = removeEqConstraint

    remove_ineq_constraint = removeIneqConstraint

    set_dimension = setDimension

    set_initial_guess = setInitialGuess

    set_lower_bounds = setLowerBounds

    set_objective = setObjective

    set_optimal_solution = setOptimalSolution

    set_optimum_value = setOptimumValue

    set_upper_bounds = setUpperBounds

class SolverProperties:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, problem: Problem) -> None: ...

    @overload
    def __init__(self, problem: Problem, tolerance: float) -> None: ...

    @overload
    def __init__(self, problem: Problem, tolerance: float, num_max_iterations: int) -> None: ...

    @overload
    def __init__(self, problem: Problem, tolerance: float, num_max_iterations: int, iterations_per_print: int) -> None: ...

    @overload
    def __init__(self, problem: Problem, tolerance: float, num_max_iterations: int, iterations_per_print: int, ostream: "std::ostream") -> None: ...

    @overload
    def __init__(self, problem: Problem, tolerance: float, num_max_iterations: int, iterations_per_print: int, ostream: "std::ostream", print_final_result: bool) -> None: ...

    @overload
    def __init__(self, problem: Problem, tolerance: float, num_max_iterations: int, iterations_per_print: int, ostream: "std::ostream", print_final_result: bool, result_file: str) -> None: ...

    @property
    def mProblem(self) -> Problem: ...

    @mProblem.setter
    def mProblem(self, arg: Problem, /) -> None: ...

    @property
    def mTolerance(self) -> float: ...

    @mTolerance.setter
    def mTolerance(self, arg: float, /) -> None: ...

    @property
    def mNumMaxIterations(self) -> int: ...

    @mNumMaxIterations.setter
    def mNumMaxIterations(self, arg: int, /) -> None: ...

    @property
    def mIterationsPerPrint(self) -> int: ...

    @mIterationsPerPrint.setter
    def mIterationsPerPrint(self, arg: int, /) -> None: ...

    @property
    def mOutStream(self) -> "std::ostream": ...

    @mOutStream.setter
    def mOutStream(self, arg: "std::ostream", /) -> None: ...

    @property
    def mPrintFinalResult(self) -> bool: ...

    @mPrintFinalResult.setter
    def mPrintFinalResult(self, arg: bool, /) -> None: ...

    @property
    def mResultFile(self) -> str: ...

    @mResultFile.setter
    def mResultFile(self, arg: str, /) -> None: ...

    @property
    def m_iterations_per_print(self) -> int: ...

    @m_iterations_per_print.setter
    def m_iterations_per_print(self, arg: int, /) -> None: ...

    @property
    def m_num_max_iterations(self) -> int: ...

    @m_num_max_iterations.setter
    def m_num_max_iterations(self, arg: int, /) -> None: ...

    @property
    def m_out_stream(self) -> "std::ostream": ...

    @m_out_stream.setter
    def m_out_stream(self, arg: "std::ostream", /) -> None: ...

    @property
    def m_print_final_result(self) -> bool: ...

    @m_print_final_result.setter
    def m_print_final_result(self, arg: bool, /) -> None: ...

    @property
    def m_problem(self) -> Problem: ...

    @m_problem.setter
    def m_problem(self, arg: Problem, /) -> None: ...

    @property
    def m_result_file(self) -> str: ...

    @m_result_file.setter
    def m_result_file(self, arg: str, /) -> None: ...

    @property
    def m_tolerance(self) -> float: ...

    @m_tolerance.setter
    def m_tolerance(self, arg: float, /) -> None: ...

class Solver:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, properties: SolverProperties) -> None: ...

    @overload
    def __init__(self, problem: Problem) -> None: ...

    def solve(self) -> bool: ...

    def getType(*args, **kwargs) -> Any: ...

    def clone(self) -> Solver: ...

    def setProperties(*args, **kwargs) -> Any: ...

    def getProperties(*args, **kwargs) -> Any: ...

    def setProblem(*args, **kwargs) -> Any: ...

    def getProblem(*args, **kwargs) -> Any: ...

    def setTolerance(*args, **kwargs) -> Any: ...

    def getTolerance(*args, **kwargs) -> Any: ...

    def setNumMaxIterations(*args, **kwargs) -> Any: ...

    def getNumMaxIterations(*args, **kwargs) -> Any: ...

    def setIterationsPerPrint(*args, **kwargs) -> Any: ...

    def getIterationsPerPrint(*args, **kwargs) -> Any: ...

    def setOutStream(*args, **kwargs) -> Any: ...

    def setPrintFinalResult(*args, **kwargs) -> Any: ...

    def getPrintFinalResult(*args, **kwargs) -> Any: ...

    def setResultFileName(*args, **kwargs) -> Any: ...

    def getResultFileName(*args, **kwargs) -> Any: ...

    get_iterations_per_print = getIterationsPerPrint

    get_num_max_iterations = getNumMaxIterations

    get_print_final_result = getPrintFinalResult

    get_problem = getProblem

    get_properties = getProperties

    get_result_file_name = getResultFileName

    get_tolerance = getTolerance

    get_type = getType

    set_iterations_per_print = setIterationsPerPrint

    set_num_max_iterations = setNumMaxIterations

    set_out_stream = setOutStream

    set_print_final_result = setPrintFinalResult

    set_problem = setProblem

    set_properties = setProperties

    set_result_file_name = setResultFileName

    set_tolerance = setTolerance

def get_solver_type_wrapper(solver: Solver) -> str: ...

class GradientDescentSolverUniqueProperties:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, step_multiplier: float) -> None: ...

    @overload
    def __init__(self, step_multiplier: float, max_attempts: int) -> None: ...

class GradientDescentSolverProperties(SolverProperties):
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, solver_properties: SolverProperties, gradient_properties: GradientDescentSolverUniqueProperties = ...) -> None: ...

    @property
    def m_iterations_per_print(self) -> int: ...

    @m_iterations_per_print.setter
    def m_iterations_per_print(self, arg: int, /) -> None: ...

    @property
    def m_num_max_iterations(self) -> int: ...

    @m_num_max_iterations.setter
    def m_num_max_iterations(self, arg: int, /) -> None: ...

    @property
    def m_out_stream(self) -> "std::ostream": ...

    @m_out_stream.setter
    def m_out_stream(self, arg: "std::ostream", /) -> None: ...

    @property
    def m_print_final_result(self) -> bool: ...

    @m_print_final_result.setter
    def m_print_final_result(self, arg: bool, /) -> None: ...

    @property
    def m_problem(self) -> Problem: ...

    @m_problem.setter
    def m_problem(self, arg: Problem, /) -> None: ...

    @property
    def m_result_file(self) -> str: ...

    @m_result_file.setter
    def m_result_file(self, arg: str, /) -> None: ...

    @property
    def m_tolerance(self) -> float: ...

    @m_tolerance.setter
    def m_tolerance(self, arg: float, /) -> None: ...

class GradientDescentSolver(Solver):
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, properties: GradientDescentSolverProperties) -> None: ...

    @overload
    def __init__(self, problem: Problem) -> None: ...

    def solve(self) -> bool: ...

    def getLastConfiguration(*args, **kwargs) -> Any: ...

    def getType(*args, **kwargs) -> Any: ...

    def clone(self) -> Solver: ...

    def setProperties(*args, **kwargs) -> Any: ...

    def getProperties(*args, **kwargs) -> Any: ...

    def setStepSize(*args, **kwargs) -> Any: ...

    def getStepSize(*args, **kwargs) -> Any: ...

    def getTolerance(*args, **kwargs) -> Any: ...

    get_iterations_per_print = getIterationsPerPrint

    def getIterationsPerPrint(*args, **kwargs) -> Any: ...

    get_last_configuration = getLastConfiguration

    get_num_max_iterations = getNumMaxIterations

    def getNumMaxIterations(*args, **kwargs) -> Any: ...

    get_print_final_result = getPrintFinalResult

    def getPrintFinalResult(*args, **kwargs) -> Any: ...

    get_problem = getProblem

    def getProblem(*args, **kwargs) -> Any: ...

    get_properties = getProperties

    get_result_file_name = getResultFileName

    def getResultFileName(*args, **kwargs) -> Any: ...

    get_step_size = getStepSize

    get_tolerance = getTolerance

    get_type = getType

    set_iterations_per_print = setIterationsPerPrint

    def setIterationsPerPrint(*args, **kwargs) -> Any: ...

    set_num_max_iterations = setNumMaxIterations

    def setNumMaxIterations(*args, **kwargs) -> Any: ...

    set_out_stream = setOutStream

    def setOutStream(*args, **kwargs) -> Any: ...

    set_print_final_result = setPrintFinalResult

    def setPrintFinalResult(*args, **kwargs) -> Any: ...

    set_problem = setProblem

    def setProblem(*args, **kwargs) -> Any: ...

    set_properties = setProperties

    set_result_file_name = setResultFileName

    def setResultFileName(*args, **kwargs) -> Any: ...

    set_step_size = setStepSize

    set_tolerance = setTolerance

    def setTolerance(*args, **kwargs) -> Any: ...
