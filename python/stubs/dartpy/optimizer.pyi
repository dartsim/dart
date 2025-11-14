from __future__ import annotations
import numpy
import typing
__all__: list[str] = ['Function', 'GradientDescentSolver', 'GradientDescentSolverProperties', 'GradientDescentSolverUniqueProperties', 'ModularFunction', 'MultiFunction', 'NloptSolver', 'NullFunction', 'Problem', 'Solver', 'SolverProperties']
M = typing.TypeVar("M", bound=int)
class Function:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, name: str) -> None:
        ...
    def getName(self) -> str:
        ...
    def setName(self, newName: str) -> None:
        ...
class GradientDescentSolver(Solver):
    Type: typing.ClassVar[str] = 'GradientDescentSolver'
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, properties: GradientDescentSolverProperties) -> None:
        ...
    @typing.overload
    def __init__(self, problem: ...) -> None:
        ...
    def clampToBoundary(self, x: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def clone(self) -> Solver:
        ...
    def getDefaultConstraintWeight(self) -> float:
        ...
    def getGradientDescentProperties(self) -> GradientDescentSolverProperties:
        ...
    def getLastConfiguration(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def getLastNumIterations(self) -> int:
        ...
    def getMaxAttempts(self) -> int:
        ...
    def getMaxPerturbationFactor(self) -> float:
        ...
    def getPerturbationStep(self) -> int:
        ...
    def getStepSize(self) -> float:
        ...
    def getType(self) -> str:
        ...
    def randomizeConfiguration(self, x: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def setDefaultConstraintWeight(self, newDefault: float) -> None:
        ...
    def setMaxAttempts(self, maxAttempts: int) -> None:
        ...
    def setMaxPerturbationFactor(self, factor: float) -> None:
        ...
    def setPerturbationStep(self, step: int) -> None:
        ...
    @typing.overload
    def setProperties(self, properties: GradientDescentSolverProperties) -> None:
        ...
    @typing.overload
    def setProperties(self, properties: GradientDescentSolverUniqueProperties) -> None:
        ...
    def setStepSize(self, newMultiplier: float) -> None:
        ...
    def solve(self) -> bool:
        ...
class GradientDescentSolverProperties(SolverProperties, GradientDescentSolverUniqueProperties):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, solverProperties: SolverProperties) -> None:
        ...
    @typing.overload
    def __init__(self, solverProperties: SolverProperties, descentProperties: GradientDescentSolverUniqueProperties) -> None:
        ...
class GradientDescentSolverUniqueProperties:
    mDefaultConstraintWeight: float
    mEqConstraintWeights: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    mIneqConstraintWeights: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    mMaxAttempts: int
    mMaxPerturbationFactor: float
    mMaxRandomizationStep: float
    mPerturbationStep: int
    mStepSize: float
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, stepMultiplier: float) -> None:
        ...
    @typing.overload
    def __init__(self, stepMultiplier: float, maxAttempts: int) -> None:
        ...
    @typing.overload
    def __init__(self, stepMultiplier: float, maxAttempts: int, perturbationStep: int) -> None:
        ...
    @typing.overload
    def __init__(self, stepMultiplier: float, maxAttempts: int, perturbationStep: int, maxPerturbationFactor: float) -> None:
        ...
    @typing.overload
    def __init__(self, stepMultiplier: float, maxAttempts: int, perturbationStep: int, maxPerturbationFactor: float, maxRandomizationStep: float) -> None:
        ...
    @typing.overload
    def __init__(self, stepMultiplier: float, maxAttempts: int, perturbationStep: int, maxPerturbationFactor: float, maxRandomizationStep: float, defaultConstraintWeight: float) -> None:
        ...
    @typing.overload
    def __init__(self, stepMultiplier: float, maxAttempts: int, perturbationStep: int, maxPerturbationFactor: float, maxRandomizationStep: float, defaultConstraintWeight: float, eqConstraintWeights: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, stepMultiplier: float, maxAttempts: int, perturbationStep: int, maxPerturbationFactor: float, maxRandomizationStep: float, defaultConstraintWeight: float, eqConstraintWeights: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], ineqConstraintWeights: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
class ModularFunction(Function):
    @typing.overload
    def clearCostFunction(self) -> None:
        ...
    @typing.overload
    def clearCostFunction(self, printWarning: bool) -> None:
        ...
    def clearGradientFunction(self) -> None:
        ...
    def clearHessianFunction(self) -> None:
        ...
    def eval(self, x: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    def setCostFunction(self, cost: ...) -> None:
        ...
    def setGradientFunction(self, gradient: ...) -> None:
        ...
    def setHessianFunction(self, hessian: ...) -> None:
        ...
class MultiFunction:
    pass
class NloptSolver(Solver):
    class Algorithm:
        """
        Members:
        
          GN_DIRECT
        
          GN_DIRECT_L
        
          GN_DIRECT_L_RAND
        
          GN_DIRECT_NOSCAL
        
          GN_DIRECT_L_NOSCAL
        
          GN_DIRECT_L_RAND_NOSCAL
        
          GN_ORIG_DIRECT
        
          GN_ORIG_DIRECT_L
        
          GD_STOGO
        
          GD_STOGO_RAND
        
          LD_LBFGS
        
          LN_PRAXIS
        
          LD_VAR1
        
          LD_VAR2
        
          LD_TNEWTON
        
          LD_TNEWTON_RESTART
        
          LD_TNEWTON_PRECOND
        
          LD_TNEWTON_PRECOND_RESTART
        
          GN_CRS2_LM
        
          GN_MLSL
        
          GD_MLSL
        
          GN_MLSL_LDS
        
          GD_MLSL_LDS
        
          LD_MMA
        
          LN_COBYLA
        
          LN_NEWUOA
        
          LN_NEWUOA_BOUND
        
          LN_NELDERMEAD
        
          LN_SBPLX
        
          LN_AUGLAG
        
          LD_AUGLAG
        
          LN_AUGLAG_EQ
        
          LD_AUGLAG_EQ
        
          LN_BOBYQA
        
          GN_ISRES
        
          AUGLAG
        
          AUGLAG_EQ
        
          G_MLSL
        
          G_MLSL_LDS
        
          LD_SLSQP
        
          LD_CCSAQ
        
          GN_ESCH
        """
        AUGLAG: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.AUGLAG: 35>
        AUGLAG_EQ: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.AUGLAG_EQ: 36>
        GD_MLSL: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GD_MLSL: 20>
        GD_MLSL_LDS: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GD_MLSL_LDS: 22>
        GD_STOGO: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GD_STOGO: 8>
        GD_STOGO_RAND: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GD_STOGO_RAND: 9>
        GN_CRS2_LM: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GN_CRS2_LM: 18>
        GN_DIRECT: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GN_DIRECT: 0>
        GN_DIRECT_L: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GN_DIRECT_L: 1>
        GN_DIRECT_L_NOSCAL: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GN_DIRECT_L_NOSCAL: 4>
        GN_DIRECT_L_RAND: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GN_DIRECT_L_RAND: 2>
        GN_DIRECT_L_RAND_NOSCAL: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GN_DIRECT_L_RAND_NOSCAL: 5>
        GN_DIRECT_NOSCAL: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GN_DIRECT_NOSCAL: 3>
        GN_ESCH: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GN_ESCH: 41>
        GN_ISRES: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GN_ISRES: 34>
        GN_MLSL: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GN_MLSL: 19>
        GN_MLSL_LDS: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GN_MLSL_LDS: 21>
        GN_ORIG_DIRECT: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GN_ORIG_DIRECT: 6>
        GN_ORIG_DIRECT_L: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.GN_ORIG_DIRECT_L: 7>
        G_MLSL: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.G_MLSL: 37>
        G_MLSL_LDS: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.G_MLSL_LDS: 38>
        LD_AUGLAG: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LD_AUGLAG: 30>
        LD_AUGLAG_EQ: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LD_AUGLAG_EQ: 32>
        LD_CCSAQ: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LD_CCSAQ: 40>
        LD_LBFGS: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LD_LBFGS: 10>
        LD_MMA: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LD_MMA: 23>
        LD_SLSQP: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LD_SLSQP: 39>
        LD_TNEWTON: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LD_TNEWTON: 14>
        LD_TNEWTON_PRECOND: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LD_TNEWTON_PRECOND: 16>
        LD_TNEWTON_PRECOND_RESTART: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LD_TNEWTON_PRECOND_RESTART: 17>
        LD_TNEWTON_RESTART: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LD_TNEWTON_RESTART: 15>
        LD_VAR1: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LD_VAR1: 12>
        LD_VAR2: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LD_VAR2: 13>
        LN_AUGLAG: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LN_AUGLAG: 29>
        LN_AUGLAG_EQ: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LN_AUGLAG_EQ: 31>
        LN_BOBYQA: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LN_BOBYQA: 33>
        LN_COBYLA: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LN_COBYLA: 24>
        LN_NELDERMEAD: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LN_NELDERMEAD: 27>
        LN_NEWUOA: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LN_NEWUOA: 25>
        LN_NEWUOA_BOUND: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LN_NEWUOA_BOUND: 26>
        LN_PRAXIS: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LN_PRAXIS: 11>
        LN_SBPLX: typing.ClassVar[NloptSolver.Algorithm]  # value = <Algorithm.LN_SBPLX: 28>
        __members__: typing.ClassVar[dict[str, NloptSolver.Algorithm]]  # value = {'GN_DIRECT': <Algorithm.GN_DIRECT: 0>, 'GN_DIRECT_L': <Algorithm.GN_DIRECT_L: 1>, 'GN_DIRECT_L_RAND': <Algorithm.GN_DIRECT_L_RAND: 2>, 'GN_DIRECT_NOSCAL': <Algorithm.GN_DIRECT_NOSCAL: 3>, 'GN_DIRECT_L_NOSCAL': <Algorithm.GN_DIRECT_L_NOSCAL: 4>, 'GN_DIRECT_L_RAND_NOSCAL': <Algorithm.GN_DIRECT_L_RAND_NOSCAL: 5>, 'GN_ORIG_DIRECT': <Algorithm.GN_ORIG_DIRECT: 6>, 'GN_ORIG_DIRECT_L': <Algorithm.GN_ORIG_DIRECT_L: 7>, 'GD_STOGO': <Algorithm.GD_STOGO: 8>, 'GD_STOGO_RAND': <Algorithm.GD_STOGO_RAND: 9>, 'LD_LBFGS': <Algorithm.LD_LBFGS: 10>, 'LN_PRAXIS': <Algorithm.LN_PRAXIS: 11>, 'LD_VAR1': <Algorithm.LD_VAR1: 12>, 'LD_VAR2': <Algorithm.LD_VAR2: 13>, 'LD_TNEWTON': <Algorithm.LD_TNEWTON: 14>, 'LD_TNEWTON_RESTART': <Algorithm.LD_TNEWTON_RESTART: 15>, 'LD_TNEWTON_PRECOND': <Algorithm.LD_TNEWTON_PRECOND: 16>, 'LD_TNEWTON_PRECOND_RESTART': <Algorithm.LD_TNEWTON_PRECOND_RESTART: 17>, 'GN_CRS2_LM': <Algorithm.GN_CRS2_LM: 18>, 'GN_MLSL': <Algorithm.GN_MLSL: 19>, 'GD_MLSL': <Algorithm.GD_MLSL: 20>, 'GN_MLSL_LDS': <Algorithm.GN_MLSL_LDS: 21>, 'GD_MLSL_LDS': <Algorithm.GD_MLSL_LDS: 22>, 'LD_MMA': <Algorithm.LD_MMA: 23>, 'LN_COBYLA': <Algorithm.LN_COBYLA: 24>, 'LN_NEWUOA': <Algorithm.LN_NEWUOA: 25>, 'LN_NEWUOA_BOUND': <Algorithm.LN_NEWUOA_BOUND: 26>, 'LN_NELDERMEAD': <Algorithm.LN_NELDERMEAD: 27>, 'LN_SBPLX': <Algorithm.LN_SBPLX: 28>, 'LN_AUGLAG': <Algorithm.LN_AUGLAG: 29>, 'LD_AUGLAG': <Algorithm.LD_AUGLAG: 30>, 'LN_AUGLAG_EQ': <Algorithm.LN_AUGLAG_EQ: 31>, 'LD_AUGLAG_EQ': <Algorithm.LD_AUGLAG_EQ: 32>, 'LN_BOBYQA': <Algorithm.LN_BOBYQA: 33>, 'GN_ISRES': <Algorithm.GN_ISRES: 34>, 'AUGLAG': <Algorithm.AUGLAG: 35>, 'AUGLAG_EQ': <Algorithm.AUGLAG_EQ: 36>, 'G_MLSL': <Algorithm.G_MLSL: 37>, 'G_MLSL_LDS': <Algorithm.G_MLSL_LDS: 38>, 'LD_SLSQP': <Algorithm.LD_SLSQP: 39>, 'LD_CCSAQ': <Algorithm.LD_CCSAQ: 40>, 'GN_ESCH': <Algorithm.GN_ESCH: 41>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, properties: SolverProperties) -> None:
        ...
    @typing.overload
    def __init__(self, properties: SolverProperties, alg: NloptSolver.Algorithm) -> None:
        ...
    @typing.overload
    def __init__(self, problem: ...) -> None:
        ...
    @typing.overload
    def __init__(self, problem: ..., alg: NloptSolver.Algorithm) -> None:
        ...
    def clone(self) -> Solver:
        ...
    def getAlgorithm(self) -> NloptSolver.Algorithm:
        ...
    def getLastConfiguration(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def getType(self) -> str:
        ...
    def setAlgorithm(self, alg: NloptSolver.Algorithm) -> None:
        ...
    def solve(self) -> bool:
        ...
class NullFunction(Function):
    def eval(self, arg0_: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
class Problem:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, dim: int) -> None:
        ...
    def addEqConstraint(self, eqConst: Function) -> None:
        ...
    def addIneqConstraint(self, ineqConst: Function) -> None:
        ...
    def addSeed(self, seed: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def clearAllSeeds(self) -> None:
        ...
    def getDimension(self) -> int:
        ...
    def getEqConstraint(self, idx: int) -> Function:
        ...
    def getIneqConstraint(self, idx: int) -> Function:
        ...
    def getInitialGuess(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def getNumEqConstraints(self) -> int:
        ...
    def getNumIneqConstraints(self) -> int:
        ...
    def getObjective(self) -> Function:
        ...
    def getOptimalSolution(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def getOptimumValue(self) -> float:
        ...
    def removeAllEqConstraints(self) -> None:
        ...
    def removeAllIneqConstraints(self) -> None:
        ...
    def removeEqConstraint(self, eqConst: Function) -> None:
        ...
    def removeIneqConstraint(self, ineqConst: Function) -> None:
        ...
    def setDimension(self, dim: int) -> None:
        ...
    def setInitialGuess(self, initGuess: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def setLowerBounds(self, lb: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def setObjective(self, obj: Function) -> None:
        ...
    def setOptimalSolution(self, optParam: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def setOptimumValue(self, val: float) -> None:
        ...
    def setUpperBounds(self, ub: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
class Solver:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, properties: SolverProperties) -> None:
        ...
    @typing.overload
    def __init__(self, problem: ...) -> None:
        ...
    def clone(self) -> Solver:
        ...
    def getIterationsPerPrint(self) -> int:
        ...
    def getNumMaxIterations(self) -> int:
        ...
    def getPrintFinalResult(self) -> bool:
        ...
    def getProblem(self) -> ...:
        ...
    def getResultFileName(self) -> str:
        ...
    def getTolerance(self) -> float:
        ...
    def getType(self) -> str:
        ...
    def setIterationsPerPrint(self, newRatio: int) -> None:
        ...
    def setNumMaxIterations(self, newMax: int) -> None:
        ...
    def setOutStream(self, os: ...) -> None:
        ...
    def setPrintFinalResult(self, print: bool) -> None:
        ...
    def setProblem(self, newProblem: ...) -> None:
        ...
    def setProperties(self, properties: SolverProperties) -> None:
        ...
    def setResultFileName(self, resultFile: str) -> None:
        ...
    def setTolerance(self, newTolerance: float) -> None:
        ...
    def solve(self) -> bool:
        ...
class SolverProperties:
    mIterationsPerPrint: int
    mNumMaxIterations: int
    mOutStream: ...
    mPrintFinalResult: bool
    mProblem: ...
    mResultFile: str
    mTolerance: float
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, problem: ...) -> None:
        ...
    @typing.overload
    def __init__(self, problem: ..., tolerance: float) -> None:
        ...
    @typing.overload
    def __init__(self, problem: ..., tolerance: float, numMaxIterations: int) -> None:
        ...
    @typing.overload
    def __init__(self, problem: ..., tolerance: float, numMaxIterations: int, iterationsPerPrint: int) -> None:
        ...
    @typing.overload
    def __init__(self, problem: ..., tolerance: float, numMaxIterations: int, iterationsPerPrint: int, ostream: ...) -> None:
        ...
    @typing.overload
    def __init__(self, problem: ..., tolerance: float, numMaxIterations: int, iterationsPerPrint: int, ostream: ..., printFinalResult: bool) -> None:
        ...
    @typing.overload
    def __init__(self, problem: ..., tolerance: float, numMaxIterations: int, iterationsPerPrint: int, ostream: ..., printFinalResult: bool, resultFile: str) -> None:
        ...
