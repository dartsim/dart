from __future__ import annotations

__all__: list[str] = [
    "AdmmSolver",
    "AdmmSolverParameters",
    "AngleAxis",
    "ApgdSolver",
    "ApgdSolverParameters",
    "BaraffSolver",
    "BgsSolver",
    "BgsSolverParameters",
    "BlockedJacobiSolver",
    "BlockedJacobiSolverParameters",
    "BoxedSemiSmoothNewtonSolver",
    "BoxedSemiSmoothNewtonSolverParameters",
    "DantzigSolver",
    "DirectSolver",
    "FischerBurmeisterNewtonSolver",
    "FischerBurmeisterNewtonSolverParameters",
    "InteriorPointSolver",
    "InteriorPointSolverParameters",
    "Isometry3",
    "JacobiSolver",
    "JacobiSolverParameters",
    "LcpOptions",
    "LcpProblem",
    "LcpProblemType",
    "LcpResult",
    "LcpSolver",
    "LcpSolverStatus",
    "LemkeSolver",
    "MinimumMapNewtonSolver",
    "MinimumMapNewtonSolverParameters",
    "MprgpSolver",
    "MprgpSolverParameters",
    "NncgSolver",
    "NncgSolverParameters",
    "PenalizedFischerBurmeisterNewtonSolver",
    "PenalizedFischerBurmeisterNewtonSolverParameters",
    "PgsSolver",
    "PgsSolverParameters",
    "Quaternion",
    "Random",
    "RedBlackGaussSeidelSolver",
    "RedBlackGaussSeidelSolverParameters",
    "SE3",
    "SE3Tangent",
    "SO3",
    "SO3Tangent",
    "SapSolver",
    "SapSolverParameters",
    "ShockPropagationSolver",
    "ShockPropagationSolverParameters",
    "StaggeringSolver",
    "SubspaceMinimizationSolver",
    "SubspaceMinimizationSolverParameters",
    "SymmetricPsorSolver",
    "SymmetricPsorSolverParameters",
    "TgsSolver",
    "TgsSolverParameters",
    "TriMesh",
    "deg2rad",
    "eps",
    "eulerXYXToMatrix",
    "eulerXYZToMatrix",
    "eulerXZXToMatrix",
    "eulerXZYToMatrix",
    "eulerYXYToMatrix",
    "eulerYXZToMatrix",
    "eulerYZXToMatrix",
    "eulerYZYToMatrix",
    "eulerZXYToMatrix",
    "eulerZXZToMatrix",
    "eulerZYXToMatrix",
    "eulerZYZToMatrix",
    "euler_xyx_to_matrix",
    "euler_xyz_to_matrix",
    "euler_xzx_to_matrix",
    "euler_xzy_to_matrix",
    "euler_yxy_to_matrix",
    "euler_yxz_to_matrix",
    "euler_yzx_to_matrix",
    "euler_yzy_to_matrix",
    "euler_zxy_to_matrix",
    "euler_zxz_to_matrix",
    "euler_zyx_to_matrix",
    "euler_zyz_to_matrix",
    "exp",
    "expAngular",
    "expMap",
    "expMapJac",
    "expMapRot",
    "expToQuat",
    "exp_angular",
    "exp_map",
    "exp_map_jac",
    "exp_map_rot",
    "exp_to_quat",
    "half_pi",
    "inf",
    "lcp_problem_type_to_string",
    "lcp_solver_status_to_string",
    "log",
    "matrixToEulerXYX",
    "matrixToEulerXYZ",
    "matrixToEulerXZY",
    "matrixToEulerYXZ",
    "matrixToEulerYZX",
    "matrixToEulerZXY",
    "matrixToEulerZYX",
    "matrix_to_euler_xyx",
    "matrix_to_euler_xyz",
    "matrix_to_euler_xzy",
    "matrix_to_euler_yxz",
    "matrix_to_euler_yzx",
    "matrix_to_euler_zxy",
    "matrix_to_euler_zyx",
    "max_val",
    "min_val",
    "phi",
    "pi",
    "pi_sq",
    "quatToExp",
    "quat_to_exp",
    "rad2deg",
    "rand_se3",
    "rand_se3_tangent",
    "rand_so3",
    "rand_so3_tangent",
    "two_pi",
    "verifyRotation",
    "verifyTransform",
    "verify_rotation",
    "verify_transform",
]

from enum import Enum
from typing import Annotated, Any, TypeAlias, overload

import numpy
from numpy.typing import NDArray

pi: float = ...

two_pi: float = ...

half_pi: float = ...

pi_sq: float = ...

phi: float = ...

inf: float = ...

max_val: float = ...

min_val: float = ...

eps: float = ...

def deg2rad(degrees: float) -> float: ...
def rad2deg(radians: float) -> float: ...

class Random:
    def __init__(self) -> None: ...
    def setSeed(*args, **kwargs): ...
    def getSeed(*args, **kwargs): ...
    @staticmethod
    def uniform(min: float, max: float) -> float: ...

    get_seed = getSeed

    set_seed = setSeed

def eulerXYXToMatrix(*args, **kwargs): ...
def eulerXYZToMatrix(*args, **kwargs): ...
def eulerXZXToMatrix(*args, **kwargs): ...
def eulerXZYToMatrix(*args, **kwargs): ...
def eulerYXYToMatrix(*args, **kwargs): ...
def eulerYXZToMatrix(*args, **kwargs): ...
def eulerYZXToMatrix(*args, **kwargs): ...
def eulerYZYToMatrix(*args, **kwargs): ...
def eulerZXYToMatrix(*args, **kwargs): ...
def eulerZYXToMatrix(*args, **kwargs): ...
def eulerZXZToMatrix(*args, **kwargs): ...
def eulerZYZToMatrix(*args, **kwargs): ...
def matrixToEulerXYX(*args, **kwargs): ...
def matrixToEulerXYZ(*args, **kwargs): ...
def matrixToEulerXZY(*args, **kwargs): ...
def matrixToEulerYXZ(*args, **kwargs): ...
def matrixToEulerYZX(*args, **kwargs): ...
def matrixToEulerZXY(*args, **kwargs): ...
def matrixToEulerZYX(*args, **kwargs): ...
def expMap(*args, **kwargs): ...
def expMapJac(*args, **kwargs): ...
def expMapRot(*args, **kwargs): ...
def expToQuat(*args, **kwargs): ...
def quatToExp(*args, **kwargs): ...
def expAngular(*args, **kwargs): ...
def verifyRotation(*args, **kwargs): ...
def verifyTransform(*args, **kwargs): ...

class LcpSolverStatus(Enum):
    SUCCESS: LcpSolverStatus
    FAILED: LcpSolverStatus
    MAX_ITERATIONS: LcpSolverStatus
    NUMERICAL_ERROR: LcpSolverStatus
    INVALID_PROBLEM: LcpSolverStatus
    DEGENERATE: LcpSolverStatus
    NOT_SOLVED: LcpSolverStatus

def lcp_solver_status_to_string(status: LcpSolverStatus) -> str: ...

class LcpProblemType(Enum):
    INVALID: LcpProblemType
    STANDARD: LcpProblemType
    BOXED: LcpProblemType
    FRICTION_INDEX: LcpProblemType

def lcp_problem_type_to_string(type: LcpProblemType) -> str: ...

class LcpResult:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, status: LcpSolverStatus) -> None: ...

    status: LcpSolverStatus
    iterations: int
    residual: float
    complementarity: float
    validated: bool
    message: str

    def succeeded(self) -> bool: ...

class LcpOptions:
    def __init__(self) -> None: ...

    max_iterations: int
    absolute_tolerance: float
    relative_tolerance: float
    complementarity_tolerance: float
    validate_solution: bool
    relaxation: float
    warm_start: bool
    early_termination: bool

    @staticmethod
    def with_relaxation(relaxation: float, max_iterations: int = ...) -> LcpOptions: ...
    @staticmethod
    def high_accuracy() -> LcpOptions: ...
    @staticmethod
    def real_time() -> LcpOptions: ...

class PgsSolverParameters:
    def __init__(self) -> None: ...

    epsilon_for_division: float
    randomize_constraint_order: bool

class SymmetricPsorSolverParameters:
    def __init__(self) -> None: ...

    epsilon_for_division: float

class JacobiSolverParameters:
    def __init__(self) -> None: ...

    epsilon_for_division: float
    worker_threads: int

class RedBlackGaussSeidelSolverParameters:
    def __init__(self) -> None: ...

    epsilon_for_division: float
    worker_threads: int

class BlockedJacobiSolverParameters:
    def __init__(self) -> None: ...

    block_sizes: list[int]
    worker_threads: int

class BgsSolverParameters:
    def __init__(self) -> None: ...

    block_sizes: list[int]

class NncgSolverParameters:
    def __init__(self) -> None: ...

    pgs_iterations: int
    restart_interval: int
    restart_threshold: float

class SubspaceMinimizationSolverParameters:
    def __init__(self) -> None: ...

    pgs_iterations: int
    active_set_tolerance: float

class ApgdSolverParameters:
    def __init__(self) -> None: ...

    epsilon_for_division: float
    adaptive_restart: bool
    restart_check_interval: int

class TgsSolverParameters:
    def __init__(self) -> None: ...

    epsilon_for_division: float

class MinimumMapNewtonSolverParameters:
    def __init__(self) -> None: ...

    max_line_search_steps: int
    step_reduction: float
    sufficient_decrease: float
    min_step: float
    max_gradient_descent_warm_start_steps: int
    max_gradient_descent_line_search_steps: int
    gradient_descent_step_reduction: float
    gradient_descent_sufficient_decrease: float
    gradient_descent_min_step: float
    max_pgs_warm_start_iterations: int
    pgs_warm_start_relaxation: float

class FischerBurmeisterNewtonSolverParameters:
    def __init__(self) -> None: ...

    smoothing_epsilon: float
    max_line_search_steps: int
    step_reduction: float
    sufficient_decrease: float
    min_step: float
    max_gradient_descent_warm_start_steps: int
    max_gradient_descent_line_search_steps: int
    gradient_descent_step_reduction: float
    gradient_descent_sufficient_decrease: float
    gradient_descent_min_step: float
    max_pgs_warm_start_iterations: int
    pgs_warm_start_relaxation: float

class PenalizedFischerBurmeisterNewtonSolverParameters:
    def __init__(self) -> None: ...

    smoothing_epsilon: float
    lambda_: float
    max_line_search_steps: int
    step_reduction: float
    sufficient_decrease: float
    min_step: float
    max_gradient_descent_warm_start_steps: int
    max_gradient_descent_line_search_steps: int
    gradient_descent_step_reduction: float
    gradient_descent_sufficient_decrease: float
    gradient_descent_min_step: float
    max_pgs_warm_start_iterations: int
    pgs_warm_start_relaxation: float

class AdmmSolverParameters:
    def __init__(self) -> None: ...

    rho_init: float
    mu_prox: float
    adaptive_rho_tolerance: float
    adaptive_rho: bool

class SapSolverParameters:
    def __init__(self) -> None: ...

    regularization: float
    armijos_parameter: float
    backtracking_factor: float
    max_line_search_iterations: int

class BoxedSemiSmoothNewtonSolverParameters:
    def __init__(self) -> None: ...

    max_line_search_steps: int
    step_reduction: float
    sufficient_decrease: float
    min_step: float
    jacobian_regularization: float
    max_pgs_warm_start_iterations: int
    pgs_warm_start_relaxation: float
    max_friction_index_exact_solve_dimension: int

class InteriorPointSolverParameters:
    def __init__(self) -> None: ...

    sigma: float
    step_scale: float

class MprgpSolverParameters:
    def __init__(self) -> None: ...

    symmetry_tolerance: float
    epsilon_for_division: float
    check_positive_definite: bool

class ShockPropagationSolverParameters:
    def __init__(self) -> None: ...

    block_sizes: list[int]
    layers: list[list[int]]

class LcpProblem:
    @overload
    def __init__(
        self,
        A: NDArray[numpy.float64],
        b: NDArray[numpy.float64],
    ) -> None: ...
    @overload
    def __init__(
        self,
        A: NDArray[numpy.float64],
        b: NDArray[numpy.float64],
        lo: NDArray[numpy.float64],
        hi: NDArray[numpy.float64],
    ) -> None: ...
    @overload
    def __init__(
        self,
        A: NDArray[numpy.float64],
        b: NDArray[numpy.float64],
        lo: NDArray[numpy.float64],
        hi: NDArray[numpy.float64],
        findex: NDArray[numpy.int32],
    ) -> None: ...

    A: NDArray[numpy.float64]
    b: NDArray[numpy.float64]
    lo: NDArray[numpy.float64]
    hi: NDArray[numpy.float64]
    findex: NDArray[numpy.int32]

    def size(self) -> int: ...
    def empty(self) -> bool: ...
    def is_valid(self) -> bool: ...
    def get_validation_message(self) -> str: ...
    def get_type(self, tolerance: float = ...) -> LcpProblemType: ...
    def is_standard_lcp(self, tolerance: float = ...) -> bool: ...
    def is_boxed_lcp(self) -> bool: ...
    def has_friction_index(self) -> bool: ...
    def get_friction_index_row_count(self) -> int: ...
    def get_friction_index_contact_count(self) -> int: ...

class LcpSolver:
    default_options: LcpOptions

    def solve(
        self,
        problem: LcpProblem,
        initial_guess: NDArray[numpy.float64] | None = ...,
        options: LcpOptions | None = ...,
    ) -> tuple[LcpResult, NDArray[numpy.float64]]: ...
    def get_name(self) -> str: ...
    def get_category(self) -> str: ...
    def supports_standard_lcp(self) -> bool: ...
    def supports_boxed_lcp(self) -> bool: ...
    def supports_friction_index(self) -> bool: ...
    def supports_problem(
        self, problem: LcpProblem, standard_tolerance: float | None = ...
    ) -> bool: ...

class DantzigSolver(LcpSolver):
    def __init__(self) -> None: ...

class LemkeSolver(LcpSolver):
    def __init__(self) -> None: ...

class BaraffSolver(LcpSolver):
    def __init__(self) -> None: ...

class DirectSolver(LcpSolver):
    def __init__(self) -> None: ...

class PgsSolver(LcpSolver):
    parameters: PgsSolverParameters

    def __init__(self) -> None: ...

class SymmetricPsorSolver(LcpSolver):
    parameters: SymmetricPsorSolverParameters

    def __init__(self) -> None: ...

class JacobiSolver(LcpSolver):
    parameters: JacobiSolverParameters

    def __init__(self) -> None: ...

class RedBlackGaussSeidelSolver(LcpSolver):
    parameters: RedBlackGaussSeidelSolverParameters

    def __init__(self) -> None: ...

class BlockedJacobiSolver(LcpSolver):
    parameters: BlockedJacobiSolverParameters

    def __init__(self) -> None: ...

class BgsSolver(LcpSolver):
    parameters: BgsSolverParameters

    def __init__(self) -> None: ...

class NncgSolver(LcpSolver):
    parameters: NncgSolverParameters

    def __init__(self) -> None: ...

class SubspaceMinimizationSolver(LcpSolver):
    parameters: SubspaceMinimizationSolverParameters

    def __init__(self) -> None: ...

class ApgdSolver(LcpSolver):
    parameters: ApgdSolverParameters

    def __init__(self) -> None: ...

class TgsSolver(LcpSolver):
    parameters: TgsSolverParameters

    def __init__(self) -> None: ...

class MinimumMapNewtonSolver(LcpSolver):
    parameters: MinimumMapNewtonSolverParameters

    def __init__(self) -> None: ...

class FischerBurmeisterNewtonSolver(LcpSolver):
    parameters: FischerBurmeisterNewtonSolverParameters

    def __init__(self) -> None: ...

class PenalizedFischerBurmeisterNewtonSolver(LcpSolver):
    parameters: PenalizedFischerBurmeisterNewtonSolverParameters

    def __init__(self) -> None: ...

class BoxedSemiSmoothNewtonSolver(LcpSolver):
    parameters: BoxedSemiSmoothNewtonSolverParameters

    def __init__(self) -> None: ...

class InteriorPointSolver(LcpSolver):
    parameters: InteriorPointSolverParameters

    def __init__(self) -> None: ...

class MprgpSolver(LcpSolver):
    parameters: MprgpSolverParameters

    def __init__(self) -> None: ...

class ShockPropagationSolver(LcpSolver):
    parameters: ShockPropagationSolverParameters

    def __init__(self) -> None: ...

class StaggeringSolver(LcpSolver):
    def __init__(self) -> None: ...

class AdmmSolver(LcpSolver):
    parameters: AdmmSolverParameters

    def __init__(self) -> None: ...

class SapSolver(LcpSolver):
    parameters: SapSolverParameters

    def __init__(self) -> None: ...

class Isometry3:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(
        self, matrix: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order="F")]
    ) -> None: ...
    @overload
    def __init__(
        self,
        rotation: Annotated[NDArray[numpy.float64], dict(shape=(3, 3), order="F")],
        translation: Annotated[NDArray[numpy.float64], dict(shape=(3), order="C")],
    ) -> None: ...
    @overload
    def __init__(
        self,
        rotation: Quaternion,
        translation: Annotated[NDArray[numpy.float64], dict(shape=(3), order="C")],
    ) -> None: ...
    def Identity(*args, **kwargs): ...
    def matrix(
        self,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order="F")]: ...
    def set_identity(self) -> None: ...
    def set_matrix(
        self, matrix: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order="F")]
    ) -> None: ...
    def translation(
        self,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order="C")]: ...
    def set_translation(self, translation: object) -> None: ...
    def rotation(
        self,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(3, 3), order="F")]: ...
    def set_rotation(
        self, rotation: Annotated[NDArray[numpy.float64], dict(shape=(3, 3), order="F")]
    ) -> None: ...
    def quaternion(self) -> Quaternion: ...
    def set_quaternion(self, quaternion: Quaternion) -> None: ...
    def __str__(self) -> str: ...
    @overload
    def multiply(self, other: Isometry3) -> Isometry3: ...
    @overload
    def multiply(
        self, position: object
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order="C")]: ...
    def inverse(self) -> Isometry3: ...
    def translate(self, shift: object) -> None: ...
    def pretranslate(self, shift: object) -> None: ...

    identity = Identity

class Quaternion:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(
        self, wxyz: Annotated[NDArray[numpy.float64], dict(shape=(4), order="C")]
    ) -> None: ...
    @overload
    def __init__(self, w: float, x: float, y: float, z: float) -> None: ...
    @overload
    def __init__(
        self, rotation: Annotated[NDArray[numpy.float64], dict(shape=(3, 3), order="F")]
    ) -> None: ...
    def Identity(*args, **kwargs): ...
    def w(self) -> float: ...
    def x(self) -> float: ...
    def y(self) -> float: ...
    def z(self) -> float: ...
    def xyz(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order="C")]: ...
    def wxyz(self) -> Annotated[NDArray[numpy.float64], dict(shape=(4), order="C")]: ...
    def set_wxyz(
        self, wxyz: Annotated[NDArray[numpy.float64], dict(shape=(4), order="C")]
    ) -> None: ...
    def rotation(
        self,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(3, 3), order="F")]: ...
    def set_rotation(
        self, rotation: Annotated[NDArray[numpy.float64], dict(shape=(3, 3), order="F")]
    ) -> None: ...
    def __str__(self) -> str: ...
    @overload
    def multiply(self, other: Quaternion) -> Quaternion: ...
    @overload
    def multiply(
        self, position: Annotated[NDArray[numpy.float64], dict(shape=(3), order="C")]
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order="C")]: ...
    def inverse(self) -> Quaternion: ...
    def conjugate(self) -> Quaternion: ...
    def to_rotation_matrix(
        self,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(3, 3), order="F")]: ...

    identity = Identity

class AngleAxis:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(
        self,
        angle: float,
        axis: Annotated[NDArray[numpy.float64], dict(shape=(3), order="C")],
    ) -> None: ...
    @overload
    def __init__(self, angle: float, axis: object) -> None: ...
    @overload
    def __init__(self, quaternion: Quaternion) -> None: ...
    def Identity(*args, **kwargs): ...
    def angle(self) -> float: ...
    def axis(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order="C")]: ...
    def to_rotation_matrix(
        self,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(3, 3), order="F")]: ...

    identity = Identity

class SO3:
    def __init__(self) -> None: ...
    def __eq__(self, arg: SO3, /) -> bool: ...
    def __ne__(self, arg: SO3, /) -> bool: ...
    def __mul__(self, arg: SO3, /) -> SO3: ...
    def __str__(
        self,
    ) -> "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >": ...
    def __repr__(
        self,
    ) -> "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >": ...
    def set_random(self) -> None: ...
    def is_approx(self, other: SO3, tol: float = ...) -> bool: ...
    def is_identity(self, tol: float = ...) -> bool: ...
    def inverse(self) -> SO3: ...
    def log(self, tol: float = ...) -> SO3Tangent: ...
    def inverse_in_place(self) -> SO3: ...
    def to_matrix(
        self,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(3, 3), order="F")]: ...

    Tangent: TypeAlias = SO3Tangent

class SE3:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(
        self,
        arg0: SO3,
        arg1: Annotated[NDArray[numpy.float64], dict(shape=(3), order="C")],
        /,
    ) -> None: ...
    def __eq__(self, arg: SE3, /) -> bool: ...
    def __ne__(self, arg: SE3, /) -> bool: ...
    def __mul__(self, arg: SE3, /) -> SE3: ...
    def __str__(
        self,
    ) -> "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >": ...
    def __repr__(
        self,
    ) -> "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >": ...
    def is_approx(self, other: SE3, tol: float = ...) -> bool: ...
    def is_identity(self, tol: float = ...) -> bool: ...
    def inverse(self) -> SE3: ...
    def log(self, tol: float = ...) -> SE3Tangent: ...
    def inverse_in_place(self) -> SE3: ...
    def to_matrix(
        self,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order="F")]: ...

    Tangent: TypeAlias = SE3Tangent

def rand_so3() -> SO3: ...
def rand_se3() -> SE3: ...

class SO3Tangent:
    def __init__(self) -> None: ...
    @staticmethod
    def zero() -> SO3Tangent: ...
    @staticmethod
    def random() -> SO3Tangent: ...
    def __eq__(self, arg: SO3Tangent, /) -> bool: ...
    def __ne__(self, arg: SO3Tangent, /) -> bool: ...
    def __str__(
        self,
    ) -> "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >": ...
    def __repr__(
        self,
    ) -> "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >": ...
    def is_approx(self, other: SO3Tangent, tol: float = ...) -> bool: ...
    def is_zero(self, tol: float = ...) -> bool: ...
    def exp(self, tol: float = ...) -> SO3: ...

class SE3Tangent:
    def __init__(self) -> None: ...
    @staticmethod
    def zero() -> SE3Tangent: ...
    @staticmethod
    def random() -> SE3Tangent: ...
    def __eq__(self, arg: SE3Tangent, /) -> bool: ...
    def __ne__(self, arg: SE3Tangent, /) -> bool: ...
    def __str__(
        self,
    ) -> "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >": ...
    def __repr__(
        self,
    ) -> "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >": ...
    def is_approx(self, other: SE3Tangent, tol: float = ...) -> bool: ...
    def is_zero(self, tol: float = ...) -> bool: ...
    def exp(self, tol: float = ...) -> SE3: ...

def rand_so3_tangent() -> SO3Tangent: ...
def rand_se3_tangent() -> SE3Tangent: ...
@overload
def exp(dx: SO3Tangent, tol: float = ...) -> SO3: ...
@overload
def exp(dx: SE3Tangent, tol: float = ...) -> SE3: ...
@overload
def log(x: SO3, tol: float = ...) -> SO3Tangent: ...
@overload
def log(x: SE3, tol: float = ...) -> SE3Tangent: ...

class TriMesh:
    def __init__(self) -> None: ...
    def addVertex(*args, **kwargs) -> Any: ...
    def addTriangle(*args, **kwargs) -> Any: ...
    def addVertexNormal(*args, **kwargs) -> Any: ...
    def reserveVertices(*args, **kwargs) -> Any: ...
    def reserveTriangles(*args, **kwargs) -> Any: ...
    def reserveVertexNormals(*args, **kwargs) -> Any: ...
    def getVertices(*args, **kwargs) -> Any: ...
    def getTriangles(*args, **kwargs) -> Any: ...
    def getVertexNormals(*args, **kwargs) -> Any: ...
    def hasVertexNormals(*args, **kwargs) -> Any: ...
    def computeVertexNormals(*args, **kwargs) -> Any: ...
    def getNumVertices(*args, **kwargs) -> Any: ...
    def getNumTriangles(*args, **kwargs) -> Any: ...
    def clear(self) -> None: ...

    add_triangle = addTriangle

    add_vertex = addVertex

    add_vertex_normal = addVertexNormal

    compute_vertex_normals = computeVertexNormals

    get_num_triangles = getNumTriangles

    get_num_vertices = getNumVertices

    get_triangles = getTriangles

    get_vertex_normals = getVertexNormals

    get_vertices = getVertices

    has_vertex_normals = hasVertexNormals

    reserve_triangles = reserveTriangles

    reserve_vertex_normals = reserveVertexNormals

    reserve_vertices = reserveVertices

euler_xyx_to_matrix = eulerXYXToMatrix

euler_xyz_to_matrix = eulerXYZToMatrix

euler_xzx_to_matrix = eulerXZXToMatrix

euler_xzy_to_matrix = eulerXZYToMatrix

euler_yxy_to_matrix = eulerYXYToMatrix

euler_yxz_to_matrix = eulerYXZToMatrix

euler_yzx_to_matrix = eulerYZXToMatrix

euler_yzy_to_matrix = eulerYZYToMatrix

euler_zxy_to_matrix = eulerZXYToMatrix

euler_zxz_to_matrix = eulerZXZToMatrix

euler_zyx_to_matrix = eulerZYXToMatrix

euler_zyz_to_matrix = eulerZYZToMatrix

exp_angular = expAngular

exp_map = expMap

exp_map_jac = expMapJac

exp_map_rot = expMapRot

exp_to_quat = expToQuat

matrix_to_euler_xyx = matrixToEulerXYX

matrix_to_euler_xyz = matrixToEulerXYZ

matrix_to_euler_xzy = matrixToEulerXZY

matrix_to_euler_yxz = matrixToEulerYXZ

matrix_to_euler_yzx = matrixToEulerYZX

matrix_to_euler_zxy = matrixToEulerZXY

matrix_to_euler_zyx = matrixToEulerZYX

quat_to_exp = quatToExp

verify_rotation = verifyRotation

verify_transform = verifyTransform
