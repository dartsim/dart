#pragma once

// New LCP solver framework (v2)
#include <dart/math/lcp/LcpSolver.hpp>
#include <dart/math/lcp/LcpTypes.hpp>

// Pivoting methods
#include <dart/math/lcp/pivoting/BaraffSolver.hpp>
#include <dart/math/lcp/pivoting/DantzigSolver.hpp>
#include <dart/math/lcp/pivoting/LemkeSolver.hpp>

// Projection methods
#include <dart/math/lcp/projection/BgsSolver.hpp>
#include <dart/math/lcp/projection/BlockedJacobiSolver.hpp>
#include <dart/math/lcp/projection/JacobiSolver.hpp>
#include <dart/math/lcp/projection/NncgSolver.hpp>
#include <dart/math/lcp/projection/PgsSolver.hpp>
#include <dart/math/lcp/projection/RedBlackGaussSeidelSolver.hpp>
#include <dart/math/lcp/projection/SubspaceMinimizationSolver.hpp>
#include <dart/math/lcp/projection/SymmetricPsorSolver.hpp>

// Newton methods
#include <dart/math/lcp/newton/FischerBurmeisterNewtonSolver.hpp>
#include <dart/math/lcp/newton/MinimumMapNewtonSolver.hpp>
#include <dart/math/lcp/newton/PenalizedFischerBurmeisterNewtonSolver.hpp>

// Other methods
#include <dart/math/lcp/other/InteriorPointSolver.hpp>
#include <dart/math/lcp/other/StaggeringSolver.hpp>

// Legacy LCP solvers (v1 - backward compatibility)
#include <dart/math/lcp/pivoting/dantzig/Lcp.hpp>
