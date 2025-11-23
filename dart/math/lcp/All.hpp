#pragma once

// New LCP solver framework (v2)
#include <dart/math/lcp/LcpSolver.hpp>
#include <dart/math/lcp/LcpTypes.hpp>

// Pivoting methods
#include <dart/math/lcp/pivoting/LemkeSolver.hpp>

// Legacy LCP solvers (v1 - backward compatibility)
#include <dart/math/lcp/Lemke.hpp>
#include <dart/math/lcp/ODELCPSolver.hpp>
#include <dart/math/lcp/dantzig/Lcp.hpp>
