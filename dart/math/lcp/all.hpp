#pragma once

// New LCP solver framework (v2)
#include <dart/math/lcp/lcp_solver.hpp>
#include <dart/math/lcp/lcp_types.hpp>

// Pivoting methods
#include <dart/math/lcp/pivoting/baraff_solver.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/pivoting/direct_solver.hpp>
#include <dart/math/lcp/pivoting/lemke_solver.hpp>

// Projection methods
#include <dart/math/lcp/projection/bgs_solver.hpp>
#include <dart/math/lcp/projection/blocked_jacobi_solver.hpp>
#include <dart/math/lcp/projection/jacobi_solver.hpp>
#include <dart/math/lcp/projection/nncg_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>
#include <dart/math/lcp/projection/red_black_gauss_seidel_solver.hpp>
#include <dart/math/lcp/projection/subspace_minimization_solver.hpp>
#include <dart/math/lcp/projection/symmetric_psor_solver.hpp>

// Newton methods
#include <dart/math/lcp/newton/fischer_burmeister_newton_solver.hpp>
#include <dart/math/lcp/newton/minimum_map_newton_solver.hpp>
#include <dart/math/lcp/newton/penalized_fischer_burmeister_newton_solver.hpp>

// Other methods
#include <dart/math/lcp/other/interior_point_solver.hpp>
#include <dart/math/lcp/other/mprgp_solver.hpp>
#include <dart/math/lcp/other/shock_propagation_solver.hpp>
#include <dart/math/lcp/other/staggering_solver.hpp>

// Legacy LCP solvers (v1 - backward compatibility)
#include <dart/math/lcp/pivoting/dantzig/lcp.hpp>
