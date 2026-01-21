/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 */

#pragma once

#include <dart/math/optimization/gradient_descent_solver.hpp>

#include <dart/common/deprecated.hpp>

namespace dart {
namespace optimizer {

using GradientDescentSolver DART_DEPRECATED(7.0) = math::GradientDescentSolver;

} // namespace optimizer
} // namespace dart
