/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 */

#pragma once

#include <dart/math/optimization/GradientDescentSolver.hpp>

#include <dart/common/Deprecated.hpp>

namespace dart {
namespace optimizer {

using GradientDescentSolver DART_DEPRECATED(7.0) = math::GradientDescentSolver;

} // namespace optimizer
} // namespace dart
