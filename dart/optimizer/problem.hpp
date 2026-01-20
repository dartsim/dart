/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the same BSD-style license as DART.
 */

#pragma once

#include <dart/math/optimization/problem.hpp>

#include <dart/common/deprecated.hpp>

namespace dart {
namespace optimizer {

using Problem DART_DEPRECATED(7.0) = math::Problem;

} // namespace optimizer
} // namespace dart
