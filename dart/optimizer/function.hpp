/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the same BSD-style license as DART.
 */

#pragma once

#include <dart/math/optimization/function.hpp>

#include <dart/common/deprecated.hpp>

namespace dart {
namespace optimizer {

using Function DART_DEPRECATED(7.0) = math::Function;
using FunctionPtr DART_DEPRECATED(7.0) = math::FunctionPtr;
using UniqueFunctionPtr DART_DEPRECATED(7.0) = math::UniqueFunctionPtr;
using CostFunction DART_DEPRECATED(7.0) = math::CostFunction;
using GradientFunction DART_DEPRECATED(7.0) = math::GradientFunction;
using HessianFunction DART_DEPRECATED(7.0) = math::HessianFunction;
using ModularFunction DART_DEPRECATED(7.0) = math::ModularFunction;
using NullFunction DART_DEPRECATED(7.0) = math::NullFunction;
using MultiFunction DART_DEPRECATED(7.0) = math::MultiFunction;

} // namespace optimizer
} // namespace dart
