/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <algorithm>
#include <limits>

#include <cmath>

namespace dart::simulation::detail::deformable_vbd {

/// Persistent scalar row state for an Augmented VBD constraint or finite
/// stiffness force row.
struct AvbdScalarRowState
{
  double stiffness = 1.0;
  double lambda = 0.0;
};

/// Force bounds for a scalar AVBD row. Contacts use a non-negative normal
/// lower bound; equality rows normally leave both bounds infinite.
struct AvbdScalarRowBounds
{
  double lower = -std::numeric_limits<double>::infinity();
  double upper = std::numeric_limits<double>::infinity();
};

//==============================================================================
inline double clampAvbdRowForce(double value, AvbdScalarRowBounds bounds)
{
  return std::clamp(value, bounds.lower, bounds.upper);
}

//==============================================================================
/// AVBD's regularized constraint value, Eq. 18 in the paper:
/// C(x) = C*(x) - alpha C*(x_t).
inline double regularizeAvbdConstraintValue(
    double currentConstraintValue,
    double previousStepConstraintValue,
    double alpha)
{
  return currentConstraintValue - alpha * previousStepConstraintValue;
}

//==============================================================================
/// Warm start a hard-constraint row using the previous timestep state, Eq. 19.
/// The stiffness decays but never below `startStiffness`; lambda additionally
/// scales by alpha so existing error correction is not re-injected every step.
inline AvbdScalarRowState warmStartAvbdHardConstraint(
    AvbdScalarRowState previous,
    double startStiffness,
    double alpha,
    double gamma,
    double maxStiffness = std::numeric_limits<double>::infinity())
{
  AvbdScalarRowState next;
  next.lambda = alpha * gamma * previous.lambda;
  const double lower = std::min(startStiffness, maxStiffness);
  next.stiffness = std::clamp(gamma * previous.stiffness, lower, maxStiffness);
  return next;
}

//==============================================================================
/// Trial force magnitude for a hard constraint row before the dual write-back,
/// Eq. 13 with optional row bounds.
inline double computeAvbdHardConstraintForce(
    const AvbdScalarRowState& row,
    double constraintValue,
    AvbdScalarRowBounds bounds = {})
{
  return clampAvbdRowForce(
      row.stiffness * constraintValue + row.lambda, bounds);
}

//==============================================================================
/// Update a hard-constraint row's dual and penalty state, Eqs. 11-12. The
/// stiffness grows only while the force is not clamped by row bounds.
inline AvbdScalarRowState updateAvbdHardConstraintRow(
    AvbdScalarRowState row,
    double constraintValue,
    double beta,
    AvbdScalarRowBounds bounds = {},
    double maxStiffness = std::numeric_limits<double>::infinity())
{
  const double nextLambda
      = computeAvbdHardConstraintForce(row, constraintValue, bounds);
  row.lambda = nextLambda;

  if (nextLambda > bounds.lower && nextLambda < bounds.upper) {
    row.stiffness = std::min(
        maxStiffness, row.stiffness + beta * std::abs(constraintValue));
  }

  return row;
}

//==============================================================================
/// Update AVBD's progressive finite-stiffness row, Eq. 16. This row has no
/// persistent lambda; it ramps the effective stiffness up to the material
/// stiffness as error is observed.
inline double updateAvbdFiniteStiffness(
    double currentStiffness,
    double constraintValue,
    double beta,
    double materialStiffness)
{
  return std::min(
      materialStiffness, currentStiffness + beta * std::abs(constraintValue));
}

} // namespace dart::simulation::detail::deformable_vbd
