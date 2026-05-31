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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

#include <dart/simulation/experimental/detail/deformable_vbd/avbd_constraint.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/neo_hookean.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/vertex_block_kernel.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <limits>

#include <cmath>
#include <cstdint>

namespace dart::simulation::experimental::detail::deformable_vbd {

/// One scalar AVBD finite-stiffness row for a deformable distance spring. The
/// row has no dual variable; its persistent state stores the current effective
/// stiffness that ramps from a soft start toward the material stiffness as
/// stretch error is observed.
struct AvbdSpringFiniteStiffnessRow
{
  std::uint32_t spring = 0;
  AvbdScalarRowState state;
  double materialStiffness = std::numeric_limits<double>::infinity();
};

/// One scalar AVBD finite-stiffness row for a tetrahedral material element.
/// The row's current stiffness is a dimensionless multiplier on the body's
/// Lamé parameters, ramping from a soft start toward 1.0 as strain error is
/// observed. It carries no dual variable.
struct AvbdTetMaterialFiniteStiffnessRow
{
  std::uint32_t tet = 0;
  AvbdScalarRowState state;
  double materialStiffness = 1.0;
};

/// Per-sweep AVBD finite-stiffness update parameters.
struct AvbdSpringFiniteStiffnessOptions
{
  double beta = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
};

/// Per-sweep AVBD tetrahedral material finite-stiffness update parameters.
struct AvbdTetMaterialFiniteStiffnessOptions
{
  double beta = 1.0;
  double maxStiffness = 1.0;
};

//==============================================================================
inline double avbdSpringConstraintValue(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b, double restLength)
{
  const double length = (b - a).norm();
  if (!std::isfinite(length)) {
    return 0.0;
  }
  return length - restLength;
}

//==============================================================================
inline double avbdTetMaterialConstraintValue(
    const TetRestShape& rest, const std::array<Eigen::Vector3d, 4>& positions)
{
  if (!(rest.restVolume > 0.0)) {
    return 0.0;
  }

  const Eigen::Matrix3d F
      = deformationGradient(rest.restShapeInverse, positions);
  const double strain = (F - Eigen::Matrix3d::Identity()).norm();
  return std::isfinite(strain) ? strain : 0.0;
}

//==============================================================================
inline LameParameters avbdScaledTetMaterial(
    double mu, double lambda, const AvbdTetMaterialFiniteStiffnessRow& row)
{
  const double scale = std::max(0.0, row.state.stiffness);
  return {scale * mu, scale * lambda};
}

//==============================================================================
inline void addAvbdSpringFiniteStiffness(
    VertexBlock& block,
    const Eigen::Vector3d& self,
    const Eigen::Vector3d& other,
    double restLength,
    const AvbdSpringFiniteStiffnessRow& row,
    bool clampToPsd = true)
{
  addSpringTerm(
      block, row.state.stiffness, restLength, self, other, clampToPsd);
}

//==============================================================================
inline AvbdScalarRowState updateAvbdSpringFiniteStiffnessRow(
    AvbdScalarRowState state,
    double constraintValue,
    const AvbdSpringFiniteStiffnessRow& row,
    const AvbdSpringFiniteStiffnessOptions& options)
{
  state.lambda = 0.0;
  state.stiffness = updateAvbdFiniteStiffness(
      state.stiffness,
      constraintValue,
      options.beta,
      std::min(row.materialStiffness, options.maxStiffness));
  return state;
}

//==============================================================================
inline AvbdScalarRowState updateAvbdTetMaterialFiniteStiffnessRow(
    AvbdScalarRowState state,
    double constraintValue,
    const AvbdTetMaterialFiniteStiffnessRow& row,
    const AvbdTetMaterialFiniteStiffnessOptions& options)
{
  state.lambda = 0.0;
  state.stiffness = updateAvbdFiniteStiffness(
      state.stiffness,
      constraintValue,
      options.beta,
      std::min(row.materialStiffness, options.maxStiffness));
  return state;
}

} // namespace dart::simulation::experimental::detail::deformable_vbd
