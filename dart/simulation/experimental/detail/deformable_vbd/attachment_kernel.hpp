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

#include <dart/simulation/experimental/detail/deformable_vbd/avbd_constraint.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/vertex_block_kernel.hpp>

#include <Eigen/Core>

#include <limits>

#include <cstdint>

namespace dart::simulation::experimental::detail::deformable_vbd {

/// One scalar hard attachment row for a deformable vertex. A full 3D point
/// attachment is represented by three rows whose axes are usually the world
/// basis vectors; keeping the primitive scalar matches AVBD's row inventory and
/// lets later row generation persist/warm-start each axis independently.
struct AvbdPointAttachmentRow
{
  std::uint32_t vertex = 0;
  Eigen::Vector3d target = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
  AvbdScalarRowState state;
  double previousConstraintValue = 0.0;
  AvbdScalarRowBounds bounds;
};

/// Per-sweep AVBD hard-attachment update parameters.
struct AvbdPointAttachmentOptions
{
  double alpha = 0.0;
  double beta = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
};

//==============================================================================
inline Eigen::Vector3d canonicalAvbdAttachmentAxis(std::uint8_t axis)
{
  switch (axis) {
    case 0:
      return Eigen::Vector3d::UnitX();
    case 1:
      return Eigen::Vector3d::UnitY();
    default:
      return Eigen::Vector3d::UnitZ();
  }
}

//==============================================================================
inline double avbdPointAttachmentConstraintValue(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& target,
    const Eigen::Vector3d& axis)
{
  return axis.dot(target - position);
}

//==============================================================================
/// Stamp one scalar AVBD point-attachment row into a VBD vertex block. The row
/// constraint is `axis . (target - x)`, so a positive row force moves the
/// vertex toward the target along `axis`. The caller supplies a normalized
/// axis.
inline double addAvbdPointAttachment(
    VertexBlock& block,
    const Eigen::Vector3d& position,
    const AvbdPointAttachmentRow& row,
    double alpha)
{
  const double constraintValue = regularizeAvbdConstraintValue(
      avbdPointAttachmentConstraintValue(position, row.target, row.axis),
      row.previousConstraintValue,
      alpha);
  const double forceMagnitude
      = computeAvbdHardConstraintForce(row.state, constraintValue, row.bounds);
  block.force.noalias() += forceMagnitude * row.axis;
  block.hessian.noalias()
      += row.state.stiffness * (row.axis * row.axis.transpose());
  return forceMagnitude;
}

//==============================================================================
inline AvbdScalarRowState updateAvbdPointAttachmentRow(
    AvbdScalarRowState state,
    const Eigen::Vector3d& position,
    const AvbdPointAttachmentRow& row,
    const AvbdPointAttachmentOptions& options)
{
  const double constraintValue = regularizeAvbdConstraintValue(
      avbdPointAttachmentConstraintValue(position, row.target, row.axis),
      row.previousConstraintValue,
      options.alpha);
  return updateAvbdHardConstraintRow(
      state, constraintValue, options.beta, row.bounds, options.maxStiffness);
}

} // namespace dart::simulation::experimental::detail::deformable_vbd
