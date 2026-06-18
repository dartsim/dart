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

#include <dart/simulation/detail/multibody_spatial_algebra.hpp>

#include <Eigen/Core>

#include <algorithm>

#include <cstddef>

namespace dart::simulation::detail {

/// Adapter payload for the shared articulated inverse-mass apply.
///
/// `parentToChild` is the spatial motion transform from the parent link frame
/// into this link's frame. The corresponding force transform is its transpose.
struct ArticulatedInverseMassLink
{
  int parentIndex = -1;
  std::size_t dof = 0;
  std::size_t dofOffset = 0;
  const Matrix6* inertia = nullptr;
  const Subspace* subspace = nullptr;
  Matrix6 parentToChild = Matrix6::Identity();
};

template <typename Scratch, typename LinkDofAt>
void reserveArticulatedInverseMassScratch(
    std::size_t linkCount,
    Eigen::Index dofCount,
    const LinkDofAt& linkDofAt,
    Scratch& scratch)
{
  scratch.articulated.resize(linkCount);
  scratch.bias.resize(linkCount);
  scratch.motionToChild.resize(linkCount);
  scratch.spatial.resize(linkCount);
  scratch.forceProjector.resize(linkCount);
  scratch.jointMatrix.resize(linkCount);
  scratch.jointMatrixInverse.resize(linkCount);
  scratch.jointRhs.resize(linkCount);

  Eigen::Index maxJointDof = 0;
  for (std::size_t i = 0; i < linkCount; ++i) {
    const auto dof = static_cast<Eigen::Index>(linkDofAt(i));
    maxJointDof = std::max(maxJointDof, dof);
    scratch.forceProjector[i].resize(6, dof);
    scratch.jointMatrix[i].resize(dof, dof);
    scratch.jointMatrixInverse[i].resize(dof, dof);
    scratch.jointRhs[i].resize(dof);
  }
  scratch.jointWork.resize(maxJointDof);
  scratch.jointSolveWork.resize(maxJointDof);
  (void)dofCount;
}

/// Apply the fixed-base articulated inverse mass, `result = M(q)^-1 impulse`.
///
/// This is Featherstone ABA with zero velocity and gravity: a backward
/// articulated-inertia sweep followed by a forward acceleration sweep. Optional
/// per-coordinate armature is added to each joint-space articulated inertia
/// block before elimination.
template <typename LinkAt, typename Scratch>
void applyArticulatedInverseMassInto(
    std::size_t linkCount,
    std::size_t dofCount,
    const Eigen::VectorXd& armature,
    const Eigen::VectorXd& impulse,
    const LinkAt& linkAt,
    Scratch& scratch,
    Eigen::VectorXd& result)
{
  const auto dofCountIndex = static_cast<Eigen::Index>(dofCount);
  reserveArticulatedInverseMassScratch(
      linkCount,
      dofCountIndex,
      [&](std::size_t i) { return linkAt(i).dof; },
      scratch);

  result.resize(dofCountIndex);
  result.setZero();
  for (std::size_t i = 0; i < linkCount; ++i) {
    const auto link = linkAt(i);
    scratch.articulated[i] = *link.inertia;
    scratch.bias[i].setZero();
    scratch.motionToChild[i].setIdentity();
    scratch.spatial[i].setZero();
  }

  for (std::size_t reverse = 0; reverse < linkCount; ++reverse) {
    const std::size_t i = linkCount - 1 - reverse;
    const auto link = linkAt(i);
    if (link.dof > 0) {
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto dof = static_cast<Eigen::Index>(link.dof);
      if (dof == 1) {
        const Vector6 subspace = link.subspace->col(0);
        scratch.forceProjector[i].col(0).noalias()
            = scratch.articulated[i] * subspace;
        scratch.jointMatrix[i](0, 0)
            = subspace.dot(scratch.forceProjector[i].col(0));
        if (armature.size() == dofCountIndex) {
          scratch.jointMatrix[i](0, 0) += armature[seg];
        }
        scratch.jointMatrixInverse[i](0, 0)
            = 1.0 / scratch.jointMatrix[i](0, 0);
        scratch.jointRhs[i][0] = impulse[seg] - subspace.dot(scratch.bias[i]);
      } else {
        scratch.forceProjector[i].noalias()
            = scratch.articulated[i] * (*link.subspace);
        scratch.jointMatrix[i].noalias()
            = link.subspace->transpose() * scratch.forceProjector[i];
        if (armature.size() == dofCountIndex) {
          scratch.jointMatrix[i].diagonal() += armature.segment(seg, dof);
        }
        scratch.jointMatrixInverse[i] = scratch.jointMatrix[i].inverse();
        scratch.jointRhs[i] = impulse.segment(seg, dof);
        scratch.jointWork.head(dof).noalias()
            = link.subspace->transpose() * scratch.bias[i];
        scratch.jointRhs[i] -= scratch.jointWork.head(dof);
      }
    }
    if (link.parentIndex >= 0) {
      Matrix6 articulated = scratch.articulated[i];
      Vector6 bias = scratch.bias[i];
      if (link.dof > 0) {
        const auto dof = static_cast<Eigen::Index>(link.dof);
        if (dof == 1) {
          const Vector6 forceProjector = scratch.forceProjector[i].col(0);
          const double inverseJointMass = scratch.jointMatrixInverse[i](0, 0);
          articulated.noalias()
              -= inverseJointMass
                 * (forceProjector * forceProjector.transpose());
          const double jointSolve = inverseJointMass * scratch.jointRhs[i][0];
          bias.noalias() += forceProjector * jointSolve;
        } else {
          articulated.noalias() -= scratch.forceProjector[i]
                                   * scratch.jointMatrixInverse[i]
                                   * scratch.forceProjector[i].transpose();
          scratch.jointWork.head(dof).noalias()
              = scratch.jointMatrixInverse[i] * scratch.jointRhs[i];
          bias.noalias()
              += scratch.forceProjector[i] * scratch.jointWork.head(dof);
        }
      }
      scratch.motionToChild[i] = link.parentToChild;
      const Matrix6 forceToParent = scratch.motionToChild[i].transpose();
      const auto parent = static_cast<std::size_t>(link.parentIndex);
      scratch.articulated[parent].noalias()
          += forceToParent * articulated * scratch.motionToChild[i];
      scratch.bias[parent].noalias() += forceToParent * bias;
    }
  }

  for (std::size_t i = 0; i < linkCount; ++i) {
    const auto link = linkAt(i);
    if (link.parentIndex < 0) {
      scratch.spatial[i].setZero();
      continue;
    }
    scratch.spatial[i].noalias()
        = scratch.motionToChild[i]
          * scratch.spatial[static_cast<std::size_t>(link.parentIndex)];
    if (link.dof > 0) {
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto dof = static_cast<Eigen::Index>(link.dof);
      if (dof == 1) {
        const double jointRhs
            = scratch.jointRhs[i][0]
              - scratch.forceProjector[i].col(0).dot(scratch.spatial[i]);
        const double jointSolve
            = scratch.jointMatrixInverse[i](0, 0) * jointRhs;
        result[seg] = jointSolve;
        scratch.spatial[i].noalias() += link.subspace->col(0) * jointSolve;
      } else {
        scratch.jointWork.head(dof) = scratch.jointRhs[i];
        scratch.jointSolveWork.head(dof).noalias()
            = scratch.forceProjector[i].transpose() * scratch.spatial[i];
        scratch.jointWork.head(dof) -= scratch.jointSolveWork.head(dof);
        scratch.jointSolveWork.head(dof).noalias()
            = scratch.jointMatrixInverse[i] * scratch.jointWork.head(dof);
        result.segment(seg, dof) = scratch.jointSolveWork.head(dof);
        scratch.spatial[i].noalias()
            += (*link.subspace) * scratch.jointSolveWork.head(dof);
      }
    }
  }
}

} // namespace dart::simulation::detail
