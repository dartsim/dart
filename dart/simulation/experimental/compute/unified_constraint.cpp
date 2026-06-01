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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/simulation/experimental/compute/unified_constraint.hpp"

#include <limits>
#include <utility>

namespace dart::simulation::experimental::compute {

namespace {

//==============================================================================
// The generalized-force Jacobian (J^T d, in the multibody's DOF space) for one
// direction of a link row.
const Eigen::VectorXd& jacobianOf(
    const MultibodyLinkContactRow& row, UnifiedContactDirection direction)
{
  switch (direction) {
    case UnifiedContactDirection::Normal:
      return row.normalJacobian;
    case UnifiedContactDirection::Tangent1:
      return row.tangentJacobian1;
    case UnifiedContactDirection::Tangent2:
      return row.tangentJacobian2;
  }
  return row.normalJacobian;
}

} // namespace

//==============================================================================
UnifiedConstraintProblem assembleUnifiedConstraintProblem(
    const RigidBodyContactProblem& rigidProblem,
    std::span<const UnifiedMultibodyContact> multibodyContacts)
{
  constexpr int kRows = UnifiedConstraintProblem::kRowsPerContact;
  constexpr double kInfinity = std::numeric_limits<double>::infinity();

  UnifiedConstraintProblem problem;

  // Retain the rigid constraints verbatim (post effective-mass filter, same
  // order) for impulse application and the rigid positional projection.
  problem.rigidConstraints = rigidProblem.constraints;
  const auto rigidContactCount
      = static_cast<Eigen::Index>(rigidProblem.constraints.size());
  const Eigen::Index rigidRows = rigidContactCount * kRows;

  // Compact each multibody's active link rows and lay out one contiguous block
  // per multibody after the rigid block.
  Eigen::Index nextBase = rigidRows;
  problem.multibodyBlocks.reserve(multibodyContacts.size());
  for (const auto& contact : multibodyContacts) {
    UnifiedMultibodyBlock block;
    block.multibody = contact.multibody;
    block.inverseMass = contact.problem.inverseMass;
    for (const auto& row : contact.problem.rows) {
      if (row.active) {
        block.rows.push_back(row);
      }
    }
    block.blockBase = nextBase;
    nextBase += static_cast<Eigen::Index>(block.rows.size()) * kRows;
    problem.multibodyBlocks.push_back(std::move(block));
  }

  const Eigen::Index size = nextBase;
  problem.delassus = Eigen::MatrixXd::Zero(size, size);
  problem.rhs.resize(size);
  problem.lo.resize(size);
  problem.hi.resize(size);
  problem.findex.resize(size);
  problem.rowOwners.assign(static_cast<std::size_t>(size), UnifiedRowOwner{});

  // --- Rigid block: copy A,b,lo,hi,findex VERBATIM. The rigid findex already
  // references 3*i, which is the correct global index because the rigid block
  // sits at global offset 0. This keeps multibody-free worlds bit-identical.
  // ---
  if (rigidRows > 0) {
    problem.delassus.topLeftCorner(rigidRows, rigidRows)
        = rigidProblem.delassus;
    problem.rhs.head(rigidRows) = rigidProblem.rhs;
    problem.lo.head(rigidRows) = rigidProblem.lo;
    problem.hi.head(rigidRows) = rigidProblem.hi;
    problem.findex.head(rigidRows) = rigidProblem.findex;
  }
  for (Eigen::Index i = 0; i < rigidContactCount; ++i) {
    const Eigen::Index normalRow = i * kRows;
    for (int t = 0; t < kRows; ++t) {
      auto& owner = problem.rowOwners[static_cast<std::size_t>(normalRow + t)];
      owner.domain = UnifiedContactDomain::Rigid;
      owner.direction = static_cast<UnifiedContactDirection>(t);
      owner.normalRowGlobalIndex = normalRow;
      owner.sourceIndex = static_cast<std::size_t>(i);
      owner.multibodyIndex = -1;
    }
  }

  // --- Link blocks: per-row bounds/findex/rhs against the COMPACTED global row
  // indices, plus the full dense within-multibody Delassus coupling
  // J_i^T M_k^-1 J_j. The diagonal reproduces the stored row denominators for a
  // contact with no dynamic obstacle (same expression, same evaluation order).
  // Shared-dynamic-obstacle off-diagonal terms are added by a later slice. ---
  for (std::size_t k = 0; k < problem.multibodyBlocks.size(); ++k) {
    const auto& block = problem.multibodyBlocks[k];
    const Eigen::MatrixXd& inverseMass = block.inverseMass;
    const auto rowCount = static_cast<Eigen::Index>(block.rows.size());

    for (Eigen::Index c = 0; c < rowCount; ++c) {
      const auto& row = block.rows[static_cast<std::size_t>(c)];
      const Eigen::Index normalRow = block.blockBase + c * kRows;

      problem.rhs[normalRow] = row.normalRhs;
      problem.rhs[normalRow + 1] = row.tangentRhs1;
      problem.rhs[normalRow + 2] = row.tangentRhs2;

      problem.lo[normalRow] = 0.0;
      problem.hi[normalRow] = kInfinity;
      problem.findex[normalRow] = -1;
      for (int t = 1; t < kRows; ++t) {
        problem.lo[normalRow + t] = -row.friction;
        problem.hi[normalRow + t] = row.friction;
        problem.findex[normalRow + t] = static_cast<int>(normalRow);
      }

      for (int t = 0; t < kRows; ++t) {
        auto& owner
            = problem.rowOwners[static_cast<std::size_t>(normalRow + t)];
        owner.domain = UnifiedContactDomain::Link;
        owner.direction = static_cast<UnifiedContactDirection>(t);
        owner.normalRowGlobalIndex = normalRow;
        owner.sourceIndex = static_cast<std::size_t>(c);
        owner.multibodyIndex = static_cast<int>(k);
      }
    }

    // Dense within-multibody block. For each (row ci, direction a) precompute
    // M_k^-1 J once, then dot with every (row cj, direction b) Jacobian, so the
    // diagonal entry is exactly `J.dot(M^-1 J)` = the stored denominator.
    for (Eigen::Index ci = 0; ci < rowCount; ++ci) {
      const auto& rowI = block.rows[static_cast<std::size_t>(ci)];
      const Eigen::Index normalRowI = block.blockBase + ci * kRows;
      for (int a = 0; a < kRows; ++a) {
        const auto direction = static_cast<UnifiedContactDirection>(a);
        const Eigen::VectorXd inverseMassJacobian
            = inverseMass * jacobianOf(rowI, direction);
        for (Eigen::Index cj = 0; cj < rowCount; ++cj) {
          const auto& rowJ = block.rows[static_cast<std::size_t>(cj)];
          const Eigen::Index normalRowJ = block.blockBase + cj * kRows;
          for (int b = 0; b < kRows; ++b) {
            problem.delassus(normalRowI + a, normalRowJ + b)
                = jacobianOf(rowJ, static_cast<UnifiedContactDirection>(b))
                      .dot(inverseMassJacobian);
          }
        }
      }
    }
  }

  return problem;
}

} // namespace dart::simulation::experimental::compute
