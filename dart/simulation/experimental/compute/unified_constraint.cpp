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

#include <Eigen/Geometry>

#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

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

//==============================================================================
const Eigen::Vector3d& directionOfRigid(
    const RigidBodyContactConstraint& constraint,
    UnifiedContactDirection direction)
{
  switch (direction) {
    case UnifiedContactDirection::Normal:
      return constraint.normal;
    case UnifiedContactDirection::Tangent1:
      return constraint.tangent1;
    case UnifiedContactDirection::Tangent2:
      return constraint.tangent2;
  }
  return constraint.normal;
}

//==============================================================================
const Eigen::Vector3d& directionOfLink(
    const MultibodyLinkContactRow& row, UnifiedContactDirection direction)
{
  switch (direction) {
    case UnifiedContactDirection::Normal:
      return row.normal;
    case UnifiedContactDirection::Tangent1:
      return row.tangent1;
    case UnifiedContactDirection::Tangent2:
      return row.tangent2;
  }
  return row.normal;
}

//==============================================================================
// One rigid body a row's impulse acts on: the body, the signed
// relative-velocity convention (-1 for bodyA, +1 for bodyB; -1 for a link's
// dynamic obstacle), and that body's world inverse mass/inertia and contact
// arm.
struct RigidEnd
{
  entt::entity entity = entt::null;
  double sign = 0.0;
  double invMass = 0.0;
  Eigen::Matrix3d invInertia = Eigen::Matrix3d::Zero();
  Eigen::Vector3d arm = Eigen::Vector3d::Zero();
};

//==============================================================================
// The contribution to a Delassus entry from a single rigid body shared between
// two rows. Mirrors the rigid `delassusEntry` kernel exactly (same operand
// order), so a within-rigid pair would reproduce the verbatim rigid block.
double sharedBodyEntry(
    const RigidEnd& endI,
    const Eigen::Vector3d& directionI,
    const RigidEnd& endJ,
    const Eigen::Vector3d& directionJ)
{
  return endI.sign * endJ.sign
         * (endI.invMass * directionI.dot(directionJ)
            + directionI.dot((endI.invInertia * endJ.arm.cross(directionJ))
                                 .cross(endI.arm)));
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

  // --- Single-source obstacle inertia. A dynamic rigid body shared between a
  // rigid contact and a link obstacle must use ONE (invMass, invInertia)
  // everywhere, or A is not a consistent Delassus of one operator. The rigid
  // and link assemblers normalize the orientation and handle LDLT failure
  // differently, so adopt the rigid path's value (canonical) for any shared
  // body and overwrite the link rows' obstacle inertia. ---
  std::unordered_map<entt::entity, std::pair<double, Eigen::Matrix3d>>
      rigidInertia;
  rigidInertia.reserve(problem.rigidConstraints.size() * 2);
  for (const auto& constraint : problem.rigidConstraints) {
    rigidInertia[constraint.bodyA]
        = {constraint.invMassA, constraint.invInertiaA};
    rigidInertia[constraint.bodyB]
        = {constraint.invMassB, constraint.invInertiaB};
  }
  for (auto& block : problem.multibodyBlocks) {
    for (auto& row : block.rows) {
      if (row.otherBody == entt::null) {
        continue;
      }
      const auto it = rigidInertia.find(row.otherBody);
      if (it != rigidInertia.end()) {
        row.otherInvMass = it->second.first;
        row.otherInvInertia = it->second.second;
      }
    }
  }

  // --- Per-row direction and rigid-body ends. A rigid row touches its two
  // contact bodies; a link row touches only its dynamic obstacle (its link side
  // is the J^T M^-1 J coupling already filled above). ---
  std::vector<Eigen::Vector3d> rowDirection(static_cast<std::size_t>(size));
  std::vector<std::vector<RigidEnd>> rowEnds(static_cast<std::size_t>(size));
  std::vector<char> rowIsLink(static_cast<std::size_t>(size), 0);
  for (Eigen::Index r = 0; r < size; ++r) {
    const auto& owner = problem.rowOwners[static_cast<std::size_t>(r)];
    auto& ends = rowEnds[static_cast<std::size_t>(r)];
    if (owner.domain == UnifiedContactDomain::Rigid) {
      const auto& constraint = problem.rigidConstraints[owner.sourceIndex];
      rowDirection[static_cast<std::size_t>(r)]
          = directionOfRigid(constraint, owner.direction);
      ends.push_back(
          {constraint.bodyA,
           -1.0,
           constraint.invMassA,
           constraint.invInertiaA,
           constraint.armA});
      ends.push_back(
          {constraint.bodyB,
           +1.0,
           constraint.invMassB,
           constraint.invInertiaB,
           constraint.armB});
    } else {
      rowIsLink[static_cast<std::size_t>(r)] = 1;
      const auto& block = problem.multibodyBlocks[static_cast<std::size_t>(
          owner.multibodyIndex)];
      const auto& row = block.rows[owner.sourceIndex];
      rowDirection[static_cast<std::size_t>(r)]
          = directionOfLink(row, owner.direction);
      if (row.otherBody != entt::null) {
        ends.push_back(
            {row.otherBody,
             -1.0,
             row.otherInvMass,
             row.otherInvInertia,
             row.otherArm});
      }
    }
  }

  // --- Cross / shared-obstacle coupling. Add a shared-rigid-body term to
  // A(i,j) for any pair where at least one row is a link row; rigid-rigid
  // coupling is already in the verbatim rigid block. This fills, over ALL
  // direction pairs: a link contact's own obstacle self-term (the diagonal
  // completes to the stored denominator when the body is not also a rigid
  // participant); link<->link coupling through a shared obstacle (same or
  // different multibody); and rigid<->link coupling through a body that is both
  // a rigid contact participant and a link obstacle. ---
  for (Eigen::Index i = 0; i < size; ++i) {
    const auto& endsI = rowEnds[static_cast<std::size_t>(i)];
    if (endsI.empty()) {
      continue;
    }
    const bool linkI = rowIsLink[static_cast<std::size_t>(i)] != 0;
    const Eigen::Vector3d& directionI
        = rowDirection[static_cast<std::size_t>(i)];
    for (Eigen::Index j = 0; j < size; ++j) {
      const auto& endsJ = rowEnds[static_cast<std::size_t>(j)];
      if (endsJ.empty()) {
        continue;
      }
      if (!linkI && rowIsLink[static_cast<std::size_t>(j)] == 0) {
        continue; // rigid-rigid coupling already in the verbatim block
      }
      const Eigen::Vector3d& directionJ
          = rowDirection[static_cast<std::size_t>(j)];
      double addend = 0.0;
      for (const auto& endI : endsI) {
        if (endI.invMass == 0.0) {
          continue; // static side contributes nothing
        }
        for (const auto& endJ : endsJ) {
          if (endI.entity != endJ.entity) {
            continue; // not the same shared body
          }
          addend += sharedBodyEntry(endI, directionI, endJ, directionJ);
        }
      }
      if (addend != 0.0) {
        problem.delassus(i, j) += addend;
      }
    }
  }

  return problem;
}

} // namespace dart::simulation::experimental::compute
