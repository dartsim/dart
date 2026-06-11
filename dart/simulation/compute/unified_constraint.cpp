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

#include "dart/simulation/compute/unified_constraint.hpp"

#include "dart/simulation/comps/dynamics.hpp"

#include <dart/math/lcp/lcp_types.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>

#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <utility>
#include <vector>

#include <cstddef>

namespace dart::simulation::compute {

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
const Eigen::VectorXd& otherJacobianOf(
    const MultibodyLinkContactRow& row, UnifiedContactDirection direction)
{
  switch (direction) {
    case UnifiedContactDirection::Normal:
      return row.otherNormalJacobian;
    case UnifiedContactDirection::Tangent1:
      return row.otherTangentJacobian1;
    case UnifiedContactDirection::Tangent2:
      return row.otherTangentJacobian2;
  }
  return row.otherNormalJacobian;
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
int findMultibodyBlockIndex(
    std::span<const UnifiedMultibodyBlock> blocks, entt::entity multibody)
{
  for (std::size_t k = 0; k < blocks.size(); ++k) {
    if (blocks[k].multibody == multibody) {
      return static_cast<int>(k);
    }
  }
  return -1;
}

//==============================================================================
const MultibodyLinkContactProblem& problemOf(
    const UnifiedMultibodyContact& contact)
{
  return contact.borrowedProblem != nullptr ? *contact.borrowedProblem
                                            : contact.problem;
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
// One articulated multibody a link row's impulse acts on. Primary link ends use
// sign +1; cross-multibody obstacle link ends use sign -1.
struct ArticulatedEnd
{
  int multibodyIndex = -1;
  double sign = 0.0;
  bool primary = false;
  const Eigen::VectorXd* jacobian = nullptr;
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

//==============================================================================
bool findRigidConstraintInertia(
    std::span<const RigidBodyContactConstraint> constraints,
    entt::entity body,
    double& invMass,
    Eigen::Matrix3d& invInertia)
{
  for (const auto& constraint : constraints) {
    if (constraint.bodyA == body) {
      invMass = constraint.invMassA;
      invInertia = constraint.invInertiaA;
      return true;
    }
    if (constraint.bodyB == body) {
      invMass = constraint.invMassB;
      invInertia = constraint.invInertiaB;
      return true;
    }
  }
  return false;
}

//==============================================================================
const Eigen::Vector3d& rowDirection(
    const UnifiedConstraintProblem& problem, Eigen::Index rowIndex)
{
  const auto& owner = problem.rowOwners[static_cast<std::size_t>(rowIndex)];
  if (owner.domain == UnifiedContactDomain::Rigid) {
    return directionOfRigid(
        problem.rigidConstraints[owner.sourceIndex], owner.direction);
  }

  const auto& block
      = problem.multibodyBlocks[static_cast<std::size_t>(owner.multibodyIndex)];
  return directionOfLink(block.rows[owner.sourceIndex], owner.direction);
}

//==============================================================================
std::size_t rowRigidEnds(
    const UnifiedConstraintProblem& problem,
    Eigen::Index rowIndex,
    std::array<RigidEnd, 2>& ends)
{
  const auto& owner = problem.rowOwners[static_cast<std::size_t>(rowIndex)];
  if (owner.domain == UnifiedContactDomain::Rigid) {
    const auto& constraint = problem.rigidConstraints[owner.sourceIndex];
    ends[0]
        = {constraint.bodyA,
           -1.0,
           constraint.invMassA,
           constraint.invInertiaA,
           constraint.armA};
    ends[1]
        = {constraint.bodyB,
           +1.0,
           constraint.invMassB,
           constraint.invInertiaB,
           constraint.armB};
    return 2;
  }

  const auto& block
      = problem.multibodyBlocks[static_cast<std::size_t>(owner.multibodyIndex)];
  const auto& row = block.rows[owner.sourceIndex];
  if (row.otherBody == entt::null) {
    return 0;
  }

  ends[0] = {
      row.otherBody, -1.0, row.otherInvMass, row.otherInvInertia, row.otherArm};
  return 1;
}

//==============================================================================
std::size_t rowArticulatedEnds(
    const UnifiedConstraintProblem& problem,
    Eigen::Index rowIndex,
    std::array<ArticulatedEnd, 2>& ends)
{
  const auto& owner = problem.rowOwners[static_cast<std::size_t>(rowIndex)];
  if (owner.domain != UnifiedContactDomain::Link) {
    return 0;
  }

  const auto& block
      = problem.multibodyBlocks[static_cast<std::size_t>(owner.multibodyIndex)];
  const auto& row = block.rows[owner.sourceIndex];

  std::size_t count = 0;
  ends[count++]
      = {owner.multibodyIndex, +1.0, true, &jacobianOf(row, owner.direction)};
  if (row.otherMultibodyIndex >= 0) {
    ends[count++]
        = {row.otherMultibodyIndex,
           -1.0,
           false,
           &otherJacobianOf(row, owner.direction)};
  }
  return count;
}

//==============================================================================
double jointSpaceEntry(
    const Eigen::MatrixXd& inverseMass,
    const Eigen::VectorXd& lhs,
    const Eigen::VectorXd& rhs)
{
  DART_ASSERT(inverseMass.rows() == lhs.size());
  DART_ASSERT(inverseMass.cols() == rhs.size());

  double value = 0.0;
  for (Eigen::Index row = 0; row < inverseMass.rows(); ++row) {
    double rowValue = 0.0;
    for (Eigen::Index col = 0; col < inverseMass.cols(); ++col) {
      rowValue += inverseMass(row, col) * rhs[col];
    }
    value += lhs[row] * rowValue;
  }
  return value;
}

//==============================================================================
void computeRowIslandsInto(
    const UnifiedConstraintProblem& problem,
    UnifiedConstraintSolveScratch& scratch)
{
  const Eigen::Index size = problem.rhs.size();
  const auto rowCount = static_cast<std::size_t>(size);
  scratch.islandRows.clear();
  scratch.islandRows.reserve(rowCount);
  scratch.islandOffsets.clear();
  scratch.islandOffsets.reserve(rowCount + 1u);
  scratch.rowStack.clear();
  scratch.rowStack.reserve(rowCount);
  scratch.visitedRows.assign(rowCount, 0);
  scratch.islandOffsets.push_back(0);

  const auto visit = [&](Eigen::Index row) {
    if (row < 0 || row >= size) {
      return false;
    }
    auto& wasVisited = scratch.visitedRows[static_cast<std::size_t>(row)];
    if (wasVisited != 0) {
      return false;
    }
    wasVisited = 1;
    scratch.rowStack.push_back(row);
    return true;
  };

  for (Eigen::Index start = 0; start < size; ++start) {
    if (!visit(start)) {
      continue;
    }

    const auto islandBegin = scratch.islandRows.size();
    while (!scratch.rowStack.empty()) {
      const Eigen::Index row = scratch.rowStack.back();
      scratch.rowStack.pop_back();
      scratch.islandRows.push_back(row);

      const Eigen::Index normalRow = problem.findex[row];
      visit(normalRow);
      for (Eigen::Index candidate = 0; candidate < size; ++candidate) {
        if (candidate == row) {
          continue;
        }
        if (problem.findex[candidate] == row
            || problem.delassus(row, candidate) != 0.0
            || problem.delassus(candidate, row) != 0.0) {
          visit(candidate);
        }
      }
    }
    std::sort(
        scratch.islandRows.begin() + static_cast<std::ptrdiff_t>(islandBegin),
        scratch.islandRows.end());
    scratch.islandOffsets.push_back(scratch.islandRows.size());
  }
}

//==============================================================================
bool solveBoxedLcpInto(
    const Eigen::MatrixXd& delassus,
    const Eigen::VectorXd& rhs,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    Eigen::VectorXd& lambda,
    UnifiedConstraintSolveScratch& scratch)
{
  const Eigen::Index size = rhs.size();
  if (size == 0) {
    lambda.resize(0);
    return true;
  }

  // Pin the options to the solver default + early termination. A
  // default-constructed LcpOptions would change the validation/tolerance fields
  // that decide succeeded(), and thus flip the rank-deficient fallback
  // decision.
  math::LcpOptions options = scratch.solver.getDefaultOptions();
  options.earlyTermination = true;
  const auto result = scratch.solver.solve(
      delassus, rhs, lo, hi, findex, lambda, scratch.dantzig, options);
  return result.succeeded() && lambda.size() == size;
}

//==============================================================================
void setScaledVector(
    Eigen::VectorXd& target, const Eigen::VectorXd& source, double scale)
{
  target.resize(source.size());
  for (Eigen::Index i = 0; i < source.size(); ++i) {
    target[i] = scale * source[i];
  }
}

//==============================================================================
void addScaledVector(
    Eigen::VectorXd& target, const Eigen::VectorXd& source, double scale)
{
  DART_ASSERT(target.size() == source.size());
  for (Eigen::Index i = 0; i < source.size(); ++i) {
    target[i] += scale * source[i];
  }
}

//==============================================================================
void addJointSpaceImpulse(
    const Eigen::MatrixXd& inverseMass,
    const Eigen::VectorXd& generalizedImpulse,
    Eigen::VectorXd& velocity,
    UnifiedConstraintSolveScratch& scratch,
    double sign)
{
  DART_ASSERT(inverseMass.rows() == velocity.size());
  DART_ASSERT(inverseMass.cols() == generalizedImpulse.size());

  scratch.velocityDelta.resize(inverseMass.rows());
  scratch.velocityDelta.noalias() = inverseMass * generalizedImpulse;
  for (Eigen::Index i = 0; i < velocity.size(); ++i) {
    velocity[i] += sign * scratch.velocityDelta[i];
  }
}

//==============================================================================
void setContactImpulse(
    UnifiedConstraintSolveScratch& scratch,
    const Eigen::VectorXd& normalJacobian,
    double normalImpulse,
    const Eigen::VectorXd& tangentJacobian1,
    double tangentImpulse1,
    const Eigen::VectorXd& tangentJacobian2,
    double tangentImpulse2)
{
  setScaledVector(scratch.generalizedImpulse, normalJacobian, normalImpulse);
  addScaledVector(
      scratch.generalizedImpulse, tangentJacobian1, tangentImpulse1);
  addScaledVector(
      scratch.generalizedImpulse, tangentJacobian2, tangentImpulse2);
}

} // namespace

//==============================================================================
void UnifiedConstraintSolveScratch::clear() noexcept
{
  dantzig.clear();
  lambda.resize(0);
  islandLambda.resize(0);
  generalizedImpulse.resize(0);
  velocityDelta.resize(0);
  islandRows.clear();
  islandOffsets.clear();
  visitedRows.clear();
  rowStack.clear();
  localIndex.clear();
  islandDelassus.resize(0, 0);
  islandRhs.resize(0);
  islandLo.resize(0);
  islandHi.resize(0);
  islandFindex.resize(0);
  normalRows.clear();
  normalA.resize(0, 0);
  normalB.resize(0);
  normalLo.resize(0);
  normalHi.resize(0);
  normalFindex.resize(0);
  normalLambda.resize(0);
  rigidTangent1.clear();
  rigidTangent2.clear();
  linkTangentOffsets.clear();
  linkTangent1.clear();
  linkTangent2.clear();
}

//==============================================================================
void assembleUnifiedConstraintProblemInto(
    UnifiedConstraintProblem& problem,
    const RigidBodyContactProblem& rigidProblem,
    std::span<const UnifiedMultibodyContact> multibodyContacts)
{
  constexpr int kRows = UnifiedConstraintProblem::kRowsPerContact;
  constexpr double kInfinity = std::numeric_limits<double>::infinity();

  // Retain the rigid constraints verbatim (post effective-mass filter, same
  // order) for impulse application and the rigid positional projection.
  problem.rigidConstraints.assign(
      rigidProblem.constraints.begin(), rigidProblem.constraints.end());
  const auto rigidContactCount
      = static_cast<Eigen::Index>(rigidProblem.constraints.size());
  const Eigen::Index rigidRows = rigidContactCount * kRows;

  // Compact each multibody's active link rows and lay out one contiguous block
  // per multibody after the rigid block.
  Eigen::Index nextBase = rigidRows;
  problem.resizeMultibodyBlocks(multibodyContacts.size());
  for (std::size_t k = 0; k < multibodyContacts.size(); ++k) {
    const auto& contact = multibodyContacts[k];
    const auto& linkProblem = problemOf(contact);
    auto& block = problem.multibodyBlocks[k];
    block.multibody = contact.multibody;
    block.inverseMass = linkProblem.inverseMass;

    std::size_t activeRowCount = 0;
    for (const auto& row : linkProblem.rows) {
      if (row.active) {
        ++activeRowCount;
      }
    }
    block.rows.resize(activeRowCount);
    std::size_t compactedRow = 0;
    for (const auto& row : linkProblem.rows) {
      if (row.active) {
        block.rows[compactedRow++] = row;
      }
    }
    block.blockBase = nextBase;
    nextBase += static_cast<Eigen::Index>(block.rows.size()) * kRows;
  }
  for (auto& block : problem.multibodyBlocks) {
    for (auto& row : block.rows) {
      if (row.otherMultibody == entt::null) {
        continue;
      }
      row.otherMultibodyIndex = findMultibodyBlockIndex(
          problem.multibodyBlocks, row.otherMultibody);
    }
  }

  const Eigen::Index size = nextBase;
  problem.delassus.resize(size, size);
  problem.delassus.setZero();
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
  // Shared-dynamic-obstacle terms are added after inertia reconciliation. ---
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
        for (Eigen::Index cj = 0; cj < rowCount; ++cj) {
          const auto& rowJ = block.rows[static_cast<std::size_t>(cj)];
          const Eigen::Index normalRowJ = block.blockBase + cj * kRows;
          for (int b = 0; b < kRows; ++b) {
            problem.delassus(normalRowI + a, normalRowJ + b) = jointSpaceEntry(
                inverseMass,
                jacobianOf(rowJ, static_cast<UnifiedContactDirection>(b)),
                jacobianOf(rowI, direction));
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
  // body and overwrite the link rows' obstacle inertia. Keep this as a small
  // linear scan instead of a per-step hash table: the stage's hot path should
  // not allocate scratch containers just to reconcile the already-assembled
  // rigid rows.
  // ---
  for (auto& block : problem.multibodyBlocks) {
    for (auto& row : block.rows) {
      if (row.otherBody == entt::null) {
        continue;
      }
      double invMass = 0.0;
      Eigen::Matrix3d invInertia = Eigen::Matrix3d::Zero();
      if (findRigidConstraintInertia(
              problem.rigidConstraints, row.otherBody, invMass, invInertia)) {
        row.otherInvMass = invMass;
        row.otherInvInertia = invInertia;
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
    std::array<RigidEnd, 2> endsI;
    const std::size_t endsICount = rowRigidEnds(problem, i, endsI);
    if (endsICount == 0u) {
      continue;
    }
    const auto& ownerI = problem.rowOwners[static_cast<std::size_t>(i)];
    const bool linkI = ownerI.domain == UnifiedContactDomain::Link;
    const Eigen::Vector3d& directionI = rowDirection(problem, i);
    for (Eigen::Index j = 0; j < size; ++j) {
      std::array<RigidEnd, 2> endsJ;
      const std::size_t endsJCount = rowRigidEnds(problem, j, endsJ);
      if (endsJCount == 0u) {
        continue;
      }
      const auto& ownerJ = problem.rowOwners[static_cast<std::size_t>(j)];
      if (!linkI && ownerJ.domain != UnifiedContactDomain::Link) {
        continue; // rigid-rigid coupling already in the verbatim block
      }
      const Eigen::Vector3d& directionJ = rowDirection(problem, j);
      double addend = 0.0;
      for (std::size_t endIIndex = 0; endIIndex < endsICount; ++endIIndex) {
        const auto& endI = endsI[endIIndex];
        if (endI.invMass == 0.0) {
          continue; // static side contributes nothing
        }
        for (std::size_t endJIndex = 0; endJIndex < endsJCount; ++endJIndex) {
          const auto& endJ = endsJ[endJIndex];
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

  // --- Cross-multibody articulated coupling. The dense within-multibody pass
  // above already filled primary-primary terms for rows owned by the same
  // block. Cross-link rows add a second articulated end with sign -1; fill
  // every term involving such non-primary ends here, including their self
  // contribution and coupling against rows owned by the other multibody.
  for (Eigen::Index i = 0; i < size; ++i) {
    std::array<ArticulatedEnd, 2> endsI;
    const std::size_t endsICount = rowArticulatedEnds(problem, i, endsI);
    if (endsICount == 0u) {
      continue;
    }
    for (Eigen::Index j = 0; j < size; ++j) {
      std::array<ArticulatedEnd, 2> endsJ;
      const std::size_t endsJCount = rowArticulatedEnds(problem, j, endsJ);
      if (endsJCount == 0u) {
        continue;
      }
      double addend = 0.0;
      for (std::size_t endIIndex = 0; endIIndex < endsICount; ++endIIndex) {
        const auto& endI = endsI[endIIndex];
        if (endI.multibodyIndex < 0) {
          continue;
        }
        for (std::size_t endJIndex = 0; endJIndex < endsJCount; ++endJIndex) {
          const auto& endJ = endsJ[endJIndex];
          if (endI.multibodyIndex != endJ.multibodyIndex) {
            continue;
          }
          if (endI.primary && endJ.primary) {
            continue; // filled by the dense within-multibody pass
          }
          const auto& block = problem.multibodyBlocks[static_cast<std::size_t>(
              endI.multibodyIndex)];
          addend += endI.sign * endJ.sign
                    * jointSpaceEntry(
                        block.inverseMass, *endJ.jacobian, *endI.jacobian);
        }
      }
      if (addend != 0.0) {
        problem.delassus(i, j) += addend;
      }
    }
  }
}

//==============================================================================
UnifiedConstraintProblem assembleUnifiedConstraintProblem(
    const RigidBodyContactProblem& rigidProblem,
    std::span<const UnifiedMultibodyContact> multibodyContacts)
{
  UnifiedConstraintProblem problem;
  assembleUnifiedConstraintProblemInto(
      problem, rigidProblem, multibodyContacts);
  return problem;
}

//==============================================================================
UnifiedConstraintSolution solveUnifiedConstraintProblem(
    const UnifiedConstraintProblem& problem)
{
  UnifiedConstraintSolveScratch scratch;
  UnifiedConstraintSolution solution;
  solution.succeeded = solveUnifiedConstraintProblemInto(problem, scratch);
  solution.lambda = std::move(scratch.lambda);
  return solution;
}

//==============================================================================
bool solveUnifiedConstraintProblemInto(
    const UnifiedConstraintProblem& problem,
    UnifiedConstraintSolveScratch& scratch)
{
  const Eigen::Index size = problem.rhs.size();
  if (size == 0) {
    scratch.lambda.resize(0);
    return true;
  }

  computeRowIslandsInto(problem, scratch);
  const auto islandCount = scratch.islandOffsets.empty()
                               ? std::size_t{0}
                               : scratch.islandOffsets.size() - 1;
  if (islandCount == 1
      && static_cast<Eigen::Index>(
             scratch.islandOffsets[1] - scratch.islandOffsets[0])
             == size) {
    return solveBoxedLcpInto(
        problem.delassus,
        problem.rhs,
        problem.lo,
        problem.hi,
        problem.findex,
        scratch.lambda,
        scratch);
  }

  scratch.lambda.setZero(size);
  scratch.localIndex.assign(static_cast<std::size_t>(size), -1);
  for (std::size_t islandIndex = 0; islandIndex < islandCount; ++islandIndex) {
    const auto islandBegin = scratch.islandOffsets[islandIndex];
    const auto islandEnd = scratch.islandOffsets[islandIndex + 1];
    const auto islandSize = static_cast<Eigen::Index>(islandEnd - islandBegin);
    for (Eigen::Index local = 0; local < islandSize; ++local) {
      scratch.localIndex[static_cast<std::size_t>(
          scratch.islandRows[islandBegin + static_cast<std::size_t>(local)])]
          = local;
    }

    scratch.islandDelassus.resize(islandSize, islandSize);
    scratch.islandRhs.resize(islandSize);
    scratch.islandLo.resize(islandSize);
    scratch.islandHi.resize(islandSize);
    scratch.islandFindex.setConstant(islandSize, -1);
    for (Eigen::Index row = 0; row < islandSize; ++row) {
      const Eigen::Index globalRow
          = scratch.islandRows[islandBegin + static_cast<std::size_t>(row)];
      scratch.islandRhs[row] = problem.rhs[globalRow];
      scratch.islandLo[row] = problem.lo[globalRow];
      scratch.islandHi[row] = problem.hi[globalRow];
      const Eigen::Index globalFindex = problem.findex[globalRow];
      if (globalFindex >= 0) {
        if (globalFindex >= size) {
          return false;
        }
        const Eigen::Index remapped
            = scratch.localIndex[static_cast<std::size_t>(globalFindex)];
        if (remapped < 0) {
          return false;
        }
        scratch.islandFindex[row] = static_cast<int>(remapped);
      }
      for (Eigen::Index col = 0; col < islandSize; ++col) {
        scratch.islandDelassus(row, col) = problem.delassus(
            globalRow,
            scratch.islandRows[islandBegin + static_cast<std::size_t>(col)]);
      }
    }

    const bool islandSucceeded = solveBoxedLcpInto(
        scratch.islandDelassus,
        scratch.islandRhs,
        scratch.islandLo,
        scratch.islandHi,
        scratch.islandFindex,
        scratch.islandLambda,
        scratch);
    if (!islandSucceeded) {
      return false;
    }
    for (Eigen::Index local = 0; local < islandSize; ++local) {
      scratch.lambda
          [scratch.islandRows[islandBegin + static_cast<std::size_t>(local)]]
          = scratch.islandLambda[local];
    }
  }

  return true;
}

//==============================================================================
bool solveUnifiedConstraintProblemInto(
    const UnifiedConstraintProblem& problem,
    UnifiedConstraintSolution& solution,
    UnifiedConstraintSolveScratch& scratch)
{
  solution.succeeded = solveUnifiedConstraintProblemInto(problem, scratch);
  solution.lambda = scratch.lambda;
  return solution.succeeded;
}

//==============================================================================
UnifiedConstraintSolution solveUnifiedConstraintProblem(
    const UnifiedConstraintProblem& problem,
    UnifiedConstraintSolveScratch& scratch)
{
  UnifiedConstraintSolution solution;
  solveUnifiedConstraintProblemInto(problem, solution, scratch);
  return solution;
}

//==============================================================================
void applyUnifiedConstraintImpulses(
    detail::WorldRegistry& registry,
    const UnifiedConstraintProblem& problem,
    const Eigen::VectorXd& lambda,
    std::span<Eigen::VectorXd> multibodyVelocities)
{
  UnifiedConstraintSolveScratch scratch;
  applyUnifiedConstraintImpulses(
      registry, problem, lambda, multibodyVelocities, scratch);
}

//==============================================================================
void applyUnifiedConstraintImpulses(
    detail::WorldRegistry& registry,
    const UnifiedConstraintProblem& problem,
    const Eigen::VectorXd& lambda,
    std::span<Eigen::VectorXd> multibodyVelocities,
    UnifiedConstraintSolveScratch& scratch)
{
  constexpr int kRows = UnifiedConstraintProblem::kRowsPerContact;

  // Rigid contacts: one world-space impulse per contact applied to both bodies.
  for (std::size_t i = 0; i < problem.rigidConstraints.size(); ++i) {
    const auto& constraint = problem.rigidConstraints[i];
    const Eigen::Index normalRow = static_cast<Eigen::Index>(i) * kRows;
    const double normalImpulse = std::max(0.0, lambda[normalRow]);
    const Eigen::Vector3d impulse
        = normalImpulse * constraint.normal
          + lambda[normalRow + 1] * constraint.tangent1
          + lambda[normalRow + 2] * constraint.tangent2;
    applyRigidBodyContactImpulse(registry, constraint, impulse);
  }

  // Link contacts: drive the owning multibody's staged generalized velocity by
  // M_k^-1 J^T lambda, and apply the equal-and-opposite impulse to a dynamic
  // obstacle's velocity.
  for (std::size_t k = 0; k < problem.multibodyBlocks.size(); ++k) {
    const auto& block = problem.multibodyBlocks[k];
    Eigen::VectorXd& velocity = multibodyVelocities[k];
    for (std::size_t c = 0; c < block.rows.size(); ++c) {
      const auto& row = block.rows[c];
      const Eigen::Index normalRow
          = block.blockBase + static_cast<Eigen::Index>(c) * kRows;
      const double normalImpulse = std::max(0.0, lambda[normalRow]);
      const double tangentImpulse1 = lambda[normalRow + 1];
      const double tangentImpulse2 = lambda[normalRow + 2];

      setContactImpulse(
          scratch,
          row.normalJacobian,
          normalImpulse,
          row.tangentJacobian1,
          tangentImpulse1,
          row.tangentJacobian2,
          tangentImpulse2);
      addJointSpaceImpulse(
          block.inverseMass,
          scratch.generalizedImpulse,
          velocity,
          scratch,
          1.0);

      if (row.otherMultibodyIndex >= 0) {
        const auto& otherBlock
            = problem.multibodyBlocks[static_cast<std::size_t>(
                row.otherMultibodyIndex)];
        Eigen::VectorXd& otherVelocity
            = multibodyVelocities[static_cast<std::size_t>(
                row.otherMultibodyIndex)];
        setContactImpulse(
            scratch,
            row.otherNormalJacobian,
            normalImpulse,
            row.otherTangentJacobian1,
            tangentImpulse1,
            row.otherTangentJacobian2,
            tangentImpulse2);
        addJointSpaceImpulse(
            otherBlock.inverseMass,
            scratch.generalizedImpulse,
            otherVelocity,
            scratch,
            -1.0);
      }

      if (row.otherBody != entt::null) {
        auto& obstacleVelocity = registry.get<comps::Velocity>(row.otherBody);
        const auto applyObstacle = [&](const Eigen::Vector3d& direction,
                                       double impulse) {
          obstacleVelocity.linear -= impulse * row.otherInvMass * direction;
          obstacleVelocity.angular
              -= impulse * row.otherInvInertia * row.otherArm.cross(direction);
        };
        applyObstacle(row.normal, normalImpulse);
        applyObstacle(row.tangent1, tangentImpulse1);
        applyObstacle(row.tangent2, tangentImpulse2);
      }
    }
  }
}

//==============================================================================
void primeUnifiedConstraintFallbackScratch(
    const UnifiedConstraintProblem& problem,
    UnifiedConstraintSolveScratch& scratch)
{
  const Eigen::Index size = problem.rhs.size();
  if (size == 0) {
    return;
  }

  auto& normalRows = scratch.normalRows;
  normalRows.clear();
  normalRows.reserve(static_cast<std::size_t>(size));
  for (Eigen::Index r = 0; r < size; ++r) {
    if (problem.findex[r] < 0) {
      normalRows.push_back(r);
    }
  }
  const auto normalCount = static_cast<Eigen::Index>(normalRows.size());

  scratch.normalLambda.setZero(normalCount);
  if (normalCount > 0) {
    scratch.normalA.resize(normalCount, normalCount);
    scratch.normalB.resize(normalCount);
    for (Eigen::Index a = 0; a < normalCount; ++a) {
      scratch.normalB[a] = problem.rhs[normalRows[static_cast<std::size_t>(a)]];
      for (Eigen::Index b = 0; b < normalCount; ++b) {
        scratch.normalA(a, b) = problem.delassus(
            normalRows[static_cast<std::size_t>(a)],
            normalRows[static_cast<std::size_t>(b)]);
      }
    }
    scratch.normalLo.setZero(normalCount);
    scratch.normalHi.setConstant(
        normalCount, std::numeric_limits<double>::infinity());
    scratch.normalFindex.setConstant(normalCount, -1);
    math::LcpOptions options = scratch.solver.getDefaultOptions();
    options.earlyTermination = true;
    const auto result = scratch.solver.solve(
        scratch.normalA,
        scratch.normalB,
        scratch.normalLo,
        scratch.normalHi,
        scratch.normalFindex,
        scratch.normalLambda,
        scratch.dantzig,
        options);
    if (!result.succeeded() || scratch.normalLambda.size() != normalCount) {
      scratch.normalLambda.resize(normalCount);
      for (Eigen::Index a = 0; a < normalCount; ++a) {
        const double diagonal = scratch.normalA(a, a);
        scratch.normalLambda[a]
            = diagonal > 0.0 ? std::max(0.0, scratch.normalB[a] / diagonal)
                             : 0.0;
      }
    }
  }

  scratch.lambda.setZero(size);
  for (Eigen::Index a = 0; a < normalCount; ++a) {
    scratch.lambda[normalRows[static_cast<std::size_t>(a)]]
        = std::max(0.0, scratch.normalLambda[a]);
  }

  const auto rigidContactCount = problem.rigidConstraints.size();
  scratch.rigidTangent1.assign(rigidContactCount, 0.0);
  scratch.rigidTangent2.assign(rigidContactCount, 0.0);
  scratch.linkTangentOffsets.resize(problem.multibodyBlocks.size() + 1);
  std::size_t totalLinkRows = 0;
  Eigen::Index maxJointDofs = 0;
  for (std::size_t k = 0; k < problem.multibodyBlocks.size(); ++k) {
    const auto& block = problem.multibodyBlocks[k];
    scratch.linkTangentOffsets[k] = totalLinkRows;
    totalLinkRows += block.rows.size();
    maxJointDofs = std::max(maxJointDofs, block.inverseMass.rows());
    maxJointDofs = std::max(maxJointDofs, block.inverseMass.cols());
  }
  scratch.linkTangentOffsets[problem.multibodyBlocks.size()] = totalLinkRows;
  scratch.linkTangent1.assign(totalLinkRows, 0.0);
  scratch.linkTangent2.assign(totalLinkRows, 0.0);
  scratch.generalizedImpulse.resize(maxJointDofs);
  scratch.velocityDelta.resize(maxJointDofs);
}

//==============================================================================
void applyUnifiedConstraintFallback(
    detail::WorldRegistry& registry,
    const UnifiedConstraintProblem& problem,
    std::span<Eigen::VectorXd> multibodyVelocities,
    std::size_t frictionIterations)
{
  UnifiedConstraintSolveScratch scratch;
  applyUnifiedConstraintFallback(
      registry, problem, multibodyVelocities, frictionIterations, scratch);
}

//==============================================================================
void applyUnifiedConstraintFallback(
    detail::WorldRegistry& registry,
    const UnifiedConstraintProblem& problem,
    std::span<Eigen::VectorXd> multibodyVelocities,
    std::size_t frictionIterations,
    UnifiedConstraintSolveScratch& scratch)
{
  constexpr int kRows = UnifiedConstraintProblem::kRowsPerContact;
  const Eigen::Index size = problem.rhs.size();
  if (size == 0) {
    return;
  }

  primeUnifiedConstraintFallbackScratch(problem, scratch);

  // 1. Apply the solved normal impulses to both domains (friction rows zero).
  applyUnifiedConstraintImpulses(
      registry, problem, scratch.lambda, multibodyVelocities);

  // 2. Sequential Coulomb friction sweep bounded by each contact's solved
  // normal impulse, over rigid contacts then link contacts (ascending global
  // order), reading live velocities each pass.
  const auto rigidContactCount = problem.rigidConstraints.size();

  const auto solveRigidTangent =
      [&](const RigidBodyContactConstraint& constraint,
          const Eigen::Vector3d& tangent,
          double tangentMass,
          double& accumulated,
          double limit) {
        if (tangentMass <= 0.0) {
          return;
        }
        const auto& velocityA = registry.get<comps::Velocity>(constraint.bodyA);
        const auto& velocityB = registry.get<comps::Velocity>(constraint.bodyB);
        const Eigen::Vector3d relativeVelocity
            = computeRigidBodyContactPointVelocity(
                  velocityB, constraint.armB, constraint.staticB)
              - computeRigidBodyContactPointVelocity(
                  velocityA, constraint.armA, constraint.staticA);
        const double tangentVelocity = relativeVelocity.dot(tangent);
        const double newImpulse = std::clamp(
            accumulated - tangentVelocity / tangentMass, -limit, limit);
        const double delta = newImpulse - accumulated;
        accumulated = newImpulse;
        applyRigidBodyContactImpulse(registry, constraint, delta * tangent);
      };

  const auto solveLinkTangent = [&](const UnifiedMultibodyBlock& block,
                                    const MultibodyLinkContactRow& row,
                                    Eigen::VectorXd& velocity,
                                    const Eigen::Vector3d& tangent,
                                    const Eigen::VectorXd& tangentJacobian,
                                    const Eigen::VectorXd& otherTangentJacobian,
                                    double tangentDenominator,
                                    double& accumulated,
                                    double limit) {
    if (tangentDenominator <= 0.0) {
      return;
    }
    double tangentVelocity = tangentJacobian.dot(velocity);
    if (row.otherBody != entt::null) {
      const auto& obstacleVelocity
          = registry.get<comps::Velocity>(row.otherBody);
      tangentVelocity -= (obstacleVelocity.linear
                          + obstacleVelocity.angular.cross(row.otherArm))
                             .dot(tangent);
    } else if (row.otherMultibodyIndex >= 0) {
      const auto& otherVelocity = multibodyVelocities[static_cast<std::size_t>(
          row.otherMultibodyIndex)];
      tangentVelocity -= otherTangentJacobian.dot(otherVelocity);
    }
    const double newImpulse = std::clamp(
        accumulated - tangentVelocity / tangentDenominator, -limit, limit);
    const double delta = newImpulse - accumulated;
    accumulated = newImpulse;
    setScaledVector(scratch.generalizedImpulse, tangentJacobian, delta);
    addJointSpaceImpulse(
        block.inverseMass, scratch.generalizedImpulse, velocity, scratch, 1.0);
    if (row.otherMultibodyIndex >= 0) {
      const auto& otherBlock = problem.multibodyBlocks[static_cast<std::size_t>(
          row.otherMultibodyIndex)];
      Eigen::VectorXd& otherVelocity
          = multibodyVelocities[static_cast<std::size_t>(
              row.otherMultibodyIndex)];
      setScaledVector(scratch.generalizedImpulse, otherTangentJacobian, delta);
      addJointSpaceImpulse(
          otherBlock.inverseMass,
          scratch.generalizedImpulse,
          otherVelocity,
          scratch,
          -1.0);
    } else if (row.otherBody != entt::null) {
      auto& obstacleVelocity = registry.get<comps::Velocity>(row.otherBody);
      obstacleVelocity.linear -= delta * row.otherInvMass * tangent;
      obstacleVelocity.angular
          -= delta * row.otherInvInertia * row.otherArm.cross(tangent);
    }
  };

  for (std::size_t iteration = 0; iteration < frictionIterations; ++iteration) {
    for (std::size_t i = 0; i < rigidContactCount; ++i) {
      const auto& constraint = problem.rigidConstraints[i];
      const Eigen::Index normalRow = static_cast<Eigen::Index>(i) * kRows;
      const double limit = constraint.friction * scratch.lambda[normalRow];
      if (limit <= 0.0) {
        continue;
      }
      solveRigidTangent(
          constraint,
          constraint.tangent1,
          constraint.tangentMass1,
          scratch.rigidTangent1[i],
          limit);
      solveRigidTangent(
          constraint,
          constraint.tangent2,
          constraint.tangentMass2,
          scratch.rigidTangent2[i],
          limit);
    }

    for (std::size_t k = 0; k < problem.multibodyBlocks.size(); ++k) {
      const auto& block = problem.multibodyBlocks[k];
      Eigen::VectorXd& velocity = multibodyVelocities[k];
      for (std::size_t c = 0; c < block.rows.size(); ++c) {
        const auto& row = block.rows[c];
        const Eigen::Index normalRow
            = block.blockBase + static_cast<Eigen::Index>(c) * kRows;
        const double limit = row.friction * scratch.lambda[normalRow];
        if (limit <= 0.0) {
          continue;
        }
        const std::size_t tangentIndex = scratch.linkTangentOffsets[k] + c;
        solveLinkTangent(
            block,
            row,
            velocity,
            row.tangent1,
            row.tangentJacobian1,
            row.otherTangentJacobian1,
            row.tangentDenominator1,
            scratch.linkTangent1[tangentIndex],
            limit);
        solveLinkTangent(
            block,
            row,
            velocity,
            row.tangent2,
            row.tangentJacobian2,
            row.otherTangentJacobian2,
            row.tangentDenominator2,
            scratch.linkTangent2[tangentIndex],
            limit);
      }
    }
  }
}

//==============================================================================
bool resolveUnifiedConstraints(
    detail::WorldRegistry& registry,
    const UnifiedConstraintProblem& problem,
    std::span<Eigen::VectorXd> multibodyVelocities,
    std::size_t frictionIterations)
{
  UnifiedConstraintSolveScratch scratch;
  return resolveUnifiedConstraints(
      registry, problem, multibodyVelocities, frictionIterations, scratch);
}

//==============================================================================
bool resolveUnifiedConstraints(
    detail::WorldRegistry& registry,
    const UnifiedConstraintProblem& problem,
    std::span<Eigen::VectorXd> multibodyVelocities,
    std::size_t frictionIterations,
    UnifiedConstraintSolveScratch& scratch)
{
  if (solveUnifiedConstraintProblemInto(problem, scratch)) {
    applyUnifiedConstraintImpulses(
        registry, problem, scratch.lambda, multibodyVelocities, scratch);
    return true;
  }
  applyUnifiedConstraintFallback(
      registry, problem, multibodyVelocities, frictionIterations, scratch);
  return false;
}

} // namespace dart::simulation::compute
