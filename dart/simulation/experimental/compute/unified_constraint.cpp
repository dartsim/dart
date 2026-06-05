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

#include "dart/simulation/experimental/comps/dynamics.hpp"

#include <dart/math/lcp/lcp_types.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>

#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
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
  Eigen::VectorXd jacobian;
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
std::vector<std::vector<Eigen::Index>> computeRowIslands(
    const UnifiedConstraintProblem& problem)
{
  const Eigen::Index size = problem.rhs.size();
  std::vector<std::vector<Eigen::Index>> islands;
  std::vector<char> visited(static_cast<std::size_t>(size), 0);
  std::vector<Eigen::Index> stack;

  const auto visit = [&](Eigen::Index row) {
    if (row < 0 || row >= size) {
      return false;
    }
    auto& wasVisited = visited[static_cast<std::size_t>(row)];
    if (wasVisited != 0) {
      return false;
    }
    wasVisited = 1;
    stack.push_back(row);
    return true;
  };

  for (Eigen::Index start = 0; start < size; ++start) {
    if (!visit(start)) {
      continue;
    }

    std::vector<Eigen::Index> island;
    while (!stack.empty()) {
      const Eigen::Index row = stack.back();
      stack.pop_back();
      island.push_back(row);

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
    std::sort(island.begin(), island.end());
    islands.push_back(std::move(island));
  }

  return islands;
}

//==============================================================================
UnifiedConstraintSolution solveBoxedLcp(
    const Eigen::MatrixXd& delassus,
    const Eigen::VectorXd& rhs,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex)
{
  UnifiedConstraintSolution solution;
  const Eigen::Index size = rhs.size();
  if (size == 0) {
    solution.succeeded = true;
    return solution;
  }

  const math::LcpProblem lcpProblem(delassus, rhs, lo, hi, findex);
  math::DantzigSolver solver;
  // Pin the options to the solver default + early termination. A
  // default-constructed LcpOptions would change the validation/tolerance fields
  // that decide succeeded(), and thus flip the rank-deficient fallback
  // decision.
  math::LcpOptions options = solver.getDefaultOptions();
  options.earlyTermination = true;
  const auto result = solver.solve(lcpProblem, solution.lambda, options);
  solution.succeeded = result.succeeded() && solution.lambda.size() == size;
  return solution;
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
  std::unordered_map<entt::entity, int> multibodyBlockIndex;
  multibodyBlockIndex.reserve(problem.multibodyBlocks.size());
  for (std::size_t k = 0; k < problem.multibodyBlocks.size(); ++k) {
    multibodyBlockIndex[problem.multibodyBlocks[k].multibody]
        = static_cast<int>(k);
  }
  for (auto& block : problem.multibodyBlocks) {
    for (auto& row : block.rows) {
      if (row.otherMultibody == entt::null) {
        continue;
      }
      const auto it = multibodyBlockIndex.find(row.otherMultibody);
      row.otherMultibodyIndex
          = it == multibodyBlockIndex.end() ? -1 : it->second;
    }
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
  std::vector<std::vector<ArticulatedEnd>> rowArticulatedEnds(
      static_cast<std::size_t>(size));
  std::vector<char> rowIsLink(static_cast<std::size_t>(size), 0);
  for (Eigen::Index r = 0; r < size; ++r) {
    const auto& owner = problem.rowOwners[static_cast<std::size_t>(r)];
    auto& ends = rowEnds[static_cast<std::size_t>(r)];
    auto& articulatedEnds = rowArticulatedEnds[static_cast<std::size_t>(r)];
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
      articulatedEnds.push_back(
          {owner.multibodyIndex, +1.0, true, jacobianOf(row, owner.direction)});
      if (row.otherMultibodyIndex >= 0) {
        articulatedEnds.push_back(
            {row.otherMultibodyIndex,
             -1.0,
             false,
             otherJacobianOf(row, owner.direction)});
      }
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

  // --- Cross-multibody articulated coupling. The dense within-multibody pass
  // above already filled primary-primary terms for rows owned by the same
  // block. Cross-link rows add a second articulated end with sign -1; fill
  // every term involving such non-primary ends here, including their self
  // contribution and coupling against rows owned by the other multibody.
  for (Eigen::Index i = 0; i < size; ++i) {
    const auto& endsI = rowArticulatedEnds[static_cast<std::size_t>(i)];
    if (endsI.empty()) {
      continue;
    }
    for (Eigen::Index j = 0; j < size; ++j) {
      const auto& endsJ = rowArticulatedEnds[static_cast<std::size_t>(j)];
      if (endsJ.empty()) {
        continue;
      }
      double addend = 0.0;
      for (const auto& endI : endsI) {
        if (endI.multibodyIndex < 0) {
          continue;
        }
        for (const auto& endJ : endsJ) {
          if (endI.multibodyIndex != endJ.multibodyIndex) {
            continue;
          }
          if (endI.primary && endJ.primary) {
            continue; // filled by the dense within-multibody pass
          }
          const auto& block = problem.multibodyBlocks[static_cast<std::size_t>(
              endI.multibodyIndex)];
          addend += endI.sign * endJ.sign
                    * endJ.jacobian.dot(block.inverseMass * endI.jacobian);
        }
      }
      if (addend != 0.0) {
        problem.delassus(i, j) += addend;
      }
    }
  }

  return problem;
}

//==============================================================================
UnifiedConstraintSolution solveUnifiedConstraintProblem(
    const UnifiedConstraintProblem& problem)
{
  UnifiedConstraintSolution solution;
  const Eigen::Index size = problem.rhs.size();
  if (size == 0) {
    solution.succeeded = true;
    return solution;
  }

  const std::vector<std::vector<Eigen::Index>> islands
      = computeRowIslands(problem);
  if (islands.size() == 1
      && static_cast<Eigen::Index>(islands[0].size()) == size) {
    return solveBoxedLcp(
        problem.delassus, problem.rhs, problem.lo, problem.hi, problem.findex);
  }

  solution.lambda = Eigen::VectorXd::Zero(size);
  for (const auto& island : islands) {
    const auto islandSize = static_cast<Eigen::Index>(island.size());
    std::vector<Eigen::Index> localIndex(static_cast<std::size_t>(size), -1);
    for (Eigen::Index local = 0; local < islandSize; ++local) {
      localIndex[static_cast<std::size_t>(
          island[static_cast<std::size_t>(local)])] = local;
    }

    Eigen::MatrixXd islandDelassus(islandSize, islandSize);
    Eigen::VectorXd islandRhs(islandSize);
    Eigen::VectorXd islandLo(islandSize);
    Eigen::VectorXd islandHi(islandSize);
    Eigen::VectorXi islandFindex = Eigen::VectorXi::Constant(islandSize, -1);
    for (Eigen::Index row = 0; row < islandSize; ++row) {
      const Eigen::Index globalRow = island[static_cast<std::size_t>(row)];
      islandRhs[row] = problem.rhs[globalRow];
      islandLo[row] = problem.lo[globalRow];
      islandHi[row] = problem.hi[globalRow];
      const Eigen::Index globalFindex = problem.findex[globalRow];
      if (globalFindex >= 0) {
        if (globalFindex >= size) {
          solution.succeeded = false;
          return solution;
        }
        const Eigen::Index remapped
            = localIndex[static_cast<std::size_t>(globalFindex)];
        if (remapped < 0) {
          solution.succeeded = false;
          return solution;
        }
        islandFindex[row] = static_cast<int>(remapped);
      }
      for (Eigen::Index col = 0; col < islandSize; ++col) {
        islandDelassus(row, col) = problem.delassus(
            globalRow, island[static_cast<std::size_t>(col)]);
      }
    }

    const UnifiedConstraintSolution islandSolution = solveBoxedLcp(
        islandDelassus, islandRhs, islandLo, islandHi, islandFindex);
    if (!islandSolution.succeeded) {
      solution.succeeded = false;
      return solution;
    }
    for (Eigen::Index local = 0; local < islandSize; ++local) {
      solution.lambda[island[static_cast<std::size_t>(local)]]
          = islandSolution.lambda[local];
    }
  }

  solution.succeeded = true;
  return solution;
}

//==============================================================================
void applyUnifiedConstraintImpulses(
    detail::WorldRegistry& registry,
    const UnifiedConstraintProblem& problem,
    const Eigen::VectorXd& lambda,
    std::span<Eigen::VectorXd> multibodyVelocities)
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

      velocity += block.inverseMass
                  * (row.normalJacobian * normalImpulse
                     + row.tangentJacobian1 * tangentImpulse1
                     + row.tangentJacobian2 * tangentImpulse2);

      if (row.otherMultibodyIndex >= 0) {
        const auto& otherBlock
            = problem.multibodyBlocks[static_cast<std::size_t>(
                row.otherMultibodyIndex)];
        Eigen::VectorXd& otherVelocity
            = multibodyVelocities[static_cast<std::size_t>(
                row.otherMultibodyIndex)];
        otherVelocity -= otherBlock.inverseMass
                         * (row.otherNormalJacobian * normalImpulse
                            + row.otherTangentJacobian1 * tangentImpulse1
                            + row.otherTangentJacobian2 * tangentImpulse2);
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
void applyUnifiedConstraintFallback(
    detail::WorldRegistry& registry,
    const UnifiedConstraintProblem& problem,
    std::span<Eigen::VectorXd> multibodyVelocities,
    std::size_t frictionIterations)
{
  constexpr int kRows = UnifiedConstraintProblem::kRowsPerContact;
  const Eigen::Index size = problem.rhs.size();
  if (size == 0) {
    return;
  }

  // 1. Coupled NORMAL-only boxed-LCP. Gather the normal rows in ASCENDING
  // global-row order so a multibody-free set reproduces the legacy i*3 stride
  // byte-for-byte. The normal-row sub-block already carries the rigid-rigid,
  // within-multibody, and shared-obstacle normal coupling.
  std::vector<Eigen::Index> normalRows;
  normalRows.reserve(static_cast<std::size_t>(size));
  for (Eigen::Index r = 0; r < size; ++r) {
    if (problem.findex[r] < 0) {
      normalRows.push_back(r);
    }
  }
  const auto normalCount = static_cast<Eigen::Index>(normalRows.size());

  Eigen::VectorXd normalLambda = Eigen::VectorXd::Zero(normalCount);
  if (normalCount > 0) {
    Eigen::MatrixXd normalA(normalCount, normalCount);
    Eigen::VectorXd normalB(normalCount);
    for (Eigen::Index a = 0; a < normalCount; ++a) {
      normalB[a] = problem.rhs[normalRows[static_cast<std::size_t>(a)]];
      for (Eigen::Index b = 0; b < normalCount; ++b) {
        normalA(a, b) = problem.delassus(
            normalRows[static_cast<std::size_t>(a)],
            normalRows[static_cast<std::size_t>(b)]);
      }
    }
    const math::LcpProblem normalProblem(
        normalA,
        normalB,
        Eigen::VectorXd::Zero(normalCount),
        Eigen::VectorXd::Constant(
            normalCount, std::numeric_limits<double>::infinity()),
        Eigen::VectorXi::Constant(normalCount, -1));
    math::DantzigSolver solver;
    math::LcpOptions options = solver.getDefaultOptions();
    options.earlyTermination = true;
    const auto result = solver.solve(normalProblem, normalLambda, options);
    if (!result.succeeded() || normalLambda.size() != normalCount) {
      // Last resort: an uncoupled diagonal projection (works for both kinds;
      // the diagonal is positive for every active contact).
      normalLambda.resize(normalCount);
      for (Eigen::Index a = 0; a < normalCount; ++a) {
        const double diagonal = normalA(a, a);
        normalLambda[a]
            = diagonal > 0.0 ? std::max(0.0, normalB[a] / diagonal) : 0.0;
      }
    }
  }

  // 2. Apply the solved normal impulses to both domains (friction rows zero).
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(size);
  for (Eigen::Index a = 0; a < normalCount; ++a) {
    lambda[normalRows[static_cast<std::size_t>(a)]]
        = std::max(0.0, normalLambda[a]);
  }
  applyUnifiedConstraintImpulses(
      registry, problem, lambda, multibodyVelocities);

  // 3. Sequential Coulomb friction sweep bounded by each contact's solved
  // normal impulse, over rigid contacts then link contacts (ascending global
  // order), reading live velocities each pass.
  const auto rigidContactCount = problem.rigidConstraints.size();
  std::vector<double> rigidTangent1(rigidContactCount, 0.0);
  std::vector<double> rigidTangent2(rigidContactCount, 0.0);
  std::vector<std::vector<double>> linkTangent1(problem.multibodyBlocks.size());
  std::vector<std::vector<double>> linkTangent2(problem.multibodyBlocks.size());
  for (std::size_t k = 0; k < problem.multibodyBlocks.size(); ++k) {
    linkTangent1[k].assign(problem.multibodyBlocks[k].rows.size(), 0.0);
    linkTangent2[k].assign(problem.multibodyBlocks[k].rows.size(), 0.0);
  }

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
    velocity += block.inverseMass * tangentJacobian * delta;
    if (row.otherMultibodyIndex >= 0) {
      const auto& otherBlock = problem.multibodyBlocks[static_cast<std::size_t>(
          row.otherMultibodyIndex)];
      Eigen::VectorXd& otherVelocity
          = multibodyVelocities[static_cast<std::size_t>(
              row.otherMultibodyIndex)];
      otherVelocity -= otherBlock.inverseMass * otherTangentJacobian * delta;
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
      const double limit = constraint.friction * lambda[normalRow];
      if (limit <= 0.0) {
        continue;
      }
      solveRigidTangent(
          constraint,
          constraint.tangent1,
          constraint.tangentMass1,
          rigidTangent1[i],
          limit);
      solveRigidTangent(
          constraint,
          constraint.tangent2,
          constraint.tangentMass2,
          rigidTangent2[i],
          limit);
    }

    for (std::size_t k = 0; k < problem.multibodyBlocks.size(); ++k) {
      const auto& block = problem.multibodyBlocks[k];
      Eigen::VectorXd& velocity = multibodyVelocities[k];
      for (std::size_t c = 0; c < block.rows.size(); ++c) {
        const auto& row = block.rows[c];
        const Eigen::Index normalRow
            = block.blockBase + static_cast<Eigen::Index>(c) * kRows;
        const double limit = row.friction * lambda[normalRow];
        if (limit <= 0.0) {
          continue;
        }
        solveLinkTangent(
            block,
            row,
            velocity,
            row.tangent1,
            row.tangentJacobian1,
            row.otherTangentJacobian1,
            row.tangentDenominator1,
            linkTangent1[k][c],
            limit);
        solveLinkTangent(
            block,
            row,
            velocity,
            row.tangent2,
            row.tangentJacobian2,
            row.otherTangentJacobian2,
            row.tangentDenominator2,
            linkTangent2[k][c],
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
  const auto solution = solveUnifiedConstraintProblem(problem);
  if (solution.succeeded) {
    applyUnifiedConstraintImpulses(
        registry, problem, solution.lambda, multibodyVelocities);
    return true;
  }
  applyUnifiedConstraintFallback(
      registry, problem, multibodyVelocities, frictionIterations);
  return false;
}

} // namespace dart::simulation::experimental::compute
