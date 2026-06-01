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

#pragma once

#include <dart/simulation/experimental/compute/multibody_dynamics.hpp>
#include <dart/simulation/experimental/compute/rigid_body_constraint.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <Eigen/Core>
#include <entt/entity/entity.hpp>

#include <span>
#include <vector>

namespace dart::simulation::experimental::compute {

/// Which solver domain a unified-constraint row belongs to.
enum class UnifiedContactDomain
{
  Rigid,
  Link,
};

/// Which direction of a contact a unified-constraint row drives.
enum class UnifiedContactDirection
{
  Normal = 0,
  Tangent1 = 1,
  Tangent2 = 2,
};

/// Provenance of one global row, so the solve can route the solved impulse back
/// to the right domain and the fallback can enumerate normal rows in ascending
/// global-row order.
struct UnifiedRowOwner
{
  UnifiedContactDomain domain = UnifiedContactDomain::Rigid;
  UnifiedContactDirection direction = UnifiedContactDirection::Normal;
  /// Global index of this contact's normal row (== own index for a normal row).
  Eigen::Index normalRowGlobalIndex = 0;
  /// Rigid: index into `rigidConstraints`. Link: index of the compacted active
  /// row within its multibody block.
  std::size_t sourceIndex = 0;
  /// -1 for a rigid row; otherwise the index into `multibodyBlocks`.
  int multibodyIndex = -1;
};

/// Input to the unified assembler: one multibody's already-assembled
/// link-contact problem paired with the `MultibodyStructure` entity that owns
/// it (used to route the solved generalized velocity back to its
/// `PendingMultibodyVelocity`).
struct UnifiedMultibodyContact
{
  entt::entity multibody = entt::null;
  MultibodyLinkContactProblem problem;
};

/// One multibody's compacted active link rows placed into the global system.
/// Inactive rows (a contact that cannot move either body, e.g. a fixed-base
/// link against an immovable obstacle) are dropped before layout so they never
/// enter the matrix nor make it singular.
struct UnifiedMultibodyBlock
{
  entt::entity multibody = entt::null;
  /// Global row index of this block's first row.
  Eigen::Index blockBase = 0;
  /// Compacted active rows, in their original (stable) order.
  std::vector<MultibodyLinkContactRow> rows;
  /// Joint-space inverse mass `M_k^-1`, the operator link impulses act through.
  Eigen::MatrixXd inverseMass;
};

/// The fully assembled unified contact-space boxed-LCP over all rigid-rigid and
/// articulated-link contacts.
///
/// Row layout is block-concatenated and deterministic: rows `[0, 3*Nr)` are the
/// rigid-rigid block in `assembleRigidBodyContactProblem` order (3-row stride:
/// normal, tangent1, tangent2), followed by one contiguous span per multibody
/// (in input order) of three rows per compacted active link contact. The
/// uniform 3-row stride holds across the whole system, and the normal rows
/// enumerated in ascending global-row order are exactly the block bases.
struct UnifiedConstraintProblem
{
  static constexpr int kRowsPerContact = 3;

  Eigen::MatrixXd delassus;
  Eigen::VectorXd rhs;
  Eigen::VectorXd lo;
  Eigen::VectorXd hi;
  Eigen::VectorXi findex;
  std::vector<UnifiedRowOwner> rowOwners;
  /// Verbatim copy of the rigid problem's accepted constraints (post
  /// effective-mass filter, same order), retained for impulse application and
  /// the rigid positional projection.
  std::vector<RigidBodyContactConstraint> rigidConstraints;
  /// Per-multibody compacted link blocks, retained for impulse application and
  /// the generalized fallback.
  std::vector<UnifiedMultibodyBlock> multibodyBlocks;
};

/// Assemble the unified contact-space boxed-LCP from an already-assembled rigid
/// problem and per-multibody link problems.
///
/// This is a pure function of its inputs: it copies the rigid problem's
/// `A,b,lo,hi,findex` into the leading block VERBATIM (so a multibody-free
/// input reproduces `assembleRigidBodyContactProblem` byte-for-byte), compacts
/// each multibody's active link rows, lays out their three-row triples after
/// the rigid block, and fills the full dense within-multibody Delassus coupling
/// `J_i^T M_k^-1 J_j`. The link rows' `lo/hi/findex` are computed against the
/// COMPACTED global row indices, and their right-hand sides are the pre-solve
/// targets carried on the rows.
///
/// It also fills the coupling through a dynamic rigid body shared between
/// contacts: a link contact's own obstacle self-term (so the diagonal completes
/// to the stored denominator when that body is not also a rigid participant),
/// link<->link coupling through a shared obstacle, and rigid<->link coupling
/// through a body that is both a rigid-contact participant and a link obstacle.
/// Such a shared body's `(invMass, invInertia)` is reconciled to one canonical
/// value (the rigid path's) so the assembled operator stays a consistent,
/// symmetric Delassus. With link-vs-link contacts dropped upstream, a shared
/// dynamic obstacle is the only cross-body coupling, so the assembled system is
/// the complete Delassus for the accepted contact set.
[[nodiscard]] DART_EXPERIMENTAL_API UnifiedConstraintProblem
assembleUnifiedConstraintProblem(
    const RigidBodyContactProblem& rigidProblem,
    std::span<const UnifiedMultibodyContact> multibodyContacts);

/// The result of a joint unified boxed-LCP solve.
struct UnifiedConstraintSolution
{
  Eigen::VectorXd
      lambda; ///< solved global impulses (empty for an empty problem)
  bool succeeded = false; ///< the joint Dantzig solve converged at full size
};

/// Solve the unified boxed-LCP `w = A*lambda - b`, `lo <= lambda <= hi` with
/// the Coulomb cone coupled through `findex`, jointly over all contacts.
///
/// Uses the Dantzig solver's default options with early termination, so a
/// rank-deficient contact set fails cleanly (`succeeded == false`) instead of
/// emitting a partial solution; the caller falls back in that case. An empty
/// problem succeeds trivially with an empty `lambda`. The options MUST come
/// from `getDefaultOptions()` (not a default-constructed `LcpOptions`), because
/// the validation/tolerance fields determine `succeeded()`.
[[nodiscard]] DART_EXPERIMENTAL_API UnifiedConstraintSolution
solveUnifiedConstraintProblem(const UnifiedConstraintProblem& problem);

/// Apply the solved global impulses back to both domains.
///
/// Rigid contacts apply their world-space impulse to both bodies' velocities
/// (via `applyRigidBodyContactImpulse`). Each link contact drives its owning
/// multibody's staged generalized velocity by `M_k^-1 J^T lambda` and applies
/// the equal-and-opposite impulse to a dynamic obstacle's velocity (Newton's
/// third law). `multibodyVelocities` is parallel to `problem.multibodyBlocks`
/// and is updated in place. Normal impulses are clamped non-negative (matching
/// the existing stages); every impulse is applied after the solve, so apply
/// order is irrelevant. A dynamic body shared between a rigid contact and a
/// link obstacle accumulates both impulses into the same velocity.
DART_EXPERIMENTAL_API void applyUnifiedConstraintImpulses(
    entt::registry& registry,
    const UnifiedConstraintProblem& problem,
    const Eigen::VectorXd& lambda,
    std::span<Eigen::VectorXd> multibodyVelocities);

} // namespace dart::simulation::experimental::compute
