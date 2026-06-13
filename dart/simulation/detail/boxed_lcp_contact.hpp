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

// @internal
//
// Implementation-detail header for the opt-in boxed-LCP rigid-body contact
// solve (PLAN-080 WS4). It is NOT a public facade header: it exposes the LCP
// Delassus system and the contact Jacobian, which must never appear in a public
// signature. It lives under `detail/` so Doxygen and the API-boundary checks
// treat it as internal, and it is not part of the DART 7 public-facade
// promotion target.
//
// The snapshot it carries is the seam PLAN-110 WS2 will differentiate: it holds
// the solved boxed LCP `{A, b, lo, hi, findex, f}` plus the stacked contact
// Jacobian `J` for the step. In-tree tests may include it directly; the solve
// entry point is marked DART_SIMULATION_API so a test/diff layer can link it
// across the experimental .so (DART applies strict symbol visibility).

#include <dart/simulation/detail/world_registry_types.hpp>
#include <dart/simulation/export.hpp>
#include <dart/simulation/fwd.hpp>

#include <dart/math/lcp/pivoting/dantzig_solver.hpp>

#include <dart/common/memory_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/fwd.hpp>

#include <span>
#include <unordered_map>
#include <vector>

#include <cstddef>

namespace dart::simulation::detail {

/// Snapshot of the boxed-LCP rigid-body contact solve for a single step.
///
/// This slice handles Coulomb-friction rigid-body contacts. The LCP stacks one
/// normal row per active contact followed by two tangential (friction) rows per
/// contact:
///
///   w = A f - b,   lo <= f <= hi,   complementary to w,
///
/// with `A = J M⁻¹ Jᵀ` (the Delassus operator over the stacked normal + tangent
/// Jacobian), `b = -(J v_free) + bias` (the restitution bias used by the
/// existing stage). The first `contactCount` rows are normal rows with
/// `lo = 0`, `hi = +∞`, `findex = -1`. The remaining `2 * contactCount` rows
/// are friction rows, each carrying an orthonormal tangent direction; a
/// friction row has `lo = -mu`, `hi = +mu` and `findex` set to its owning
/// normal row so the boxed solver applies the `±mu·f_normal` Coulomb box
/// automatically. `f` is the solved impulse (normal and tangential) and `J` is
/// the stacked contact Jacobian whose normal/tangent rows map stacked body
/// velocities to the per-contact normal/tangential relative velocity. The body
/// velocity update applied to the World is `Δv = M⁻¹ Jᵀ f`.
///
/// The plain Eigen-typed members are what PLAN-110 WS2 differentiates; the
/// member matrices are empty when there is no active rigid-body normal
/// constraint.
struct DART_SIMULATION_API BoxedLcpContactSnapshot
{
  /// Delassus operator `A = J M⁻¹ Jᵀ`, size `n x n` (`n` = active normals).
  Eigen::MatrixXd A;
  /// Right-hand side `b = J v_free + bias`, size `n`.
  Eigen::VectorXd b;
  /// Lower impulse bounds (all zero for normal constraints), size `n`.
  Eigen::VectorXd lo;
  /// Upper impulse bounds (all +∞ for normal constraints), size `n`.
  Eigen::VectorXd hi;
  /// Friction index per row: `-1` for normal rows, the owning normal row index
  /// for friction rows, size `n`.
  Eigen::VectorXi findex;
  /// Solved impulses (normal rows then friction rows), size `n`.
  Eigen::VectorXd f;
  /// Stacked contact Jacobian, size `n x (6 * bodyCount)` mapping the stacked
  /// body twists `[v; ω]` (per body) to the per-row normal/tangential relative
  /// speed.
  Eigen::MatrixXd J;

  /// Number of dynamic bodies whose velocities back the columns of `J`.
  std::size_t bodyCount = 0;
  /// Number of active contacts (the normal rows occupy `[0, contactCount)`;
  /// friction rows occupy `[contactCount, contactCount + 2 * contactCount)`).
  std::size_t contactCount = 0;

  [[nodiscard]] Eigen::Index size() const
  {
    return b.size();
  }
};

// Per-contact constraint, mirroring the sequential-impulse stage so the two
// paths assemble the same physics: a normal row plus two tangential friction
// rows spanning the contact plane (box Coulomb model).
struct BoxedLcpContactNormal
{
  entt::entity bodyA{entt::null};
  entt::entity bodyB{entt::null};
  bool staticA = false;
  bool staticB = false;
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d tangent1 = Eigen::Vector3d::UnitX();
  Eigen::Vector3d tangent2 = Eigen::Vector3d::UnitY();
  Eigen::Vector3d armA = Eigen::Vector3d::Zero();
  Eigen::Vector3d armB = Eigen::Vector3d::Zero();
  double bias = 0.0; // Baumgarte/restitution bias as a target normal velocity.
  double friction = 0.0; // Combined Coulomb friction coefficient mu.
};

struct DART_SIMULATION_API BoxedLcpContactScratch
{
  using NormalAllocator = dart::common::StlAllocator<BoxedLcpContactNormal>;
  using BodyColumnPair = std::pair<const entt::entity, std::size_t>;
  using BodyColumnAllocator = dart::common::StlAllocator<BodyColumnPair>;
  using BodyColumnMap = std::unordered_map<
      entt::entity,
      std::size_t,
      std::hash<entt::entity>,
      std::equal_to<entt::entity>,
      BodyColumnAllocator>;

  BoxedLcpContactScratch() = default;

  explicit BoxedLcpContactScratch(dart::common::MemoryAllocator& allocator)
    : normals(NormalAllocator{allocator}),
      bodyColumn(
          0u,
          std::hash<entt::entity>{},
          std::equal_to<entt::entity>{},
          BodyColumnAllocator{allocator}),
      dantzig(allocator)
  {
  }

  void reserve(std::size_t contactCapacity, std::size_t bodyCapacity);
  void clearProblem();

  std::vector<BoxedLcpContactNormal, NormalAllocator> normals;
  BodyColumnMap bodyColumn;
  BoxedLcpContactSnapshot snapshot;
  Eigen::MatrixXd Minv;
  Eigen::VectorXd vFree;
  Eigen::MatrixXd JMinv;
  Eigen::VectorXd jtImpulse;
  Eigen::VectorXd deltaV;
  math::DantzigSolver::Scratch dantzig;
};

DART_SIMULATION_API void reserveBoxedLcpContactScratch(
    const detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    BoxedLcpContactScratch& scratch);

/// Assemble and solve the Coulomb-friction boxed LCP for the active rigid-body
/// contacts of `world` (one normal row plus two tangential friction rows per
/// contact), apply the resulting impulses to the affected body velocities
/// (`Δv = M⁻¹ Jᵀ f`), and return the snapshot.
///
/// `contacts` is the active set from the existing collision query, filtered to
/// rigid-body/rigid-body pairs internally. `timeStep` drives the Baumgarte
/// position-error bias consistent with the sequential-impulse stage. The solve
/// uses the pivoting Dantzig boxed-LCP solver (`dart::math::DantzigSolver`).
///
/// @param registry  The world registry owning the contacting bodies (body
///                   velocities are mutated by the impulse application).
/// @param contacts  Active contacts from `World::collide()`.
/// @param timeStep  Integration time step (> 0), for the Baumgarte bias.
/// @return The solved `{A, b, lo, hi, findex, f, J}` snapshot. Empty matrices
///         when there is no active rigid-body normal constraint.
[[nodiscard]] DART_SIMULATION_API BoxedLcpContactSnapshot solveBoxedLcpContacts(
    detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    double timeStep);

[[nodiscard]] DART_SIMULATION_API BoxedLcpContactSnapshot&
solveBoxedLcpContacts(
    detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    double timeStep,
    BoxedLcpContactScratch& scratch);

} // namespace dart::simulation::detail
