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

#include "dart/simulation/detail/contact_jacobians.hpp"

#include "dart/simulation/body/collision_body.hpp"
#include "dart/simulation/body/contact.hpp"
#include "dart/simulation/comps/contact_material.hpp"
#include "dart/simulation/comps/dynamics.hpp"
#include "dart/simulation/comps/rigid_body.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/rigid_contact/rigid_contact_assembly.hpp"

#include <dart/math/lcp/lcp_solver.hpp>
#include <dart/math/lcp/lcp_types.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>

#include <dart/common/stl_allocator.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <limits>
#include <span>
#include <unordered_map>
#include <vector>

#include <cmath>
#include <cstddef>

namespace dart::simulation::detail {

namespace {

// Clamping threshold: a normal impulse above this (in absolute terms) marks a
// CLAMPING (active) row whose complementarity slack is zero. Below it the row
// is SEPARATING and contributes zero impulse gradient. Matches the plan's 1e-9.
constexpr double kClampingTolerance = 1e-9;

// Constraint-force-mixing applied to the friction-row diagonal, mirroring
// solveBoxedLcpContacts so the differentiated Delassus matches the forward
// boxed-LCP step exactly.
constexpr double kConstraintForceMixing = kRigidContactFrictionCfm;

// PreContactSurrogate (PLAN-110 WS5): the fraction of the approach-direction
// velocity sensitivity the surrogate arrests for a body that is approaching but
// not yet touching. 1.0 mimics a hard clamp the instant before contact; this is
// a backward-only straight-through surrogate, not the true (zero) derivative.
constexpr double kPreContactSurrogateWeight = 1.0;

// PreContactSurrogate: a free linear speed below this (m/s) is treated as "not
// meaningfully approaching"; the surrogate then falls back to the gravity
// direction so a body released from rest still gets a toward-contact gradient.
constexpr double kPreContactApproachSpeedTolerance = 1e-9;

//==============================================================================
// A frozen frictionless normal contact: the geometry (which bodies, the world
// normal, the lever arms) is captured once at the pre-step state and held fixed
// while the body positions/velocities are perturbed for finite differencing of
// the smooth terms. This keeps the active set fixed (no collision requery), so
// the FD-of-terms inputs to the analytic A_CC⁻¹ formula stay smooth.
struct FrozenContact
{
  entt::entity bodyA{entt::null};
  entt::entity bodyB{entt::null};
  bool prescribedA = false;
  bool prescribedB = false;
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d tangent1 = Eigen::Vector3d::UnitX();
  Eigen::Vector3d tangent2 = Eigen::Vector3d::UnitY();
  Eigen::Vector3d armA = Eigen::Vector3d::Zero();
  Eigen::Vector3d armB = Eigen::Vector3d::Zero();
  double restitution = 0.0;
  double friction = 0.0; // Combined Coulomb friction coefficient mu.
};

using FrozenContactAllocator = common::StlAllocator<FrozenContact>;
using FrozenContactVector = std::vector<FrozenContact, FrozenContactAllocator>;
using ContactEntityAllocator = common::StlAllocator<entt::entity>;
using ContactEntityVector = std::vector<entt::entity, ContactEntityAllocator>;
using ContactBodyColumnEntry = std::pair<const entt::entity, std::size_t>;
using ContactBodyColumnAllocator = common::StlAllocator<ContactBodyColumnEntry>;
using ContactBodyColumnMap = std::unordered_map<
    entt::entity,
    std::size_t,
    std::hash<entt::entity>,
    std::equal_to<entt::entity>,
    ContactBodyColumnAllocator>;
using DoubleAllocator = common::StlAllocator<double>;
using DoubleVector = std::vector<double, DoubleAllocator>;
using InverseWorldInertiaAllocator = common::StlAllocator<Eigen::Matrix3d>;
using InverseWorldInertiaVector
    = std::vector<Eigen::Matrix3d, InverseWorldInertiaAllocator>;
using BoolAllocator = common::StlAllocator<bool>;
using BoolVector = std::vector<bool, BoolAllocator>;
using IndexAllocator = common::StlAllocator<Eigen::Index>;
using IndexVector = std::vector<Eigen::Index, IndexAllocator>;

//==============================================================================
// Scene captured at the pre-step state: the frozen contacts, the dynamic-body
// column layout (in stable order), and the bodies' inverse mass / inverse world
// inertia. The differentiated OUTPUT state is translational (3 DOFs per body),
// but the LCP solve and the impulse->velocity map use the full 6-DOF dynamics
// per body, so both normal and tangential impulses pick up the correct
// rotational effective mass through the screw-axis term arm x dir in the
// angular rows of J. This makes the gradient correct for contacts whose lever
// arm is NOT parallel to the normal (off-COM contacts that excite rotation) and
// for multiple simultaneous contacts (e.g. a box on its corners): the angular
// coupling enters A, the solved impulse, and the linear part of Minv Jᵀ f that
// drives the translational output. The output stays translational because the
// acquired angular velocity does not feed back into the translational state
// within a single step.
struct ContactScene
{
  ContactScene() : ContactScene(common::MemoryAllocator::GetDefault()) {}

  explicit ContactScene(common::MemoryAllocator& allocator)
    : contacts(FrozenContactAllocator{allocator}),
      dynamicBodies(ContactEntityAllocator{allocator}),
      bodyColumn(
          0,
          std::hash<entt::entity>{},
          std::equal_to<entt::entity>{},
          ContactBodyColumnAllocator{allocator}),
      inverseMass(DoubleAllocator{allocator}),
      inverseWorldInertia(InverseWorldInertiaAllocator{allocator})
  {
  }

  FrozenContactVector contacts;
  ContactEntityVector dynamicBodies; // column k -> entity
  ContactBodyColumnMap bodyColumn;
  DoubleVector inverseMass;                      // per dynamic body
  InverseWorldInertiaVector inverseWorldInertia; // per dynamic body
};

//==============================================================================
// A single nonzero 6-column block of one contact-Jacobian row: the row's
// contribution to one dynamic body's stacked twist [v; ω]. A frozen contact row
// touches at most two bodies (its bodyA and bodyB), so each row carries at most
// two of these. Storing J this way (instead of a dense rows x 6*nb matrix) is
// what makes the assembly scale: products with the block-diagonal Minv and the
// Delassus operator A = J Minv Jᵀ only ever touch the few bodies a contact
// actually references, never the full 6*nb velocity space.
struct JacobianBlock
{
  Eigen::Index body;                 // dynamic body (column) index
  Eigen::Matrix<double, 6, 1> value; // [direction; arm x direction]
};

using JacobianBlockAllocator = common::StlAllocator<JacobianBlock>;
using JacobianBlockVector = std::vector<JacobianBlock, JacobianBlockAllocator>;
using JacobianRowAllocator = common::StlAllocator<JacobianBlockVector>;
using JacobianRows = std::vector<JacobianBlockVector, JacobianRowAllocator>;
using IndexRowAllocator = common::StlAllocator<IndexVector>;
using IndexRows = std::vector<IndexVector, IndexRowAllocator>;
using IndexMapEntry = std::pair<const Eigen::Index, Eigen::Index>;
using IndexMapAllocator = common::StlAllocator<IndexMapEntry>;
using IndexMap = std::unordered_map<
    Eigen::Index,
    Eigen::Index,
    std::hash<Eigen::Index>,
    std::equal_to<Eigen::Index>,
    IndexMapAllocator>;

//==============================================================================
// Stacked quantities derived from the (possibly perturbed) registry state for a
// frozen scene. The LCP has `rows = n + 2*n` rows: the first n are normal rows,
// the following 2*n are friction rows (two tangents per contact, friction row
// 2*i+t owned by normal row i). The velocity space is the full 6-DOF-per-body
// twist `[v; ω]` (size `vdofs = 6*nb`). The inverse mass/inertia (Minv) is
// block-diagonal and lives on the ContactScene as per-body 3x3 blocks; it is
// applied block-wise rather than as a dense 6*nb x 6*nb matrix. J is stored
// block-sparsely (per row, the few nonzero body blocks). A is a dense rows x
// rows matrix (the intrinsic LCP system size) but is ASSEMBLED block-sparsely
// from J and Minv, so its cost is O(contacts) not O(bodies^2). b is rows.
struct ContactTerms
{
  ContactTerms() : ContactTerms(common::MemoryAllocator::GetDefault()) {}

  explicit ContactTerms(common::MemoryAllocator& allocator)
    : memoryAllocator(&allocator),
      J(JacobianRowAllocator{allocator}),
      rowsOfBody(IndexRowAllocator{allocator})
  {
  }

  void resizeRows(std::size_t rows, std::size_t bodies)
  {
    J.clear();
    J.reserve(rows);
    for (std::size_t i = 0; i < rows; ++i) {
      J.emplace_back(JacobianBlockAllocator{*memoryAllocator});
    }

    rowsOfBody.clear();
    rowsOfBody.reserve(bodies);
    for (std::size_t i = 0; i < bodies; ++i) {
      rowsOfBody.emplace_back(IndexAllocator{*memoryAllocator});
    }
  }

  common::MemoryAllocator* memoryAllocator;
  Eigen::VectorXd vFree; // free twist, stacked 6*nb
  JacobianRows J;        // per row, nonzero body blocks
  IndexRows rowsOfBody;  // body -> rows touching it
  Eigen::MatrixXd
      A; // Delassus operator J Minv Jᵀ (+ friction CFM), rows x rows
  Eigen::VectorXd b; // rhs -(J vFree) + bias, rows
};

//==============================================================================
// Apply the block-diagonal inverse mass/inertia to one body's stacked twist
// block: linear part scaled by inverse mass, angular part by the inverse world
// inertia. Mirrors the dense Minv block layout (invMass*I, then invInertia).
inline Eigen::Matrix<double, 6, 1> applyBodyMinv(
    const ContactScene& scene,
    Eigen::Index body,
    const Eigen::Matrix<double, 6, 1>& twist)
{
  const auto b = static_cast<std::size_t>(body);
  Eigen::Matrix<double, 6, 1> out;
  out.head<3>() = scene.inverseMass[b] * twist.head<3>();
  out.tail<3>() = scene.inverseWorldInertia[b] * twist.tail<3>();
  return out;
}

//==============================================================================
// Dot a block-sparse Jacobian row with a stacked 6*nb twist vector: sum the
// row's body blocks against the corresponding body segments. This is the
// block-sparse equivalent of J.row(r).dot(v).
inline double rowDotTwist(
    const JacobianBlockVector& row, const Eigen::VectorXd& twist)
{
  double value = 0.0;
  for (const auto& block : row) {
    value += block.value.dot(twist.segment<6>(6 * block.body));
  }
  return value;
}

//==============================================================================
// Compute the stacked twist Minv Jᵀ f (size 6*nb) block-wise. (Jᵀ f) for a body
// is the sum over the rows touching it of that row's block scaled by the row's
// impulse; Minv is then applied per body. Only the bodies that contacts touch
// are visited, so this is O(contacts) rather than the dense vdofs x rows
// product. This is the impulse->velocity map used by the constrained next state
// and the ∂x'/∂f contribution.
inline Eigen::VectorXd minvJtTimes(
    const ContactTerms& terms,
    const ContactScene& scene,
    Eigen::Index vdofs,
    const Eigen::VectorXd& f)
{
  Eigen::VectorXd out = Eigen::VectorXd::Zero(vdofs);
  const auto rows = static_cast<Eigen::Index>(terms.J.size());
  for (Eigen::Index r = 0; r < rows; ++r) {
    const double fr = f[r];
    if (fr == 0.0) {
      continue;
    }
    for (const auto& block : terms.J[static_cast<std::size_t>(r)]) {
      out.segment<6>(6 * block.body) += fr * block.value;
    }
  }
  for (Eigen::Index body = 0; body < vdofs / 6; ++body) {
    out.segment<6>(6 * body)
        = applyBodyMinv(scene, body, out.segment<6>(6 * body));
  }
  return out;
}

//==============================================================================
ContactScene buildScene(
    detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    common::MemoryAllocator& allocator)
{
  ContactScene scene(allocator);

  const auto registerBody = [&](entt::entity entity, bool isPrescribed) {
    if (isPrescribed) {
      return;
    }
    if (scene.bodyColumn.find(entity) == scene.bodyColumn.end()) {
      scene.bodyColumn.emplace(entity, scene.dynamicBodies.size());
      scene.dynamicBodies.push_back(entity);
    }
  };

  // Match World's baked dense rigid-body index order. Contact-touching bodies
  // no longer lead the state layout; every dynamic body is registered by
  // creation-order index first, then contacts only reference those columns.
  ContactEntityVector dynamicEntities(ContactEntityAllocator{allocator});
  auto dynamicView = registry.view<
      comps::RigidBodyTag,
      comps::Transform,
      comps::Velocity,
      comps::MassProperties,
      comps::Force>();
  dynamicEntities.reserve(dynamicView.size_hint());
  for (const auto entity : dynamicView) {
    dynamicEntities.push_back(entity);
  }
  std::ranges::sort(dynamicEntities, [](auto lhs, auto rhs) {
    return entt::to_integral(lhs) < entt::to_integral(rhs);
  });
  for (const auto entity : dynamicEntities) {
    if (hasPrescribedRigidBodyContactResponse(registry, entity)) {
      continue;
    }
    registerBody(entity, false);
  }

  for (const auto& contact : contacts) {
    const auto entityA = detail::toRegistryEntity(contact.bodyA.getEntity());
    const auto entityB = detail::toRegistryEntity(contact.bodyB.getEntity());

    // Rigid-body pairs only (parity with solveBoxedLcpContacts).
    if (!registry.all_of<comps::RigidBodyTag>(entityA)
        || !registry.all_of<comps::RigidBodyTag>(entityB)) {
      continue;
    }
    const bool prescribedA
        = hasPrescribedRigidBodyContactResponse(registry, entityA);
    const bool prescribedB
        = hasPrescribedRigidBodyContactResponse(registry, entityB);
    if (prescribedA && prescribedB) {
      continue;
    }

    const auto& transformA = registry.get<comps::Transform>(entityA);
    const auto& transformB = registry.get<comps::Transform>(entityB);

    FrozenContact frozen;
    frozen.bodyA = entityA;
    frozen.bodyB = entityB;
    frozen.prescribedA = prescribedA;
    frozen.prescribedB = prescribedB;
    frozen.normal = contact.normal;
    frozen.tangent1 = contact.normal.unitOrthogonal();
    frozen.tangent2 = contact.normal.cross(frozen.tangent1);
    frozen.armA = contact.point - transformA.position;
    frozen.armB = contact.point - transformB.position;
    frozen.restitution = std::max(
        restitutionOf(registry, entityA), restitutionOf(registry, entityB));
    frozen.friction = std::sqrt(
        frictionOf(registry, entityA) * frictionOf(registry, entityB));

    registerBody(entityA, prescribedA);
    registerBody(entityB, prescribedB);
    scene.contacts.push_back(frozen);
  }

  scene.inverseMass.resize(scene.dynamicBodies.size());
  scene.inverseWorldInertia.resize(scene.dynamicBodies.size());
  for (std::size_t k = 0; k < scene.dynamicBodies.size(); ++k) {
    const auto& mass
        = registry.get<comps::MassProperties>(scene.dynamicBodies[k]);
    const auto& transform
        = registry.get<comps::Transform>(scene.dynamicBodies[k]);
    scene.inverseMass[k] = inverseMassOf(mass);
    scene.inverseWorldInertia[k] = inverseWorldInertiaOf(mass, transform);
  }

  return scene;
}

//==============================================================================
// Recompute the smooth contact terms (vFree, J, A, b) for the frozen scene at
// the registry's current body state. The contact geometry (normal, arms, body
// membership) is held fixed from the scene; only the velocity-dependent bias
// and the free velocity read the live state. J is configuration independent for
// a frozen normal contact, so it is constant across perturbations; recomputing
// it keeps the assembly self-contained.
//
// SCALING: J is stored block-sparsely (each row keeps only the few body blocks
// it touches) and the block-diagonal inverse mass/inertia is applied per body
// from the scene. The Delassus operator A = J Minv Jᵀ is therefore assembled
// only over row pairs that SHARE a body (independent contacts never couple), so
// computeTerms is O(contacts) rather than the previous O(bodies^3) dense
// rows x 6*nb x 6*nb products.
ContactTerms computeTerms(
    const detail::WorldRegistry& registry,
    const ContactScene& scene,
    const Eigen::Vector3d& gravity,
    double timeStep,
    common::MemoryAllocator& allocator)
{
  const Eigen::Index nb = static_cast<Eigen::Index>(scene.dynamicBodies.size());
  const Eigen::Index vdofs = 6 * nb; // full 6-DOF twist per body
  const Eigen::Index n = static_cast<Eigen::Index>(scene.contacts.size());

  ContactTerms terms(allocator);
  terms.vFree = Eigen::VectorXd::Zero(vdofs);

  // Current (pre-integration) twist, stacked like vFree. The forward boxed-LCP
  // step computes its restitution approach velocity from the CURRENT body
  // velocity (solveBoxedLcpContacts reads velocity.linear/angular directly, not
  // the post-gravity free velocity), so the gradient's restitution bias below
  // must use the same quantity to match the forward step and to differentiate
  // the bias correctly.
  Eigen::VectorXd vCurrent = Eigen::VectorXd::Zero(vdofs);

  // Free (unconstrained) twist per dynamic body:
  //   v_free = v + Δt (F/m + g),   ω_free = ω + Δt invInertia τ.
  for (Eigen::Index k = 0; k < nb; ++k) {
    const auto entity = scene.dynamicBodies[static_cast<std::size_t>(k)];
    const auto& velocity = registry.get<comps::Velocity>(entity);
    const auto& force = registry.get<comps::Force>(entity);
    const double invMass = scene.inverseMass[static_cast<std::size_t>(k)];
    const double mass = (invMass > 0.0) ? 1.0 / invMass : 0.0;
    const Eigen::Matrix3d invInertia
        = scene.inverseWorldInertia[static_cast<std::size_t>(k)];

    const Eigen::Index base = 6 * k;
    vCurrent.segment<3>(base) = velocity.linear;
    vCurrent.segment<3>(base + 3) = velocity.angular;

    Eigen::Vector3d acceleration = invMass * force.force;
    if (mass > 0.0) {
      acceleration += gravity;
    }
    terms.vFree.segment<3>(base) = velocity.linear + timeStep * acceleration;
    terms.vFree.segment<3>(base + 3)
        = velocity.angular + timeStep * (invInertia * force.torque);
  }

  // Contact Jacobian (full 6-DOF rows), stored block-sparsely. A normal row
  // maps the stacked body twists to (v_B + ω_B × armB - v_A - ω_A × armA) . n;
  // a friction row uses the tangent direction. Row layout matches
  // solveBoxedLcpContacts: normal rows [0, n), then friction rows n + 2*i
  // (tangent1) and n + 2*i + 1 (tangent2). Each row keeps at most two body
  // blocks (its bodyB with +direction, its bodyA with -direction). rowsOfBody
  // is the inverse index used to assemble A only over coupled row pairs.
  const Eigen::Index rows = n + 2 * n;
  terms.resizeRows(
      static_cast<std::size_t>(rows), static_cast<std::size_t>(nb));
  const auto fillRow = [&](Eigen::Index row,
                           const FrozenContact& contact,
                           const Eigen::Vector3d& direction) {
    auto& rowBlocks = terms.J[static_cast<std::size_t>(row)];
    const auto addBlock
        = [&](Eigen::Index body, double sign, const Eigen::Vector3d& arm) {
            Eigen::Matrix<double, 6, 1> value;
            value.head<3>() = sign * direction;
            value.tail<3>() = sign * arm.cross(direction);
            // A row could in principle reference the same body for both A and
            // B; sum into the existing block to keep the dense-J equivalence
            // exact.
            for (auto& existing : rowBlocks) {
              if (existing.body == body) {
                existing.value += value;
                return;
              }
            }
            rowBlocks.push_back({body, value});
            terms.rowsOfBody[static_cast<std::size_t>(body)].push_back(row);
          };
    if (!contact.prescribedB) {
      addBlock(
          static_cast<Eigen::Index>(scene.bodyColumn.at(contact.bodyB)),
          +1.0,
          contact.armB);
    }
    if (!contact.prescribedA) {
      addBlock(
          static_cast<Eigen::Index>(scene.bodyColumn.at(contact.bodyA)),
          -1.0,
          contact.armA);
    }
  };
  for (Eigen::Index i = 0; i < n; ++i) {
    const auto& contact = scene.contacts[static_cast<std::size_t>(i)];
    fillRow(i, contact, contact.normal);
    fillRow(n + 2 * i, contact, contact.tangent1);
    fillRow(n + 2 * i + 1, contact, contact.tangent2);
  }

  // Delassus operator A = J Minv Jᵀ and rhs b = -(J vFree) + bias. The bias is
  // the restitution target from the pre-solve approach velocity (matches
  // solveBoxedLcpContacts). For a firmly resting (clamping) contact the
  // approach is non-negative, so the restitution bias is zero and contributes
  // no gradient. Friction rows carry no bias.
  //
  // A(r,c) = Σ_b J_r[b]ᵀ Minv_b J_c[b], which is nonzero only when rows r and c
  // share a body b. Iterating per body over the rows that touch it accumulates
  // exactly these couplings (a row pair sharing two bodies gets a contribution
  // from each), so A is filled in O(Σ_b rowsOfBody[b]^2) instead of by a dense
  // triple product over the full velocity space.
  terms.A = Eigen::MatrixXd::Zero(rows, rows);
  for (Eigen::Index body = 0; body < nb; ++body) {
    const auto& bodyRows = terms.rowsOfBody[static_cast<std::size_t>(body)];
    for (const Eigen::Index r : bodyRows) {
      // Find r's block for this body (each row has at most one block per body).
      const auto& rBlocks = terms.J[static_cast<std::size_t>(r)];
      const JacobianBlock* rBlock = nullptr;
      for (const auto& blk : rBlocks) {
        if (blk.body == body) {
          rBlock = &blk;
          break;
        }
      }
      const Eigen::Matrix<double, 6, 1> Minv_r
          = applyBodyMinv(scene, body, rBlock->value);
      for (const Eigen::Index c : bodyRows) {
        const auto& cBlocks = terms.J[static_cast<std::size_t>(c)];
        for (const auto& blk : cBlocks) {
          if (blk.body == body) {
            terms.A(r, c) += blk.value.dot(Minv_r);
            break;
          }
        }
      }
    }
  }
  // Friction-row CFM, mirroring solveBoxedLcpContacts so the differentiated
  // system equals the forward step's.
  for (Eigen::Index row = n; row < rows; ++row) {
    terms.A(row, row) += terms.A(row, row) * kConstraintForceMixing;
  }

  terms.b = Eigen::VectorXd::Zero(rows);
  for (Eigen::Index row = 0; row < rows; ++row) {
    terms.b[row]
        = -rowDotTwist(terms.J[static_cast<std::size_t>(row)], terms.vFree);
  }

  constexpr double restitutionThreshold = kRigidContactRestitutionThreshold;
  for (Eigen::Index i = 0; i < n; ++i) {
    const auto& contact = scene.contacts[static_cast<std::size_t>(i)];
    if (!(contact.restitution > 0.0)) {
      continue;
    }
    // Approach velocity at the CURRENT (pre-integration) twist: J_i vCurrent
    // (row i is the normal row, so this is the normal approach (v_B - v_A) . n
    // including any angular contribution). This matches solveBoxedLcpContacts,
    // which reads the current velocity for its restitution target; using vFree
    // here would mismatch the forward bias by the gravity/force term and give a
    // wrong restitution gradient. In a STABLE clamping regime (approach more
    // negative than -restitutionThreshold throughout the FD perturbations) the
    // -e * approach term is smooth, so the central FD of b captures ∂b/∂z and
    // the analytic restitution gradient stays correct. The make/break instant
    // (approach crossing -restitutionThreshold) is a non-smooth boundary and is
    // deliberately out of scope.
    const double approach
        = rowDotTwist(terms.J[static_cast<std::size_t>(i)], vCurrent);
    if (approach < -restitutionThreshold) {
      terms.b[i] += -contact.restitution * approach;
    }
  }

  return terms;
}

//==============================================================================
// Solve the frozen friction-augmented boxed LCP for the contact impulses with
// the same pivoting Dantzig solver the forward step uses. Normal rows: lo = 0,
// hi = +inf, findex = -1. Friction rows: lo = -mu, hi = +mu, findex = owning
// normal row. The solution equals the forward step's solved impulse for the
// given terms.
Eigen::VectorXd solveBoxedLcp(const ContactScene& scene, const ContactTerms& t)
{
  const Eigen::Index n = static_cast<Eigen::Index>(scene.contacts.size());
  const Eigen::Index rows = n + 2 * n;
  Eigen::VectorXd impulse = Eigen::VectorXd::Zero(rows);
  if (rows == 0) {
    return impulse;
  }

  Eigen::VectorXd loBounds = Eigen::VectorXd::Zero(rows);
  Eigen::VectorXd hiBounds = Eigen::VectorXd::Constant(
      rows, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(rows, -1);
  for (Eigen::Index i = 0; i < n; ++i) {
    const double mu = scene.contacts[static_cast<std::size_t>(i)].friction;
    for (Eigen::Index r = 0; r < 2; ++r) {
      const Eigen::Index row = n + 2 * i + r;
      loBounds[row] = -mu;
      hiBounds[row] = mu;
      findex[row] = static_cast<int>(i);
    }
  }

  math::DantzigSolver solver;
  math::LcpOptions options;
  options.warmStart = false;
  options.validateSolution = false;
  const math::LcpProblem problem(t.A, t.b, loBounds, hiBounds, findex);
  solver.solve(problem, impulse, options);
  for (Eigen::Index i = 0; i < n; ++i) {
    if (!std::isfinite(impulse[i]) || impulse[i] < 0.0) {
      impulse[i] = 0.0;
    }
  }
  for (Eigen::Index row = n; row < rows; ++row) {
    if (!std::isfinite(impulse[row])) {
      impulse[row] = 0.0;
    }
  }
  return impulse;
}

//==============================================================================
// Evaluate the full constrained translational next state x' = [q'; q̇'] for the
// frozen scene at the registry's current body state, with a FRESH boxed-LCP
// solve (so a change in a body parameter that alters the impulse is captured
// exactly). q̇' = linearOf(vFree + Minv Jᵀ f); q' = q + Δt q̇'. This mirrors the
// translational output the state/control Jacobian differentiates and is the map
// the parameter Jacobian central-differences.
Eigen::VectorXd constrainedNextState(
    const detail::WorldRegistry& registry,
    const ContactScene& scene,
    const Eigen::Vector3d& gravity,
    double timeStep,
    common::MemoryAllocator& allocator)
{
  const Eigen::Index nb = static_cast<Eigen::Index>(scene.dynamicBodies.size());
  const Eigen::Index dofs = 3 * nb;
  const Eigen::Index vdofs = 6 * nb;
  const ContactTerms t
      = computeTerms(registry, scene, gravity, timeStep, allocator);
  const Eigen::VectorXd f = solveBoxedLcp(scene, t);
  const Eigen::VectorXd twistNext = t.vFree + minvJtTimes(t, scene, vdofs, f);

  Eigen::VectorXd qdotNext(dofs);
  Eigen::VectorXd q(dofs);
  for (Eigen::Index k = 0; k < nb; ++k) {
    qdotNext.segment<3>(3 * k) = twistNext.segment<3>(6 * k);
    const auto entity = scene.dynamicBodies[static_cast<std::size_t>(k)];
    q.segment<3>(3 * k) = registry.get<comps::Transform>(entity).position;
  }

  Eigen::VectorXd state(2 * dofs);
  state.head(dofs) = q + timeStep * qdotNext;
  state.tail(dofs) = qdotNext;
  return state;
}

} // namespace

//==============================================================================
StepDerivatives contactStepDerivatives(
    detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    const Eigen::Vector3d& gravity,
    double timeStep,
    ContactGradientMode mode)
{
  return contactStepDerivatives(
      registry,
      contacts,
      gravity,
      timeStep,
      mode,
      common::MemoryAllocator::GetDefault());
}

//==============================================================================
StepDerivatives contactStepDerivatives(
    detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    const Eigen::Vector3d& gravity,
    double timeStep,
    ContactGradientMode mode,
    common::MemoryAllocator& allocator)
{
  StepDerivatives derivatives;

  const ContactScene scene = buildScene(registry, contacts, allocator);
  const Eigen::Index nb = static_cast<Eigen::Index>(scene.dynamicBodies.size());
  if (nb == 0) {
    return derivatives;
  }

  const Eigen::Index dofs = 3 * nb;        // translational output DOFs per body
  const Eigen::Index vdofs = 6 * nb;       // full 6-DOF twist per body
  const Eigen::Index stateSize = 2 * dofs; // [q; q̇]
  const Eigen::Index n = static_cast<Eigen::Index>(scene.contacts.size());
  const Eigen::Index rows = n + 2 * n; // normal rows then friction rows

  // Extract the stacked linear-velocity components (3 per body) from a 6-DOF
  // twist vector (the angular components are internal to the solve and are not
  // part of the differentiated translational state).
  const auto linearOf = [&](const Eigen::VectorXd& twist) {
    Eigen::VectorXd linear(dofs);
    for (Eigen::Index k = 0; k < nb; ++k) {
      linear.segment<3>(3 * k) = twist.segment<3>(6 * k);
    }
    return linear;
  };

  // Base-state terms and the solved base impulse (from the forward boxed-LCP
  // solve of the frozen Delassus system; equals the forward step's solution).
  const ContactTerms base
      = computeTerms(registry, scene, gravity, timeStep, allocator);

  // Solve the base LCP on the frozen friction-augmented system with the same
  // pivoting Dantzig boxed solver the forward step uses, so the active-set
  // classification below matches the forward solution exactly. Normal rows:
  // lo = 0, hi = +∞, findex = -1. Friction rows: lo = -mu, hi = +mu, findex =
  // owning normal row.
  Eigen::VectorXd baseImpulse = Eigen::VectorXd::Zero(rows);
  Eigen::VectorXd loBounds = Eigen::VectorXd::Zero(rows);
  Eigen::VectorXd hiBounds = Eigen::VectorXd::Constant(
      rows, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(rows, -1);
  for (Eigen::Index i = 0; i < n; ++i) {
    const double mu = scene.contacts[static_cast<std::size_t>(i)].friction;
    for (Eigen::Index t = 0; t < 2; ++t) {
      const Eigen::Index row = n + 2 * i + t;
      loBounds[row] = -mu;
      hiBounds[row] = mu;
      findex[row] = static_cast<int>(i);
    }
  }
  {
    math::DantzigSolver solver;
    math::LcpOptions options;
    options.warmStart = false;
    options.validateSolution = false;
    const math::LcpProblem problem(base.A, base.b, loBounds, hiBounds, findex);
    solver.solve(problem, baseImpulse, options);
    for (Eigen::Index i = 0; i < n; ++i) {
      if (!std::isfinite(baseImpulse[i]) || baseImpulse[i] < 0.0) {
        baseImpulse[i] = 0.0;
      }
    }
    for (Eigen::Index row = n; row < rows; ++row) {
      if (!std::isfinite(baseImpulse[row])) {
        baseImpulse[row] = 0.0;
      }
    }
  }
  const Eigen::VectorXd baseF = baseImpulse;

  // Active-set classification (Nimble construction):
  //   * CLAMPING set C: normal rows with f_n > tol (interior, w = 0) and
  //     friction rows that stick (|f_t| < mu f_n, interior, w = 0). These obey
  //     the equality w = A_C* f - b_C = 0.
  //   * UPPER-BOUNDED set U: friction rows pinned at the cone edge
  //     (|f_t| >= mu f_n, sliding) whose owning normal row is clamping. These
  //     obey f_t = sign * mu * f_n, coupling to the normal through findex.
  //   * Everything else is separating/inactive and contributes zero gradient.
  BoolVector normalClamping(
      static_cast<std::size_t>(n), false, BoolAllocator{allocator});
  for (Eigen::Index i = 0; i < n; ++i) {
    normalClamping[static_cast<std::size_t>(i)] = baseF[i] > kClampingTolerance;
  }

  // ComplementarityAware (PLAN-110 WS5, Nimble heuristic; NOT the true
  // derivative): a firmly clamping normal contact is a complementarity saddle
  // -- the analytic gradient of the post-step normal velocity w.r.t. the
  // incoming normal velocity is identically zero (the impulse exactly cancels
  // the approach), so an optimizer pulling the body apart through the contact
  // sees no search direction. This mode RECLASSIFIES such clamping rows as
  // separating for the BACKWARD pass only, so the contact contributes the
  // contact-free (free-fall-like) sensitivity instead of a zero one, yielding a
  // non-zero descent direction. The forward step is untouched. We apply it to
  // every clamping normal row (and, transitively, its friction rows): each is a
  // saddle in the normal direction by the clamping condition above.
  if (mode == ContactGradientMode::ComplementarityAware) {
    for (Eigen::Index i = 0; i < n; ++i) {
      normalClamping[static_cast<std::size_t>(i)] = false;
    }
  }

  IndexVector clampingRows(IndexAllocator{allocator}); // C
  IndexVector upperRows(IndexAllocator{allocator});    // U (sliding friction)
  DoubleVector upperSign(DoubleAllocator{allocator});  // sign of f_t per U row
  IndexVector upperNormal(
      IndexAllocator{allocator}); // owning normal row for each U row
  for (Eigen::Index i = 0; i < n; ++i) {
    if (normalClamping[static_cast<std::size_t>(i)]) {
      clampingRows.push_back(i);
    }
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    const double mu = scene.contacts[static_cast<std::size_t>(i)].friction;
    const double bound = mu * baseF[i];
    for (Eigen::Index t = 0; t < 2; ++t) {
      const Eigen::Index row = n + 2 * i + t;
      const double ft = baseF[row];
      const bool ownerClamping = normalClamping[static_cast<std::size_t>(i)];
      if (!ownerClamping || bound <= kClampingTolerance) {
        // No (or vanishing) normal force: friction row is inactive.
        continue;
      }
      if (std::abs(ft) >= bound - kClampingTolerance) {
        // Sliding: pinned at the cone edge, mapped to the normal row.
        upperRows.push_back(row);
        upperSign.push_back(ft >= 0.0 ? 1.0 : -1.0);
        upperNormal.push_back(i);
      } else {
        // Sticking: interior friction row behaves like an equality.
        clampingRows.push_back(row);
      }
    }
  }
  const Eigen::Index nc = static_cast<Eigen::Index>(clampingRows.size());
  const Eigen::Index nu = static_cast<Eigen::Index>(upperRows.size());

  // Index of each clamping row within C (for the upper-bound mapping E, which
  // routes a sliding friction row's sensitivity through its normal row).
  IndexMap clampingIndexOf(
      0,
      std::hash<Eigen::Index>{},
      std::equal_to<Eigen::Index>{},
      IndexMapAllocator{allocator});
  for (Eigen::Index r = 0; r < nc; ++r) {
    clampingIndexOf.emplace(clampingRows[static_cast<std::size_t>(r)], r);
  }

  // Upper-bound mapping E (nu x nc): E[u, c] = sign_u * mu_u for the column c
  // that is the sliding row's owning normal row (when that normal is clamping),
  // so f_U = E f_C. Reduced clamping Delassus Ahat = A_CC + A_CU E and its
  // rank-revealing pseudo-inverse.
  Eigen::MatrixXd E = Eigen::MatrixXd::Zero(nu, nc);
  for (Eigen::Index u = 0; u < nu; ++u) {
    const Eigen::Index normalRow = upperNormal[static_cast<std::size_t>(u)];
    const auto it = clampingIndexOf.find(normalRow);
    if (it == clampingIndexOf.end()) {
      continue;
    }
    const double mu
        = scene.contacts[static_cast<std::size_t>(normalRow)].friction;
    E(u, it->second) = upperSign[static_cast<std::size_t>(u)] * mu;
  }

  const auto restrict =
      [](const Eigen::MatrixXd& M, const IndexVector& r, const IndexVector& c) {
        Eigen::MatrixXd out(
            static_cast<Eigen::Index>(r.size()),
            static_cast<Eigen::Index>(c.size()));
        for (std::size_t i = 0; i < r.size(); ++i) {
          for (std::size_t j = 0; j < c.size(); ++j) {
            out(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j))
                = M(r[i], c[j]);
          }
        }
        return out;
      };

  Eigen::MatrixXd AhatInv;
  if (nc > 0) {
    const Eigen::MatrixXd Acc = restrict(base.A, clampingRows, clampingRows);
    Eigen::MatrixXd Ahat = Acc;
    if (nu > 0) {
      const Eigen::MatrixXd Acu = restrict(base.A, clampingRows, upperRows);
      Ahat += Acu * E;
    }
    AhatInv = Ahat.completeOrthogonalDecomposition().pseudoInverse();
  }

  Eigen::VectorXd baseFc(nc);
  for (Eigen::Index r = 0; r < nc; ++r) {
    baseFc[r] = baseF[clampingRows[static_cast<std::size_t>(r)]];
  }

  // ---------------------------------------------------------------------------
  // Per-state-component finite differencing of the SMOOTH forward pieces over a
  // frozen active set, combined with the ANALYTIC ∂f/∂z via the reduced
  // clamping system Ahat = A_CC + A_CU E (upper-bound mapping for sliding
  // friction).
  //
  // For state/control component z the next state is
  //   q̇' = vFree(z) + Minv Jᵀ f(z)
  //   q'  = q(z) + correction(z) + Δt q̇'
  // where the only non-smooth piece is f (normal + friction impulses). We
  // central-difference the smooth map
  //   h(z) = [ q + correction + Δt (vFree + Minv Jᵀ f_base) ;
  //            vFree + Minv Jᵀ f_base ]            (f held at f_base)
  // and add the analytic contribution of ∂f/∂z through the constant linear map
  //   ∂x'/∂f = [ Δt Minv Jᵀ ; Minv Jᵀ ]   (over all normal + friction rows).
  //
  // Sliding friction rows are pinned: f_U = E f_C, so ∂f_U/∂z = E ∂f_C/∂z, and
  // the clamping rows satisfy Ahat f_C = b_C with Ahat = A_CC + A_CU E.
  // Implicit differentiation (E, the frozen signs and mu, held constant) gives
  //   ∂f_C/∂z = Ahat⁻¹ (∂b_C/∂z - (∂A_CC/∂z + (∂A_CU/∂z) E) f_C).
  // ---------------------------------------------------------------------------

  // ∂x'/∂f maps an impulse perturbation through the constant linear map Minv Jᵀ
  // (applied block-wise via minvJtTimes from the base scene/terms), avoiding
  // the dense vdofs x rows matrix that previously scaled with the body count.

  // Analytic ∂f/∂z over all rows from the central-differenced ∂A/∂z, ∂b/∂z.
  const auto frictionImpulseGradient
      = [&](const Eigen::MatrixXd& dA, const Eigen::VectorXd& db) {
          Eigen::VectorXd dfFull = Eigen::VectorXd::Zero(rows);
          if (nc == 0) {
            return dfFull;
          }
          Eigen::VectorXd dbcc(nc);
          for (Eigen::Index r = 0; r < nc; ++r) {
            dbcc[r] = db[clampingRows[static_cast<std::size_t>(r)]];
          }
          Eigen::MatrixXd dAhat = restrict(dA, clampingRows, clampingRows);
          if (nu > 0) {
            dAhat += restrict(dA, clampingRows, upperRows) * E;
          }
          // ∂f_C/∂z = Ahat⁻¹ (∂b_C/∂z - (∂Ahat/∂z) f_C).
          const Eigen::VectorXd dfc = AhatInv * (dbcc - dAhat * baseFc);
          for (Eigen::Index r = 0; r < nc; ++r) {
            dfFull[clampingRows[static_cast<std::size_t>(r)]] = dfc[r];
          }
          // ∂f_U/∂z = E ∂f_C/∂z (sliding friction follows its normal row).
          if (nu > 0) {
            const Eigen::VectorXd dfu = E * dfc;
            for (Eigen::Index u = 0; u < nu; ++u) {
              dfFull[upperRows[static_cast<std::size_t>(u)]] = dfu[u];
            }
          }
          return dfFull;
        };

  // Coordinate refs for the dynamic bodies' translational state: q components
  // map to body positions, q̇ components to body linear velocities, in the same
  // stacked order as dofs.
  derivatives.stateJacobian = Eigen::MatrixXd::Zero(stateSize, stateSize);
  derivatives.controlJacobian = Eigen::MatrixXd::Zero(stateSize, dofs);

  // Helper: evaluate the smooth next-state map h(z) at the current registry
  // state with f frozen at f_base. Returns [q'; q̇'] of size stateSize.
  const auto evalSmoothNextState = [&]() {
    const ContactTerms t
        = computeTerms(registry, scene, gravity, timeStep, allocator);
    // q̇' (linear part) = linearOf(vFree + Minv Jᵀ f_base)
    const Eigen::VectorXd qdotNext
        = linearOf(t.vFree + minvJtTimes(t, scene, vdofs, baseF));
    // q' = q + correction(penetration) + Δt q̇'. The penetration projection is
    // configuration dependent; we read the live positions for q and recompute
    // the projection contribution from the frozen contacts below.
    Eigen::VectorXd q(dofs);
    for (Eigen::Index k = 0; k < nb; ++k) {
      const auto entity = scene.dynamicBodies[static_cast<std::size_t>(k)];
      q.segment<3>(3 * k) = registry.get<comps::Transform>(entity).position;
    }
    Eigen::VectorXd state(stateSize);
    state.head(dofs) = q + timeStep * qdotNext;
    state.tail(dofs) = qdotNext;
    return state;
  };

  // State columns: positions (0..dofs-1) then velocities (dofs..2dofs-1).
  for (Eigen::Index col = 0; col < stateSize; ++col) {
    const bool isPosition = col < dofs;
    const Eigen::Index bodyDof = isPosition ? col : col - dofs;
    const Eigen::Index bodyIndex = bodyDof / 3;
    const Eigen::Index axis = bodyDof % 3;
    const auto entity
        = scene.dynamicBodies[static_cast<std::size_t>(bodyIndex)];

    auto& transform = registry.get<comps::Transform>(entity);
    auto& velocity = registry.get<comps::Velocity>(entity);

    double* target
        = isPosition ? &transform.position[axis] : &velocity.linear[axis];
    const double original = *target;
    const double h = 1e-6 * (1.0 + std::abs(original));

    // Smooth term (f frozen): central FD of h(z).
    *target = original + h;
    const Eigen::VectorXd plus = evalSmoothNextState();
    *target = original - h;
    const Eigen::VectorXd minus = evalSmoothNextState();
    *target = original; // restore exactly

    Eigen::VectorXd dxNext_smooth = (plus - minus) / (2.0 * h);

    // Analytic ∂f/∂z (normal + friction) via FD of the A, b terms.
    Eigen::VectorXd dfFull = Eigen::VectorXd::Zero(rows);
    if (nc > 0) {
      *target = original + h;
      const ContactTerms tPlus
          = computeTerms(registry, scene, gravity, timeStep, allocator);
      *target = original - h;
      const ContactTerms tMinus
          = computeTerms(registry, scene, gravity, timeStep, allocator);
      *target = original; // restore exactly

      const Eigen::MatrixXd dA = (tPlus.A - tMinus.A) / (2.0 * h);
      const Eigen::VectorXd db = (tPlus.b - tMinus.b) / (2.0 * h);
      dfFull = frictionImpulseGradient(dA, db);
    }

    // ∂x'/∂f contribution: q̇' adds the linear part of Minv Jᵀ ∂f, q' adds Δt
    // times that.
    const Eigen::VectorXd dQdot_f
        = linearOf(minvJtTimes(base, scene, vdofs, dfFull)); // dofs
    Eigen::VectorXd dxNext_f(stateSize);
    dxNext_f.head(dofs) = timeStep * dQdot_f;
    dxNext_f.tail(dofs) = dQdot_f;

    derivatives.stateJacobian.col(col) = dxNext_smooth + dxNext_f;
  }

  // Control columns: applied force per dynamic body axis. Force enters only
  // through vFree = v + Δt (F/m + g); the LCP rhs b = -(J vFree) depends on it,
  // so ∂f/∂F follows the same analytic A_CC⁻¹ formula.
  for (Eigen::Index col = 0; col < dofs; ++col) {
    const Eigen::Index bodyIndex = col / 3;
    const Eigen::Index axis = col % 3;
    const auto entity
        = scene.dynamicBodies[static_cast<std::size_t>(bodyIndex)];

    auto& force = registry.get<comps::Force>(entity);
    double* target = &force.force[axis];
    const double original = *target;
    const double h = 1e-6 * (1.0 + std::abs(original));

    *target = original + h;
    const Eigen::VectorXd plus = evalSmoothNextState();
    const ContactTerms tPlus
        = computeTerms(registry, scene, gravity, timeStep, allocator);
    *target = original - h;
    const Eigen::VectorXd minus = evalSmoothNextState();
    const ContactTerms tMinus
        = computeTerms(registry, scene, gravity, timeStep, allocator);
    *target = original; // restore exactly

    Eigen::VectorXd dxNext_smooth = (plus - minus) / (2.0 * h);

    Eigen::VectorXd dfFull = Eigen::VectorXd::Zero(rows);
    if (nc > 0) {
      const Eigen::MatrixXd dA = (tPlus.A - tMinus.A) / (2.0 * h);
      const Eigen::VectorXd db = (tPlus.b - tMinus.b) / (2.0 * h);
      dfFull = frictionImpulseGradient(dA, db);
    }

    const Eigen::VectorXd dQdot_f
        = linearOf(minvJtTimes(base, scene, vdofs, dfFull));
    Eigen::VectorXd dxNext_f(stateSize);
    dxNext_f.head(dofs) = timeStep * dQdot_f;
    dxNext_f.tail(dofs) = dQdot_f;

    derivatives.controlJacobian.col(col) = dxNext_smooth + dxNext_f;
  }

  // PreContactSurrogate (PLAN-110 WS5; backward-only, NOT the true derivative).
  // A dynamic body that is approaching but has no active (clamping) contact has
  // an identically-zero analytic contact gradient: the block above equals the
  // contact-free free-fall Jacobian. In this mode we ADD a distance/velocity-
  // based straight-through surrogate so the gradient is non-zero before contact
  // and points along the body's approach direction (toward contact). For each
  // such body we arrest the approach-direction velocity sensitivity by a rank-1
  // projection d dᵀ (d = unit free linear velocity, falling back to the gravity
  // direction), mimicking the clamp that is about to happen:
  //   ∂q̇'/∂q̇ -= w d dᵀ,   ∂q'/∂q̇ -= Δt w d dᵀ   (consistent with q' = q+Δt q̇').
  // The forward step is unchanged; only the cached backward Jacobian carries
  // the surrogate. This is a surrogate, not the genuine derivative (which is
  // zero before contact), and is documented as such on ContactGradientMode.
  if (mode == ContactGradientMode::PreContactSurrogate) {
    Eigen::Vector3d gravityDir = Eigen::Vector3d::Zero();
    if (gravity.norm() > kPreContactApproachSpeedTolerance) {
      gravityDir = gravity.normalized();
    }
    for (Eigen::Index k = 0; k < nb; ++k) {
      // Skip bodies that are part of an active clamping contact: their analytic
      // gradient is already the (non-surrogate) contact gradient. A body is
      // "in contact" here if any clamping normal row references it.
      bool bodyInContact = false;
      for (const Eigen::Index row : clampingRows) {
        if (row >= n) {
          continue; // friction row; its owning normal row already covers this
        }
        const auto& contact = scene.contacts[static_cast<std::size_t>(row)];
        const auto entity = scene.dynamicBodies[static_cast<std::size_t>(k)];
        if (contact.bodyA == entity || contact.bodyB == entity) {
          bodyInContact = true;
          break;
        }
      }
      if (bodyInContact) {
        continue;
      }

      // Approach direction: where the body is heading after the free
      // (gravity/force) velocity integration. Fall back to the gravity
      // direction for a body at rest so it still gets a toward-contact
      // gradient.
      const Eigen::Vector3d freeLinear = base.vFree.segment<3>(6 * k);
      Eigen::Vector3d d = Eigen::Vector3d::Zero();
      if (freeLinear.norm() > kPreContactApproachSpeedTolerance) {
        d = freeLinear.normalized();
      } else {
        d = gravityDir;
      }
      if (d.norm() <= kPreContactApproachSpeedTolerance) {
        continue; // no meaningful approach direction; leave free-fall block.
      }

      const Eigen::Matrix3d projection
          = kPreContactSurrogateWeight * (d * d.transpose());
      const Eigen::Index posBase = 3 * k;        // q rows for this body
      const Eigen::Index velBase = dofs + 3 * k; // q̇ rows for this body
      const Eigen::Index velCol = dofs + 3 * k;  // q̇ cols for this body

      // ∂q̇'/∂q̇ -= w d dᵀ.
      derivatives.stateJacobian.block<3, 3>(velBase, velCol) -= projection;
      // ∂q'/∂q̇ -= Δt w d dᵀ (keep q' = q + Δt q̇' consistent).
      derivatives.stateJacobian.block<3, 3>(posBase, velCol)
          -= timeStep * projection;
    }
  }

  return derivatives;
}

//==============================================================================
// Number of scalar Jacobian columns a single registered parameter spans. MASS
// and FRICTION are one scalar each; INERTIA contributes the three diagonal
// principal moments (Ixx, Iyy, Izz), so it spans three columns.
std::size_t parameterColumnCount(PhysicalParameter parameter)
{
  switch (parameter) {
    case PhysicalParameter::MASS:
    case PhysicalParameter::FRICTION:
      return 1;
    case PhysicalParameter::INERTIA:
      return 3;
    case PhysicalParameter::CENTER_OF_MASS:
      // Rejected at registration: the rigid-body forward step assumes the COM
      // at the body origin (localCenterOfMass is unused), so its FD-of-step is
      // identically zero and there is no gradient to assemble. Returning 0
      // keeps the column layout consistent if it ever reaches here.
      return 0;
  }
  return 0;
}

//==============================================================================
// Central FD of the full constrained next state with respect to a single scalar
// quantity addressed by `target`. A fresh scene is built on each perturbed side
// so the change flows into the cached inverse mass / inverse world inertia, the
// recomputed terms, and a fresh boxed-LCP solve, matching the forward step's
// dependence on that quantity. Restores `*target` exactly.
Eigen::VectorXd centralDifferenceColumn(
    detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    const Eigen::Vector3d& gravity,
    double timeStep,
    double* target,
    common::MemoryAllocator& allocator)
{
  const double original = *target;
  const double h = 1e-6 * (1.0 + std::abs(original));

  *target = original + h;
  const ContactScene scenePlus = buildScene(registry, contacts, allocator);
  const Eigen::VectorXd plus
      = constrainedNextState(registry, scenePlus, gravity, timeStep, allocator);

  *target = original - h;
  const ContactScene sceneMinus = buildScene(registry, contacts, allocator);
  const Eigen::VectorXd minus = constrainedNextState(
      registry, sceneMinus, gravity, timeStep, allocator);

  *target = original; // restore exactly

  return (plus - minus) / (2.0 * h);
}

//==============================================================================
StepDerivatives contactStepDerivativesWithParameters(
    detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    const Eigen::Vector3d& gravity,
    double timeStep,
    std::span<const ParameterRegistration> parameters,
    ContactGradientMode mode)
{
  return contactStepDerivativesWithParameters(
      registry,
      contacts,
      gravity,
      timeStep,
      parameters,
      mode,
      common::MemoryAllocator::GetDefault());
}

//==============================================================================
StepDerivatives contactStepDerivativesWithParameters(
    detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    const Eigen::Vector3d& gravity,
    double timeStep,
    std::span<const ParameterRegistration> parameters,
    ContactGradientMode mode,
    common::MemoryAllocator& allocator)
{
  // State/control Jacobians via the existing analytic assembly. The gradient
  // mode applies to the state/control blocks; the parameter Jacobian below is
  // the analytic FD-of-step column regardless of mode.
  StepDerivatives derivatives = contactStepDerivatives(
      registry, contacts, gravity, timeStep, mode, allocator);

  if (parameters.empty() || derivatives.stateJacobian.size() == 0) {
    // No parameters requested, or no dynamic rigid body in scope: leave the
    // parameter Jacobian empty (the OFF-build / no-registration contract).
    return derivatives;
  }

  // The differentiated scene defines the column layout and active set. Build it
  // once at the base state so the FD perturbations share the SAME frozen
  // contact geometry and body ordering as the state/control Jacobians.
  //
  // Each registration spans one or more scalar columns: MASS and FRICTION one
  // each, INERTIA three (the diagonal principal moments Ixx, Iyy, Izz). Columns
  // are laid out in registration order, with a registration's scalar columns
  // contiguous.
  const Eigen::Index stateSize = derivatives.stateJacobian.rows();
  Eigen::Index totalColumns = 0;
  for (const auto& registration : parameters) {
    totalColumns
        += static_cast<Eigen::Index>(parameterColumnCount(registration.second));
  }
  derivatives.parameterJacobian
      = Eigen::MatrixXd::Zero(stateSize, totalColumns);

  // Central FD of the full constrained next state over each registered scalar
  // parameter. A fresh scene is built on each side so the perturbed quantity
  // flows into the cached inverse mass / inverse world inertia, the recomputed
  // terms, and a fresh LCP solve, matching the forward step's dependence on the
  // parameter.
  Eigen::Index col = 0;
  for (const auto& registration : parameters) {
    const auto entity = registration.first;
    const PhysicalParameter parameter = registration.second;
    const Eigen::Index span
        = static_cast<Eigen::Index>(parameterColumnCount(parameter));

    switch (parameter) {
      case PhysicalParameter::MASS: {
        if (auto* massProps = registry.try_get<comps::MassProperties>(entity)) {
          const Eigen::VectorXd column = centralDifferenceColumn(
              registry,
              contacts,
              gravity,
              timeStep,
              &massProps->mass,
              allocator);
          if (column.size() == stateSize) {
            derivatives.parameterJacobian.col(col) = column;
          }
        }
        break;
      }
      case PhysicalParameter::INERTIA: {
        if (auto* massProps = registry.try_get<comps::MassProperties>(entity)) {
          // Three diagonal principal moments Ixx, Iyy, Izz, in axis order.
          for (Eigen::Index axis = 0; axis < 3; ++axis) {
            const Eigen::VectorXd column = centralDifferenceColumn(
                registry,
                contacts,
                gravity,
                timeStep,
                &massProps->inertia(axis, axis),
                allocator);
            if (column.size() == stateSize) {
              derivatives.parameterJacobian.col(col + axis) = column;
            }
          }
        }
        break;
      }
      case PhysicalParameter::FRICTION: {
        // Friction lives on the body's ContactMaterial; the boxed-LCP step
        // reads the per-pair combined mu = sqrt(mu_A * mu_B). Perturb this
        // body's mu and central-difference; the column is nonzero only when the
        // body is in a sliding (cone-edge) contact.
        if (auto* material = registry.try_get<comps::ContactMaterial>(entity)) {
          const Eigen::VectorXd column = centralDifferenceColumn(
              registry,
              contacts,
              gravity,
              timeStep,
              &material->friction,
              allocator);
          if (column.size() == stateSize) {
            derivatives.parameterJacobian.col(col) = column;
          }
        }
        break;
      }
      case PhysicalParameter::CENTER_OF_MASS:
        // Rejected at registration; unreachable. No columns.
        break;
    }

    col += span;
  }

  return derivatives;
}

} // namespace dart::simulation::detail
