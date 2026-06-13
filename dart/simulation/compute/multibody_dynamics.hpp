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

#include <dart/simulation/compute/compute_stage_metadata.hpp>
#include <dart/simulation/compute/world_step_stage.hpp>
#include <dart/simulation/detail/world_registry_types.hpp>
#include <dart/simulation/export.hpp>
#include <dart/simulation/fwd.hpp>

#include <dart/common/stl_allocator.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/entity/entity.hpp> // entt::entity and entt::null for the
                                  // link-contact structs below
#include <entt/fwd.hpp>

#include <memory>
#include <span>
#include <string_view>
#include <vector>

namespace dart::common {
class MemoryAllocator;
class MemoryManager;
} // namespace dart::common

namespace dart::simulation::comps {
struct MultibodyStructure;
} // namespace dart::simulation::comps

namespace dart::simulation::compute {

/// Generalized-coordinate dynamics terms for one multibody at its current
/// configuration and velocity, ordered by joint construction order.
///
/// The terms satisfy the joint-space equation of motion
/// `massMatrix * qddot + coriolisForces + gravityForces = tau`, where `tau` is
/// the applied generalized force. This matches the legacy DART decomposition of
/// `Skeleton::getMassMatrix`, `getCoriolisForces`, and `getGravityForces`.
struct MultibodyDynamicsTerms
{
  Eigen::MatrixXd massMatrix;     ///< M(q), size dof x dof
  Eigen::VectorXd coriolisForces; ///< C(q, qdot) qdot, size dof
  Eigen::VectorXd gravityForces;  ///< g(q), size dof
};

/// Reusable storage for generalized-coordinate dynamics-term evaluation.
///
/// The return-by-value wrapper below remains convenient for one-shot callers.
/// Repeated diagnostics or stage-loop callers should retain this scratch and
/// pass caller-owned result storage to `computeMultibodyDynamicsTermsInto()` so
/// dynamics-tree, RNEA, and output capacity can be reused across same-shape
/// calls without exposing private dynamics-tree types.
class DART_SIMULATION_API MultibodyDynamicsTermsScratch final
{
public:
  MultibodyDynamicsTermsScratch();
  explicit MultibodyDynamicsTermsScratch(common::MemoryAllocator& allocator);
  ~MultibodyDynamicsTermsScratch();

  MultibodyDynamicsTermsScratch(const MultibodyDynamicsTermsScratch&) = delete;
  MultibodyDynamicsTermsScratch& operator=(const MultibodyDynamicsTermsScratch&)
      = delete;
  MultibodyDynamicsTermsScratch(MultibodyDynamicsTermsScratch&&) noexcept;
  MultibodyDynamicsTermsScratch& operator=(
      MultibodyDynamicsTermsScratch&&) noexcept;

  void setAllocator(common::MemoryAllocator& allocator);
  [[nodiscard]] const common::MemoryAllocator& getAllocator() const noexcept;

private:
  struct Impl;
  struct ImplDeleter
  {
    common::MemoryAllocator* allocator = nullptr;

    void operator()(Impl* impl) const noexcept;
  };

  common::MemoryAllocator* m_allocator = nullptr;
  std::unique_ptr<Impl, ImplDeleter> m_impl;

  friend DART_SIMULATION_API void reserveMultibodyDynamicsTermsScratch(
      MultibodyDynamicsTermsScratch& scratch,
      detail::WorldRegistry& registry,
      const comps::MultibodyStructure& structure);

  friend DART_SIMULATION_API void computeMultibodyDynamicsTermsInto(
      MultibodyDynamicsTermsScratch& scratch,
      detail::WorldRegistry& registry,
      const comps::MultibodyStructure& structure,
      const Eigen::Vector3d& gravity,
      MultibodyDynamicsTerms& result);
};

struct InverseDynamicsDerivatives;

/// Compute the joint-space mass matrix and bias (Coriolis/centrifugal and
/// gravity) generalized forces for a single multibody.
///
/// Uses the same recursive Newton-Euler formulation as the forward-dynamics
/// stage, evaluated at the multibody's current joint positions and velocities.
/// Fixed-base trees with fixed, revolute, prismatic, screw, universal, planar,
/// ball (Spherical), and free (Floating) joints are supported, matching the
/// joint types handled by the forward dynamics. For a multibody with no movable
/// degrees of freedom the returned matrix and vectors are empty.
[[nodiscard]] DART_SIMULATION_API MultibodyDynamicsTerms
computeMultibodyDynamicsTerms(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity);

/// Reserve dynamics-terms scratch for the current multibody shape.
DART_SIMULATION_API void reserveMultibodyDynamicsTermsScratch(
    MultibodyDynamicsTermsScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure);

/// Compute joint-space dynamics terms into reusable caller-owned storage.
///
/// `result` is resized and overwritten. For a multibody with no movable
/// degrees of freedom, the matrix and vectors are empty.
DART_SIMULATION_API void computeMultibodyDynamicsTermsInto(
    MultibodyDynamicsTermsScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    MultibodyDynamicsTerms& result);

/// Compute the generalized joint forces required to produce a desired
/// generalized acceleration at the multibody's current configuration and
/// velocity (inverse dynamics): `tau = M(q) qddot + C(q, qdot) qdot + g(q)`,
/// including any joint armature on the mass-matrix diagonal.
///
/// Uses the recursive Newton-Euler algorithm directly. `desiredAcceleration`
/// must have size equal to the multibody's movable degree-of-freedom count,
/// ordered by joint construction order; the result has the same ordering. For a
/// multibody with no movable degrees of freedom the result is empty.
[[nodiscard]] DART_SIMULATION_API Eigen::VectorXd
computeMultibodyInverseDynamics(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    const Eigen::VectorXd& desiredAcceleration);

/// Reusable storage for inverse-dynamics evaluation.
///
/// The return-by-value wrapper above remains convenient for one-shot callers.
/// Repeated or stage-loop callers should keep this scratch and pass it to
/// `computeMultibodyInverseDynamicsInto()` so dynamics-tree, RNEA, and output
/// storage can be retained across same-shape steps without exposing private
/// dynamics-tree types.
class DART_SIMULATION_API MultibodyInverseDynamicsScratch final
{
public:
  MultibodyInverseDynamicsScratch();
  explicit MultibodyInverseDynamicsScratch(common::MemoryAllocator& allocator);
  ~MultibodyInverseDynamicsScratch();

  MultibodyInverseDynamicsScratch(const MultibodyInverseDynamicsScratch&)
      = delete;
  MultibodyInverseDynamicsScratch& operator=(
      const MultibodyInverseDynamicsScratch&) = delete;
  MultibodyInverseDynamicsScratch(MultibodyInverseDynamicsScratch&&) noexcept;
  MultibodyInverseDynamicsScratch& operator=(
      MultibodyInverseDynamicsScratch&&) noexcept;

  void setAllocator(common::MemoryAllocator& allocator);
  [[nodiscard]] const common::MemoryAllocator& getAllocator() const noexcept;

private:
  struct Impl;
  struct ImplDeleter
  {
    common::MemoryAllocator* allocator = nullptr;

    void operator()(Impl* impl) const noexcept;
  };

  common::MemoryAllocator* m_allocator = nullptr;
  std::unique_ptr<Impl, ImplDeleter> m_impl;

  friend DART_SIMULATION_API void reserveMultibodyInverseDynamicsScratch(
      MultibodyInverseDynamicsScratch& scratch,
      detail::WorldRegistry& registry,
      const comps::MultibodyStructure& structure);

  friend DART_SIMULATION_API void computeMultibodyInverseDynamicsInto(
      MultibodyInverseDynamicsScratch& scratch,
      detail::WorldRegistry& registry,
      const comps::MultibodyStructure& structure,
      const Eigen::Vector3d& gravity,
      const Eigen::VectorXd& desiredAcceleration,
      Eigen::VectorXd& result);

  friend DART_SIMULATION_API void
  computeMultibodyInverseDynamicsDerivativesInto(
      MultibodyInverseDynamicsScratch& scratch,
      detail::WorldRegistry& registry,
      const comps::MultibodyStructure& structure,
      const Eigen::Vector3d& gravity,
      const Eigen::VectorXd& generalizedAcceleration,
      InverseDynamicsDerivatives& result);
};

/// Reserve inverse-dynamics scratch for the current multibody shape.
DART_SIMULATION_API void reserveMultibodyInverseDynamicsScratch(
    MultibodyInverseDynamicsScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure);

/// Compute inverse dynamics into reusable caller-owned storage.
///
/// `result` is resized and overwritten with the generalized force vector. For a
/// multibody with no movable degrees of freedom, `result` is empty.
DART_SIMULATION_API void computeMultibodyInverseDynamicsInto(
    MultibodyInverseDynamicsScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    const Eigen::VectorXd& desiredAcceleration,
    Eigen::VectorXd& result);

/// Analytic inverse-dynamics partial derivatives, ∂τ/∂q and ∂τ/∂q̇, evaluated
/// at the multibody's current `(q, q̇)` and the supplied generalized
/// acceleration `qddot`, where `τ = M(q) qddot + C(q, q̇) q̇ + g(q)`.
struct InverseDynamicsDerivatives
{
  Eigen::MatrixXd dTau_dq;    ///< ∂τ/∂q, size dof x dof
  Eigen::MatrixXd dTau_dqdot; ///< ∂τ/∂q̇, size dof x dof
  bool valid = false;         ///< false when the analytic path does not apply
};

/// Compute `∂τ/∂q` and `∂τ/∂q̇` analytically in `O(dof²)` via spatial-algebra
/// (Recursive-Newton-Euler) derivative recursions, avoiding the `O(dof³)`
/// finite differencing of the dynamics terms.
///
/// `valid` is true only when every movable joint has a constant unit-twist
/// motion subspace (Fixed, Revolute, Prismatic, Screw). For trees containing
/// configuration-dependent or manifold subspaces (Universal, Planar, Spherical,
/// Floating), or a degenerate configuration, the result has `valid == false`
/// and empty matrices, signalling the caller to fall back to finite
/// differencing. `generalizedAcceleration` must match the movable DOF count.
[[nodiscard]] DART_SIMULATION_API InverseDynamicsDerivatives
computeMultibodyInverseDynamicsDerivatives(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    const Eigen::VectorXd& generalizedAcceleration);

/// Compute analytic inverse-dynamics partial derivatives into reusable scratch.
///
/// `result` is overwritten. When the analytic path does not apply, `valid` is
/// false and the matrices are empty, matching the return-by-value wrapper.
DART_SIMULATION_API void computeMultibodyInverseDynamicsDerivativesInto(
    MultibodyInverseDynamicsScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    const Eigen::VectorXd& generalizedAcceleration,
    InverseDynamicsDerivatives& result);

/// Compute the body-frame spatial Jacobian of one link of a multibody.
///
/// The returned 6 x DOF matrix maps the multibody's generalized velocity to the
/// link's spatial velocity `[angular; linear]` expressed in the link's own
/// frame, with columns ordered by joint construction order. It depends only on
/// the current joint configuration (not on world transforms), so it is robust
/// to stale frame caches. Columns of joints that do not move the link are zero.
///
/// @throws InvalidArgumentException if `linkEntity` is not part of `structure`.
[[nodiscard]] DART_SIMULATION_API Eigen::MatrixXd computeMultibodyLinkJacobian(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    entt::entity linkEntity);

/// Reusable storage for link-Jacobian evaluation.
///
/// The return-by-value wrappers remain convenient for one-shot diagnostics.
/// Repeated callers should retain this scratch and pass output storage to the
/// `Into` overloads so dynamics-tree and per-link body-Jacobian capacity can be
/// reused across same-shape calls.
class DART_SIMULATION_API MultibodyLinkJacobianScratch final
{
public:
  MultibodyLinkJacobianScratch();
  explicit MultibodyLinkJacobianScratch(common::MemoryAllocator& allocator);
  ~MultibodyLinkJacobianScratch();

  MultibodyLinkJacobianScratch(const MultibodyLinkJacobianScratch&) = delete;
  MultibodyLinkJacobianScratch& operator=(const MultibodyLinkJacobianScratch&)
      = delete;
  MultibodyLinkJacobianScratch(MultibodyLinkJacobianScratch&&) noexcept;
  MultibodyLinkJacobianScratch& operator=(
      MultibodyLinkJacobianScratch&&) noexcept;

  void setAllocator(common::MemoryAllocator& allocator);
  [[nodiscard]] const common::MemoryAllocator& getAllocator() const noexcept;

private:
  struct Impl;
  struct ImplDeleter
  {
    common::MemoryAllocator* allocator = nullptr;

    void operator()(Impl* impl) const noexcept;
  };

  common::MemoryAllocator* m_allocator = nullptr;
  std::unique_ptr<Impl, ImplDeleter> m_impl;

  friend DART_SIMULATION_API void reserveMultibodyLinkJacobianScratch(
      MultibodyLinkJacobianScratch& scratch,
      detail::WorldRegistry& registry,
      const comps::MultibodyStructure& structure);

  friend DART_SIMULATION_API void computeMultibodyLinkJacobianInto(
      MultibodyLinkJacobianScratch& scratch,
      detail::WorldRegistry& registry,
      const comps::MultibodyStructure& structure,
      entt::entity linkEntity,
      Eigen::MatrixXd& result);

  friend DART_SIMULATION_API void computeMultibodyLinkWorldJacobianInto(
      MultibodyLinkJacobianScratch& scratch,
      detail::WorldRegistry& registry,
      const comps::MultibodyStructure& structure,
      entt::entity linkEntity,
      Eigen::MatrixXd& result);
};

/// Reserve link-Jacobian scratch for the current multibody shape.
DART_SIMULATION_API void reserveMultibodyLinkJacobianScratch(
    MultibodyLinkJacobianScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure);

/// Compute a body-frame link Jacobian into reusable caller-owned storage.
DART_SIMULATION_API void computeMultibodyLinkJacobianInto(
    MultibodyLinkJacobianScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    entt::entity linkEntity,
    Eigen::MatrixXd& result);

/// Compute the world-frame geometric Jacobian of one link of a multibody.
///
/// The returned 6 x DOF matrix maps the multibody's generalized velocity to the
/// link's spatial velocity `[angular; linear]` expressed in world axes, with
/// the link-frame origin as the linear reference point. It is the body Jacobian
/// with both blocks rotated into world axes. The link world transform is
/// derived from the joint configuration and the base world transform.
///
/// @throws InvalidArgumentException if `linkEntity` is not part of `structure`.
[[nodiscard]] DART_SIMULATION_API Eigen::MatrixXd
computeMultibodyLinkWorldJacobian(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    entt::entity linkEntity);

/// Compute a world-frame link Jacobian into reusable caller-owned storage.
DART_SIMULATION_API void computeMultibodyLinkWorldJacobianInto(
    MultibodyLinkJacobianScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    entt::entity linkEntity,
    Eigen::MatrixXd& result);

/// One contact acting on a single link of a multibody, oriented so the normal
/// points from the obstacle into the link. The obstacle is either immovable
/// (`otherBody == entt::null` and `otherLink == entt::null`, a one-sided
/// solve), a dynamic rigid body that receives the equal-and-opposite impulse,
/// another link in the same multibody whose point Jacobian is subtracted from
/// the primary link's point Jacobian, or another link in a different multibody
/// whose articulated end is carried separately.
struct LinkContact
{
  entt::entity link = entt::null;
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ(); ///< world, into the link
  Eigen::Vector3d point = Eigen::Vector3d::Zero();   ///< world contact point
  double depth = 0.0;
  double friction = 1.0;    ///< combined Coulomb friction coefficient
  double restitution = 0.0; ///< combined normal restitution coefficient
  entt::entity otherBody = entt::null; ///< dynamic rigid-body obstacle, or null
  entt::entity otherLink
      = entt::null; ///< same-multibody link obstacle, or null
  entt::entity otherMultibody
      = entt::null; ///< owner of cross-multibody otherLink, or null
};

/// One link contact after Jacobian and inverse-mass precomputation, ready for
/// the velocity-level solve.
///
/// Each Jacobian maps a world-space contact impulse along its direction into
/// the owning multibody's generalized-velocity space (`J^T d`, size DOF). The
/// denominators are the diagonal Delassus entries `J M^-1 J^T`, where `J` is
/// either the primary link's point Jacobian, the relative point Jacobian
/// against another link in the same multibody, or the primary side of a
/// cross-multibody contact. Dynamic rigid obstacles and cross-multibody
/// articulated ends augment the denominator with their point inverse mass. A
/// contact that cannot move either body (e.g. a fixed-base link against an
/// immovable obstacle) is left inactive.
struct MultibodyLinkContactRow
{
  Eigen::VectorXd normalJacobian;
  Eigen::VectorXd tangentJacobian1;
  Eigen::VectorXd tangentJacobian2;
  Eigen::VectorXd otherNormalJacobian;
  Eigen::VectorXd otherTangentJacobian1;
  Eigen::VectorXd otherTangentJacobian2;
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d tangent1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d tangent2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  double normalDenominator = 0.0;
  double tangentDenominator1 = 0.0;
  double tangentDenominator2 = 0.0;
  double bias = 0.0; ///< Baumgarte penetration-recovery velocity target
  double restitutionTarget = 0.0;
  double friction = 1.0;
  /// Pre-solve boxed-LCP right-hand sides (`w = A*lambda - b` convention), the
  /// velocity-level targets the unified constraint solve drives to. The normal
  /// row uses `-v_n + max(bias, restitutionTarget)` (matching the Gauss-Seidel
  /// normal update); the tangent rows drive the tangential relative velocity to
  /// zero (`-v_t`). Computed once from the staged pre-solve velocity.
  double normalRhs = 0.0;
  double tangentRhs1 = 0.0;
  double tangentRhs2 = 0.0;
  double normalImpulse = 0.0;
  double tangentImpulse1 = 0.0;
  double tangentImpulse2 = 0.0;
  double restitution = 0.0;
  entt::entity otherBody = entt::null;
  entt::entity otherLink = entt::null;
  entt::entity otherMultibody = entt::null;
  int otherMultibodyIndex = -1;
  double otherInvMass = 0.0;
  Eigen::Matrix3d otherInvInertia = Eigen::Matrix3d::Zero();
  Eigen::Vector3d otherArm = Eigen::Vector3d::Zero();
  bool active = false;
};

/// The assembled link-contact rows for one multibody plus its joint-space
/// inverse mass, the operator the contact impulses act through.
struct MultibodyLinkContactProblem
{
  using RowAllocator = common::StlAllocator<MultibodyLinkContactRow>;

  MultibodyLinkContactProblem() = default;

  explicit MultibodyLinkContactProblem(common::MemoryAllocator& allocator)
    : rows(RowAllocator{allocator})
  {
  }

  std::vector<MultibodyLinkContactRow, RowAllocator> rows;
  Eigen::MatrixXd inverseMass; ///< joint-space M^-1 (size DOF x DOF)
};

/// Reusable storage for public link-contact boxed-LCP assembly.
///
/// The return-by-value assembly wrapper below remains convenient for one-shot
/// tests and tools. Repeated or hot-loop callers should keep this scratch and
/// pass it to `assembleMultibodyLinkContactProblemInto()`, then borrow
/// `getProblem()` into the unified constraint assembler. The scratch owns the
/// intermediate dynamics work arrays as well as the assembled
/// `MultibodyLinkContactProblem`, so same-shape calls can retain vector and
/// Eigen storage across frames without exposing private dynamics-tree types.
class DART_SIMULATION_API MultibodyLinkContactAssemblyScratch final
{
public:
  MultibodyLinkContactAssemblyScratch();
  ~MultibodyLinkContactAssemblyScratch();

  MultibodyLinkContactAssemblyScratch(
      const MultibodyLinkContactAssemblyScratch&) = delete;
  MultibodyLinkContactAssemblyScratch& operator=(
      const MultibodyLinkContactAssemblyScratch&) = delete;
  MultibodyLinkContactAssemblyScratch(
      MultibodyLinkContactAssemblyScratch&&) noexcept;
  MultibodyLinkContactAssemblyScratch& operator=(
      MultibodyLinkContactAssemblyScratch&&) noexcept;

  [[nodiscard]] MultibodyLinkContactProblem& getProblem() noexcept;
  [[nodiscard]] const MultibodyLinkContactProblem& getProblem() const noexcept;

private:
  struct Impl;

  std::unique_ptr<Impl> m_impl;

  friend DART_SIMULATION_API bool assembleMultibodyLinkContactProblemInto(
      MultibodyLinkContactAssemblyScratch& scratch,
      const detail::WorldRegistry& registry,
      const comps::MultibodyStructure& structure,
      const Eigen::VectorXd& nextVelocity,
      double timeStep,
      std::span<const LinkContact> linkContacts);
};

/// Assemble the link-contact rows for a single multibody at its current
/// configuration.
///
/// Builds the multibody's dynamics tree, joint-space inverse mass, and per-link
/// body Jacobians, then precomputes each contact's point Jacobian, normal and
/// tangent directions, diagonal Delassus denominators, Baumgarte bias, and
/// restitution target (plus a dynamic rigid obstacle's coupling for a two-sided
/// contact). This is the link-side counterpart of
/// `assembleRigidBodyContactProblem`; the velocity solve consumes the returned
/// rows and inverse mass. Contacts whose link is not part of `structure`, or
/// that cannot move either body, are skipped (left inactive).
///
/// `nextVelocity` is the staged generalized velocity (joint construction order)
/// used as the pre-solve baseline for the restitution target. For a multibody
/// with no movable degrees of freedom the returned problem is empty.
///
/// @throws InvalidArgumentException if `nextVelocity` does not match the
///         multibody's movable degree-of-freedom count.
[[nodiscard]] DART_SIMULATION_API MultibodyLinkContactProblem
assembleMultibodyLinkContactProblem(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::VectorXd& nextVelocity,
    double timeStep,
    std::span<const LinkContact> linkContacts);

/// Assemble link-contact rows into reusable caller-owned scratch.
///
/// Returns `true` when the multibody structure has a valid dynamics layout for
/// contact assembly. The assembled problem is available through
/// `scratch.getProblem()` even when no active rows remain.
DART_SIMULATION_API bool assembleMultibodyLinkContactProblemInto(
    MultibodyLinkContactAssemblyScratch& scratch,
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::VectorXd& nextVelocity,
    double timeStep,
    std::span<const LinkContact> linkContacts);

/// Fixed-base articulated-body forward-dynamics stage.
///
/// For each multibody, this stage computes generalized joint accelerations from
/// joint efforts, gravity, and velocity-dependent (Coriolis/centrifugal) terms,
/// then integrates joint velocities and positions with semi-implicit Euler.
///
/// The dynamics use a recursive Newton-Euler formulation to build the
/// joint-space mass matrix and bias forces, then solve `M qddot = tau - bias`.
/// This matches the legacy DART rigid-body articulated-dynamics results for
/// open chains while keeping the implementation backend-neutral.
///
/// Scope and assumptions:
/// - Fixed-base trees only: the root link is fixed to the world (it has no
///   parent joint). Floating-base (free root joint) dynamics are not yet
///   implemented.
/// - Supported joints: fixed, revolute, and prismatic. Other joint types have
///   kinematics but their dynamics are rejected until implemented.
/// - Each link's center of mass is at the link frame origin, and the link
///   inertia tensor is expressed about that origin.
class DART_SIMULATION_API MultibodyForwardDynamicsStage final
  : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Computes unconstrained articulated-body velocities without advancing
/// positions.
///
/// This is the velocity half of the semi-implicit multibody step. It leaves the
/// solved generalized velocities in a transient internal staging component so
/// contact/constraint stages can operate before positions are written back.
class DART_SIMULATION_API MultibodyVelocityStage final : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// The built-in semi-implicit `World::step()` schedule uses
/// `UnifiedConstraintStage` for articulated contact coupling. This stage
/// remains available for focused custom-pipeline tests and experiments that
/// need the older per-multibody velocity-level pass.
class DART_SIMULATION_API MultibodyContactStage final : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Applies multibody velocity limits and advances joint positions from the
/// staged semi-implicit velocities.
class DART_SIMULATION_API MultibodyPositionStage final : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Resolves all rigid-rigid and articulated link-vs-rigid-body contacts with a
/// single coupled boxed-LCP, replacing the separate `RigidBodyContactStage` and
/// `MultibodyContactStage` velocity-level passes.
///
/// Each step it queries collisions once, assembles the rigid-rigid contact
/// problem and each multibody's link-contact problem (recomputing the dynamics
/// tree and inverse mass in-stage, never cached across stages), stacks them
/// into one unified system whose shared-dynamic-obstacle coupling is
/// consistent, and solves it jointly (with a rank-deficient fallback). Rigid
/// impulses are applied to body velocities and link impulses to each
/// multibody's staged generalized velocity (`PendingMultibodyVelocity`), so a
/// position stage can run afterwards; a rigid positional projection then
/// removes residual penetration. This is a semi-implicit-pipeline stage only —
/// the variational path keeps its own contact handling.
class DART_SIMULATION_API UnifiedConstraintStage final : public WorldStepStage
{
public:
  explicit UnifiedConstraintStage(std::size_t frictionIterations = 8);
  UnifiedConstraintStage(
      std::size_t frictionIterations, common::MemoryManager* memoryManager);
  ~UnifiedConstraintStage() override;

  UnifiedConstraintStage(const UnifiedConstraintStage&) = delete;
  UnifiedConstraintStage& operator=(const UnifiedConstraintStage&) = delete;
  UnifiedConstraintStage(UnifiedConstraintStage&&) noexcept;
  UnifiedConstraintStage& operator=(UnifiedConstraintStage&&) noexcept;

  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void prepare(World& world);
  void execute(World& world, ComputeExecutor& executor) override;

  [[nodiscard]] std::size_t getFrictionIterations() const noexcept;

private:
  struct Scratch;
  struct ScratchDeleter
  {
    common::MemoryManager* memoryManager = nullptr;
    void operator()(Scratch* scratch) const noexcept;
  };

  using ScratchPtr = std::unique_ptr<Scratch, ScratchDeleter>;

  bool assembleProblemIntoScratch(
      World& world, std::span<const Contact> contacts);

  std::size_t m_frictionIterations;
  common::MemoryManager* m_memoryManager = nullptr;
  ScratchPtr m_scratch;
};

} // namespace dart::simulation::compute
