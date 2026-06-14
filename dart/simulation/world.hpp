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

#include <dart/config.hpp>

#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/body/deformable_body_options.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/compute/world_step_profile.hpp>
#include <dart/simulation/constraint/loop_closure.hpp>
#include <dart/simulation/diff/step_derivatives.hpp>
#include <dart/simulation/diff/step_gradient.hpp>
#include <dart/simulation/entity.hpp>
#include <dart/simulation/fwd.hpp>
#include <dart/simulation/multibody/multibody_options.hpp>
#include <dart/simulation/world_options.hpp>
#include <dart/simulation/world_sync_stage.hpp>

#include <dart/common/memory_manager.hpp>

#include <Eigen/Geometry>

#include <iosfwd>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation {

namespace io::detail {
class SkeletonLoaderWorldAccess;
} // namespace io::detail

namespace detail {
struct WorldStorage;
[[nodiscard]] DART_SIMULATION_API WorldStorage& storageOf(World& world);
[[nodiscard]] DART_SIMULATION_API const WorldStorage& storageOf(
    const World& world);
} // namespace detail

namespace compute {
class MultibodyContactStage;
class MultibodyForwardDynamicsStage;
class RigidBodyContactStage;
class RigidIpcContactStage;
class UnifiedConstraintStage;
} // namespace compute

struct WorldOptions;

/// Options controlling `World::collide()` query filtering.
struct CollisionQueryOptions
{
  /// Include contacts between different links that belong to the same
  /// multibody. Enabled by default to preserve explicit self-contact queries
  /// and the articulated same-multibody contact solver path.
  bool includeSameMultibodyLinkPairs = true;

  /// Include contacts between two rigid bodies.
  bool includeRigidBodyPairs = true;

  /// Include contacts between a rigid body and a multibody link.
  bool includeRigidBodyLinkPairs = true;

  /// Include contacts between two multibody links. If this is enabled,
  /// `includeSameMultibodyLinkPairs` can still filter the same-multibody
  /// subset.
  bool includeLinkPairs = true;
};

/// A read-only snapshot of the deformable solver's per-step diagnostics, folded
/// across all deformable bodies on the most recent ``World::step``. This is a
/// curated, stable subset of the internal ``compute::DeformableSolverStats``
/// (mesh sizes, projected-Newton convergence, self-contact activity, and the
/// contact closest-approach diagnostic), surfaced for observability from tools
/// and bindings without exposing the full internal counter set. All counters
/// are zero before the first step that runs a deformable solve.
struct DeformableSolverDiagnostics
{
  /// Number of deformable bodies stepped.
  std::size_t bodyCount = 0;
  /// Total deformable node and (mass-spring) edge counts.
  std::size_t nodeCount = 0;
  std::size_t edgeCount = 0;
  /// Cumulative projected-Newton iterations over the step (summed per body).
  std::size_t solverIterations = 0;
  /// Objective (energy/gradient) evaluations and line-search trials.
  std::size_t objectiveEvaluations = 0;
  std::size_t lineSearchTrials = 0;
  /// Projected-Newton search directions taken, and steepest-descent fallbacks
  /// used when the Newton direction was not a descent direction.
  std::size_t projectedNewtonSteps = 0;
  std::size_t projectedNewtonFallbacks = 0;
  /// Maximum compressed sparse Hessian matrix footprint assembled by the
  /// projected-Newton solve in this step. This is an Eigen matrix-storage
  /// estimate (values + sparse indices), not a full process peak-memory value.
  std::size_t projectedNewtonHessianNonZeros = 0;
  std::size_t projectedNewtonHessianStorageBytes = 0;
  /// Newton iterations whose linear solve took the iterative
  /// (incomplete-Cholesky preconditioned conjugate-gradient) path instead of
  /// the sparse Cholesky factorization -- either because the mesh exceeds the
  /// direct-solve node cap or because the body opted in via
  /// ``DeformableMaterialProperties.useIterativeLinearSolver``. Zero means
  /// every solve used the direct factorization.
  std::size_t projectedNewtonIterativeSolves = 0;
  /// Iterative solves that used matrix-free Hessian-vector products instead of
  /// an assembled sparse Hessian. This is a subset of
  /// ``projectedNewtonIterativeSolves``.
  std::size_t projectedNewtonMatrixFreeSolves = 0;
  /// Total conjugate-gradient iterations consumed by those iterative linear
  /// solves, and the largest Eigen-reported relative residual estimate among
  /// them. Both are zero when every solve used the direct factorization.
  std::size_t projectedNewtonIterativeIterations = 0;
  double projectedNewtonIterativeMaxError = 0.0;
  /// Self-contact barrier active contacts summed over every solver iteration.
  std::size_t selfContactBarrierActiveContacts = 0;
  /// Coulomb friction energy dissipated at the converged iterate.
  double frictionDissipation = 0.0;
  /// Contact closest-approach diagnostic at the converged iterate: the smallest
  /// active contact distance, and the size of the active set at termination.
  /// ``minActiveContactDistance`` is meaningful only when
  /// ``convergedActiveContactCount > 0`` (otherwise it is 0).
  double minActiveContactDistance = 0.0;
  std::size_t convergedActiveContactCount = 0;
};

/// Per-component-storage ECS memory diagnostics. Storage IDs are internal
/// diagnostic tokens for grouping one snapshot; callers should not persist them
/// as stable public component identifiers.
struct WorldEcsStorageDiagnostics
{
  /// Internal diagnostic ID for this component storage.
  std::size_t storageId = 0;
  /// Number of live components in this storage.
  std::size_t size = 0;
  /// Current storage capacity before another component insertion may grow it.
  std::size_t capacity = 0;
};

/// Aggregate ECS registry storage diagnostics for profiler/debugger surfaces.
struct WorldEcsDiagnostics
{
  /// Number of live entities in the World registry.
  std::size_t entityCount = 0;
  /// Current registry entity storage capacity.
  std::size_t entityCapacity = 0;
  /// Number of component storages currently materialized by the registry.
  std::size_t storageCount = 0;
  /// Sum of live component counts across materialized component storages.
  std::size_t componentCount = 0;
  /// Sum of component capacities across materialized component storages.
  std::size_t componentCapacity = 0;
  /// Per-storage live/capacity counters for layout debugging and UI grouping.
  std::vector<WorldEcsStorageDiagnostics> storages;
};

/// Snapshot of the DART 7 World's CPU memory hierarchy diagnostics.
///
/// This snapshot reports the World-owned frame allocator used for per-step
/// scratch, structured debug counters for direct free/pool allocations, and
/// ECS registry storage layout counters for memory debugger/profiler tools.
struct WorldMemoryDiagnostics
{
  /// Debug counters for the World-owned MemoryManager hierarchy.
  common::MemoryManager::DebugDiagnostics allocatorDebugDiagnostics;

  /// ECS registry/component storage diagnostics.
  WorldEcsDiagnostics ecsDiagnostics;

  /// Current usable frame-scratch arena capacity after alignment padding.
  std::size_t frameScratchCapacityBytes = 0;
  /// Bytes consumed in the current simulation frame, including overflow blocks.
  std::size_t frameScratchUsedBytes = 0;
  /// Maximum frame-scratch bytes observed since construction or clear().
  std::size_t frameScratchPeakUsedBytes = 0;
  /// Number of overflow allocations the frame allocator holds for the current
  /// frame. Nonzero means the current step exceeded the reserved arena.
  std::size_t frameScratchOverflowCount = 0;
  /// Bytes currently held by overflow allocations for the current frame.
  std::size_t frameScratchOverflowBytes = 0;
  /// Number of times the World reset frame scratch at step boundaries since
  /// construction or clear().
  std::size_t frameScratchResetCount = 0;
};

class DART_SIMULATION_API World
{
public:
  World();

  /// Construct a World from value options (time step, gravity, and the opt-in
  /// differentiable flag). The existing default constructor and setters remain
  /// valid; this only initializes members from `options`.
  explicit World(const WorldOptions& options);

  ~World();

  World(const World&) = delete;
  World& operator=(const World&) = delete;
  World(World&&) = delete;
  World& operator=(World&&) = delete;

  //--------------------------------------------------------------------------
  // Frame creation
  //--------------------------------------------------------------------------
  FreeFrame addFreeFrame();
  FreeFrame addFreeFrame(std::string_view name);
  FreeFrame addFreeFrame(std::string_view name, const Frame& parent);

  FixedFrame addFixedFrame(std::string_view name, const Frame& parent);
  FixedFrame addFixedFrame(
      std::string_view name,
      const Frame& parent,
      const Eigen::Isometry3d& offset);

  //--------------------------------------------------------------------------
  // Multibody management
  //--------------------------------------------------------------------------
  /// Create an articulated multibody container in design mode.
  ///
  /// @throws InvalidOperationException if public rigid-body fixed joints
  ///         already exist, because the mixed multibody pipeline does not yet
  ///         project those fixed-joint rows.
  Multibody addMultibody(std::string_view name);
  std::optional<Multibody> getMultibody(std::string_view name);
  bool hasMultibody(std::string_view name) const;
  std::size_t getMultibodyCount() const;

  /// Create a fixed non-topology constraint between two links in one
  /// multibody.
  ///
  /// The current relative pose is captured when the World enters simulation
  /// mode, after kinematics has been baked. During simulation steps the
  /// variational articulated path projects the child endpoint back toward that
  /// captured pose. This is design-mode only and requires the variational
  /// multibody integration family before stepping.
  Joint addArticulatedFixedJoint(
      std::string_view name, const Link& parent, const Link& child);
  Joint addArticulatedFixedJoint(
      std::string_view name,
      const Link& parent,
      const Link& child,
      const Eigen::Vector3d& parentAnchor,
      const Eigen::Vector3d& childAnchor);

  /// Create a fixed non-topology constraint between world and one link.
  ///
  /// The current world pose is captured when the World enters simulation mode.
  /// During simulation steps the variational articulated path projects the link
  /// endpoint back toward that captured world pose. This is design-mode only
  /// and requires the variational multibody integration family before stepping.
  Joint addArticulatedFixedJoint(std::string_view name, const Link& child);
  Joint addArticulatedFixedJoint(
      std::string_view name,
      const Link& child,
      const Eigen::Vector3d& worldAnchor,
      const Eigen::Vector3d& childAnchor);

  /// Create a revolute non-topology constraint between two links in one
  /// multibody.
  ///
  /// The captured anchor is preserved while rotation around the parent-frame
  /// `axis` remains free. This is design-mode only and requires the variational
  /// multibody integration family before stepping.
  Joint addArticulatedRevoluteJoint(
      std::string_view name,
      const Link& parent,
      const Link& child,
      const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ());
  Joint addArticulatedRevoluteJoint(
      std::string_view name,
      const Link& parent,
      const Link& child,
      const Eigen::Vector3d& axis,
      const Eigen::Vector3d& parentAnchor,
      const Eigen::Vector3d& childAnchor);

  /// Create a revolute non-topology constraint between world and one link.
  ///
  /// The captured anchor is preserved while rotation around the world-frame
  /// `axis` remains free. This is design-mode only and requires the
  /// variational multibody integration family before stepping.
  Joint addArticulatedRevoluteJoint(
      std::string_view name,
      const Link& child,
      const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ());
  Joint addArticulatedRevoluteJoint(
      std::string_view name,
      const Link& child,
      const Eigen::Vector3d& axis,
      const Eigen::Vector3d& worldAnchor,
      const Eigen::Vector3d& childAnchor);

  /// Create a prismatic non-topology constraint between two links in one
  /// multibody.
  ///
  /// The captured anchor is preserved while translation along the parent-frame
  /// `axis` remains free. This is design-mode only and requires the variational
  /// multibody integration family before stepping.
  Joint addArticulatedPrismaticJoint(
      std::string_view name,
      const Link& parent,
      const Link& child,
      const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ());
  Joint addArticulatedPrismaticJoint(
      std::string_view name,
      const Link& parent,
      const Link& child,
      const Eigen::Vector3d& axis,
      const Eigen::Vector3d& parentAnchor,
      const Eigen::Vector3d& childAnchor);

  /// Create a prismatic non-topology constraint between world and one link.
  ///
  /// The captured anchor is preserved while translation along the world-frame
  /// `axis` remains free. This is design-mode only and requires the
  /// variational multibody integration family before stepping.
  Joint addArticulatedPrismaticJoint(
      std::string_view name,
      const Link& child,
      const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ());
  Joint addArticulatedPrismaticJoint(
      std::string_view name,
      const Link& child,
      const Eigen::Vector3d& axis,
      const Eigen::Vector3d& worldAnchor,
      const Eigen::Vector3d& childAnchor);

  /// Create a spherical non-topology point constraint between two links in one
  /// multibody.
  ///
  /// The captured anchor is preserved while relative orientation remains free.
  /// This is design-mode only and requires the variational multibody
  /// integration family before stepping.
  Joint addArticulatedSphericalJoint(
      std::string_view name, const Link& parent, const Link& child);
  Joint addArticulatedSphericalJoint(
      std::string_view name,
      const Link& parent,
      const Link& child,
      const Eigen::Vector3d& parentAnchor,
      const Eigen::Vector3d& childAnchor);

  /// Create a spherical non-topology point constraint between world and one
  /// link.
  ///
  /// The captured anchor is preserved while relative orientation remains free.
  /// This is design-mode only and requires the variational multibody
  /// integration family before stepping.
  Joint addArticulatedSphericalJoint(std::string_view name, const Link& child);
  Joint addArticulatedSphericalJoint(
      std::string_view name,
      const Link& child,
      const Eigen::Vector3d& worldAnchor,
      const Eigen::Vector3d& childAnchor);

  /// Get a supported non-topology articulated point joint by name.
  ///
  /// This returns fixed, revolute, prismatic, and spherical link-link or
  /// world-link joints created through the articulated World facade. It does
  /// not return tree
  /// topology joints; use Multibody::getJoint() for parent joints.
  std::optional<Joint> getArticulatedJoint(std::string_view name);
  bool hasArticulatedJoint(std::string_view name) const;
  std::size_t getArticulatedJointCount() const;
  std::vector<Joint> getArticulatedJoints();

  //--------------------------------------------------------------------------
  // Loop-closure management
  //--------------------------------------------------------------------------
  LoopClosure addLoopClosure(
      std::string_view name, const LoopClosureSpec& spec);
  std::optional<LoopClosure> getLoopClosure(std::string_view name);
  bool hasLoopClosure(std::string_view name) const;
  std::size_t getLoopClosureCount() const;

  //--------------------------------------------------------------------------
  // Rigid body management
  //--------------------------------------------------------------------------
  RigidBody addRigidBody(
      std::string_view name,
      const RigidBodyOptions& options = RigidBodyOptions{});
  /// Create a fixed constraint between two free rigid bodies.
  ///
  /// The current relative pose is captured when the joint is created. During
  /// simulation steps the experimental rigid-body constraint path projects the
  /// child body back toward that captured pose. This is design-mode only:
  /// create bodies and fixed joints before `enterSimulationMode()`.
  ///
  /// @throws InvalidArgumentException if either body is invalid, belongs to a
  ///         different World, both handles refer to the same body, or the name
  ///         is already used by another joint.
  /// @throws InvalidOperationException if the World is in simulation mode,
  ///         configured for the IPC rigid-body solver, already contains
  ///         multibody structures, or if the internal fixed-joint row
  ///         configuration fails.
  Joint addRigidBodyFixedJoint(
      std::string_view name, const RigidBody& parent, const RigidBody& child);
  /// Create a revolute constraint between two free rigid bodies.
  ///
  /// The current relative pose is captured when the joint is created. During
  /// simulation steps the experimental rigid-body constraint path projects the
  /// child body back toward that captured anchor while leaving rotation around
  /// the parent-frame `axis` free. This is design-mode only.
  Joint addRigidBodyRevoluteJoint(
      std::string_view name,
      const RigidBody& parent,
      const RigidBody& child,
      const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ());
  /// Create a prismatic constraint between two free rigid bodies.
  ///
  /// The current relative pose is captured when the joint is created. During
  /// simulation steps the experimental rigid-body constraint path projects the
  /// child body back toward that captured anchor while leaving translation
  /// along the parent-frame `axis` free. This is design-mode only.
  Joint addRigidBodyPrismaticJoint(
      std::string_view name,
      const RigidBody& parent,
      const RigidBody& child,
      const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ());
  /// Create a spherical point constraint between two free rigid bodies.
  ///
  /// The current anchor is captured when the joint is created. During
  /// simulation steps the experimental rigid-body constraint path projects the
  /// child anchor back toward the parent anchor while leaving relative
  /// orientation free. This is design-mode only.
  Joint addRigidBodySphericalJoint(
      std::string_view name, const RigidBody& parent, const RigidBody& child);
  Joint addRigidBodySphericalJoint(
      std::string_view name,
      const RigidBody& parent,
      const RigidBody& child,
      const Eigen::Vector3d& parentAnchor,
      const Eigen::Vector3d& childAnchor);
  /// Create a finite-stiffness radial spring between two free rigid bodies.
  ///
  /// The spring connects each body's origin by default. The overload with
  /// anchors accepts body-local anchor points. This experimental AVBD path is
  /// design-mode only and is projected by the rigid-body contact stage.
  void addRigidBodyDistanceSpring(
      std::string_view name,
      const RigidBody& parent,
      const RigidBody& child,
      double restLength,
      double stiffness);
  void addRigidBodyDistanceSpring(
      std::string_view name,
      const RigidBody& parent,
      const RigidBody& child,
      double restLength,
      double stiffness,
      const Eigen::Vector3d& parentAnchor,
      const Eigen::Vector3d& childAnchor);
  /// Check whether a named rigid-body distance spring exists.
  bool hasRigidBodyDistanceSpring(std::string_view name) const;
  /// Read the rest length and material stiffness for a named distance spring.
  ///
  /// Throws if no spring with `name` exists.
  std::pair<double, double> getRigidBodyDistanceSpringParameters(
      std::string_view name) const;
  /// Retune an existing rigid-body distance spring.
  ///
  /// This may be called after `enterSimulationMode()` so interactive tooling
  /// can expose the spring's public rest-length and stiffness controls.
  void setRigidBodyDistanceSpringParameters(
      std::string_view name, double restLength, double stiffness);
  /// Get any supported public joint between free rigid bodies by name.
  ///
  /// This returns fixed, revolute, prismatic, and spherical rigid-body joints.
  /// It does not return multibody joints; use Multibody::getJoint() for
  /// articulated links.
  std::optional<Joint> getRigidBodyJoint(std::string_view name);
  bool hasRigidBodyJoint(std::string_view name) const;
  std::size_t getRigidBodyJointCount() const;
  std::vector<Joint> getRigidBodyJoints();
  /// Get a public fixed joint between free rigid bodies by name.
  ///
  /// This does not return multibody joints; use Multibody::getJoint() for
  /// articulated links.
  std::optional<Joint> getRigidBodyFixedJoint(std::string_view name);
  bool hasRigidBodyFixedJoint(std::string_view name) const;
  std::size_t getRigidBodyFixedJointCount() const;
  std::vector<Joint> getRigidBodyFixedJoints();
  std::optional<RigidBody> getRigidBody(std::string_view name);
  bool hasRigidBody(std::string_view name) const;
  std::size_t getRigidBodyCount() const;

  //--------------------------------------------------------------------------
  // Deformable body management
  //--------------------------------------------------------------------------
  DeformableBody addDeformableBody(
      std::string_view name, const DeformableBodyOptions& options);
  std::optional<DeformableBody> getDeformableBody(std::string_view name);
  bool hasDeformableBody(std::string_view name) const;
  std::size_t getDeformableBodyCount() const;

  /// Opt the named deformable body into the iterative block-coordinate inner
  /// solver and configure it with solver-agnostic quality/performance options.
  /// Without this call the body uses the default per-step solver. Throws if no
  /// deformable body has the given name.
  void configureDeformableSolver(
      std::string_view name, const DeformableSolverOptions& options);

  //--------------------------------------------------------------------------
  // Simulation control
  //--------------------------------------------------------------------------
  [[nodiscard]] bool isSimulationMode() const
  {
    return m_simulationMode;
  }

  void enterSimulationMode();

  /// Set the uniform gravitational acceleration applied to dynamic bodies.
  ///
  /// Gravity is applied as an acceleration during rigid-body integration
  /// (`a = force / mass + gravity`); it is not stored in any per-body force
  /// accumulator. Bodies with non-positive or non-finite mass are unaffected.
  /// The value must contain only finite coordinates. Defaults to
  /// `(0, 0, -9.81)`.
  void setGravity(const Eigen::Vector3d& gravity);

  /// Get the uniform gravitational acceleration applied to dynamic bodies.
  [[nodiscard]] const Eigen::Vector3d& getGravity() const noexcept;

  /// Select the solver family used by the default rigid-body step pipeline.
  ///
  /// The default remains SequentialImpulse. Ipc is experimental and currently
  /// handles free mesh-like rigid bodies through the internal rigid IPC stage.
  /// Throws if Ipc is selected while public rigid-body fixed joints exist.
  void setRigidBodySolver(RigidBodySolver solver);

  /// Get the solver family used by the default rigid-body step pipeline.
  [[nodiscard]] RigidBodySolver getRigidBodySolver() const noexcept;

  void setTimeStep(double timeStep);
  [[nodiscard]] double getTimeStep() const noexcept;

  /// Whether this World opted in to differentiable simulation.
  ///
  /// Differentiability is opt-in via `WorldOptions::differentiable` and
  /// defaults false. The flag does not change the forward step result; when
  /// false the step executes the identical non-differentiable path.
  [[nodiscard]] bool isDifferentiable() const noexcept;

  /// The rigid-body contact resolution method this World uses.
  ///
  /// Selected via `WorldOptions::contactSolverMethod` (default
  /// `SequentialImpulse`) and independent of the differentiable flag. The
  /// `BoxedLcp` value opts the rigid-body contact stage into the boxed-LCP
  /// normal solve; all other behavior is unchanged.
  [[nodiscard]] ContactSolverMethod getContactSolverMethod() const noexcept;

  /// The backward-pass contact gradient mode this World uses.
  ///
  /// Selected via `WorldOptions::contactGradientMode` (default `Analytic`) and
  /// independent of the differentiable flag and the contact-solver method. It
  /// affects ONLY the gradient returned by `getStepDerivatives()` /
  /// `applyStepVjp()`; the forward step result is identical for every value.
  /// See `ContactGradientMode` for the per-mode semantics (the non-`Analytic`
  /// modes are research-grade heuristics, not the true derivative).
  [[nodiscard]] ContactGradientMode getContactGradientMode() const noexcept;

  /// Set the backward-pass contact gradient mode.
  ///
  /// Safe to change at any time (including mid-simulation): the mode affects
  /// ONLY the gradient assembled by the next `getStepDerivatives()` /
  /// `applyStepVjp()`, never the forward step or any stored state. It does not
  /// require the differentiable opt-in to be set (it is simply inert without
  /// it).
  /// @throws InvalidArgumentException if `mode` is not a valid
  ///         `ContactGradientMode` enumerator.
  void setContactGradientMode(ContactGradientMode mode);

  /// Get the explicit Jacobian blocks of the most recent step.
  ///
  /// With the default `SequentialImpulse` contact solver this returns the
  /// contact-free (free-fall / multibody) step Jacobian. With
  /// `ContactSolverMethod::BoxedLcp` it returns the contact-aware Jacobian,
  /// including the analytic frictionless normal-contact gradient, whenever the
  /// step had active contacts; with no active contacts the result reduces
  /// exactly to the contact-free Jacobian.
  ///
  /// @throws InvalidOperationException if this World did not opt in to
  ///         differentiable simulation (`isDifferentiable() == false`), or if
  ///         differentiable support was not compiled (`DART_BUILD_DIFF=OFF`).
  /// @throws NotImplementedException if, under `BoxedLcp`, the step had active
  ///         contacts outside the supported slice (multibody/articulated-link
  ///         contact, or rotational contact whose lever arm is not parallel to
  ///         the contact normal). The supported slice is frictionless
  ///         translational rigid-body contact (PLAN-110 WS2 first slice); the
  ///         API throws rather than returning a wrong (contact-omitting)
  ///         matrix.
  [[nodiscard]] StepDerivatives getStepDerivatives() const;

  /// Apply the reverse-mode (vector-Jacobian product) rule of the most recent
  /// step to an upstream loss gradient.
  ///
  /// Given `dLossDNextState = dL/dx'` (size `2*ndof`, matching the next-state
  /// layout `x' = [q'; q̇']`), this returns the pullback to the step inputs
  /// using the Jacobians cached by the most recent step:
  ///   - `state   = stateJacobianᵀ   · dLossDNextState` (size `2*ndof`),
  ///   - `control = controlJacobianᵀ · dLossDNextState` (size `ndof`).
  ///
  /// This is the explicit vector-Jacobian product of `getStepDerivatives()`; it
  /// is exposed as a first-class reverse-mode primitive so a Python autograd
  /// bridge does not have to materialize and transpose the full Jacobians.
  ///
  /// @throws InvalidArgumentException if `dLossDNextState` does not have size
  ///         `2*ndof` (the next-state dimension of the cached Jacobians).
  /// @throws InvalidOperationException / NotImplementedException under the same
  ///         conditions as getStepDerivatives() (not differentiable, not built,
  ///         or no derivatives cached yet).
  [[nodiscard]] StepGradient applyStepVjp(
      const Eigen::VectorXd& dLossDNextState) const;

  //--------------------------------------------------------------------------
  // Differentiable physical parameters (system identification)
  //--------------------------------------------------------------------------
  /// Register a PHYSICAL PARAMETER of a rigid body for differentiation.
  ///
  /// After registration, each subsequent step's `getStepDerivatives()`
  /// populates `StepDerivatives::parameterJacobian` with `∂x'/∂θ`: the columns
  /// are the partial derivatives of the next state with respect to the
  /// registered scalar parameters, in registration order. A registration spans
  /// one or more contiguous columns by parameter kind: `MASS` and `FRICTION`
  /// one column each, `INERTIA` three columns (the diagonal principal moments
  /// Ixx, Iyy, Izz, in axis order). With no registered parameter the parameter
  /// Jacobian stays empty and the state/control Jacobians are unaffected.
  ///
  /// Supported (PLAN-110 WS4): `MASS`, `INERTIA`, and `FRICTION`.
  /// `CENTER_OF_MASS` is rejected (the rigid-body step fixes the COM at the
  /// body origin, so its gradient is identically zero). The optional bounds on
  /// `selector` are advisory metadata for downstream optimizers and are not
  /// enforced here.
  ///
  /// @throws InvalidOperationException if this World did not opt in to
  ///         differentiable simulation, or differentiable support was not
  ///         compiled (`DART_BUILD_DIFF=OFF`).
  /// @throws InvalidArgumentException if `selector.body` does not belong to
  /// this
  ///         World or is not a valid rigid body.
  /// @throws NotImplementedException if `selector.parameter` is not supported
  ///         (`CENTER_OF_MASS`).
  void addDifferentiableParameter(const PhysicalParameterSelector& selector);

  /// Convenience overload: register `parameter` of `body` with no bounds.
  void addDifferentiableParameter(
      const RigidBody& body, PhysicalParameter parameter);

  /// Number of registered differentiable parameters (registration count, not
  /// the column count). A registration may span more than one column of
  /// `StepDerivatives::parameterJacobian`: `MASS`/`FRICTION` one column each,
  /// `INERTIA` three. Read `parameterJacobian.cols()` for the total column
  /// count.
  [[nodiscard]] std::size_t getNumDifferentiableParameters() const noexcept;

  //--------------------------------------------------------------------------
  // State / control vectors
  //--------------------------------------------------------------------------
  /// Number of generalized coordinates `ndof` exposed to the differentiable
  /// state/control interface.
  ///
  /// First slice: the translational rigid-body reduction. `ndof = 3 * (number
  /// of dynamic, non-static rigid bodies)`, matching the `[q; q̇]` layout of the
  /// contact-aware step Jacobian. The full state vector therefore has size
  /// `2 * ndof` and the control vector has size `ndof`.
  [[nodiscard]] std::size_t getNumDofs() const;

  /// Number of control (effort) coordinates. Equal to `getNumDofs()`: the
  /// control `u = τ` for the translational reduction is the applied force on
  /// each dynamic rigid body, three components per body.
  [[nodiscard]] std::size_t getNumEfforts() const;

  /// The current state vector `x = [q; q̇]`, size `2 * getNumDofs()`.
  ///
  /// `q` is the stacked translational position and `q̇` the stacked linear
  /// velocity of the dynamic (non-static) rigid bodies in registration order,
  /// matching the layout of `getStepDerivatives()`.
  [[nodiscard]] Eigen::VectorXd getStateVector() const;

  /// Overwrite the state vector `x = [q; q̇]` (size `2 * getNumDofs()`).
  ///
  /// @throws InvalidArgumentException if `state` does not have size
  ///         `2 * getNumDofs()`.
  void setStateVector(const Eigen::VectorXd& state);

  /// The current control vector `u = τ`, size `getNumEfforts()`: the applied
  /// force on each dynamic rigid body, three components per body, in
  /// registration order.
  [[nodiscard]] Eigen::VectorXd getControlVector() const;

  /// Overwrite the control vector `u = τ` (size `getNumEfforts()`).
  ///
  /// @throws InvalidArgumentException if `control` does not have size
  ///         `getNumEfforts()`.
  void setControlVector(const Eigen::VectorXd& control);

  void setTime(double time);
  [[nodiscard]] double getTime() const noexcept;
  [[nodiscard]] std::size_t getFrame() const noexcept;
  void sync(WorldSyncStage stage = WorldSyncStage::Kinematics);
  void sync(WorldSyncStage stage, compute::ComputeExecutor& executor);
  void updateKinematics();
  void updateKinematics(compute::ComputeExecutor& executor);
  void step();
  void step(std::size_t count);
  void step(compute::ComputeExecutor& executor);
  void step(std::size_t count, compute::ComputeExecutor& executor);
  void step(compute::ComputeExecutor& executor, compute::WorldStepStage& stage);
  void step(
      std::size_t count,
      compute::ComputeExecutor& executor,
      compute::WorldStepStage& stage);
  void step(
      compute::ComputeExecutor& executor, compute::WorldStepPipeline& pipeline);
  void step(
      std::size_t count,
      compute::ComputeExecutor& executor,
      compute::WorldStepPipeline& pipeline);

  //--------------------------------------------------------------------------
  // Simulation replay
  //--------------------------------------------------------------------------

  /// Enable or disable opt-in simulation replay recording.
  ///
  /// Replay recording is off by default. Enabling it captures the current world
  /// state as the first replay frame, then records one frame after each
  /// timestep completed by the built-in or user-supplied step pipeline.
  /// Restoring a recorded frame writes the saved state back into the existing
  /// world entities, so public handles remain valid and no physics step is
  /// re-run.
  void setReplayRecordingEnabled(bool enabled);

  /// Whether this world is currently appending replay frames after steps.
  [[nodiscard]] bool isReplayRecordingEnabled() const noexcept;

  /// Clear all recorded replay frames.
  ///
  /// If recording is currently enabled, the current world state is immediately
  /// captured as the new frame zero.
  void clearReplayRecording();

  /// Number of states currently available for replay.
  [[nodiscard]] std::size_t getReplayFrameCount() const noexcept;

  /// Current replay cursor, if a frame has been recorded or restored.
  [[nodiscard]] std::optional<std::size_t> getReplayCursor() const noexcept;

  /// Simulation time stored at replay frame `index`.
  ///
  /// @throws InvalidArgumentException if `index` is out of range.
  [[nodiscard]] double getReplayFrameTime(std::size_t index) const;

  /// Simulation frame number stored at replay frame `index`.
  ///
  /// @throws InvalidArgumentException if `index` is out of range.
  [[nodiscard]] std::size_t getReplaySimulationFrame(std::size_t index) const;

  /// Restore a recorded frame without re-running physics.
  ///
  /// The restore is in-place with respect to existing entities and handles. If
  /// recording is enabled and the caller subsequently steps from the restored
  /// frame, frames after the cursor are discarded before the new branch is
  /// appended.
  ///
  /// @throws InvalidArgumentException if `index` is out of range.
  void restoreReplayFrame(std::size_t index);

  //--------------------------------------------------------------------------
  // Memory
  //--------------------------------------------------------------------------
  /// Returns the memory manager owned by this World.
  ///
  /// Components and stages should borrow allocators from this manager instead
  /// of owning allocator roots. This mirrors the classic World ownership model
  /// while leaving the later EnTT storage allocator integration behind an
  /// internal implementation boundary.
  [[nodiscard]] common::MemoryManager& getMemoryManager();
  [[nodiscard]] const common::MemoryManager& getMemoryManager() const;

  /// Returns current frame-scratch diagnostics for this World.
  [[nodiscard]] WorldMemoryDiagnostics getMemoryDiagnostics() const;

  /// Diagnostics from the deformable solve on the most recent ``step`` that
  /// used the built-in pipeline (the ``step()`` / ``step(count)`` /
  /// ``step(executor)`` overloads). For a multi-step call it reflects the last
  /// step. The user-supplied-pipeline overloads do not update this (read the
  /// stage's own ``getLastStats`` there); it stays at its previous value.
  const DeformableSolverDiagnostics& getLastDeformableSolverDiagnostics() const;

  //--------------------------------------------------------------------------
  // Profiling
  //--------------------------------------------------------------------------
  /// Enables or disables per-stage step profiling. When enabled, every ``step``
  /// records the wall-clock time of each pipeline stage into a snapshot
  /// retrievable via ``getLastStepProfile``. This is the DART 7 World's
  /// non-GUI, text-first performance surface, intended for tools, bindings, and
  /// AI agents optimizing a step. Requires ``DART_BUILD_PROFILE=ON``; when that
  /// build option is off this toggle is a no-op, ``World`` stores no profiling
  /// cache fields, and the step path has no compiled profiling branch. Disabled
  /// by default in profiling-enabled
  /// builds; when off the step path is unchanged and adds no profiling
  /// overhead.
  void setStepProfilingEnabled(bool enabled) noexcept;

  /// Whether per-stage step profiling is currently enabled. Always false when
  /// DART was built with ``DART_BUILD_PROFILE=OFF``.
  [[nodiscard]] bool isStepProfilingEnabled() const noexcept;

  /// Per-stage wall-clock profile of the most recent ``step`` taken while
  /// profiling was enabled. Empty before the first such step. For a multi-step
  /// call it reflects the last step. Use ``WorldStepProfile::toSummaryText``
  /// for a compact, readable breakdown.
  [[nodiscard]] const compute::WorldStepProfile& getLastStepProfile()
      const noexcept;

  //--------------------------------------------------------------------------
  // Multibody solver configuration
  //--------------------------------------------------------------------------

  /// Set the multibody solver/integration configuration as a whole (see
  /// `MultibodyOptions`). Configuration is by documented method-family name, so
  /// new capabilities are added as `MultibodyOptions` fields rather than as new
  /// World methods, and no solver/stage types are exposed. Throws
  /// InvalidArgumentException for an unknown `integrationFamily`. Selection is
  /// parsed to an internal representation here, so the per-step path carries no
  /// configuration-parsing cost.
  void setMultibodyOptions(const MultibodyOptions& options);

  /// The current multibody solver/integration configuration. The
  /// `integrationFamily` defaults to `"semi-implicit"`.
  [[nodiscard]] MultibodyOptions getMultibodyOptions() const;

  //--------------------------------------------------------------------------
  // Collision queries
  //--------------------------------------------------------------------------

  /// Add or remove a persistent collision-query exclusion between two collision
  /// bodies.
  ///
  /// Endpoints must be valid rigid-body or multibody-link frame handles owned
  /// by this World. The filter is order-independent and is applied by
  /// `collide()` and the built-in contact stages before narrow-phase contact
  /// generation.
  void setCollisionPairIgnored(
      const Frame& first, const Frame& second, bool ignored = true);

  /// Return whether a persistent collision-query exclusion exists between two
  /// rigid-body or multibody-link frame handles.
  [[nodiscard]] bool isCollisionPairIgnored(
      const Frame& first, const Frame& second) const;

  /// Clear all persistent collision-query exclusions.
  void clearIgnoredCollisionPairs();

  /// Return the number of persistent collision-query exclusions.
  [[nodiscard]] std::size_t getIgnoredCollisionPairCount() const noexcept;

  /// Run a collision query over all bodies that have a collision shape.
  ///
  /// This is a query, not a solver: it reports contact points (position,
  /// world-frame normal from bodyA toward bodyB, and penetration depth) using
  /// the current body world transforms. It does not modify body state. Rigid
  /// bodies and multibody links without a collision shape are ignored.
  [[nodiscard]] std::vector<Contact> collide();

  /// Run a collision query with explicit filtering options.
  ///
  /// Body-type switches can include or exclude rigid-body pairs,
  /// rigid-body/link pairs, and link/link pairs independently.
  ///
  /// `includeSameMultibodyLinkPairs=false` filters link-vs-link contacts within
  /// the same multibody while preserving rigid-body pairs, link-vs-rigid-body
  /// pairs, and link-vs-link contacts across different multibodies.
  [[nodiscard]] std::vector<Contact> collide(
      const CollisionQueryOptions& options);

  //--------------------------------------------------------------------------
  // Serialization
  //--------------------------------------------------------------------------
  void saveBinary(std::ostream& output) const;
  void loadBinary(std::istream& input);

  //--------------------------------------------------------------------------
  // Utilities
  //--------------------------------------------------------------------------
  void clear();

private:
  friend class Frame;
  friend class FreeFrame;
  friend class FixedFrame;
  friend class Link;
  friend class Joint;
  friend class LoopClosure;
  friend class Multibody;
  friend class RigidBody;
  friend class DeformableBody;
  friend class io::detail::SkeletonLoaderWorldAccess;
  friend class compute::MultibodyContactStage;
  friend class compute::MultibodyForwardDynamicsStage;
  friend class compute::RigidBodyContactStage;
  friend class compute::WorldKinematicsGraph;
  friend class compute::RigidIpcContactStage;
  friend class compute::UnifiedConstraintStage;

  /// Internal storage seam. `detail::storageOf` reaches the privately-held,
  /// ECS-typed `WorldStorage` without exposing it on the public surface; the
  /// internal `detail::registryOf` escape hatch is built on top of it.
  /// Friending these `detail` free functions names no EnTT symbols, so it is
  /// leak-free.
  friend detail::WorldStorage& detail::storageOf(World& world);
  friend const detail::WorldStorage& detail::storageOf(const World& world);

  struct WorldStorageDeleter
  {
    common::MemoryManager* memoryManager = nullptr;
    void operator()(void* storage) const noexcept;
  };

  using WorldStoragePtr
      = std::unique_ptr<detail::WorldStorage, WorldStorageDeleter>;

  Frame resolveParentFrame(const Frame& parent) const;
  struct CollisionQueryCache;
  struct CollisionQueryCacheDeleter
  {
    common::MemoryManager* memoryManager = nullptr;
    void operator()(void* cache) const noexcept;
  };
  using CollisionQueryCachePtr
      = std::unique_ptr<CollisionQueryCache, CollisionQueryCacheDeleter>;
  struct StepPipelineCache;
  struct StepPipelineCacheDeleter
  {
    common::MemoryManager* memoryManager = nullptr;
    void operator()(void* cache) const noexcept;
  };
  using StepPipelineCachePtr
      = std::unique_ptr<StepPipelineCache, StepPipelineCacheDeleter>;
  static WorldStoragePtr makeWorldStorage(common::MemoryManager& memoryManager);
  static CollisionQueryCachePtr makeCollisionQueryCache(
      common::MemoryManager& memoryManager);
  static StepPipelineCachePtr makeStepPipelineCache(
      common::MemoryManager& memoryManager);
  struct ReplayState;
  struct ReplayStateDeleter
  {
    common::MemoryManager* memoryManager = nullptr;
    void operator()(void* replayState) const noexcept;
  };
  using ReplayStatePtr = std::unique_ptr<ReplayState, ReplayStateDeleter>;
  static ReplayStatePtr makeReplayState(common::MemoryManager& memoryManager);
  Entity createFrameEntity(
      std::string_view name,
      const Frame& parentFrame,
      const Eigen::Isometry3d& localTransform,
      std::size_t* autoNameCounter,
      std::string_view autoNamePrefix,
      bool isFixedFrame,
      std::string& outName);
  Joint addRigidBodyJoint(
      std::string_view name,
      const RigidBody& parent,
      const RigidBody& child,
      JointType type,
      const Eigen::Vector3d& axis,
      std::optional<Eigen::Vector3d> parentAnchor = std::nullopt,
      std::optional<Eigen::Vector3d> childAnchor = std::nullopt);
  void addRigidBodyDistanceSpringImpl(
      std::string_view name,
      const RigidBody& parent,
      const RigidBody& child,
      double restLength,
      double stiffness,
      std::optional<Eigen::Vector3d> parentAnchor,
      std::optional<Eigen::Vector3d> childAnchor);
  Joint addArticulatedJoint(
      std::string_view name,
      const Link* parent,
      const Link& child,
      JointType type,
      const Eigen::Vector3d& axis,
      std::optional<Eigen::Vector3d> parentAnchor = std::nullopt,
      std::optional<Eigen::Vector3d> childAnchor = std::nullopt);

  void ensureDesignMode() const;
  [[nodiscard]] std::span<const Contact> queryContacts(
      const CollisionQueryOptions& options);
  void markFrameTopologyChanged() noexcept;
  [[nodiscard]] std::uint64_t getFrameTopologyRevision() const noexcept;
  void reserveRegistryStorageForSimulation();
  void prepareStepPipelineCacheForCurrentConfiguration();
  void resetCountersFromRegistry();
  bool tryStepCleanNoWorkDefaultPipeline();
  void stepPipelineOnce(
      compute::ComputeExecutor& executor, compute::WorldStepPipeline& pipeline);
  void recordReplayFrame();
  void resetFrameScratchForStep();
  void refreshMemoryDiagnostics();

  /// Record the analytic step Jacobians at the current (pre-step) state into
  /// the cached step derivatives. Under `ContactSolverMethod::BoxedLcp` this
  /// captures
  /// the active contacts from `collide()` and routes through the contact-aware
  /// Jacobian (`detail::contactStepDerivatives`), reducing to the contact-free
  /// result when there are no active contacts and throwing
  /// `NotImplementedException` for active contacts outside the WS2 slice;
  /// otherwise it records the contact-free multibody Jacobian. A no-op unless
  /// differentiable support is compiled (`DART_BUILD_DIFF`); callers gate on
  /// `m_differentiable`.
  void captureStepDerivatives();
  double getRigidIpcAdaptiveBarrierStiffnessLowerBound() const noexcept;
  void setRigidIpcAdaptiveBarrierStiffnessLowerBound(double value) noexcept;
  void resetRigidIpcAdaptiveBarrierStiffnessLowerBound() noexcept;

  common::MemoryManager m_memoryManager;
  /// Opaque, ECS-typed state (the EnTT registry, the registered differentiable
  /// parameters, and the cached step Jacobians). Held by pointer so the
  /// promoted public `world.hpp` names no EnTT symbols; the complete type lives
  /// in the internal `detail/world_storage.hpp`. Because this is a `unique_ptr`
  /// to an incomplete type with a root-allocator deleter, `~World()` (and the
  /// move operations, were they enabled) must be declared here and defined
  /// out-of-line in `world.cpp`.
  /// Always non-null after construction.
  WorldStoragePtr m_storage;
  bool m_simulationMode{false};
  Eigen::Vector3d m_gravity{0.0, 0.0, -9.81};
  RigidBodySolver m_rigidBodySolver{RigidBodySolver::SequentialImpulse};
  double m_timeStep{0.001};
  bool m_differentiable{false};
  ContactSolverMethod m_contactSolverMethod{
      ContactSolverMethod::SequentialImpulse};
  ContactGradientMode m_contactGradientMode{ContactGradientMode::Analytic};
  double m_time{0.0};
  DeformableSolverDiagnostics m_lastDeformableSolverDiagnostics{};
  WorldMemoryDiagnostics m_memoryDiagnostics{};
#if DART_BUILD_PROFILE
  bool m_stepProfilingEnabled{false};
  compute::WorldStepProfile m_lastStepProfile{};
  compute::WorldStepProfile m_stepProfileScratch{};
#endif
  double m_rigidIpcAdaptiveBarrierStiffnessLowerBound{1.0};
  std::uint64_t m_frameTopologyRevision{0};
  enum class MultibodyIntegrationMethod
  {
    SemiImplicit,
    Variational
  };
  MultibodyIntegrationMethod m_multibodyIntegrationMethod{
      MultibodyIntegrationMethod::SemiImplicit};
  std::size_t m_variationalIntegratorMaxIterations{100};
  double m_variationalIntegratorTolerance{1e-10};
  std::size_t m_frame{0};

  std::size_t m_freeFrameCounter{0};
  std::size_t m_fixedFrameCounter{0};
  std::size_t m_multibodyCounter{0};
  std::size_t m_loopClosureCounter{0};
  std::size_t m_rigidBodyCounter{0};
  std::size_t m_deformableBodyCounter{0};
  std::size_t m_linkCounter{0};
  std::size_t m_jointCounter{0};
  mutable CollisionQueryCachePtr m_collisionQueryCache;
  StepPipelineCachePtr m_stepPipelineCache;
  ReplayStatePtr m_replay;
};

} // namespace dart::simulation
