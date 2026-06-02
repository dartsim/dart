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

#include <dart/simulation/experimental/fwd.hpp>

#include <dart/simulation/experimental/body/deformable_body_options.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/constraint/loop_closure.hpp>
#include <dart/simulation/experimental/diff/step_derivatives.hpp>
#include <dart/simulation/experimental/diff/step_gradient.hpp>
#include <dart/simulation/experimental/multibody/multibody_options.hpp>
#include <dart/simulation/experimental/world_options.hpp>
#include <dart/simulation/experimental/world_sync_stage.hpp>

#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <iosfwd>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cstddef>

namespace dart::simulation::experimental {

namespace io::detail {
class SkeletonLoaderWorldAccess;
} // namespace io::detail

struct WorldOptions;

/// Solver family used for free rigid-body dynamics in the default experimental
/// World step pipeline.
enum class RigidBodySolver
{
  SequentialImpulse,
  Ipc,
};

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

class DART_EXPERIMENTAL_API World
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
  Multibody addMultibody(std::string_view name);
  std::optional<Multibody> getMultibody(std::string_view name);
  bool hasMultibody(std::string_view name) const;
  std::size_t getMultibodyCount() const;

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
  /// @throws InvalidOperationException if the World is in simulation mode or
  ///         the internal fixed-joint row configuration fails.
  Joint addRigidBodyFixedJoint(
      std::string_view name, const RigidBody& parent, const RigidBody& child);
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
  void setContactGradientMode(ContactGradientMode mode) noexcept;

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

  /// Diagnostics from the deformable solve on the most recent ``step`` that
  /// used the built-in pipeline (the ``step()`` / ``step(count)`` /
  /// ``step(executor)`` overloads). For a multi-step call it reflects the last
  /// step. The user-supplied-pipeline overloads do not update this (read the
  /// stage's own ``getLastStats`` there); it stays at its previous value.
  const DeformableSolverDiagnostics& getLastDeformableSolverDiagnostics() const;

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
  // Registry access
  //--------------------------------------------------------------------------
  /// @internal
  /// DART 7 implementation escape hatch for tests and subsystem bring-up.
  /// This is not part of the DART 7 public World facade promotion target;
  /// prefer public handles and accessors for user-facing code.
  entt::registry& getRegistry();
  /// @internal
  /// See the non-const overload.
  const entt::registry& getRegistry() const;

  //--------------------------------------------------------------------------
  // Collision queries
  //--------------------------------------------------------------------------

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

  Frame resolveParentFrame(const Frame& parent) const;
  struct CollisionQueryCache;
  entt::entity createFrameEntity(
      std::string_view name,
      const Frame& parentFrame,
      const Eigen::Isometry3d& localTransform,
      std::size_t* autoNameCounter,
      std::string_view autoNamePrefix,
      bool isFixedFrame,
      std::string& outName);

  void ensureDesignMode() const;
  void resetCountersFromRegistry();

  /// Record the analytic step Jacobians at the current (pre-step) state into
  /// `m_stepDerivatives`. Under `ContactSolverMethod::BoxedLcp` this captures
  /// the active contacts from `collide()` and routes through the contact-aware
  /// Jacobian (`detail::contactStepDerivatives`), reducing to the contact-free
  /// result when there are no active contacts and throwing
  /// `NotImplementedException` for active contacts outside the WS2 slice;
  /// otherwise it records the contact-free multibody Jacobian. A no-op unless
  /// differentiable support is compiled (`DART_BUILD_DIFF`); callers gate on
  /// `m_differentiable`.
  void captureStepDerivatives();

  entt::registry m_registry;
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
  enum class MultibodyIntegrationMethod
  {
    SemiImplicit,
    Variational
  };
  MultibodyIntegrationMethod m_multibodyIntegrationMethod{
      MultibodyIntegrationMethod::SemiImplicit};
  std::size_t m_frame{0};

  /// Cached explicit Jacobians of the most recent differentiable step.
  /// Populated only when `m_differentiable` is true and differentiable support
  /// is compiled
  /// (`DART_BUILD_DIFF`); always empty otherwise.
  std::optional<StepDerivatives> m_stepDerivatives;

  /// Registered differentiable physical parameters, in registration order. Each
  /// entry pairs the owning rigid-body entity with the parameter to
  /// differentiate. Columns of `StepDerivatives::parameterJacobian` follow this
  /// order. Stored in both build configs (the public registration API exists
  /// either way); only consumed when differentiable support is compiled.
  std::vector<std::pair<entt::entity, PhysicalParameter>>
      m_differentiableParameters;

  std::size_t m_freeFrameCounter{0};
  std::size_t m_fixedFrameCounter{0};
  std::size_t m_multibodyCounter{0};
  std::size_t m_loopClosureCounter{0};
  std::size_t m_rigidBodyCounter{0};
  std::size_t m_deformableBodyCounter{0};
  std::size_t m_linkCounter{0};
  std::size_t m_jointCounter{0};
  mutable std::unique_ptr<CollisionQueryCache> m_collisionQueryCache;
};

} // namespace dart::simulation::experimental
