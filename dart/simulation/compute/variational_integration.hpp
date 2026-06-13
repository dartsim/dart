/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#pragma once

#include <dart/simulation/comps/component_category.hpp>
#include <dart/simulation/compute/compute_stage_metadata.hpp>
#include <dart/simulation/compute/multibody_dynamics.hpp>
#include <dart/simulation/compute/world_step_stage.hpp>
#include <dart/simulation/detail/world_registry_types.hpp>
#include <dart/simulation/export.hpp>
#include <dart/simulation/fwd.hpp>

#include <dart/common/memory_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::comps {
struct MultibodyStructure;
} // namespace dart::simulation::comps

namespace dart::simulation::compute {

struct MultibodyVariationalTreeScratchAccess;

struct VariationalProjectionRowBounds
{
  double lower = -std::numeric_limits<double>::infinity();
  double upper = std::numeric_limits<double>::infinity();
};

/// Persistent two-step discrete-mechanics history for one multibody, indexed by
/// link construction order (parent-before-child).
///
/// The linear-time variational integrator is a two-step method: evaluating the
/// forced discrete Euler-Lagrange residual at step k needs the relative
/// configuration displacement and the discrete spatial momentum carried in from
/// step k-1. This is dynamic State, owned by the integrator and hidden from the
/// public facade.
///
/// **Internal Implementation Detail** - Not exposed in public API. Serialized
/// (State category) so a saved/loaded trajectory resumes without
/// re-bootstrapping the two-step history. It holds no entity references, so no
/// remapping pass is needed.
struct MultibodyVariationalState
{
  DART_SIMULATION_STATE_COMPONENT(MultibodyVariationalState);

  using DeltaTransformVector = std::
      vector<Eigen::Isometry3d, dart::common::StlAllocator<Eigen::Isometry3d>>;
  using MomentumVector = std::vector<
      Eigen::Matrix<double, 6, 1>,
      dart::common::StlAllocator<Eigen::Matrix<double, 6, 1>>>;

  /// Whether `previousDeltaTransform`/`previousMomentum` have been seeded from
  /// a consistent prior step. The first integration bootstraps them from the
  /// current generalized velocity.
  bool bootstrapped = false;

  /// Per-link relative configuration displacement dT from the previous step
  /// (the SE(3) transform whose log gives the previous average velocity).
  DeltaTransformVector previousDeltaTransform;

  /// Per-link discrete spatial momentum from the previous step (link frame).
  MomentumVector previousMomentum;
};

/// Diagnostics from one RIQN (recursive impulse-based quasi-Newton) solve.
///
/// **Internal Implementation Detail** - Not exposed in public API.
struct VariationalSolveReport
{
  std::size_t iterations = 0; ///< RIQN iterations performed.
  double residualNorm = 0.0;  ///< Final forced-DEL residual L2 norm.
  bool converged = false;     ///< Whether the residual fell below tolerance.
};

/// A holonomic loop-closure constraint between two endpoint frames fixed on two
/// links of the same multibody (or an endpoint on a link and a fixed world
/// anchor when `linkB`/`linkA` is null). Point closures constrain the 3D offset
/// to zero (3 rows); distance closures constrain the separation to `length`
/// (1 row); rigid closures additionally constrain the relative orientation
/// (6 rows: 3 position + 3 world-frame rotation residual).
///
/// **Internal Implementation Detail** - Not exposed in public API.
struct VariationalLoopConstraint
{
  entt::entity linkA = entt::null; ///< null => `pointA` is a world anchor.
  Eigen::Vector3d pointA
      = Eigen::Vector3d::Zero();   ///< local on linkA (or world).
  entt::entity linkB = entt::null; ///< null => `pointB` is a world anchor.
  Eigen::Vector3d pointB
      = Eigen::Vector3d::Zero(); ///< local on linkB (or world).
  bool distance = false;         ///< false: point (3 rows); true: distance (1).
  double length = 0.0;           ///< target separation when `distance`.
  bool rigid
      = false; ///< true: also constrain relative orientation (3 extra rows).
  Eigen::Matrix3d rotationA
      = Eigen::Matrix3d::Identity(); ///< endpoint-A offset
                                     ///< rotation (rigid).
  Eigen::Matrix3d rotationB
      = Eigen::Matrix3d::Identity(); ///< endpoint-B offset
                                     ///< rotation (rigid).
  Eigen::Matrix3d linearAxes
      = Eigen::Matrix3d::Identity(); ///< world-frame linear row axes.
  Eigen::Matrix3d angularAxes
      = Eigen::Matrix3d::Identity();   ///< world-frame angular row axes.
  std::uint8_t linearAxisMask = 0x7u;  ///< active columns in linearAxes.
  std::uint8_t angularAxisMask = 0x7u; ///< active columns in angularAxes.
  bool linearMotor = false;            ///< true: project one linear motor.
  Eigen::Vector3d linearMotorAxis
      = Eigen::Vector3d::UnitX(); ///< world-frame motor row axis.
  double linearMotorReferencePosition
      = 0.0; ///< start-step relative linear coordinate.
  double linearMotorTargetSpeed = 0.0; ///< target relative speed (m/s).
  double linearMotorMaxForce
      = std::numeric_limits<double>::infinity(); ///< motor row force bound.
  bool angularMotor = false; ///< true: project one angular motor.
  Eigen::Vector3d angularMotorAxis
      = Eigen::Vector3d::UnitZ(); ///< world-frame motor row axis.
  double angularMotorReferencePosition
      = 0.0; ///< start-step relative angular coordinate.
  double angularMotorTargetSpeed = 0.0; ///< target relative speed (rad/s).
  double angularMotorMaxTorque
      = std::numeric_limits<double>::infinity(); ///< motor row torque bound.
  entt::entity sourceJoint = entt::null;         ///< private AVBD source joint.
  double breakForce = 0.0; ///< >0 marks sourceJoint broken.
};

/// **EXPERIMENTAL SPIKE (PLAN-082 contact-roadmap gate 2).** Kinematics of the
/// *trial* configuration `q^{k+1}` passed to an in-loop contact-force hook on
/// each RIQN iteration, so the hook can evaluate a contact potential's
/// generalized force `Q_c(q^{k+1})` at the current iterate (the coupling that
/// folds the potential's curvature into the root-find). All
/// transforms/Jacobians are at the trial configuration; the body Jacobians are
/// body-frame `[angular; linear]` (`6 x dofCount`), in the integrator's
/// generalized-coordinate order. This is probe scaffolding for the
/// compliant-contact go/no-go spike, not a production contact surface.
struct VariationalContactContext
{
  /// Per-link world transform at the trial configuration (link construction
  /// order, parent-before-child).
  std::span<const Eigen::Isometry3d> linkWorldTransforms;
  /// Per-link body-frame spatial Jacobian (`6 x dofCount`, `[angular; linear]`)
  /// at the trial configuration.
  std::span<const Eigen::MatrixXd> linkBodyJacobians;
  /// Movable generalized-coordinate count (the `Q_c` the hook returns has this
  /// length).
  std::size_t dofCount;
  /// Per-link world transform at the *previous* configuration `q^k` (the start
  /// of the step), same link order. Lagged contact laws (friction) use it for
  /// the contact-point position (normal magnitude) at the step start.
  std::span<const Eigen::Isometry3d> previousLinkWorldTransforms;
  /// Per-link body-frame spatial Jacobian at `q^k`. With `previousVelocity` it
  /// gives the lagged contact-point sliding velocity that fixes the friction
  /// direction over the step (so friction stays constant across RIQN iterates,
  /// hence smooth for the root-find -- the roadmap's "lagged friction").
  std::span<const Eigen::MatrixXd> previousLinkBodyJacobians;
  /// Generalized velocity at `q^k` (lagged).
  const Eigen::VectorXd& previousVelocity;
  /// The integration time step (s).
  double timeStep;
};

/// **EXPERIMENTAL SPIKE.** In-loop contact-force hook for the variational
/// integrator: given the trial configuration's kinematics, returns a per-DOF
/// generalized contact force `Q_c` (a `dofCount`-vector) folded into the forced
/// DEL residual exactly like the applied/external force (`residual -= dt *
/// Q_c`, same sign/scaling). An empty hook (the default) is zero overhead and
/// leaves the numerics byte-for-byte identical to the no-contact integrator.
using VariationalContactHook
    = std::function<Eigen::VectorXd(const VariationalContactContext&)>;

/// **EXPERIMENTAL SPIKE.** Generalized force `J(p)^T * F` of a world-frame
/// force `worldForce` applied at the body-frame point `localPoint` on link
/// `linkIndex`, using the trial-configuration kinematics in `context` (the
/// world-frame translational point Jacobian `J(p) = R_i (J_linear - [p]
/// J_angular)` mapped back to generalized coordinates). The glue a contact hook
/// uses to turn a Cartesian contact force into the reduced-coordinate `Q_c`.
[[nodiscard]] DART_SIMULATION_API Eigen::VectorXd variationalContactPointForce(
    const VariationalContactContext& context,
    std::size_t linkIndex,
    const Eigen::Vector3d& localPoint,
    const Eigen::Vector3d& worldForce);

/// **EXPERIMENTAL (PLAN-082 Phase C).** A body-fixed contact point: the point
/// at body-frame position `localPoint` on link `linkIndex`, evaluated against
/// the contact geometry at the trial configuration.
struct VariationalContactPoint
{
  std::size_t linkIndex = 0; ///< link carrying the point
  Eigen::Vector3d localPoint = Eigen::Vector3d::Zero(); ///< body-frame position
};

/// **EXPERIMENTAL (PLAN-082 Phase C, rung C2 — compliant contact).** A static
/// ground half-space `{x : n . (x - p0) >= 0}` and a set of body-fixed contact
/// points repelled by a one-sided quadratic penalty potential
/// `E = 1/2 k max(0,-d)^2`, where `d = n . (p - p0)` is the signed distance of
/// a world contact point `p` to the plane. Its gradient is the compliant normal
/// force `F = k max(0,-d) n` (the VBD/XPBD compliant-contact force law). This
/// is the real distance/gradient query the gate-2 spike's hard-coded plane
/// stood in for, for the link-point-vs-analytic-ground case: bounded curvature
/// RIQN absorbs, leaving a small `mg/k` residual penetration (see the contact
/// roadmap's `k <= 1e4 mg` envelope).
struct VariationalGroundContact
{
  using PointAllocator = dart::common::StlAllocator<VariationalContactPoint>;
  using PointVector = std::vector<VariationalContactPoint, PointAllocator>;

  VariationalGroundContact()
    : VariationalGroundContact(dart::common::MemoryAllocator::GetDefault())
  {
  }

  explicit VariationalGroundContact(dart::common::MemoryAllocator& allocator)
    : points(PointAllocator{allocator})
  {
  }

  Eigen::Vector3d planeNormal
      = Eigen::Vector3d::UnitZ(); ///< unit normal, out of the ground
  Eigen::Vector3d planePoint
      = Eigen::Vector3d::Zero(); ///< any point on the plane
  double stiffness = 0.0;        ///< penalty stiffness k (N/m), >= 0
  double frictionCoefficient
      = 0.0; ///< Coulomb friction mu (>= 0); 0 disables friction (rung C1)
  double frictionRegularization = 1.0e-4; ///< friction velocity scale eps_v
                                          ///< (m/s); > 0 required when mu > 0
  double dampingCoefficient = 0.0; ///< Kelvin-Voigt normal damping c (N*s/m,
                                   ///< >= 0); dissipates the contact transient
                                   ///< (needed for clean AL settling). Honored
                                   ///< by VariationalGroundContactSolver.
  PointVector points;              ///< body-fixed contact points
};

/// **EXPERIMENTAL (PLAN-082 Phase C, rung C2).** Build an in-loop
/// `VariationalContactHook` for compliant ground contact. Each RIQN iteration
/// it evaluates every contact point's world position at the trial
/// configuration, computes the signed plane distance, and for penetrating
/// points (`d < 0`) accumulates the compliant normal force `k(-d) n` mapped to
/// a generalized force by `variationalContactPointForce`. Empty `points` or
/// `stiffness == 0` yields a no-op hook (byte-for-byte identical to the
/// no-contact integrator). Throws if the plane normal is zero or the stiffness
/// is negative; the normal is normalized defensively.
[[nodiscard]] DART_SIMULATION_API VariationalContactHook
makeVariationalGroundContactHook(VariationalGroundContact contact);

/// **EXPERIMENTAL (PLAN-082 Phase C, rung C3 — augmented Lagrangian).** A
/// stateful augmented-Lagrangian wrapper over ground contact that drives the
/// penetration toward zero at finite stiffness (unlike the pure-penalty `mg/k`
/// residual). It holds a per-contact-point dual `lambda >= 0`: each step the
/// normal force is `max(0, lambda + k(-d))` (penalty + dual), with friction
/// bounded by that AL normal magnitude. After each converged step, call
/// `updateDuals` with the converged per-link world transforms to advance
/// `lambda <- max(0, lambda + k(-d))` (dual ascent), warm-started across steps;
/// over a few steps `lambda` carries the steady contact load and the
/// penetration drives to ~0. `lambda = 0` (the initial state) reduces to the C2
/// compliant penalty. Usage: build once, then each step `integrate(...,
/// solver.hook())` followed by
/// `solver.updateDuals(convergedLinkWorldTransforms)`.
class DART_SIMULATION_API VariationalGroundContactSolver
{
public:
  using DualAllocator = dart::common::StlAllocator<double>;
  using DualVector = std::vector<double, DualAllocator>;

  explicit VariationalGroundContactSolver(VariationalGroundContact contact);
  VariationalGroundContactSolver(
      VariationalGroundContact contact,
      dart::common::MemoryAllocator& allocator);

  /// In-loop contact hook reading the current duals (constant across the step's
  /// RIQN iterates, so the AL force is smooth for the root-find). The returned
  /// hook reads this solver's live duals, so build it once and reuse it.
  [[nodiscard]] VariationalContactHook hook() const;

  void setAllocator(dart::common::MemoryAllocator& allocator);
  [[nodiscard]] const dart::common::MemoryAllocator& getAllocator()
      const noexcept;

  /// Evaluate the current contact force into caller-owned storage. The output
  /// is resized to the context DOF count and overwritten.
  void computeForceInto(
      const VariationalContactContext& context,
      Eigen::VectorXd& generalizedForce) const;

  /// Advance the duals after a converged step, from the per-link world
  /// transforms at the converged configuration (same link order as the
  /// context).
  void updateDuals(std::span<const Eigen::Isometry3d> linkWorldTransforms);

  /// The current per-contact-point duals (the accumulated AL contact forces).
  [[nodiscard]] const DualVector& duals() const
  {
    return mDuals;
  }

  /// Seed the duals, e.g. warm-started from persisted state across a save/load
  /// (or across the `World::step()` stage's per-step solver rebuild). The size
  /// must match the contact-point count; throws otherwise.
  void setDuals(std::span<const double> duals);

  /// Replace the contact geometry while preserving existing vector capacity.
  void resetContact(const VariationalGroundContact& contact);

private:
  dart::common::MemoryAllocator* m_allocator = nullptr;
  VariationalGroundContact mContact;
  DualVector mDuals; ///< per contact point, >= 0
};

/// Reusable storage for variational contact-force evaluation.
struct VariationalContactEvaluationScratch
{
  using TransformAllocator = dart::common::StlAllocator<Eigen::Isometry3d>;
  using TransformVector = std::vector<Eigen::Isometry3d, TransformAllocator>;
  using JacobianAllocator = dart::common::StlAllocator<Eigen::MatrixXd>;
  using JacobianVector = std::vector<Eigen::MatrixXd, JacobianAllocator>;

  VariationalContactEvaluationScratch()
    : VariationalContactEvaluationScratch(
          dart::common::MemoryAllocator::GetDefault())
  {
  }

  explicit VariationalContactEvaluationScratch(
      dart::common::MemoryAllocator& allocator)
    : previousWorldTransforms(TransformAllocator{allocator}),
      trialRelativeTransforms(TransformAllocator{allocator}),
      trialWorldTransforms(TransformAllocator{allocator}),
      previousJacobians(JacobianAllocator{allocator}),
      trialJacobians(JacobianAllocator{allocator})
  {
  }

  TransformVector previousWorldTransforms;
  TransformVector trialRelativeTransforms;
  TransformVector trialWorldTransforms;
  JacobianVector previousJacobians;
  JacobianVector trialJacobians;
  Eigen::VectorXd contactForce;
  Eigen::VectorXd forcing;
};

/// Reusable storage for variational articulated linear solves.
struct VariationalLinearSolveScratch
{
  using Matrix6 = Eigen::Matrix<double, 6, 6>;
  using Vector6 = Eigen::Matrix<double, 6, 1>;
  using Matrix6Allocator = dart::common::StlAllocator<Matrix6>;
  using Matrix6Vector = std::vector<Matrix6, Matrix6Allocator>;
  using Vector6Allocator = dart::common::StlAllocator<Vector6>;
  using Vector6Vector = std::vector<Vector6, Vector6Allocator>;
  using MatrixAllocator = dart::common::StlAllocator<Eigen::MatrixXd>;
  using MatrixVector = std::vector<Eigen::MatrixXd, MatrixAllocator>;
  using VectorAllocator = dart::common::StlAllocator<Eigen::VectorXd>;
  using VectorVector = std::vector<Eigen::VectorXd, VectorAllocator>;

  VariationalLinearSolveScratch()
    : VariationalLinearSolveScratch(dart::common::MemoryAllocator::GetDefault())
  {
  }

  explicit VariationalLinearSolveScratch(
      dart::common::MemoryAllocator& allocator)
    : articulated(Matrix6Allocator{allocator}),
      bias(Vector6Allocator{allocator}),
      motionToChild(Matrix6Allocator{allocator}),
      spatial(Vector6Allocator{allocator}),
      forceProjector(MatrixAllocator{allocator}),
      motionProjector(MatrixAllocator{allocator}),
      jointMatrix(MatrixAllocator{allocator}),
      jointMatrixInverse(MatrixAllocator{allocator}),
      jointRhs(VectorAllocator{allocator})
  {
  }

  Matrix6Vector articulated;
  Vector6Vector bias;
  Matrix6Vector motionToChild;
  Vector6Vector spatial;
  MatrixVector forceProjector;
  MatrixVector motionProjector;
  MatrixVector jointMatrix;
  MatrixVector jointMatrixInverse;
  VectorVector jointRhs;
  Eigen::VectorXd jointWork;
  Eigen::VectorXd jointSolveWork;
  Eigen::VectorXd rhs;
  Eigen::VectorXd result;
};

/// Reusable storage for one variational multibody step.
struct VariationalStepScratch
{
  using SpatialVelocityAllocator
      = dart::common::StlAllocator<Eigen::Matrix<double, 6, 1>>;
  using SpatialVelocityVector
      = std::vector<Eigen::Matrix<double, 6, 1>, SpatialVelocityAllocator>;

  VariationalStepScratch()
    : VariationalStepScratch(dart::common::MemoryAllocator::GetDefault())
  {
  }

  explicit VariationalStepScratch(dart::common::MemoryAllocator& allocator)
    : currentSpatialVelocities(SpatialVelocityAllocator{allocator})
  {
  }

  SpatialVelocityVector currentSpatialVelocities;
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd appliedForce;
  Eigen::VectorXd nextPosition;
  Eigen::VectorXd residual;
  Eigen::VectorXd zeroAcceleration;
  Eigen::VectorXd previousJointVelocity;
};

/// Reusable storage for variational multibody tree construction.
class DART_SIMULATION_API MultibodyVariationalTreeScratch final
{
public:
  MultibodyVariationalTreeScratch();
  explicit MultibodyVariationalTreeScratch(
      dart::common::MemoryAllocator& allocator);
  ~MultibodyVariationalTreeScratch();

  MultibodyVariationalTreeScratch(const MultibodyVariationalTreeScratch&)
      = delete;
  MultibodyVariationalTreeScratch& operator=(
      const MultibodyVariationalTreeScratch&) = delete;
  MultibodyVariationalTreeScratch(MultibodyVariationalTreeScratch&&) noexcept;
  MultibodyVariationalTreeScratch& operator=(
      MultibodyVariationalTreeScratch&&) noexcept;

  void setAllocator(dart::common::MemoryAllocator& allocator);
  [[nodiscard]] const dart::common::MemoryAllocator& getAllocator()
      const noexcept;
  [[nodiscard]] std::size_t linkCount() const noexcept;
  [[nodiscard]] std::size_t dofCount() const noexcept;

private:
  struct Impl;
  struct ImplDeleter
  {
    dart::common::MemoryAllocator* allocator = nullptr;

    void operator()(Impl* impl) const noexcept;
  };

  dart::common::MemoryAllocator* m_allocator = nullptr;
  std::unique_ptr<Impl, ImplDeleter> m_impl;

  friend struct MultibodyVariationalTreeScratchAccess;
};

/// Reusable storage for variational loop-closure projection.
struct VariationalConstraintProjectionScratch
{
  using JacobianAllocator = dart::common::StlAllocator<Eigen::MatrixXd>;
  using JacobianVector = std::vector<Eigen::MatrixXd, JacobianAllocator>;
  using ProjectionBoundsAllocator
      = dart::common::StlAllocator<VariationalProjectionRowBounds>;
  using ProjectionBoundsVector
      = std::vector<VariationalProjectionRowBounds, ProjectionBoundsAllocator>;
  using RowIndexAllocator = dart::common::StlAllocator<Eigen::Index>;
  using RowIndexVector = std::vector<Eigen::Index, RowIndexAllocator>;

  VariationalConstraintProjectionScratch()
    : VariationalConstraintProjectionScratch(
          dart::common::MemoryAllocator::GetDefault())
  {
  }

  explicit VariationalConstraintProjectionScratch(
      dart::common::MemoryAllocator& allocator)
    : jacobians(JacobianAllocator{allocator}),
      projectionBounds(ProjectionBoundsAllocator{allocator}),
      hardRows(RowIndexAllocator{allocator}),
      boundedRows(RowIndexAllocator{allocator})
  {
  }

  JacobianVector jacobians;
  Eigen::MatrixXd pointJacobianA;
  Eigen::MatrixXd pointJacobianB;
  Eigen::MatrixXd angularJacobianA;
  Eigen::MatrixXd angularJacobianB;
  Eigen::MatrixXd pointJacobianWork;
  Eigen::VectorXd residual;
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd inverseMassJt;
  Eigen::MatrixXd constraintMass;
  Eigen::LDLT<Eigen::MatrixXd> constraintFactorization;
  Eigen::VectorXd lambdaRhs;
  Eigen::VectorXd lambda;
  Eigen::VectorXd correction;
  ProjectionBoundsVector projectionBounds;
  RowIndexVector hardRows;
  RowIndexVector boundedRows;
  Eigen::MatrixXd hardConstraintMass;
  Eigen::LDLT<Eigen::MatrixXd> hardConstraintFactorization;
  Eigen::VectorXd hardConstraintRhs;
  Eigen::VectorXd hardConstraintLambda;
  Eigen::VectorXd projectionLambda;
};

/// Reusable storage for manifold Anderson acceleration in the variational
/// multibody stage.
struct VariationalAndersonScratch
{
  using VectorAllocator = dart::common::StlAllocator<Eigen::VectorXd>;
  using VectorVector = std::vector<Eigen::VectorXd, VectorAllocator>;

  VariationalAndersonScratch()
    : VariationalAndersonScratch(dart::common::MemoryAllocator::GetDefault())
  {
  }

  explicit VariationalAndersonScratch(dart::common::MemoryAllocator& allocator)
    : stepDeltas(VectorAllocator{allocator}),
      iterateDeltas(VectorAllocator{allocator})
  {
  }

  VectorVector stepDeltas;
  VectorVector iterateDeltas;
  std::size_t historyCount = 0;
  Eigen::VectorXd previousStep;
  Eigen::VectorXd previousPosition;
  Eigen::VectorXd jointDelta;
  Eigen::VectorXd trialPosition;
  Eigen::VectorXd andersonPosition;
  Eigen::VectorXd andersonIncrement;
  Eigen::MatrixXd stepMatrix;
  Eigen::MatrixXd mixMatrix;
  Eigen::MatrixXd ftf;
  Eigen::MatrixXd regularized;
  Eigen::VectorXd normalRhs;
  Eigen::VectorXd gamma;
};

/// Cache-only scratch reused by the variational multibody stage.
///
/// The persisted `MultibodyVariationalState` remains the two-step dynamics
/// history; this component owns rebuildable contact/constraint buffers that
/// should be sized during bake and reused by the step loop.
struct MultibodyVariationalScratch
{
  DART_SIMULATION_CACHE_COMPONENT(MultibodyVariationalScratch);

  using PostContactTransformAllocator
      = dart::common::StlAllocator<Eigen::Isometry3d>;
  using PostContactTransformVector
      = std::vector<Eigen::Isometry3d, PostContactTransformAllocator>;
  using ConstraintAllocator
      = dart::common::StlAllocator<VariationalLoopConstraint>;
  using ConstraintVector
      = std::vector<VariationalLoopConstraint, ConstraintAllocator>;

  ConstraintVector constraints;
  VariationalGroundContact groundContact;
  std::optional<VariationalGroundContactSolver> groundContactSolver;
  MultibodyVariationalTreeScratch tree;
  VariationalContactEvaluationScratch contactEvaluation;
  VariationalStepScratch step;
  VariationalLinearSolveScratch linearSolve;
  MultibodyInverseDynamicsScratch inverseDynamics;
  VariationalConstraintProjectionScratch projection;
  VariationalAndersonScratch anderson;
  PostContactTransformVector postContactTransforms;
};

/// **EXPERIMENTAL (PLAN-082 Phase C -- link-vs-link).** A sphere-sphere contact
/// pair between two links of a multibody (self-contact): spheres of radius
/// `radiusA`/`radiusB` fixed at body-frame centers `centerA`/`centerB` on links
/// `linkA`/`linkB` (structure link indices).
struct VariationalSphereContactPair
{
  std::size_t linkA = 0;
  Eigen::Vector3d centerA = Eigen::Vector3d::Zero();
  double radiusA = 0.0;
  std::size_t linkB = 0;
  Eigen::Vector3d centerB = Eigen::Vector3d::Zero();
  double radiusB = 0.0;
};

/// **EXPERIMENTAL (PLAN-082 Phase C -- link-vs-link contact, first slice).**
/// Build an in-loop hook for compliant sphere-sphere contact between links of
/// the same multibody. Each RIQN iteration, at the trial configuration, every
/// overlapping pair contributes an equal-and-opposite penalty force
/// `k*penetration` (with Kelvin-Voigt damping `c` along the center line)
/// pushing the two link spheres apart, mapped to a generalized force via the
/// per-link point Jacobians. This is the simplest link-vs-link query (sphere
/// primitives); arbitrary link geometry reuses the rigid IPC contact stack (the
/// gate-1 adapter, see the contact roadmap). Throws on negative stiffness or
/// damping.
[[nodiscard]] DART_SIMULATION_API VariationalContactHook
makeVariationalLinkSphereContactHook(
    double stiffness,
    double dampingCoefficient,
    std::vector<VariationalSphereContactPair> pairs);

/// Advance one multibody by one step with the linear-time variational
/// integrator (Lee, Liu, Park, Srinivasa, WAFR 2016 / arXiv:1609.02898).
///
/// Each step solves the forced discrete Euler-Lagrange equation
/// `D2 Ld(q^{k-1}, q^k) + D1 Ld(q^k, q^{k+1}) + F^k = 0` for the next
/// configuration by a Newton root-find: the residual is evaluated in O(n) by a
/// discrete recursive Newton-Euler sweep (DRNEA), and the Newton update applies
/// the inverse Jacobian in O(n) via an articulated-body recursion, so the whole
/// step is linear-time in the degree-of-freedom count. Gravity enters as a
/// forcing-side spatial impulse (not a Lagrangian potential).
///
/// Scope (Phase A1): fixed-base open chains with fixed, revolute, and prismatic
/// joints; fixed time step. Other joint types and floating bases are rejected.
///
/// Updates the multibody's joint positions, velocities, and accelerations in
/// the registry, and advances `state`. Returns solve diagnostics. `tolerance`
/// is a per-coordinate accuracy; convergence tests the residual L2 norm against
/// `tolerance * sqrt(dofCount)` so the accuracy is uniform across chain
/// lengths. Throws InvalidOperationException if RIQN fails to converge within
/// `maxIterations` (non-convergence is a hard error, not a silent best-effort
/// step).
///
/// When `constraints` is non-empty, holonomic loop closures are enforced after
/// the unconstrained step by an impulse-based position projection onto the
/// constraint manifold `g(q) = 0` (the paper's Sec. 5 extension), reusing the
/// O(n) inverse-mass apply: `lambda = (J M^{-1} J^T)^{-1} (-g)`,
/// `dq = M^{-1} J^T lambda`. This keeps closed loops satisfied each step.
///
/// Preconditioner policy: when the generalized coordinates form a vector space
/// (every movable joint is revolute/prismatic) the update is the *exact*
/// recursive-Jacobian Newton step -- it solves `J(q^{k+1}) dq = residual` for
/// the exact forced-DEL Jacobian via a non-symmetric articulated-body recursion
/// in O(n) -- so the iteration count stays bounded and length-independent, and
/// long/stiff chains converge well within the iteration budget. Spherical and
/// floating coordinates live on a manifold the exact step does not yet handle,
/// so they fall back to the fixed `dt * M(q^k)^{-1}` quasi-Newton step (only an
/// approximate inverse Jacobian, manifold-correct), accelerated by
/// tangent-space Anderson mixing (see `andersonDepth`). The converged DEL root
/// is independent of the preconditioner, so this never changes the integrated
/// trajectory, only the iteration count.
///
/// `andersonDepth` is the depth-m history of the tangent-space Anderson
/// acceleration applied to the manifold (spherical/floating) quasi-Newton
/// fixed-point iteration: the per-iteration tangent updates and the tangent
/// log-differences between consecutive iterates are mixed (a least-squares
/// secant/Anderson(m) extrapolation, retracted with the per-joint manifold map)
/// before the manifold retraction. On a long, stiff spherical/floating chain
/// the plain `dt * M^{-1}` fixed point reaches a per-step residual plateau and
/// cannot drive the residual below a length-dependent floor; the acceleration
/// pushes through that plateau, so the manifold path converges to tolerances
/// (and on steps) the plain step cannot reach, at a comparable iteration count.
/// `0` disables it (the plain quasi-Newton fixed point), exposed so callers and
/// tests can compare the plain and accelerated paths. It has no effect on the
/// Euclidean exact-Newton path.
///
/// **EXPERIMENTAL SPIKE (contact-roadmap gate 2).** `contactHook`, when set,
/// supplies a per-DOF generalized contact force `Q_c` re-evaluated at the
/// *trial* configuration on every RIQN iteration (including each line-search
/// trial) and folded into the forced-DEL residual exactly like the applied
/// force (`residual -= dt * Q_c`), so a bounded (compliant/penalty) contact
/// potential's curvature couples into the root-find. The default empty hook is
/// zero overhead and leaves the no-contact numerics byte-for-byte identical;
/// this parameter is probe scaffolding for the compliant-contact go/no-go
/// spike, not a production contact surface.
DART_SIMULATION_API VariationalSolveReport integrateMultibodyVariational(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    MultibodyVariationalState& state,
    int maxIterations = 100,
    double tolerance = 1e-10,
    const std::vector<VariationalLoopConstraint>& constraints = {},
    std::size_t andersonDepth = 5,
    const VariationalContactHook& contactHook = {});

/// Total mechanical energy (kinetic + gravitational potential) of one multibody
/// at its current configuration and velocity.
///
/// Used to verify the variational integrator's near-energy-conserving behavior
/// on passive systems. Kinetic energy is `0.5 * sum_i V_i^T G_i V_i` over link
/// spatial velocities; potential energy is `- sum_i m_i * g . r_i^com` (zero
/// reference at the world origin). Fixed-base open chains with fixed/revolute/
/// prismatic joints.
[[nodiscard]] DART_SIMULATION_API double computeMultibodyMechanicalEnergy(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity);

/// Scratch-aware overload for repeated diagnostics on the same multibody shape.
/// The caller retains tree and step scratch capacity across calls; the
/// no-scratch overload above remains source-compatible and uses local default
/// scratch objects.
[[nodiscard]] DART_SIMULATION_API double computeMultibodyMechanicalEnergy(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    MultibodyVariationalTreeScratch& treeScratch,
    VariationalStepScratch& stepScratch);

/// O(n) product `M(q)^{-1} * impulse` for one multibody at its current
/// configuration, via the articulated-body algorithm (zero velocity/gravity).
/// This is the linear-time inverse-mass apply that powers the variational
/// integrator's RIQN step; exposed for testing against the dense mass-matrix
/// solve. Throws InvalidArgumentException if `impulse` size does not equal the
/// movable degree-of-freedom count.
[[nodiscard]] DART_SIMULATION_API Eigen::VectorXd
computeMultibodyInverseMassProduct(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::VectorXd& impulse);

/// Scratch-aware inverse-mass product for repeated calls on the same
/// multibody shape. The caller retains tree, linear-solve, and output vector
/// capacity across calls; the no-scratch overload remains source-compatible.
DART_SIMULATION_API void computeMultibodyInverseMassProductInto(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::VectorXd& impulse,
    MultibodyVariationalTreeScratch& treeScratch,
    VariationalLinearSolveScratch& linearSolveScratch,
    Eigen::VectorXd& result);

/// Holonomic loop-closure residual `g(q)` and its Jacobian `J = dg/dq`
/// evaluated at the multibody's current configuration, for the given closures.
/// The Jacobian columns are in the integrator's generalized-coordinate order
/// (the same ordering as `computeMultibodyInverseMassProduct`). Exposed for
/// finite-difference verification of the constraint Jacobian used by the
/// constrained variational step (the impulse-based loop-closure projection).
struct VariationalConstraintLinearization
{
  Eigen::VectorXd residual; ///< `g(q)`, stacked per constraint.
  Eigen::MatrixXd jacobian; ///< `dg/dq` (rows = constraint dim, cols = DOF).
};

[[nodiscard]] DART_SIMULATION_API VariationalConstraintLinearization
computeVariationalConstraintLinearization(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const std::vector<VariationalLoopConstraint>& constraints);

/// Scratch-aware loop-closure linearization for repeated diagnostics on the
/// same multibody shape. The caller retains tree, projection-scratch, and
/// result payload capacity across calls; the no-scratch overload remains
/// source-compatible.
DART_SIMULATION_API void computeVariationalConstraintLinearizationInto(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const std::vector<VariationalLoopConstraint>& constraints,
    MultibodyVariationalTreeScratch& treeScratch,
    VariationalConstraintProjectionScratch& projectionScratch,
    VariationalConstraintLinearization& result);

/// Classification of one public loop closure (`comps::LoopClosure`) for the
/// variational loop-closure solver. Point closures whose endpoints are links of
/// a single multibody and/or a fixed world anchor are `Supported`; everything
/// the variational stage cannot honestly enforce yet (the Rigid and Distance
/// families, rigid-body endpoints, or a closure spanning two multibodies) is
/// `Unsupported` with a human-readable `reason`.
struct VariationalLoopClosureBinding
{
  enum class Status
  {
    Ignored,     ///< Disabled, or not requesting dynamic solving.
    Supported,   ///< Translatable; `structure` and `constraint` are set.
    Unsupported, ///< Requests solving but this stage cannot enforce it.
  };

  Status status = Status::Ignored;
  entt::entity structure = entt::null; ///< Owning multibody (when Supported).
  VariationalLoopConstraint
      constraint;          ///< Translated closure (when Supported).
  std::string_view reason; ///< Why rejected (when Unsupported).
};

/// Classify a loop closure (the `comps::LoopClosure` on `closureEntity`) for
/// the variational loop-closure solver. The World's dynamics-policy validation
/// uses it to accept/reject a `Solve` closure under the variational method, and
/// the variational stage uses it to gather the constraints to enforce, so the
/// two can never disagree. Returns `Ignored` when there is no such component,
/// the closure is disabled, or it does not request `Solve`.
[[nodiscard]] DART_SIMULATION_API VariationalLoopClosureBinding
bindVariationalLoopClosure(
    const detail::WorldRegistry& registry, entt::entity closureEntity);

/// Reserve variational-stage registry storage and per-multibody scratch for the
/// current baked multibody/contact shape.
DART_SIMULATION_API void reserveMultibodyVariationalRegistryStorage(
    detail::WorldRegistry& registry,
    std::size_t multibodyCount,
    dart::common::MemoryAllocator& allocator
    = dart::common::MemoryAllocator::GetDefault());

/// Variational-integrator multibody stage (a peer of
/// `MultibodyForwardDynamicsStage`, selected by the `variational integrator`
/// integration-family method name).
///
/// For each multibody, advances the dynamics with
/// `integrateMultibodyVariational`, persisting the two-step discrete-mechanics
/// history as a `MultibodyVariationalState` component on the multibody entity.
///
/// Scope and assumptions match `integrateMultibodyVariational` (Phase A1:
/// fixed-base open chains; fixed/revolute/prismatic joints; no contact).
class DART_SIMULATION_API MultibodyVariationalIntegrationStage final
  : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  /// Pre-reserve per-multibody variational state storage before baked steps.
  void prepare(World& world);
  void execute(World& world, ComputeExecutor& executor) override;

private:
  std::size_t m_maxIterations{100};
  double m_tolerance{1e-10};
};

} // namespace dart::simulation::compute
