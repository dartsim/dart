/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#include "dart/simulation/experimental/compute/variational_integration.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/dynamics.hpp"
#include "dart/simulation/experimental/comps/frame_types.hpp"
#include "dart/simulation/experimental/comps/joint.hpp"
#include "dart/simulation/experimental/comps/link.hpp"
#include "dart/simulation/experimental/comps/multibody.hpp"
#include "dart/simulation/experimental/compute/multibody_dynamics.hpp"
#include "dart/simulation/experimental/detail/variational/discrete_mechanics_math.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <unordered_map>
#include <vector>

namespace dart::simulation::experimental::compute {

namespace {

namespace dm = detail::variational;
using dm::Matrix6;
using dm::Vector6;
using Subspace = Eigen::Matrix<double, 6, Eigen::Dynamic>;

// NOTE: skew/adjoint/spatialInertia and the joint relative-transform/subspace
// helpers below duplicate the Phase-A1 subset of
// compute/multibody_dynamics.cpp. They are kept local to avoid refactoring that
// working, tested translation unit mid-implementation; a follow-up should hoist
// the shared spatial-algebra and kinematic-tree machinery into an internal
// header used by both stages.

Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

// SO(3) exponential / logarithm on rotation vectors (axis * angle).
Eigen::Matrix3d rotationExp3(const Eigen::Vector3d& v)
{
  const double angle = v.norm();
  if (angle < 1e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, v / angle).toRotationMatrix();
}

Eigen::Vector3d rotationLog3(const Eigen::Matrix3d& rotation)
{
  const Eigen::AngleAxisd angleAxis(rotation);
  return angleAxis.angle() * angleAxis.axis();
}

// Spatial motion adjoint Ad_T for the [angular; linear] convention.
Matrix6 adjoint(const Eigen::Isometry3d& transform)
{
  const Eigen::Matrix3d rotation = transform.linear();
  const Eigen::Vector3d translation = transform.translation();
  Matrix6 result = Matrix6::Zero();
  result.topLeftCorner<3, 3>() = rotation;
  result.bottomLeftCorner<3, 3>() = skew(translation) * rotation;
  result.bottomRightCorner<3, 3>() = rotation;
  return result;
}

// Spatial inertia about the link frame origin, [angular; linear] convention.
Matrix6 spatialInertia(const comps::MassProperties& mass)
{
  const Eigen::Matrix3d comCross = skew(mass.localCenterOfMass);
  Matrix6 result = Matrix6::Zero();
  result.topLeftCorner<3, 3>() = mass.inertia - mass.mass * comCross * comCross;
  result.topRightCorner<3, 3>() = mass.mass * comCross;
  result.bottomLeftCorner<3, 3>() = -mass.mass * comCross;
  result.bottomRightCorner<3, 3>() = mass.mass * Eigen::Matrix3d::Identity();
  return result;
}

// Relative transform produced by a joint at the given generalized position
// (Phase A1: fixed/revolute/prismatic), before the post-joint link offset.
Eigen::Isometry3d jointMotionTransform(
    const comps::Joint& joint, const Eigen::VectorXd& position)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  switch (joint.type) {
    case comps::JointType::Fixed:
      return transform;
    case comps::JointType::Revolute:
      transform.linear()
          = Eigen::AngleAxisd(position[0], joint.axis).toRotationMatrix();
      return transform;
    case comps::JointType::Prismatic:
      transform.translation() = joint.axis * position[0];
      return transform;
    case comps::JointType::Spherical:
      // Orientation stored as a rotation vector (exponential coordinates).
      transform.linear() = rotationExp3(position.head<3>());
      return transform;
    case comps::JointType::Floating:
      // 6-DOF pose: translation (position[0..2]) then orientation as a rotation
      // vector (position[3..5]), matching the kinematics convention.
      transform.linear() = rotationExp3(position.tail<3>());
      transform.translation() = position.head<3>();
      return transform;
    default:
      DART_EXPERIMENTAL_THROW_T(
          InvalidOperationException,
          "The variational integrator supports fixed, revolute, prismatic, "
          "spherical, and floating joints");
  }
}

// Joint motion subspace in the joint frame (Phase A1), [angular; linear].
Subspace jointSubspaceInJointFrame(const comps::Joint& joint)
{
  switch (joint.type) {
    case comps::JointType::Fixed:
      return Subspace(6, 0);
    case comps::JointType::Revolute: {
      Subspace subspace(6, 1);
      subspace.col(0).head<3>() = joint.axis;
      subspace.col(0).tail<3>().setZero();
      return subspace;
    }
    case comps::JointType::Prismatic: {
      Subspace subspace(6, 1);
      subspace.col(0).head<3>().setZero();
      subspace.col(0).tail<3>() = joint.axis;
      return subspace;
    }
    case comps::JointType::Spherical: {
      // Generalized velocity is the body angular velocity (matching the
      // rotation-vector position): angular = I, linear = 0.
      Subspace subspace = Subspace::Zero(6, 3);
      subspace.topRows<3>() = Eigen::Matrix3d::Identity();
      return subspace;
    }
    case comps::JointType::Floating: {
      // Generalized velocity is [linear; angular] body twist (matching the
      // position layout [translation; rotation vector]); the subspace permutes
      // it into the [angular; linear] spatial convention.
      Subspace subspace = Subspace::Zero(6, 6);
      subspace.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
      subspace.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity();
      return subspace;
    }
    default:
      DART_EXPERIMENTAL_THROW_T(
          InvalidOperationException,
          "The variational integrator supports fixed, revolute, prismatic, "
          "spherical, and floating joints");
  }
}

// Retract a joint configuration `q` along a generalized-coordinate increment
// `delta` (the same space as generalized velocity). Euclidean joints add
// directly; spherical/floating joints apply the increment on the SO(3)/SE(3)
// manifold, matching the semi-implicit integration convention
// (R_new = R exp(omega), p_new = p + R v).
Eigen::VectorXd jointRetract(
    const comps::Joint& joint,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& delta)
{
  switch (joint.type) {
    case comps::JointType::Spherical:
      return rotationLog3(
          rotationExp3(q.head<3>()) * rotationExp3(delta.head<3>()));
    case comps::JointType::Floating: {
      const Eigen::Matrix3d rotation = rotationExp3(q.tail<3>());
      Eigen::VectorXd result(6);
      result.head<3>() = q.head<3>() + rotation * delta.head<3>();
      result.tail<3>() = rotationLog3(rotation * rotationExp3(delta.tail<3>()));
      return result;
    }
    default:
      return q + delta;
  }
}

// The generalized-coordinate tangent `tau` with `jointRetract(q, tau) == qNext`
// (so the generalized velocity is `tau / dt`). Inverse of jointRetract.
Eigen::VectorXd jointLogDifference(
    const comps::Joint& joint,
    const Eigen::VectorXd& qNext,
    const Eigen::VectorXd& q)
{
  switch (joint.type) {
    case comps::JointType::Spherical:
      return rotationLog3(
          rotationExp3(q.head<3>()).transpose()
          * rotationExp3(qNext.head<3>()));
    case comps::JointType::Floating: {
      const Eigen::Matrix3d rotation = rotationExp3(q.tail<3>());
      Eigen::VectorXd result(6);
      result.head<3>() = rotation.transpose() * (qNext.head<3>() - q.head<3>());
      result.tail<3>()
          = rotationLog3(rotation.transpose() * rotationExp3(qNext.tail<3>()));
      return result;
    }
    default:
      return qNext - q;
  }
}

// Per-link data for the discrete recursion, in construction order
// (parent-before-child).
struct VarLink
{
  int parent = -1;
  entt::entity joint = entt::null;
  std::size_t dof = 0;
  std::size_t dofOffset = 0;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity(); // link from joint
  Matrix6 inertia = Matrix6::Zero();                        // G_i (link frame)
  Subspace subspace{6, 0};                                  // S_i (link frame)
  Eigen::Isometry3d currentRelative
      = Eigen::Isometry3d::Identity(); // T_{lambda(i),i} at q^k
  Eigen::Isometry3d worldTransform = Eigen::Isometry3d::Identity();
  std::vector<int> children;
  // Per-residual-evaluation scratch.
  Eigen::Isometry3d deltaTransform = Eigen::Isometry3d::Identity();
  Vector6 averageVelocity = Vector6::Zero();
  Vector6 momentum = Vector6::Zero();
};

struct VarTree
{
  std::vector<VarLink> links;
  std::size_t dofCount = 0;
};

VarTree buildVarTree(
    const entt::registry& registry, const comps::MultibodyStructure& structure)
{
  const auto& linkEntities = structure.links;
  VarTree tree;
  tree.links.assign(linkEntities.size(), VarLink{});

  // O(1) link-index lookup so building the tree stays O(n); a linear scan per
  // link would make the whole step O(n^2) and defeat the linear-time solve.
  std::unordered_map<entt::entity, std::size_t> indexOf;
  indexOf.reserve(linkEntities.size());
  for (std::size_t i = 0; i < linkEntities.size(); ++i) {
    indexOf.emplace(linkEntities[i], i);
  }

  for (std::size_t i = 0; i < linkEntities.size(); ++i) {
    const auto linkEntity = linkEntities[i];
    const auto& linkComp = registry.get<comps::Link>(linkEntity);
    auto& link = tree.links[i];
    link.inertia = spatialInertia(linkComp.mass);
    link.offset = linkComp.transformFromParentJoint;

    if (linkComp.parentJoint == entt::null) {
      const auto& cache = registry.get<comps::FrameCache>(linkEntity);
      link.worldTransform = cache.worldTransform;
      link.parent = -1;
      continue;
    }

    const auto& joint = registry.get<comps::Joint>(linkComp.parentJoint);
    link.joint = linkComp.parentJoint;
    const auto parentIt = indexOf.find(joint.parentLink);
    DART_EXPERIMENTAL_THROW_T_IF(
        parentIt == indexOf.end(),
        InvalidOperationException,
        "Multibody link parent is not part of the same multibody");
    link.parent = static_cast<int>(parentIt->second);

    const Subspace jointFrameSubspace = jointSubspaceInJointFrame(joint);
    link.dof = static_cast<std::size_t>(jointFrameSubspace.cols());
    link.dofOffset = tree.dofCount;
    tree.dofCount += link.dof;
    link.subspace = adjoint(link.offset.inverse()) * jointFrameSubspace;
    link.currentRelative
        = jointMotionTransform(joint, joint.position) * link.offset;
    link.worldTransform
        = tree.links[static_cast<std::size_t>(link.parent)].worldTransform
          * link.currentRelative;
    tree.links[static_cast<std::size_t>(link.parent)].children.push_back(
        static_cast<int>(i));
  }
  return tree;
}

// Gather the generalized velocity and the applied (forcing-side) generalized
// effort: commanded effort (Force actuator, clamped to limits) minus passive
// spring and damping forces. Coriolis/gravity are handled by the integrator.
void gatherState(
    const entt::registry& registry,
    const VarTree& tree,
    Eigen::VectorXd& position,
    Eigen::VectorXd& velocity,
    Eigen::VectorXd& appliedForce)
{
  const auto dof = static_cast<Eigen::Index>(tree.dofCount);
  position = Eigen::VectorXd::Zero(dof);
  velocity = Eigen::VectorXd::Zero(dof);
  appliedForce = Eigen::VectorXd::Zero(dof);
  for (const auto& link : tree.links) {
    if (link.dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(link.joint);
    const auto seg = static_cast<Eigen::Index>(link.dofOffset);
    const auto n = static_cast<Eigen::Index>(link.dof);
    position.segment(seg, n) = joint.position;
    velocity.segment(seg, n) = joint.velocity;

    Eigen::VectorXd effort = Eigen::VectorXd::Zero(n);
    if (joint.actuatorType == comps::ActuatorType::Force
        && joint.torque.size() == n) {
      effort = joint.torque;
      if (joint.limits.effortLower.size() == n
          && joint.limits.effortUpper.size() == n) {
        effort = effort.cwiseMax(joint.limits.effortLower)
                     .cwiseMin(joint.limits.effortUpper);
      }
    }
    if (joint.springStiffness.size() == n && joint.restPosition.size() == n) {
      effort -= joint.springStiffness.cwiseProduct(
          joint.position - joint.restPosition);
    }
    if (joint.dampingCoefficient.size() == n) {
      effort -= joint.dampingCoefficient.cwiseProduct(joint.velocity);
    }
    appliedForce.segment(seg, n) = effort;
  }
}

// Forward velocity recursion at the current configuration: body spatial
// velocity V_i = Ad(T_i^{-1}) V_parent + S_i qdot_i.
std::vector<Vector6> currentSpatialVelocities(
    const VarTree& tree, const Eigen::VectorXd& velocity)
{
  std::vector<Vector6> v(tree.links.size(), Vector6::Zero());
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto& link = tree.links[i];
    if (link.parent < 0) {
      v[i].setZero();
    } else {
      v[i] = adjoint(link.currentRelative.inverse())
             * v[static_cast<std::size_t>(link.parent)];
    }
    if (link.dof > 0) {
      v[i] += link.subspace
              * velocity.segment(
                  static_cast<Eigen::Index>(link.dofOffset),
                  static_cast<Eigen::Index>(link.dof));
    }
  }
  return v;
}

// Evaluate the forced discrete Euler-Lagrange residual f(qNext) in O(n) via a
// forward (average-velocity) sweep and a backward (momentum/impulse) sweep.
Eigen::VectorXd computeResidual(
    const entt::registry& registry,
    VarTree& tree,
    const Eigen::VectorXd& nextPosition,
    const MultibodyVariationalState& state,
    const Eigen::Vector3d& gravity,
    double timeStep,
    const Eigen::VectorXd& appliedForce)
{
  // Forward sweep: relative displacement dT_i and average velocity V_i.
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    auto& link = tree.links[i];
    if (link.parent < 0) {
      link.deltaTransform = Eigen::Isometry3d::Identity();
      link.averageVelocity.setZero();
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(link.joint);
    const Eigen::VectorXd seg = nextPosition.segment(
        static_cast<Eigen::Index>(link.dofOffset),
        static_cast<Eigen::Index>(link.dof));
    const Eigen::Isometry3d nextRelative
        = jointMotionTransform(joint, seg) * link.offset;
    link.deltaTransform
        = link.currentRelative.inverse()
          * tree.links[static_cast<std::size_t>(link.parent)].deltaTransform
          * nextRelative;
    link.averageVelocity = dm::se3Log(link.deltaTransform) / timeStep;
  }

  // Backward sweep: discrete momentum, transmitted impulse, joint residual.
  Eigen::VectorXd residual
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount));
  for (std::size_t reverse = 0; reverse < tree.links.size(); ++reverse) {
    const auto i = tree.links.size() - 1 - reverse;
    auto& link = tree.links[i];
    const Matrix6& g = link.inertia;
    link.momentum = dm::dexpInvTranspose(
        timeStep * link.averageVelocity, g * link.averageVelocity);

    Vector6 force = link.momentum;
    force
        -= dm::dAdT(state.previousDeltaTransform[i], state.previousMomentum[i]);
    // Gravity as a forcing-side spatial impulse (not a Lagrangian potential).
    force -= (g * dm::adInvRLinear(link.worldTransform, gravity)) * timeStep;
    for (const int child : link.children) {
      force += dm::dAdInvT(
          tree.links[static_cast<std::size_t>(child)].currentRelative,
          tree.links[static_cast<std::size_t>(child)].momentum);
    }
    link.momentum = force; // store transmitted impulse for child accumulation

    if (link.dof > 0) {
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto n = static_cast<Eigen::Index>(link.dof);
      residual.segment(seg, n) = link.subspace.transpose() * force
                                 - timeStep * appliedForce.segment(seg, n);
    }
  }
  return residual;
}

// O(n) articulated-body-inertia solve of M(q)^{-1} * b for the precomputed tree
// (Featherstone ABA with zero velocity and gravity): a backward articulated-
// inertia sweep followed by a forward acceleration sweep. This is the
// linear-time RIQN quasi-Newton step (replacing a dense factorization) and
// equals massMatrix.ldlt().solve(b) to machine precision (cross-checked by
// test). Frames follow the [angular; linear] convention: parentToChild maps a
// spatial motion parent->child, its transpose maps a spatial force
// child->parent.
Eigen::VectorXd applyArticulatedInverseMass(
    const VarTree& tree, const Eigen::VectorXd& b)
{
  const std::size_t n = tree.links.size();
  std::vector<Matrix6> articulatedInertia(n);
  std::vector<Vector6> biasForce(n, Vector6::Zero());
  std::vector<Eigen::MatrixXd> u(n); // U_i = I^A_i S_i        (6 x dof)
  std::vector<Eigen::MatrixXd> dInverse(
      n);                                 // (S_i^T U_i)^{-1}       (dof x dof)
  std::vector<Eigen::VectorXd> uForce(n); // u_i = b_i - S_i^T p^A_i (dof)
  std::vector<Matrix6> parentToChild(n, Matrix6::Identity());
  for (std::size_t i = 0; i < n; ++i) {
    articulatedInertia[i] = tree.links[i].inertia;
  }

  // Backward sweep: accumulate articulated inertia and bias toward the root.
  for (std::size_t reverse = 0; reverse < n; ++reverse) {
    const std::size_t i = n - 1 - reverse;
    const VarLink& link = tree.links[i];
    if (link.dof > 0) {
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto dof = static_cast<Eigen::Index>(link.dof);
      u[i] = articulatedInertia[i] * link.subspace;
      const Eigen::MatrixXd d = link.subspace.transpose() * u[i];
      dInverse[i] = d.inverse();
      uForce[i]
          = b.segment(seg, dof) - link.subspace.transpose() * biasForce[i];
    }
    if (link.parent >= 0) {
      Matrix6 ia = articulatedInertia[i];
      Vector6 pa = biasForce[i];
      if (link.dof > 0) {
        ia.noalias() -= u[i] * dInverse[i] * u[i].transpose();
        pa.noalias() += u[i] * (dInverse[i] * uForce[i]);
      }
      parentToChild[i] = adjoint(link.currentRelative.inverse());
      const Matrix6 forceToParent = parentToChild[i].transpose();
      const auto p = static_cast<std::size_t>(link.parent);
      articulatedInertia[p].noalias() += forceToParent * ia * parentToChild[i];
      biasForce[p].noalias() += forceToParent * pa;
    }
  }

  // Forward sweep: propagate spatial accelerations and read joint
  // accelerations.
  Eigen::VectorXd qddot
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount));
  std::vector<Vector6> acceleration(n, Vector6::Zero());
  for (std::size_t i = 0; i < n; ++i) {
    const VarLink& link = tree.links[i];
    if (link.parent < 0) {
      acceleration[i].setZero();
      continue;
    }
    const Vector6 parentAccel
        = parentToChild[i]
          * acceleration[static_cast<std::size_t>(link.parent)];
    if (link.dof > 0) {
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto dof = static_cast<Eigen::Index>(link.dof);
      const Eigen::VectorXd jointAccel
          = dInverse[i] * (uForce[i] - u[i].transpose() * parentAccel);
      qddot.segment(seg, dof) = jointAccel;
      acceleration[i] = parentAccel + link.subspace * jointAccel;
    } else {
      acceleration[i] = parentAccel;
    }
  }
  return qddot;
}

} // namespace

//==============================================================================
VariationalSolveReport integrateMultibodyVariational(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    MultibodyVariationalState& state,
    int maxIterations,
    double tolerance)
{
  VariationalSolveReport report;
  if (structure.links.empty()) {
    return report;
  }

  VarTree tree = buildVarTree(registry, structure);
  if (tree.dofCount == 0) {
    return report;
  }

  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd appliedForce;
  gatherState(registry, tree, position, velocity, appliedForce);

  // Bootstrap the two-step history from the current generalized velocity: seed
  // dT_prev = exp(dt * V_i) and mu_prev = dexp^{-T}(dt V_i, G V_i) so the first
  // step's momentum-transport term is consistent.
  if (!state.bootstrapped
      || state.previousDeltaTransform.size() != tree.links.size()) {
    const std::vector<Vector6> v0 = currentSpatialVelocities(tree, velocity);
    state.previousDeltaTransform.assign(
        tree.links.size(), Eigen::Isometry3d::Identity());
    state.previousMomentum.assign(tree.links.size(), Vector6::Zero());
    for (std::size_t i = 0; i < tree.links.size(); ++i) {
      const Vector6 scaled = timeStep * v0[i];
      state.previousDeltaTransform[i] = dm::se3Exp(scaled);
      state.previousMomentum[i]
          = dm::dexpInvTranspose(scaled, tree.links[i].inertia * v0[i]);
    }
    state.bootstrapped = true;
  }

  // Initial guess IG3 (semi-implicit Euler from forward dynamics): far closer
  // to the DEL root than explicit Euler, which keeps the RIQN iteration count
  // low and roughly uniform across chain length. The forward-dynamics
  // acceleration ddq = M(q)^{-1}(appliedForce - (C(q,qdot) qdot + g(q))) reuses
  // the O(n) inverse-mass apply, so the initial guess stays linear-time. A1
  // joints are Euclidean, so the guess uses plain vector arithmetic.
  const Eigen::VectorXd bias = computeMultibodyInverseDynamics(
      registry,
      structure,
      gravity,
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount)));
  const Eigen::VectorXd guessAcceleration
      = applyArticulatedInverseMass(tree, appliedForce - bias);
  Eigen::VectorXd nextPosition = position;
  for (const auto& link : tree.links) {
    if (link.dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(link.joint);
    const auto seg = static_cast<Eigen::Index>(link.dofOffset);
    const auto n = static_cast<Eigen::Index>(link.dof);
    const Eigen::VectorXd tangent
        = timeStep * velocity.segment(seg, n)
          + timeStep * timeStep * guessAcceleration.segment(seg, n);
    nextPosition.segment(seg, n)
        = jointRetract(joint, position.segment(seg, n), tangent);
  }

  for (int iteration = 0; iteration < maxIterations; ++iteration) {
    const Eigen::VectorXd residual = computeResidual(
        registry, tree, nextPosition, state, gravity, timeStep, appliedForce);
    report.iterations = static_cast<std::size_t>(iteration) + 1;
    report.residualNorm = residual.norm();
    if (report.residualNorm <= tolerance) {
      report.converged = true;
      break;
    }
    // RIQN quasi-Newton step dt * M(q^k)^{-1} * e via the O(n) articulated-body
    // inverse-mass apply (the linear-time root update; no dense factorization).
    // The increment is retracted per joint so spherical/floating coordinates
    // stay on their SO(3)/SE(3) manifolds.
    const Eigen::VectorXd delta
        = applyArticulatedInverseMass(tree, timeStep * residual);
    for (const auto& link : tree.links) {
      if (link.dof == 0) {
        continue;
      }
      const auto& joint = registry.get<comps::Joint>(link.joint);
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto n = static_cast<Eigen::Index>(link.dof);
      nextPosition.segment(seg, n) = jointRetract(
          joint, nextPosition.segment(seg, n), -delta.segment(seg, n));
    }
  }

  // Refresh the per-link scratch at the accepted configuration so the history
  // shift uses dT and momentum consistent with nextPosition.
  const Eigen::VectorXd finalResidual = computeResidual(
      registry, tree, nextPosition, state, gravity, timeStep, appliedForce);
  report.residualNorm = finalResidual.norm();

  // Write back joint position/velocity/acceleration (Euclidean, Phase A1).
  for (auto& link : tree.links) {
    if (link.dof == 0) {
      continue;
    }
    auto& joint = registry.get<comps::Joint>(link.joint);
    const auto seg = static_cast<Eigen::Index>(link.dofOffset);
    const auto n = static_cast<Eigen::Index>(link.dof);
    const Eigen::VectorXd previousVelocity = joint.velocity;
    const Eigen::VectorXd newPosition = nextPosition.segment(seg, n);
    joint.velocity
        = jointLogDifference(joint, newPosition, joint.position) / timeStep;
    joint.position = newPosition;
    if (previousVelocity.size() == n) {
      joint.acceleration = (joint.velocity - previousVelocity) / timeStep;
    }
  }

  // Shift the two-step history: dT_prev <- dT, mu_prev <- discrete momentum.
  // Note: computeResidual stored the transmitted impulse in `momentum`; the
  // discrete momentum we must carry forward is dexp^{-T}(dt V, G V), recomputed
  // here from the accepted average velocity.
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto& link = tree.links[i];
    state.previousDeltaTransform[i] = link.deltaTransform;
    state.previousMomentum[i] = dm::dexpInvTranspose(
        timeStep * link.averageVelocity, link.inertia * link.averageVelocity);
  }

  return report;
}

//==============================================================================
double computeMultibodyMechanicalEnergy(
    const entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity)
{
  if (structure.links.empty()) {
    return 0.0;
  }
  const VarTree tree = buildVarTree(registry, structure);

  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd appliedForce;
  gatherState(registry, tree, position, velocity, appliedForce);
  const std::vector<Vector6> v = currentSpatialVelocities(tree, velocity);

  double kinetic = 0.0;
  double potential = 0.0;
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto& link = tree.links[i];
    kinetic += 0.5 * v[i].dot(link.inertia * v[i]);
    const auto& linkComp = registry.get<comps::Link>(structure.links[i]);
    const Eigen::Vector3d comWorld
        = link.worldTransform * linkComp.mass.localCenterOfMass;
    potential -= linkComp.mass.mass * gravity.dot(comWorld);
  }
  return kinetic + potential;
}

//==============================================================================
Eigen::VectorXd computeMultibodyInverseMassProduct(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::VectorXd& impulse)
{
  if (structure.links.empty()) {
    return {};
  }
  const VarTree tree = buildVarTree(registry, structure);
  if (tree.dofCount == 0) {
    return {};
  }
  DART_EXPERIMENTAL_THROW_T_IF(
      impulse.size() != static_cast<Eigen::Index>(tree.dofCount),
      InvalidArgumentException,
      "Impulse dimension must match the multibody movable DOF count");
  return applyArticulatedInverseMass(tree, impulse);
}

//==============================================================================
std::string_view MultibodyVariationalIntegrationStage::getName() const noexcept
{
  return "multibody_variational_integration";
}

//==============================================================================
ComputeStageMetadata MultibodyVariationalIntegrationStage::getMetadata()
    const noexcept
{
  return {
      ComputeStageDomain::ArticulatedBody,
      toMask(ComputeStageAcceleration::TaskParallel)};
}

//==============================================================================
void MultibodyVariationalIntegrationStage::execute(
    World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = world.getRegistry();
  const Eigen::Vector3d gravity = world.getGravity();
  const double timeStep = world.getTimeStep();

  auto view = registry.view<comps::MultibodyStructure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    auto& state = registry.get_or_emplace<MultibodyVariationalState>(entity);
    integrateMultibodyVariational(
        registry, structure, gravity, timeStep, state);
  }
}

} // namespace dart::simulation::experimental::compute
