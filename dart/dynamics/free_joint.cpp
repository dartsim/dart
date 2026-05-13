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

#include "dart/dynamics/free_joint.hpp"

#include "dart/common/macros.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/degree_of_freedom.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/math/geometry.hpp"
#include "dart/math/helpers.hpp"

#include <algorithm>
#include <array>
#include <string>
#include <utility>

#include <cmath>

namespace dart {
namespace dynamics {
namespace {

constexpr double kTorqueFreeTolerance = 1e-12;
constexpr int kTorqueFreeMaxIterations = 32;

template <class Derived>
bool isExactlyZero(const Eigen::MatrixBase<Derived>& vector)
{
  return vector.size() == 0 || vector.cwiseAbs().maxCoeff() == 0.0;
}

bool isExactlyZero(double value)
{
  return value == 0.0;
}

bool isExactlyIdentity(const Eigen::Isometry3d& transform)
{
  return transform.translation().isZero(0.0)
         && transform.linear().isIdentity(0.0);
}

bool isTorqueFreeMidpointResidualConverged(
    const Eigen::Matrix3d& inertia,
    const Eigen::Vector3d& omega,
    const Eigen::Vector3d& midpoint,
    double dt,
    double residualNorm)
{
  constexpr double absoluteTolerance = 1e-13;
  constexpr double relativeTolerance = 1e-12;

  const Eigen::Vector3d angularMomentum = inertia * midpoint;
  const double residualScale
      = ((2.0 / dt) * inertia * (midpoint - omega)).norm()
        + midpoint.cross(angularMomentum).norm();
  const double tolerance
      = absoluteTolerance + relativeTolerance * std::max(1.0, residualScale);

  if (!std::isfinite(residualNorm) || !std::isfinite(tolerance)) {
    return false;
  }

  return residualNorm <= tolerance;
}

bool integrateTorqueFreeAngularVelocity(
    const Eigen::Matrix3d& inertia,
    const Eigen::Vector3d& omega,
    double dt,
    Eigen::Vector3d& nextOmega)
{
  Eigen::LDLT<Eigen::Matrix3d> inertiaSolver(inertia);
  if (inertiaSolver.info() != Eigen::Success || !inertiaSolver.isPositive()) {
    return false;
  }

  Eigen::Vector3d midpoint = omega;
  bool converged = false;

  for (int i = 0; i < kTorqueFreeMaxIterations; ++i) {
    const Eigen::Vector3d angularMomentum = inertia * midpoint;
    const Eigen::Vector3d residual = (2.0 / dt) * inertia * (midpoint - omega)
                                     + midpoint.cross(angularMomentum);
    const double residualNorm = residual.norm();

    if (isTorqueFreeMidpointResidualConverged(
            inertia, omega, midpoint, dt, residualNorm)) {
      converged = true;
      break;
    }

    const Eigen::Matrix3d jacobian
        = (2.0 / dt) * inertia - math::makeSkewSymmetric(angularMomentum)
          + math::makeSkewSymmetric(midpoint) * inertia;
    const Eigen::Vector3d step = jacobian.ldlt().solve(residual);

    if (!step.allFinite()) {
      return false;
    }

    midpoint -= step;

    if (step.norm() < 1e-13) {
      const Eigen::Vector3d nextAngularMomentum = inertia * midpoint;
      const Eigen::Vector3d nextResidual
          = (2.0 / dt) * inertia * (midpoint - omega)
            + midpoint.cross(nextAngularMomentum);
      converged = isTorqueFreeMidpointResidualConverged(
          inertia, omega, midpoint, dt, nextResidual.norm());
      break;
    }
  }

  if (!converged) {
    const Eigen::Vector3d angularMomentum = inertia * midpoint;
    const Eigen::Vector3d residual = (2.0 / dt) * inertia * (midpoint - omega)
                                     + midpoint.cross(angularMomentum);
    converged = isTorqueFreeMidpointResidualConverged(
        inertia, omega, midpoint, dt, residual.norm());
  }

  nextOmega = 2.0 * midpoint - omega;
  return converged && nextOmega.allFinite();
}

void projectTorqueFreeAngularVelocityInvariants(
    const Eigen::Matrix3d& inertia,
    const Eigen::Vector3d& referenceOmega,
    Eigen::Vector3d& omega)
{
  const Eigen::Vector3d referenceAngularMomentum = inertia * referenceOmega;
  const double targetEnergy2 = referenceOmega.dot(referenceAngularMomentum);
  const double targetMomentum2 = referenceAngularMomentum.squaredNorm();

  if (!std::isfinite(targetEnergy2) || !std::isfinite(targetMomentum2)
      || targetEnergy2 <= 0.0 || targetMomentum2 <= 0.0) {
    return;
  }

  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector3d angularMomentum = inertia * omega;
    const Eigen::Vector2d residual(
        omega.dot(angularMomentum) - targetEnergy2,
        angularMomentum.squaredNorm() - targetMomentum2);
    const double scaledResidual = std::max(
        std::abs(residual[0]) / std::max(1.0, std::abs(targetEnergy2)),
        std::abs(residual[1]) / std::max(1.0, std::abs(targetMomentum2)));

    if (!std::isfinite(scaledResidual) || scaledResidual <= 1e-15) {
      return;
    }

    Eigen::Matrix<double, 2, 3> jacobian;
    jacobian.row(0) = (2.0 * angularMomentum).transpose();
    jacobian.row(1) = (2.0 * inertia * angularMomentum).transpose();

    const Eigen::Matrix2d normal = jacobian * jacobian.transpose();
    Eigen::LDLT<Eigen::Matrix2d> solver(normal);
    if (solver.info() != Eigen::Success) {
      return;
    }

    const Eigen::Vector2d multipliers = solver.solve(-residual);
    const Eigen::Vector3d correction = jacobian.transpose() * multipliers;
    if (!correction.allFinite()) {
      return;
    }

    const double correctionScale = std::max(1.0, omega.norm());
    if (correction.norm() > 1e-9 * correctionScale) {
      return;
    }

    omega += correction;
    if (!omega.allFinite()) {
      return;
    }
  }
}

bool canUseTorqueFreeIntegration(const FreeJoint& joint)
{
  if (joint.getParentBodyNode() != nullptr) {
    return false;
  }

  const BodyNode* bodyNode = joint.getChildBodyNode();
  if (bodyNode == nullptr || bodyNode->getNumChildBodyNodes() != 0) {
    return false;
  }

  const auto skeleton = joint.getSkeleton();
  if (skeleton && bodyNode->getGravityMode()
      && !isExactlyZero(skeleton->getGravity())) {
    return false;
  }

  if (!isExactlyZero(bodyNode->getExternalForceLocal())) {
    return false;
  }

  if (!isExactlyIdentity(joint.getTransformFromParentBodyNode())
      || !isExactlyIdentity(joint.getTransformFromChildBodyNode())) {
    return false;
  }

  for (std::size_t i = 0; i < joint.getNumDofs(); ++i) {
    const Joint::ActuatorType actuatorType = joint.getActuatorType(i);
    if (actuatorType != Joint::FORCE && actuatorType != Joint::PASSIVE) {
      return false;
    }

    if (!isExactlyZero(joint.getForce(i)) || !isExactlyZero(joint.getCommand(i))
        || !isExactlyZero(joint.getSpringStiffness(i))
        || !isExactlyZero(joint.getDampingCoefficient(i))
        || !isExactlyZero(joint.getCoulombFriction(i))) {
      return false;
    }
  }

  return true;
}

bool computeTorqueFreeAcceleration(
    const Eigen::Matrix3d& inertia,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& angularVelocityWorld,
    const Eigen::Vector3d& comOffsetWorld,
    Eigen::Vector6d& acceleration)
{
  Eigen::LDLT<Eigen::Matrix3d> inertiaSolver(inertia);
  if (inertiaSolver.info() != Eigen::Success || !inertiaSolver.isPositive()) {
    return false;
  }

  const Eigen::Vector3d angularVelocityBody
      = rotation.transpose() * angularVelocityWorld;
  const Eigen::Vector3d angularMomentumBody = inertia * angularVelocityBody;
  const Eigen::Vector3d angularAccelerationBody
      = inertiaSolver.solve(-angularVelocityBody.cross(angularMomentumBody));

  if (!angularAccelerationBody.allFinite()) {
    return false;
  }

  acceleration.head<3>() = rotation * angularAccelerationBody;
  acceleration.tail<3>() = -acceleration.head<3>().cross(comOffsetWorld)
                           - angularVelocityWorld.cross(
                               angularVelocityWorld.cross(comOffsetWorld));

  return acceleration.allFinite();
}

bool isConsistentWithTorqueFreeAcceleration(
    const FreeJoint& joint,
    const BodyNode& bodyNode,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& comOffsetWorld)
{
  Eigen::Vector6d expectedAcceleration;
  if (!computeTorqueFreeAcceleration(
          bodyNode.getInertia().getMoment(),
          rotation,
          joint.getVelocitiesStatic().head<3>(),
          comOffsetWorld,
          expectedAcceleration)) {
    return false;
  }

  const Eigen::Vector6d error
      = joint.getAccelerationsStatic() - expectedAcceleration;
  const double scale
      = std::max(1.0, expectedAcceleration.cwiseAbs().maxCoeff());

  return error.cwiseAbs().maxCoeff() <= 1e-9 * scale;
}

void alignWorldAngularMomentum(
    Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& targetWorldAngularMomentum,
    const Eigen::Vector3d& nextAngularMomentumBody)
{
  if (!targetWorldAngularMomentum.allFinite()
      || !nextAngularMomentumBody.allFinite()) {
    return;
  }

  const Eigen::Vector3d currentWorldAngularMomentum
      = rotation * nextAngularMomentumBody;
  const double targetNorm = targetWorldAngularMomentum.norm();
  const double currentNorm = currentWorldAngularMomentum.norm();
  if (targetNorm <= kTorqueFreeTolerance
      || currentNorm <= kTorqueFreeTolerance) {
    return;
  }

  Eigen::Quaterniond correction = Eigen::Quaterniond::FromTwoVectors(
      currentWorldAngularMomentum, targetWorldAngularMomentum);
  correction.normalize();

  if (!correction.coeffs().allFinite()) {
    return;
  }

  const Eigen::Matrix3d correctedRotation
      = correction.toRotationMatrix() * rotation;
  if (correctedRotation.allFinite()) {
    rotation = correctedRotation;
  }
}

} // namespace

//==============================================================================
FreeJoint::~FreeJoint()
{
  // Do nothing
}

//==============================================================================
void FreeJoint::setProperties(const Properties& properties)
{
  Base::setProperties(static_cast<const Base::Properties&>(properties));
  setProperties(static_cast<const UniqueProperties&>(properties));
}

//==============================================================================
void FreeJoint::setProperties(const UniqueProperties& properties)
{
  setAspectProperties(properties);
}

//==============================================================================
void FreeJoint::setAspectProperties(const AspectProperties& properties)
{
  setCoordinateChart(properties.mCoordinateChart);
}

//==============================================================================
FreeJoint::Properties FreeJoint::getFreeJointProperties() const
{
  return FreeJoint::Properties(
      getGenericJointProperties(), getFreeJointAspect()->getProperties());
}

//==============================================================================
void FreeJoint::copy(const FreeJoint& otherJoint)
{
  if (this == &otherJoint) {
    return;
  }

  setProperties(otherJoint.getFreeJointProperties());
}

//==============================================================================
void FreeJoint::copy(const FreeJoint* otherJoint)
{
  if (nullptr == otherJoint) {
    return;
  }

  copy(*otherJoint);
}

//==============================================================================
FreeJoint& FreeJoint::operator=(const FreeJoint& otherJoint)
{
  copy(otherJoint);
  return *this;
}

//==============================================================================
void FreeJoint::setCoordinateChart(CoordinateChart chart)
{
  if (mAspectProperties.mCoordinateChart == chart) {
    return;
  }

  const CoordinateChart previousChart = mAspectProperties.mCoordinateChart;
  const Eigen::Isometry3d transform
      = convertToTransform(getPositionsStatic(), previousChart);

  mAspectProperties.mCoordinateChart = chart;
  updateDegreeOfFreedomNames();

  setPositionsStatic(convertToPositions(transform, chart));
  updateRelativeJacobian(true);
  Joint::incrementVersion();
}

//==============================================================================
FreeJoint::CoordinateChart FreeJoint::getCoordinateChart() const
{
  return mAspectProperties.mCoordinateChart;
}

//==============================================================================
Eigen::Vector6d FreeJoint::convertToPositions(const Eigen::Isometry3d& _tf)
{
  return convertToPositions(_tf, CoordinateChart::EXP_MAP);
}

//==============================================================================
Eigen::Vector6d FreeJoint::convertToPositions(
    const Eigen::Isometry3d& _tf, CoordinateChart chart)
{
  Eigen::Vector6d x;
  switch (chart) {
    case CoordinateChart::EXP_MAP:
      x.head<3>() = math::logMap(_tf.linear());
      break;
    case CoordinateChart::EULER_XYZ:
      x.head<3>() = math::matrixToEulerXYZ(_tf.linear());
      break;
    case CoordinateChart::EULER_ZYX:
      x.head<3>() = math::matrixToEulerZYX(_tf.linear());
      break;
    default:
      DART_WARN(
          "Unsupported coordinate chart ({}); returning zero rotation",
          static_cast<int>(chart));
      x.head<3>().setZero();
      break;
  }
  x.tail<3>() = _tf.translation();
  return x;
}

//==============================================================================
Eigen::Isometry3d FreeJoint::convertToTransform(
    const Eigen::Vector6d& _positions)
{
  return convertToTransform(_positions, CoordinateChart::EXP_MAP);
}

//==============================================================================
Eigen::Isometry3d FreeJoint::convertToTransform(
    const Eigen::Vector6d& _positions, CoordinateChart chart)
{
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  switch (chart) {
    case CoordinateChart::EXP_MAP:
      tf.linear() = math::expMapRot(_positions.head<3>());
      break;
    case CoordinateChart::EULER_XYZ:
      tf.linear() = math::eulerXYZToMatrix(_positions.head<3>());
      break;
    case CoordinateChart::EULER_ZYX:
      tf.linear() = math::eulerZYXToMatrix(_positions.head<3>());
      break;
    default:
      DART_ERROR(
          "Invalid coordinate chart specified ({})", static_cast<int>(chart));
      tf.linear().setIdentity();
      break;
  }
  tf.translation() = _positions.tail<3>();
  return tf;
}

//==============================================================================
//==============================================================================
void FreeJoint::setTransformOf(
    Joint* joint, const Eigen::Isometry3d& tf, const Frame* withRespectTo)
{
  if (nullptr == joint) {
    return;
  }

  FreeJoint* freeJoint = dynamic_cast<FreeJoint*>(joint);

  if (nullptr == freeJoint) {
    DART_WARN(
        "Invalid joint type. Setting transform is only allowed to FreeJoint. "
        "The joint type of given joint [{}] is [{}].",
        joint->getName(),
        joint->getType());
    return;
  }

  freeJoint->setTransform(tf, withRespectTo);
}

//==============================================================================
void FreeJoint::setTransformOf(
    BodyNode* bodyNode, const Eigen::Isometry3d& tf, const Frame* withRespectTo)
{
  if (nullptr == bodyNode) {
    return;
  }

  setTransformOf(bodyNode->getParentJoint(), tf, withRespectTo);
}

//==============================================================================
void FreeJoint::setTransformOf(
    Skeleton* skeleton,
    const Eigen::Isometry3d& tf,
    const Frame* withRespectTo,
    bool applyToAllRootBodies)
{
  if (nullptr == skeleton) {
    return;
  }

  const std::size_t numTrees = skeleton->getNumTrees();

  if (0 == numTrees) {
    return;
  }

  if (!applyToAllRootBodies) {
    setTransformOf(skeleton->getRootBodyNode(), tf, withRespectTo);
    return;
  }

  for (std::size_t i = 0; i < numTrees; ++i) {
    setTransformOf(skeleton->getRootBodyNode(i), tf, withRespectTo);
  }
}

//==============================================================================
void FreeJoint::setSpatialMotion(
    const Eigen::Isometry3d* newTransform,
    const Frame* withRespectTo,
    const Eigen::Vector6d* newSpatialVelocity,
    const Frame* velRelativeTo,
    const Frame* velInCoordinatesOf,
    const Eigen::Vector6d* newSpatialAcceleration,
    const Frame* accRelativeTo,
    const Frame* accInCoordinatesOf)
{
  if (newTransform) {
    setTransform(*newTransform, withRespectTo);
  }

  if (newSpatialVelocity) {
    setSpatialVelocity(*newSpatialVelocity, velRelativeTo, velInCoordinatesOf);
  }

  if (newSpatialAcceleration) {
    setSpatialAcceleration(
        *newSpatialAcceleration, accRelativeTo, accInCoordinatesOf);
  }
}

//==============================================================================
void FreeJoint::setRelativeTransform(const Eigen::Isometry3d& newTransform)
{
  setPositionsStatic(convertToPositions(
      Joint::mAspectProperties.mT_ParentBodyToJoint.inverse() * newTransform
          * Joint::mAspectProperties.mT_ChildBodyToJoint,
      getCoordinateChart()));
}

//==============================================================================
void FreeJoint::setTransform(
    const Eigen::Isometry3d& newTransform, const Frame* withRespectTo)
{
  DART_ASSERT(nullptr != withRespectTo);

  setRelativeTransform(
      withRespectTo->getTransform(getChildBodyNode()->getParentFrame())
      * newTransform);
}

//==============================================================================
void FreeJoint::setRelativeSpatialVelocity(
    const Eigen::Vector6d& newSpatialVelocity)
{
  const Eigen::Matrix6d& J = getRelativeJacobianStatic();
  const Eigen::Vector6d jointVelocities = J.inverse() * newSpatialVelocity;
  setVelocities(jointVelocities);
}

//==============================================================================
void FreeJoint::setRelativeSpatialVelocity(
    const Eigen::Vector6d& newSpatialVelocity, const Frame* inCoordinatesOf)
{
  DART_ASSERT(nullptr != inCoordinatesOf);

  if (getChildBodyNode() == inCoordinatesOf) {
    setRelativeSpatialVelocity(newSpatialVelocity);
  } else {
    setRelativeSpatialVelocity(
        math::AdR(
            inCoordinatesOf->getTransform(getChildBodyNode()),
            newSpatialVelocity));
  }
}

//==============================================================================
void FreeJoint::setSpatialVelocity(
    const Eigen::Vector6d& newSpatialVelocity,
    const Frame* relativeTo,
    const Frame* inCoordinatesOf)
{
  DART_ASSERT(nullptr != relativeTo);
  DART_ASSERT(nullptr != inCoordinatesOf);

  if (getChildBodyNode() == relativeTo) {
    DART_WARN(
        "{}",
        "Invalid reference frame for newSpatialVelocity. It shouldn't be the "
        "child BodyNode.\n");
    return;
  }

  // Change the reference frame of "newSpatialVelocity" to the child body node
  // frame.
  Eigen::Vector6d targetRelSpatialVel = newSpatialVelocity;
  if (getChildBodyNode() != inCoordinatesOf) {
    targetRelSpatialVel = math::AdR(
        inCoordinatesOf->getTransform(getChildBodyNode()), newSpatialVelocity);
  }

  // Compute the target relative spatial velocity from the parent body node to
  // the child body node.
  if (getChildBodyNode()->getParentFrame() != relativeTo) {
    if (relativeTo->isWorld()) {
      const Eigen::Vector6d parentVelocity = math::AdInvT(
          getRelativeTransform(),
          getChildBodyNode()->getParentFrame()->getSpatialVelocity());

      targetRelSpatialVel -= parentVelocity;
    } else {
      const Eigen::Vector6d parentVelocity = math::AdInvT(
          getRelativeTransform(),
          getChildBodyNode()->getParentFrame()->getSpatialVelocity());
      const Eigen::Vector6d arbitraryVelocity = math::AdT(
          relativeTo->getTransform(getChildBodyNode()),
          relativeTo->getSpatialVelocity());

      targetRelSpatialVel += -parentVelocity + arbitraryVelocity;
    }
  }

  setRelativeSpatialVelocity(targetRelSpatialVel);
}

//==============================================================================
void FreeJoint::setLinearVelocity(
    const Eigen::Vector3d& newLinearVelocity,
    const Frame* relativeTo,
    const Frame* inCoordinatesOf)
{
  DART_ASSERT(nullptr != relativeTo);
  DART_ASSERT(nullptr != inCoordinatesOf);

  Eigen::Vector6d targetSpatialVelocity;

  if (Frame::World() == relativeTo) {
    targetSpatialVelocity.head<3>()
        = getChildBodyNode()->getSpatialVelocity().head<3>();
  } else {
    targetSpatialVelocity.head<3>()
        = getChildBodyNode()
              ->getSpatialVelocity(relativeTo, getChildBodyNode())
              .head<3>();
  }

  targetSpatialVelocity.tail<3>()
      = getChildBodyNode()->getWorldTransform().linear().transpose()
        * inCoordinatesOf->getWorldTransform().linear() * newLinearVelocity;
  // Above code is equivalent to:
  // targetSpatialVelocity.tail<3>()
  //     = getChildBodyNode()->getTransform(
  //         inCoordinatesOf).linear().transpose()
  //       * newLinearVelocity;
  // but faster.

  setSpatialVelocity(targetSpatialVelocity, relativeTo, getChildBodyNode());
}

//==============================================================================
void FreeJoint::setAngularVelocity(
    const Eigen::Vector3d& newAngularVelocity,
    const Frame* relativeTo,
    const Frame* inCoordinatesOf)
{
  DART_ASSERT(nullptr != relativeTo);
  DART_ASSERT(nullptr != inCoordinatesOf);

  Eigen::Vector6d targetSpatialVelocity;

  targetSpatialVelocity.head<3>()
      = getChildBodyNode()->getWorldTransform().linear().transpose()
        * inCoordinatesOf->getWorldTransform().linear() * newAngularVelocity;
  // Above code is equivalent to:
  // targetSpatialVelocity.head<3>()
  //     = getChildBodyNode()->getTransform(
  //         inCoordinatesOf).linear().transpose()
  //       * newAngularVelocity;
  // but faster.

  if (Frame::World() == relativeTo) {
    targetSpatialVelocity.tail<3>()
        = getChildBodyNode()->getSpatialVelocity().tail<3>();
  } else {
    targetSpatialVelocity.tail<3>()
        = getChildBodyNode()
              ->getSpatialVelocity(relativeTo, getChildBodyNode())
              .tail<3>();
  }

  setSpatialVelocity(targetSpatialVelocity, relativeTo, getChildBodyNode());
}

//==============================================================================
void FreeJoint::setRelativeSpatialAcceleration(
    const Eigen::Vector6d& newSpatialAcceleration)
{
  const Eigen::Matrix6d& J = getRelativeJacobianStatic();
  const Eigen::Matrix6d& dJ = getRelativeJacobianTimeDerivStatic();

  const Eigen::Vector6d jointAccelerations
      = J.inverse() * (newSpatialAcceleration - dJ * getVelocitiesStatic());
  setAccelerations(jointAccelerations);
}

//==============================================================================
void FreeJoint::setRelativeSpatialAcceleration(
    const Eigen::Vector6d& newSpatialAcceleration, const Frame* inCoordinatesOf)
{
  DART_ASSERT(nullptr != inCoordinatesOf);

  if (getChildBodyNode() == inCoordinatesOf) {
    setRelativeSpatialAcceleration(newSpatialAcceleration);
  } else {
    setRelativeSpatialAcceleration(
        math::AdR(
            inCoordinatesOf->getTransform(getChildBodyNode()),
            newSpatialAcceleration));
  }
}

//==============================================================================
void FreeJoint::setSpatialAcceleration(
    const Eigen::Vector6d& newSpatialAcceleration,
    const Frame* relativeTo,
    const Frame* inCoordinatesOf)
{
  DART_ASSERT(nullptr != relativeTo);
  DART_ASSERT(nullptr != inCoordinatesOf);

  if (getChildBodyNode() == relativeTo) {
    DART_WARN(
        "Invalid reference frame for newSpatialAcceleration. It shouldn't be "
        "the child BodyNode.");
    return;
  }

  // Change the reference frame of "newSpatialAcceleration" to the child body
  // node frame.
  Eigen::Vector6d targetRelSpatialAcc = newSpatialAcceleration;
  if (getChildBodyNode() != inCoordinatesOf) {
    targetRelSpatialAcc = math::AdR(
        inCoordinatesOf->getTransform(getChildBodyNode()),
        newSpatialAcceleration);
  }

  // Compute the target relative spatial acceleration from the parent body node
  // to the child body node.
  if (getChildBodyNode()->getParentFrame() != relativeTo) {
    if (relativeTo->isWorld()) {
      const Eigen::Vector6d parentAcceleration
          = math::AdInvT(
                getRelativeTransform(),
                getChildBodyNode()->getParentFrame()->getSpatialAcceleration())
            + math::ad(
                getChildBodyNode()->getSpatialVelocity(),
                getRelativeJacobianStatic() * getVelocitiesStatic());

      targetRelSpatialAcc -= parentAcceleration;
    } else {
      const Eigen::Vector6d parentAcceleration
          = math::AdInvT(
                getRelativeTransform(),
                getChildBodyNode()->getParentFrame()->getSpatialAcceleration())
            + math::ad(
                getChildBodyNode()->getSpatialVelocity(),
                getRelativeJacobianStatic() * getVelocitiesStatic());
      const Eigen::Vector6d arbitraryAcceleration
          = math::AdT(
                relativeTo->getTransform(getChildBodyNode()),
                relativeTo->getSpatialAcceleration())
            - math::ad(
                getChildBodyNode()->getSpatialVelocity(),
                math::AdT(
                    relativeTo->getTransform(getChildBodyNode()),
                    relativeTo->getSpatialVelocity()));

      targetRelSpatialAcc += -parentAcceleration + arbitraryAcceleration;
    }
  }

  setRelativeSpatialAcceleration(targetRelSpatialAcc);
}

//==============================================================================
void FreeJoint::setLinearAcceleration(
    const Eigen::Vector3d& newLinearAcceleration,
    const Frame* relativeTo,
    const Frame* inCoordinatesOf)
{
  DART_ASSERT(nullptr != relativeTo);
  DART_ASSERT(nullptr != inCoordinatesOf);

  Eigen::Vector6d targetSpatialAcceleration;

  if (Frame::World() == relativeTo) {
    targetSpatialAcceleration.head<3>()
        = getChildBodyNode()->getSpatialAcceleration().head<3>();
  } else {
    targetSpatialAcceleration.head<3>()
        = getChildBodyNode()
              ->getSpatialAcceleration(relativeTo, getChildBodyNode())
              .head<3>();
  }

  const Eigen::Vector6d& V
      = getChildBodyNode()->getSpatialVelocity(relativeTo, inCoordinatesOf);
  targetSpatialAcceleration.tail<3>()
      = getChildBodyNode()->getWorldTransform().linear().transpose()
        * inCoordinatesOf->getWorldTransform().linear()
        * (newLinearAcceleration - V.head<3>().cross(V.tail<3>()));
  // Above code is equivalent to:
  // targetSpatialAcceleration.tail<3>()
  //     = getChildBodyNode()->getTransform(
  //         inCoordinatesOf).linear().transpose()
  //       * (newLinearAcceleration - V.head<3>().cross(V.tail<3>()));
  // but faster.

  setSpatialAcceleration(
      targetSpatialAcceleration, relativeTo, getChildBodyNode());
}

//==============================================================================
void FreeJoint::setAngularAcceleration(
    const Eigen::Vector3d& newAngularAcceleration,
    const Frame* relativeTo,
    const Frame* inCoordinatesOf)
{
  DART_ASSERT(nullptr != relativeTo);
  DART_ASSERT(nullptr != inCoordinatesOf);

  Eigen::Vector6d targetSpatialAcceleration;

  targetSpatialAcceleration.head<3>()
      = getChildBodyNode()->getWorldTransform().linear().transpose()
        * inCoordinatesOf->getWorldTransform().linear()
        * newAngularAcceleration;
  // Above code is equivalent to:
  // targetSpatialAcceleration.head<3>()
  //     = getChildBodyNode()->getTransform(
  //         inCoordinatesOf).linear().transpose()
  //       * newAngularAcceleration;
  // but faster.

  if (Frame::World() == relativeTo) {
    targetSpatialAcceleration.tail<3>()
        = getChildBodyNode()->getSpatialAcceleration().tail<3>();
  } else {
    targetSpatialAcceleration.tail<3>()
        = getChildBodyNode()
              ->getSpatialAcceleration(relativeTo, getChildBodyNode())
              .tail<3>();
  }

  setSpatialAcceleration(
      targetSpatialAcceleration, relativeTo, getChildBodyNode());
}

//==============================================================================
Eigen::Matrix6d FreeJoint::getRelativeJacobianStatic(
    const Eigen::Vector6d& positions) const
{
  const Eigen::Isometry3d Q
      = convertToTransform(positions, getCoordinateChart());
  const Eigen::Isometry3d T
      = Joint::mAspectProperties.mT_ParentBodyToJoint * Q
        * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();

  const Eigen::Matrix3d rotationTranspose = T.linear().transpose();
  const Eigen::Matrix6d baseJac
      = math::getAdTMatrix(Joint::mAspectProperties.mT_ChildBodyToJoint);

  Eigen::Matrix6d jacobian;
  jacobian.topRows<3>() = rotationTranspose * baseJac.topRows<3>();
  jacobian.bottomRows<3>() = rotationTranspose * baseJac.bottomRows<3>();
  return jacobian;
}

//==============================================================================
Eigen::Vector6d FreeJoint::getPositionDifferencesStatic(
    const Eigen::Vector6d& _q2, const Eigen::Vector6d& _q1) const
{
  const Eigen::Isometry3d Q1 = convertToTransform(_q1, getCoordinateChart());
  const Eigen::Isometry3d Q2 = convertToTransform(_q2, getCoordinateChart());

  Eigen::Vector6d diff = Eigen::Vector6d::Zero();

  const Eigen::Matrix3d rotationChange = Q2.linear() * Q1.linear().transpose();
  Eigen::Isometry3d rotationOnly = Eigen::Isometry3d::Identity();
  rotationOnly.linear() = rotationChange;
  diff.head<3>()
      = convertToPositions(rotationOnly, getCoordinateChart()).head<3>();
  diff.tail<3>() = Q2.translation() - Q1.translation();

  return diff;
}

//==============================================================================
FreeJoint::FreeJoint(const Properties& properties)
  : Base(properties), mQ(Eigen::Isometry3d::Identity())
{
  mJacobianDeriv = Eigen::Matrix6d::Zero();

  // Inherited Aspects must be created in the final joint class in reverse order
  // or else we get pure virtual function calls
  createFreeJointAspect(properties);
  createGenericJointAspect(properties);
  createJointAspect(properties);
}

//==============================================================================
Joint* FreeJoint::clone() const
{
  return new FreeJoint(getFreeJointProperties());
}

//==============================================================================
std::string_view FreeJoint::getType() const
{
  return getStaticType();
}

//==============================================================================
std::string_view FreeJoint::getStaticType()
{
  static constexpr std::string_view name = "FreeJoint";
  return name;
}

//==============================================================================
bool FreeJoint::isCyclic(std::size_t _index) const
{
  return _index < 3 && !hasPositionLimit(0) && !hasPositionLimit(1)
         && !hasPositionLimit(2);
}

//==============================================================================
void FreeJoint::integratePositions(double _dt)
{
  Eigen::Vector6d spatialVelocity = getRelativeSpatialVelocity();
  const bool useTorqueFreePositionIntegration
      = mUseTorqueFreePositionIntegration
        && isExactlyZero(
            Eigen::Vector6d(getPositionsStatic() - mTorqueFreeInitialPositions))
        && isExactlyZero(
            Eigen::Vector6d(
                getVelocitiesStatic() - mTorqueFreeIntegratedVelocities))
        && canUseTorqueFreeIntegration(*this);

  if (useTorqueFreePositionIntegration) {
    spatialVelocity.head<3>() = mTorqueFreeMidpointAngularVelocityBody;
  }

  mUseTorqueFreePositionIntegration = false;

  const Eigen::Isometry3d relativeTransform = getRelativeTransform();

  Eigen::Isometry3d nextRelativeTransform = relativeTransform;
  const Eigen::Matrix3d rotation = relativeTransform.linear();
  nextRelativeTransform.linear()
      = rotation * math::expMapRot(spatialVelocity.head<3>() * _dt);
  nextRelativeTransform.translation()
      += rotation * spatialVelocity.tail<3>() * _dt;

  Eigen::Vector3d nextComOffset = Eigen::Vector3d::Zero();
  if (useTorqueFreePositionIntegration) {
    Eigen::Matrix3d correctedRotation = nextRelativeTransform.linear();
    alignWorldAngularMomentum(
        correctedRotation,
        mTorqueFreeInitialAngularMomentumWorld,
        mTorqueFreeNextAngularMomentumBody);
    nextRelativeTransform.linear() = correctedRotation;
    nextComOffset
        = nextRelativeTransform.linear() * getChildBodyNode()->getLocalCOM();
    const Eigen::Vector3d startComPosition
        = relativeTransform.translation()
          + relativeTransform.linear() * getChildBodyNode()->getLocalCOM();
    nextRelativeTransform.translation()
        = startComPosition + mTorqueFreeComLinearVelocityWorld * _dt
          - nextComOffset;
  }

  const Eigen::Isometry3d& parentBodyToJoint
      = Joint::mAspectProperties.mT_ParentBodyToJoint;
  const Eigen::Isometry3d& childBodyToJoint
      = Joint::mAspectProperties.mT_ChildBodyToJoint;

  const Eigen::Isometry3d nextQ
      = parentBodyToJoint.inverse() * nextRelativeTransform * childBodyToJoint;

  setPositionsStatic(convertToPositions(nextQ, getCoordinateChart()));

  if (useTorqueFreePositionIntegration) {
    // FreeJoint stores angular velocity in world coordinates. The midpoint pose
    // update advances the body frame, so keep the stored velocity consistent
    // with the final body-frame angular velocity.
    Eigen::Vector6d correctedVelocities = getVelocitiesStatic();
    const Eigen::Vector3d nextAngularVelocityWorld
        = nextRelativeTransform.linear() * mTorqueFreeNextAngularVelocityBody;
    correctedVelocities.head<3>() = nextAngularVelocityWorld;
    correctedVelocities.tail<3>()
        = mTorqueFreeComLinearVelocityWorld
          - nextAngularVelocityWorld.cross(nextComOffset);
    setVelocitiesStatic(correctedVelocities);
  }
}

//==============================================================================
void FreeJoint::integratePositions(
    const Eigen::VectorXd& q0,
    const Eigen::VectorXd& v,
    double dt,
    Eigen::VectorXd& result) const
{
  if (!std::cmp_equal(q0.size(), getNumDofs())
      || !std::cmp_equal(v.size(), getNumDofs())) {
    DART_ERROR(
        "q0's size [{}] and v's size [{}] must both equal the dof [{}] for "
        "Joint [{}].",
        q0.size(),
        v.size(),
        this->getNumDofs(),
        this->getName());
    DART_ASSERT(false);
    result = Eigen::VectorXd::Zero(getNumDofs());
    return;
  }

  const Eigen::Vector6d q0Static = q0;
  const Eigen::Vector6d vStatic = v;

  const Eigen::Isometry3d& parentBodyToJoint
      = Joint::mAspectProperties.mT_ParentBodyToJoint;
  const Eigen::Isometry3d& childBodyToJoint
      = Joint::mAspectProperties.mT_ChildBodyToJoint;

  const Eigen::Isometry3d Q0
      = convertToTransform(q0Static, getCoordinateChart());
  const Eigen::Isometry3d relativeTransform0
      = parentBodyToJoint * Q0 * childBodyToJoint.inverse();

  const Eigen::Vector6d spatialVelocity
      = getRelativeJacobianStatic(q0Static) * vStatic;

  Eigen::Isometry3d nextRelativeTransform = relativeTransform0;
  const Eigen::Matrix3d rotation = relativeTransform0.linear();
  nextRelativeTransform.linear()
      = rotation * math::expMapRot(spatialVelocity.head<3>() * dt);
  nextRelativeTransform.translation()
      += rotation * spatialVelocity.tail<3>() * dt;

  const Eigen::Isometry3d Qnext
      = parentBodyToJoint.inverse() * nextRelativeTransform * childBodyToJoint;

  result = convertToPositions(Qnext, getCoordinateChart());
}

//==============================================================================
void FreeJoint::integrateVelocities(double _dt)
{
  mUseTorqueFreePositionIntegration = false;

  if (_dt > 0.0 && canUseTorqueFreeIntegration(*this)) {
    const BodyNode* bodyNode = getChildBodyNode();
    const Eigen::Isometry3d transform
        = convertToTransform(getPositionsStatic(), getCoordinateChart());
    const Eigen::Matrix3d rotation = transform.linear();
    const Eigen::Vector3d comOffset = rotation * bodyNode->getLocalCOM();
    const Eigen::Vector3d angularVelocityBody
        = rotation.transpose() * getVelocitiesStatic().head<3>();
    const Eigen::Vector3d comLinearVelocityWorld
        = getVelocitiesStatic().tail<3>()
          + getVelocitiesStatic().head<3>().cross(comOffset);
    const Eigen::Matrix3d inertia = bodyNode->getInertia().getMoment();

    Eigen::Vector3d nextAngularVelocityBody;
    if (isConsistentWithTorqueFreeAcceleration(
            *this, *bodyNode, rotation, comOffset)
        && integrateTorqueFreeAngularVelocity(
            inertia, angularVelocityBody, _dt, nextAngularVelocityBody)) {
      projectTorqueFreeAngularVelocityInvariants(
          inertia, angularVelocityBody, nextAngularVelocityBody);
      Eigen::Vector6d nextVelocities = getVelocitiesStatic();
      const Eigen::Vector3d nextAngularVelocityWorld
          = rotation * nextAngularVelocityBody;
      nextVelocities.head<3>() = nextAngularVelocityWorld;
      nextVelocities.tail<3>()
          = comLinearVelocityWorld - nextAngularVelocityWorld.cross(comOffset);
      setVelocitiesStatic(nextVelocities);
      mUseTorqueFreePositionIntegration = true;
      mTorqueFreeInitialPositions = getPositionsStatic();
      mTorqueFreeIntegratedVelocities = nextVelocities;
      mTorqueFreeMidpointAngularVelocityBody
          = 0.5 * (angularVelocityBody + nextAngularVelocityBody);
      mTorqueFreeNextAngularVelocityBody = nextAngularVelocityBody;
      mTorqueFreeInitialAngularMomentumWorld
          = rotation * inertia * angularVelocityBody;
      mTorqueFreeNextAngularMomentumBody = inertia * nextAngularVelocityBody;
      mTorqueFreeComLinearVelocityWorld = comLinearVelocityWorld;
      return;
    }
  }

  setVelocitiesStatic(
      math::integrateVelocity<math::RealVectorSpace<6>>(
          getVelocitiesStatic(), getAccelerationsStatic(), _dt));
}

//==============================================================================
void FreeJoint::updateConstrainedTerms(double timeStep)
{
  Base::updateConstrainedTerms(timeStep);
}

//==============================================================================
void FreeJoint::updateDegreeOfFreedomNames()
{
  const auto rotAffixes
      = getCoordinateChart() == CoordinateChart::EULER_ZYX
            ? std::to_array<std::string>({"_rot_z", "_rot_y", "_rot_x"})
            : std::to_array<std::string>({"_rot_x", "_rot_y", "_rot_z"});

  for (std::size_t i = 0; i < rotAffixes.size(); ++i) {
    if (!mDofs[i]->isNamePreserved()) {
      mDofs[i]->setName(Joint::mAspectProperties.mName + rotAffixes[i], false);
    }
  }

  if (!mDofs[3]->isNamePreserved()) {
    mDofs[3]->setName(Joint::mAspectProperties.mName + "_pos_x", false);
  }
  if (!mDofs[4]->isNamePreserved()) {
    mDofs[4]->setName(Joint::mAspectProperties.mName + "_pos_y", false);
  }
  if (!mDofs[5]->isNamePreserved()) {
    mDofs[5]->setName(Joint::mAspectProperties.mName + "_pos_z", false);
  }
}

//==============================================================================
void FreeJoint::updateRelativeTransform() const
{
  mQ = convertToTransform(getPositionsStatic(), getCoordinateChart());

  mT = Joint::mAspectProperties.mT_ParentBodyToJoint * mQ
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();

  if (!math::verifyTransform(mT)) {
    DART_WARN_ONCE(
        "[FreeJoint::updateRelativeTransform] Non-finite relative transform "
        "detected in '{}'. Using identity.",
        this->getName());
    mT = Eigen::Isometry3d::Identity();
  }
}

//==============================================================================
void FreeJoint::updateRelativeJacobian(bool) const
{
  const Eigen::Matrix3d rotationTranspose
      = getRelativeTransform().linear().transpose();
  const Eigen::Matrix6d baseJac
      = math::getAdTMatrix(Joint::mAspectProperties.mT_ChildBodyToJoint);

  mJacobian.topRows<3>() = rotationTranspose * baseJac.topRows<3>();
  mJacobian.bottomRows<3>() = rotationTranspose * baseJac.bottomRows<3>();
}

//==============================================================================
void FreeJoint::updateRelativeJacobianTimeDeriv() const
{
  const Eigen::Matrix3d rotationTranspose
      = getRelativeTransform().linear().transpose();
  const Eigen::Matrix6d baseJac
      = math::getAdTMatrix(Joint::mAspectProperties.mT_ChildBodyToJoint);

  const Eigen::Vector3d omega = getRelativeSpatialVelocity().head<3>();
  const Eigen::Matrix3d rotationDeriv
      = -math::makeSkewSymmetric(omega) * rotationTranspose;

  mJacobianDeriv.setZero();
  mJacobianDeriv.topRows<3>() = rotationDeriv * baseJac.topRows<3>();
  mJacobianDeriv.bottomRows<3>() = rotationDeriv * baseJac.bottomRows<3>();
}

//==============================================================================
const Eigen::Isometry3d& FreeJoint::getQ() const
{
  if (mNeedTransformUpdate) {
    updateRelativeTransform();
    mNeedTransformUpdate = false;
  }

  return mQ;
}

} // namespace dynamics
} // namespace dart
