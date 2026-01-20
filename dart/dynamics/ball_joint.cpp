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

#include "dart/dynamics/ball_joint.hpp"

#include "dart/common/macros.hpp"
#include "dart/dynamics/degree_of_freedom.hpp"
#include "dart/math/geometry.hpp"
#include "dart/math/helpers.hpp"

#include <array>
#include <string>

namespace dart {
namespace dynamics {

//==============================================================================
BallJoint::~BallJoint()
{
  // Do nothing
}

//==============================================================================
void BallJoint::setProperties(const Properties& properties)
{
  Base::setProperties(static_cast<const Base::Properties&>(properties));
  setProperties(static_cast<const UniqueProperties&>(properties));
}

//==============================================================================
void BallJoint::setProperties(const UniqueProperties& properties)
{
  setAspectProperties(properties);
}

//==============================================================================
void BallJoint::setAspectProperties(const AspectProperties& properties)
{
  setCoordinateChart(properties.mCoordinateChart);
}

//==============================================================================
std::string_view BallJoint::getType() const
{
  return getStaticType();
}

//==============================================================================
std::string_view BallJoint::getStaticType()
{
  static constexpr std::string_view name = "BallJoint";
  return name;
}

//==============================================================================
bool BallJoint::isCyclic(std::size_t _index) const
{
  return _index < 3 && !hasPositionLimit(0) && !hasPositionLimit(1)
         && !hasPositionLimit(2);
}

//==============================================================================
BallJoint::Properties BallJoint::getBallJointProperties() const
{
  return BallJoint::Properties(
      getGenericJointProperties(), getBallJointAspect()->getProperties());
}

//==============================================================================
void BallJoint::copy(const BallJoint& otherJoint)
{
  if (this == &otherJoint)
    return;

  setProperties(otherJoint.getBallJointProperties());
}

//==============================================================================
void BallJoint::copy(const BallJoint* otherJoint)
{
  if (nullptr == otherJoint)
    return;

  copy(*otherJoint);
}

//==============================================================================
BallJoint& BallJoint::operator=(const BallJoint& otherJoint)
{
  copy(otherJoint);
  return *this;
}

//==============================================================================
void BallJoint::setCoordinateChart(CoordinateChart chart)
{
  if (mAspectProperties.mCoordinateChart == chart)
    return;

  const CoordinateChart previousChart = mAspectProperties.mCoordinateChart;
  const Eigen::Matrix3d rotation
      = convertToRotation(getPositionsStatic(), previousChart);

  mAspectProperties.mCoordinateChart = chart;
  updateDegreeOfFreedomNames();

  setPositionsStatic(convertToPositions(rotation, chart));
  updateRelativeJacobian(true);
  Joint::incrementVersion();
}

//==============================================================================
BallJoint::CoordinateChart BallJoint::getCoordinateChart() const
{
  return mAspectProperties.mCoordinateChart;
}

//==============================================================================
Eigen::Isometry3d BallJoint::convertToTransform(
    const Eigen::Vector3d& _positions)
{
  return convertToTransform(_positions, CoordinateChart::EXP_MAP);
}

//==============================================================================
Eigen::Isometry3d BallJoint::convertToTransform(
    const Eigen::Vector3d& _positions, CoordinateChart chart)
{
  return Eigen::Isometry3d(convertToRotation(_positions, chart));
}

//==============================================================================
Eigen::Matrix3d BallJoint::convertToRotation(const Eigen::Vector3d& _positions)
{
  return convertToRotation(_positions, CoordinateChart::EXP_MAP);
}

//==============================================================================
Eigen::Matrix3d BallJoint::convertToRotation(
    const Eigen::Vector3d& _positions, CoordinateChart chart)
{
  switch (chart) {
    case CoordinateChart::EXP_MAP:
      return math::expMapRot(_positions);
    case CoordinateChart::EULER_XYZ:
      return math::eulerXYZToMatrix(_positions);
    case CoordinateChart::EULER_ZYX:
      return math::eulerZYXToMatrix(_positions);
    default:
      DART_ERROR(
          "Invalid coordinate chart specified ({})", static_cast<int>(chart));
      return Eigen::Matrix3d::Identity();
  }
}

//==============================================================================
BallJoint::BallJoint(const Properties& properties)
  : Base(properties), mR(Eigen::Isometry3d::Identity())
{
  mJacobianDeriv = Eigen::Matrix<double, 6, 3>::Zero();

  // Inherited Aspects must be created in the final joint class in reverse order
  // or else we get pure virtual function calls
  createBallJointAspect(properties);
  createGenericJointAspect(properties);
  createJointAspect(properties);
}

//==============================================================================
Joint* BallJoint::clone() const
{
  return new BallJoint(getBallJointProperties());
}

//==============================================================================
Eigen::Matrix<double, 6, 3> BallJoint::getRelativeJacobianStatic(
    const Eigen::Vector3d& /*positions*/) const
{
  return mJacobian;
}

//==============================================================================
Eigen::Vector3d BallJoint::getPositionDifferencesStatic(
    const Eigen::Vector3d& _q2, const Eigen::Vector3d& _q1) const
{
  const Eigen::Matrix3d R1 = convertToRotation(_q1, getCoordinateChart());
  const Eigen::Matrix3d R2 = convertToRotation(_q2, getCoordinateChart());

  return convertToPositions(R1.transpose() * R2, getCoordinateChart());
}

//==============================================================================
void BallJoint::integratePositions(double _dt)
{
  const Eigen::Matrix3d Rnext
      = getR().linear() * math::expMapRot(getVelocitiesStatic() * _dt);

  setPositionsStatic(convertToPositions(Rnext, getCoordinateChart()));
}

//==============================================================================
void BallJoint::integratePositions(
    const Eigen::VectorXd& q0,
    const Eigen::VectorXd& v,
    double dt,
    Eigen::VectorXd& result) const
{
  if (static_cast<std::size_t>(q0.size()) != getNumDofs()
      || static_cast<std::size_t>(v.size()) != getNumDofs()) {
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

  const Eigen::Vector3d q0Static = q0;
  const Eigen::Vector3d vStatic = v;

  const Eigen::Matrix3d R0 = convertToRotation(q0Static, getCoordinateChart());
  const Eigen::Matrix3d Rnext = R0 * math::expMapRot(vStatic * dt);

  result = convertToPositions(Rnext, getCoordinateChart());
}

//==============================================================================
void BallJoint::updateDegreeOfFreedomNames()
{
  std::array<std::string, 3> affixes{"_x", "_y", "_z"};

  if (getCoordinateChart() == CoordinateChart::EULER_ZYX)
    affixes = {"_z", "_y", "_x"};

  for (std::size_t i = 0; i < affixes.size(); ++i) {
    if (!mDofs[i]->isNamePreserved())
      mDofs[i]->setName(Joint::mAspectProperties.mName + affixes[i], false);
  }
}

//==============================================================================
void BallJoint::updateRelativeTransform() const
{
  mR.linear() = convertToRotation(getPositionsStatic(), getCoordinateChart());

  mT = Joint::mAspectProperties.mT_ParentBodyToJoint * mR
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();

  DART_ASSERT(math::verifyTransform(mT));
}

//==============================================================================
void BallJoint::updateRelativeJacobian(bool _mandatory) const
{
  if (_mandatory) {
    mJacobian = math::getAdTMatrix(Joint::mAspectProperties.mT_ChildBodyToJoint)
                    .leftCols<3>();
  }
}

//==============================================================================
void BallJoint::updateRelativeJacobianTimeDeriv() const
{
  DART_ASSERT(Eigen::Matrix6d::Zero().leftCols<3>() == mJacobianDeriv);
}

//==============================================================================
const Eigen::Isometry3d& BallJoint::getR() const
{
  if (mNeedTransformUpdate) {
    updateRelativeTransform();
    mNeedTransformUpdate = false;
  }

  return mR;
}

} // namespace dynamics
} // namespace dart
