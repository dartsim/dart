/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/dynamics/PlanarJoint.hpp"

#include <string>

#include "dart/common/Console.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
PlanarJoint::~PlanarJoint()
{
  // Do nothing
}

//==============================================================================
void PlanarJoint::setProperties(const Properties& _properties)
{
  Base::setProperties(
        static_cast<const Base::Properties&>(_properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void PlanarJoint::setProperties(const UniqueProperties& _properties)
{
  setAspectProperties(_properties);
}

//==============================================================================
void PlanarJoint::setAspectProperties(const AspectProperties& properties)
{
  mAspectProperties = properties;
  Joint::notifyPositionUpdate();
  updateRelativeJacobian(true);
  Joint::incrementVersion();
}

//==============================================================================
PlanarJoint::Properties PlanarJoint::getPlanarJointProperties() const
{
  return Properties(getGenericJointProperties(), mAspectProperties);
}

//==============================================================================
void PlanarJoint::copy(const PlanarJoint& _otherJoint)
{
  if(this == &_otherJoint)
    return;

  setProperties(_otherJoint.getPlanarJointProperties());
}

//==============================================================================
void PlanarJoint::copy(const PlanarJoint* _otherJoint)
{
  if(nullptr == _otherJoint)
    return;

  copy(*_otherJoint);
}

//==============================================================================
PlanarJoint& PlanarJoint::operator=(const PlanarJoint& _otherJoint)
{
  copy(_otherJoint);
  return *this;
}

//==============================================================================
const std::string& PlanarJoint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& PlanarJoint::getStaticType()
{
  static const std::string name = "PlanarJoint";
  return name;
}

//==============================================================================
bool PlanarJoint::isCyclic(std::size_t _index) const
{
  return _index == 2 && !hasPositionLimit(_index);
}

//==============================================================================
void PlanarJoint::setXYPlane(bool _renameDofs)
{
  mAspectProperties.setXYPlane();

  if (_renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdate();
}

//==============================================================================
void PlanarJoint::setYZPlane(bool _renameDofs)
{
  mAspectProperties.setYZPlane();

  if (_renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdate();
}

//==============================================================================
void PlanarJoint::setZXPlane(bool _renameDofs)
{
  mAspectProperties.setZXPlane();

  if (_renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdate();
}

//==============================================================================
void PlanarJoint::setArbitraryPlane(const Eigen::Vector3d& _transAxis1,
                                    const Eigen::Vector3d& _transAxis2,
                                    bool _renameDofs)
{
  mAspectProperties.setArbitraryPlane(_transAxis1, _transAxis2);

  if (_renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdate();
}

//==============================================================================
PlanarJoint::PlaneType PlanarJoint::getPlaneType() const
{
  return mAspectProperties.mPlaneType;
}

//==============================================================================
const Eigen::Vector3d& PlanarJoint::getRotationalAxis() const
{
  return mAspectProperties.mRotAxis;
}

//==============================================================================
const Eigen::Vector3d& PlanarJoint::getTranslationalAxis1() const
{
  return mAspectProperties.mTransAxis1;
}

//==============================================================================
const Eigen::Vector3d& PlanarJoint::getTranslationalAxis2() const
{
  return mAspectProperties.mTransAxis2;
}

//==============================================================================
Eigen::Matrix<double, 6, 3> PlanarJoint::getRelativeJacobianStatic(
    const Eigen::Vector3d& _positions) const
{
  Eigen::Matrix<double, 6, 3> J = Eigen::Matrix<double, 6, 3>::Zero();
  J.block<3, 1>(3, 0) = mAspectProperties.mTransAxis1;
  J.block<3, 1>(3, 1) = mAspectProperties.mTransAxis2;
  J.block<3, 1>(0, 2) = mAspectProperties.mRotAxis;

  J.leftCols<2>()
      = math::AdTJacFixed(Joint::mAspectProperties.mT_ChildBodyToJoint
                          * math::expAngular(mAspectProperties.mRotAxis
                                             * -_positions[2]),
                          J.leftCols<2>());
  J.col(2) = math::AdTJac(Joint::mAspectProperties.mT_ChildBodyToJoint, J.col(2));

  // Verification
  assert(!math::isNan(J));

  return J;
}

//==============================================================================
PlanarJoint::PlanarJoint(const Properties& properties)
  : detail::PlanarJointBase(properties)
{
  // Inherited Aspects must be created in the final joint class in reverse order
  // or else we get pure virtual function calls
  createPlanarJointAspect(properties);
  createGenericJointAspect(properties);
  createJointAspect(properties);
}

//==============================================================================
Joint* PlanarJoint::clone() const
{
  return new PlanarJoint(getPlanarJointProperties());
}

//==============================================================================
void PlanarJoint::updateDegreeOfFreedomNames()
{
  std::vector<std::string> affixes;
  switch (mAspectProperties.mPlaneType)
  {
    case PlaneType::XY:
      affixes.push_back("_x");
      affixes.push_back("_y");
      break;
    case PlaneType::YZ:
      affixes.push_back("_y");
      affixes.push_back("_z");
      break;
    case PlaneType::ZX:
      affixes.push_back("_z");
      affixes.push_back("_x");
      break;
    case PlaneType::ARBITRARY:
      affixes.push_back("_1");
      affixes.push_back("_2");
      break;
    default:
      dterr << "Unsupported plane type in PlanarJoint named '" << Joint::mAspectProperties.mName
            << "' (" << static_cast<int>(mAspectProperties.mPlaneType)
            << ")\n";
  }

  if (affixes.size() == 2)
  {
    for (std::size_t i = 0; i < 2; ++i)
    {
      if (!mDofs[i]->isNamePreserved())
        mDofs[i]->setName(Joint::mAspectProperties.mName + affixes[i], false);
    }
  }
}

//==============================================================================
void PlanarJoint::updateRelativeTransform() const
{
  const Eigen::Vector3d& positions = getPositionsStatic();
  mT = Joint::mAspectProperties.mT_ParentBodyToJoint
       * Eigen::Translation3d(mAspectProperties.mTransAxis1 * positions[0])
       * Eigen::Translation3d(mAspectProperties.mTransAxis2 * positions[1])
       * math::expAngular    (mAspectProperties.mRotAxis    * positions[2])
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();

  // Verification
  assert(math::verifyTransform(mT));
}

//==============================================================================
void PlanarJoint::updateRelativeJacobian(bool) const
{
  mJacobian = getRelativeJacobianStatic(getPositionsStatic());
}

//==============================================================================
void PlanarJoint::updateRelativeJacobianTimeDeriv() const
{
  Eigen::Matrix<double, 6, 3> J = Eigen::Matrix<double, 6, 3>::Zero();
  J.block<3, 1>(3, 0) = mAspectProperties.mTransAxis1;
  J.block<3, 1>(3, 1) = mAspectProperties.mTransAxis2;
  J.block<3, 1>(0, 2) = mAspectProperties.mRotAxis;

  const Eigen::Matrix<double, 6, 3>& Jacobian = getRelativeJacobianStatic();
  const Eigen::Vector3d& velocities = getVelocitiesStatic();
  mJacobianDeriv.col(0)
      = -math::ad(Jacobian.col(2) * velocities[2],
                  math::AdT(Joint::mAspectProperties.mT_ChildBodyToJoint
                            * math::expAngular(mAspectProperties.mRotAxis
                                               * -getPositionsStatic()[2]),
                            J.col(0)));

  mJacobianDeriv.col(1)
      = -math::ad(Jacobian.col(2) * velocities[2],
                  math::AdT(Joint::mAspectProperties.mT_ChildBodyToJoint
                            * math::expAngular(mAspectProperties.mRotAxis
                                               * -getPositionsStatic()[2]),
                            J.col(1)));

  assert(mJacobianDeriv.col(2) == Eigen::Vector6d::Zero());
  assert(!math::isNan(mJacobianDeriv.col(0)));
  assert(!math::isNan(mJacobianDeriv.col(1)));
}

}  // namespace dynamics
}  // namespace dart
