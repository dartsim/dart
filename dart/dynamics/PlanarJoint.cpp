/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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
  MultiDofJoint<3>::setProperties(
        static_cast<const MultiDofJoint<3>::Properties&>(_properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void PlanarJoint::setProperties(const UniqueProperties& _properties)
{
  getPlanarJointAddon()->setProperties(_properties);
}

//==============================================================================
PlanarJoint::Properties PlanarJoint::getPlanarJointProperties() const
{
  return Properties(getMultiDofJointProperties(),
                    getPlanarJointAddon()->getProperties());
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
bool PlanarJoint::isCyclic(size_t _index) const
{
  return _index == 2 && !hasPositionLimit(_index);
}

//==============================================================================
void PlanarJoint::setXYPlane(bool _renameDofs)
{
  getPlanarJointAddon()->setXYPlane();

  if (_renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdate();
}

//==============================================================================
void PlanarJoint::setYZPlane(bool _renameDofs)
{
  getPlanarJointAddon()->setYZPlane();

  if (_renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdate();
}

//==============================================================================
void PlanarJoint::setZXPlane(bool _renameDofs)
{
  getPlanarJointAddon()->setZXPlane();

  if (_renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdate();
}

//==============================================================================
void PlanarJoint::setArbitraryPlane(const Eigen::Vector3d& _transAxis1,
                                    const Eigen::Vector3d& _transAxis2,
                                    bool _renameDofs)
{
  getPlanarJointAddon()->setArbitraryPlane(_transAxis1, _transAxis2);

  if (_renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdate();
}

//==============================================================================
PlanarJoint::PlaneType PlanarJoint::getPlaneType() const
{
  return getPlanarJointAddon()->getPlaneType();
}

//==============================================================================
const Eigen::Vector3d& PlanarJoint::getRotationalAxis() const
{
  return getPlanarJointAddon()->getRotAxis();
}

//==============================================================================
const Eigen::Vector3d& PlanarJoint::getTranslationalAxis1() const
{
  return getPlanarJointAddon()->getTransAxis1();
}

//==============================================================================
const Eigen::Vector3d& PlanarJoint::getTranslationalAxis2() const
{
  return getPlanarJointAddon()->getTransAxis2();
}

//==============================================================================
Eigen::Matrix<double, 6, 3> PlanarJoint::getLocalJacobianStatic(
    const Eigen::Vector3d& _positions) const
{
  Eigen::Matrix<double, 6, 3> J = Eigen::Matrix<double, 6, 3>::Zero();
  J.block<3, 1>(3, 0) = getPlanarJointAddon()->getTransAxis1();
  J.block<3, 1>(3, 1) = getPlanarJointAddon()->getTransAxis2();
  J.block<3, 1>(0, 2) = getPlanarJointAddon()->getRotAxis();

  J.leftCols<2>()
      = math::AdTJacFixed(mJointP.mT_ChildBodyToJoint
                          * math::expAngular(getPlanarJointAddon()->getRotAxis()
                                             * -_positions[2]),
                          J.leftCols<2>());
  J.col(2) = math::AdTJac(mJointP.mT_ChildBodyToJoint, J.col(2));

  // Verification
  assert(!math::isNan(J));

  return J;
}

//==============================================================================
PlanarJoint::PlanarJoint(const Properties& _properties)
  : detail::PlanarJointBase(_properties, common::NoArg)
{
  createPlanarJointAddon(_properties);

  // Inherited Joint Properties must be set in the final joint class or else we
  // get pure virtual function calls
  MultiDofJoint<3>::setProperties(_properties);
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
  switch (getPlanarJointAddon()->getPlaneType())
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
      dterr << "Unsupported plane type in PlanarJoint named '" << mJointP.mName
            << "' (" << static_cast<int>(getPlanarJointAddon()->getPlaneType())
            << ")\n";
  }

  if (affixes.size() == 2)
  {
    for (size_t i = 0; i < 2; ++i)
    {
      if (!mDofs[i]->isNamePreserved())
        mDofs[i]->setName(mJointP.mName + affixes[i], false);
    }
  }
}

//==============================================================================
void PlanarJoint::updateLocalTransform() const
{
  const Eigen::Vector3d& positions = getPositionsStatic();
  mT = mJointP.mT_ParentBodyToJoint
       * Eigen::Translation3d(getPlanarJointAddon()->getTransAxis1() * positions[0])
       * Eigen::Translation3d(getPlanarJointAddon()->getTransAxis2() * positions[1])
       * math::expAngular    (getPlanarJointAddon()->getRotAxis()    * positions[2])
       * mJointP.mT_ChildBodyToJoint.inverse();

  // Verification
  assert(math::verifyTransform(mT));
}

//==============================================================================
void PlanarJoint::updateLocalJacobian(bool) const
{
  mJacobian = getLocalJacobianStatic(getPositionsStatic());
}

//==============================================================================
void PlanarJoint::updateLocalJacobianTimeDeriv() const
{
  Eigen::Matrix<double, 6, 3> J = Eigen::Matrix<double, 6, 3>::Zero();
  J.block<3, 1>(3, 0) = getPlanarJointAddon()->getTransAxis1();
  J.block<3, 1>(3, 1) = getPlanarJointAddon()->getTransAxis2();
  J.block<3, 1>(0, 2) = getPlanarJointAddon()->getRotAxis();

  const Eigen::Matrix<double, 6, 3>& Jacobian = getLocalJacobianStatic();
  const Eigen::Vector3d& velocities = getVelocitiesStatic();
  mJacobianDeriv.col(0)
      = -math::ad(Jacobian.col(2) * velocities[2],
                  math::AdT(mJointP.mT_ChildBodyToJoint
                            * math::expAngular(getPlanarJointAddon()->getRotAxis()
                                               * -getPositionsStatic()[2]),
                            J.col(0)));

  mJacobianDeriv.col(1)
      = -math::ad(Jacobian.col(2) * velocities[2],
                  math::AdT(mJointP.mT_ChildBodyToJoint
                            * math::expAngular(getPlanarJointAddon()->getRotAxis()
                                               * -getPositionsStatic()[2]),
                            J.col(1)));

  assert(mJacobianDeriv.col(2) == Eigen::Vector6d::Zero());
  assert(!math::isNan(mJacobianDeriv.col(0)));
  assert(!math::isNan(mJacobianDeriv.col(1)));
}

}  // namespace dynamics
}  // namespace dart
