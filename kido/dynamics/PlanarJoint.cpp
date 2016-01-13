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

#include "kido/dynamics/PlanarJoint.hpp"

#include <string>

#include "kido/common/Console.hpp"
#include "kido/math/Geometry.hpp"
#include "kido/math/Helpers.hpp"
#include "kido/dynamics/DegreeOfFreedom.hpp"

namespace kido {
namespace dynamics {

//==============================================================================
PlanarJoint::UniqueProperties::UniqueProperties(PlaneType _planeType)
{
  switch(_planeType)
  {
    case PT_ARBITRARY:
    case PT_XY:
      setXYPlane();
      mPlaneType = _planeType; // In case the PlaneType was supposed to be arbitrary
      break;
    case PT_YZ:
      setYZPlane();
       break;
    case PT_ZX:
      setZXPlane();
      break;
  }
}

//==============================================================================
PlanarJoint::UniqueProperties::UniqueProperties(
    const Eigen::Vector3d& _transAxis1,
    const Eigen::Vector3d& _transAxis2)
{
  setArbitraryPlane(_transAxis1, _transAxis2);
}

//==============================================================================
void PlanarJoint::UniqueProperties::setXYPlane()
{
  mPlaneType = PT_XY;
  mRotAxis   = Eigen::Vector3d::UnitZ();
  mTransAxis1 = Eigen::Vector3d::UnitX();
  mTransAxis2 = Eigen::Vector3d::UnitY();
}

//==============================================================================
void PlanarJoint::UniqueProperties::setYZPlane()
{
  mPlaneType = PT_YZ;
  mRotAxis   = Eigen::Vector3d::UnitX();
  mTransAxis1 = Eigen::Vector3d::UnitY();
  mTransAxis2 = Eigen::Vector3d::UnitZ();
}

//==============================================================================
void PlanarJoint::UniqueProperties::setZXPlane()
{
  mPlaneType = PT_ZX;
  mRotAxis   = Eigen::Vector3d::UnitY();
  mTransAxis1 = Eigen::Vector3d::UnitZ();
  mTransAxis2 = Eigen::Vector3d::UnitX();
}

//==============================================================================
void PlanarJoint::UniqueProperties::setArbitraryPlane(
    const Eigen::Vector3d& _transAxis1,
    const Eigen::Vector3d& _transAxis2)
{
  // Set plane type as arbitrary plane
  mPlaneType = PT_ARBITRARY;

  // First translational axis
  mTransAxis1 = _transAxis1.normalized();

  // Second translational axis
  mTransAxis2 = _transAxis2.normalized();

  // Orthogonalize translational axese
  double dotProduct = mTransAxis1.dot(mTransAxis2);
  assert(std::abs(dotProduct) < 1.0 - 1e-6);
  if (std::abs(dotProduct) > 1e-6)
    mTransAxis2 = (mTransAxis2 - dotProduct * mTransAxis1).normalized();

  // Rotational axis
  mRotAxis = (mTransAxis1.cross(mTransAxis2)).normalized();
}

//==============================================================================
PlanarJoint::Properties::Properties(
    const MultiDofJoint<3>::Properties& _multiDofProperties,
    const PlanarJoint::UniqueProperties& _planarProperties)
  : MultiDofJoint<3>::Properties(_multiDofProperties),
    PlanarJoint::UniqueProperties(_planarProperties)
{
  // Do nothing
}

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
  switch(_properties.mPlaneType)
  {
    case PT_XY:
      setXYPlane();
      break;
    case PT_YZ:
      setYZPlane();
      break;
    case PT_ZX:
      setZXPlane();
      break;
    case PT_ARBITRARY:
      setArbitraryPlane(_properties.mTransAxis1, _properties.mTransAxis2);
      break;
  }
}

//==============================================================================
PlanarJoint::Properties PlanarJoint::getPlanarJointProperties() const
{
  return Properties(getMultiDofJointProperties(), mPlanarP);
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
  mPlanarP.setXYPlane();

  if (_renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdate();
}

//==============================================================================
void PlanarJoint::setYZPlane(bool _renameDofs)
{
  mPlanarP.setYZPlane();

  if (_renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdate();
}

//==============================================================================
void PlanarJoint::setZXPlane(bool _renameDofs)
{
  mPlanarP.setZXPlane();

  if (_renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdate();
}

//==============================================================================
void PlanarJoint::setArbitraryPlane(const Eigen::Vector3d& _transAxis1,
                                    const Eigen::Vector3d& _transAxis2,
                                    bool _renameDofs)
{
  mPlanarP.setArbitraryPlane(_transAxis1, _transAxis2);

  if (_renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdate();
}

//==============================================================================
PlanarJoint::PlaneType PlanarJoint::getPlaneType() const
{
  return mPlanarP.mPlaneType;
}

//==============================================================================
const Eigen::Vector3d& PlanarJoint::getRotationalAxis() const
{
  return mPlanarP.mRotAxis;
}

//==============================================================================
const Eigen::Vector3d& PlanarJoint::getTranslationalAxis1() const
{
  return mPlanarP.mTransAxis1;
}

//==============================================================================
const Eigen::Vector3d& PlanarJoint::getTranslationalAxis2() const
{
  return mPlanarP.mTransAxis2;
}

//==============================================================================
Eigen::Matrix<double, 6, 3> PlanarJoint::getLocalJacobianStatic(
    const Eigen::Vector3d& _positions) const
{
  Eigen::Matrix<double, 6, 3> J = Eigen::Matrix<double, 6, 3>::Zero();
  J.block<3, 1>(3, 0) = mPlanarP.mTransAxis1;
  J.block<3, 1>(3, 1) = mPlanarP.mTransAxis2;
  J.block<3, 1>(0, 2) = mPlanarP.mRotAxis;

  J.leftCols<2>()
      = math::AdTJacFixed(mJointP.mT_ChildBodyToJoint
                          * math::expAngular(mPlanarP.mRotAxis * -_positions[2]),
                          J.leftCols<2>());
  J.col(2) = math::AdTJac(mJointP.mT_ChildBodyToJoint, J.col(2));

  // Verification
  assert(!math::isNan(J));

  return J;
}

//==============================================================================
PlanarJoint::PlanarJoint(const Properties& _properties)
  : MultiDofJoint<3>(_properties)
{
  setProperties(_properties);
  updateDegreeOfFreedomNames();
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
  switch (mPlanarP.mPlaneType)
  {
    case PT_XY:
      affixes.push_back("_x");
      affixes.push_back("_y");
      break;
    case PT_YZ:
      affixes.push_back("_y");
      affixes.push_back("_z");
      break;
    case PT_ZX:
      affixes.push_back("_z");
      affixes.push_back("_x");
      break;
    case PT_ARBITRARY:
      affixes.push_back("_1");
      affixes.push_back("_2");
      break;
    default:
      dterr << "Unsupported plane type in PlanarJoint named '" << mJointP.mName
            << "' (" << mPlanarP.mPlaneType << ")\n";
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
       * Eigen::Translation3d(mPlanarP.mTransAxis1 * positions[0])
       * Eigen::Translation3d(mPlanarP.mTransAxis2 * positions[1])
       * math::expAngular    (mPlanarP.mRotAxis    * positions[2])
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
  J.block<3, 1>(3, 0) = mPlanarP.mTransAxis1;
  J.block<3, 1>(3, 1) = mPlanarP.mTransAxis2;
  J.block<3, 1>(0, 2) = mPlanarP.mRotAxis;

  const Eigen::Matrix<double, 6, 3>& Jacobian = getLocalJacobianStatic();
  const Eigen::Vector3d& velocities = getVelocitiesStatic();
  mJacobianDeriv.col(0)
      = -math::ad(Jacobian.col(2) * velocities[2],
                  math::AdT(mJointP.mT_ChildBodyToJoint
                            * math::expAngular(mPlanarP.mRotAxis
                                               * -getPositionsStatic()[2]),
                            J.col(0)));

  mJacobianDeriv.col(1)
      = -math::ad(Jacobian.col(2) * velocities[2],
                  math::AdT(mJointP.mT_ChildBodyToJoint
                            * math::expAngular(mPlanarP.mRotAxis
                                               * -getPositionsStatic()[2]),
                            J.col(1)));

  assert(mJacobianDeriv.col(2) == Eigen::Vector6d::Zero());
  assert(!math::isNan(mJacobianDeriv.col(0)));
  assert(!math::isNan(mJacobianDeriv.col(1)));
}

}  // namespace dynamics
}  // namespace kido
