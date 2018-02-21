/*
 * Copyright (c) 2011-2017, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/dynamics/TranslationalJoint2d.hpp"

#include <string>

#include "dart/common/Console.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/Helpers.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
TranslationalJoint2d::~TranslationalJoint2d()
{
  // Do nothing
}

//==============================================================================
void TranslationalJoint2d::setProperties(const Properties& properties)
{
  Base::setProperties(static_cast<const Base::Properties&>(properties));
  setProperties(static_cast<const UniqueProperties&>(properties));
}

//==============================================================================
void TranslationalJoint2d::setProperties(const UniqueProperties& properties)
{
  setAspectProperties(properties);
}

//==============================================================================
void TranslationalJoint2d::setAspectProperties(
    const AspectProperties& properties)
{
  mAspectProperties = properties;
  Joint::notifyPositionUpdated();
  updateRelativeJacobian(true);
  Joint::incrementVersion();
}

//==============================================================================
TranslationalJoint2d::Properties
TranslationalJoint2d::getTranslationalJoint2dProperties() const
{
  return Properties(getGenericJointProperties(), mAspectProperties);
}

//==============================================================================
void TranslationalJoint2d::copy(const TranslationalJoint2d& otherJoint)
{
  if (this == &otherJoint)
    return;

  setProperties(otherJoint.getTranslationalJoint2dProperties());
}

//==============================================================================
void TranslationalJoint2d::copy(const TranslationalJoint2d* otherJoint)
{
  if (nullptr == otherJoint)
    return;

  copy(*this);
}

//==============================================================================
TranslationalJoint2d& TranslationalJoint2d::operator=(
    const TranslationalJoint2d& otherJoint)
{
  copy(otherJoint);
  return *this;
}

//==============================================================================
const std::string& TranslationalJoint2d::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& TranslationalJoint2d::getStaticType()
{
  static const std::string name = "TranslationalJoint2d";
  return name;
}

//==============================================================================
bool TranslationalJoint2d::isCyclic(std::size_t /*index*/) const
{
  return false;
}

//==============================================================================
void TranslationalJoint2d::setXYPlane(bool renameDofs)
{
  mAspectProperties.setXYPlane();

  if (renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdated();
}

//==============================================================================
void TranslationalJoint2d::setYZPlane(bool renameDofs)
{
  mAspectProperties.setYZPlane();

  if (renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdated();
}

//==============================================================================
void TranslationalJoint2d::setZXPlane(bool renameDofs)
{
  mAspectProperties.setZXPlane();

  if (renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdated();
}

//==============================================================================
void TranslationalJoint2d::setArbitraryPlane(
    const Eigen::Vector3d& transAxis1,
    const Eigen::Vector3d& transAxis2,
    bool renameDofs)
{
  mAspectProperties.setArbitraryPlane(transAxis1, transAxis2);

  if (renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdated();
}

//==============================================================================
TranslationalJoint2d::PlaneType TranslationalJoint2d::getPlaneType() const
{
  return mAspectProperties.mPlaneType;
}

//==============================================================================
Eigen::Vector3d TranslationalJoint2d::getTranslationalAxis1() const
{
  return mAspectProperties.mTransAxes.col(0);
}

//==============================================================================
Eigen::Vector3d TranslationalJoint2d::getTranslationalAxis2() const
{
  return mAspectProperties.mTransAxes.col(0);
}

//==============================================================================
Eigen::Matrix<double, 6, 2> TranslationalJoint2d::getRelativeJacobianStatic(
    const Eigen::Vector2d& /*positions*/) const
{
  // The Jacobian is always constant w.r.t. the generalized coordinates.
  return getRelativeJacobianStatic();
}

//==============================================================================
TranslationalJoint2d::TranslationalJoint2d(const Properties& properties)
  : detail::TranslationalJoint2dBase(properties)
{
  // Inherited Aspects must be created in the final joint class in reverse order
  // or else we get pure virtual function calls
  createTranslationalJoint2dAspect(properties);
  createGenericJointAspect(properties);
  createJointAspect(properties);
}

//==============================================================================
Joint* TranslationalJoint2d::clone() const
{
  return new TranslationalJoint2d(getTranslationalJoint2dProperties());
}

//==============================================================================
void TranslationalJoint2d::updateDegreeOfFreedomNames()
{
  if (!mDofs[0]->isNamePreserved())
    mDofs[0]->setName(Joint::mAspectProperties.mName + "_1", false);
  if (!mDofs[1]->isNamePreserved())
    mDofs[1]->setName(Joint::mAspectProperties.mName + "_2", false);
}

//==============================================================================
void TranslationalJoint2d::updateRelativeTransform() const
{
  const Eigen::Vector2d& positions = getPositionsStatic();
  mT = Joint::mAspectProperties.mT_ParentBodyToJoint
       * Eigen::Translation3d(
             mAspectProperties.mTransAxes.col(0) * positions[0])
       * Eigen::Translation3d(
             mAspectProperties.mTransAxes.col(1) * positions[1])
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();

  // Verification
  assert(math::verifyTransform(mT));
}

//==============================================================================
void TranslationalJoint2d::updateRelativeJacobian(bool mandatory) const
{
  if (mandatory)
  {
    mJacobian.bottomRows<3>()
        = Joint::mAspectProperties.mT_ChildBodyToJoint.linear()
          * mAspectProperties.mTransAxes;

    // Verification
    assert(mJacobian.topRows<3>() == (Eigen::Matrix<double, 3, 2>::Zero()));
    assert(!math::isNan(mJacobian.bottomRows<3>()));
  }
}

//==============================================================================
void TranslationalJoint2d::updateRelativeJacobianTimeDeriv() const
{
  // Time derivative of translational joint is always zero
  assert(mJacobianDeriv == (Eigen::Matrix<double, 6, 2>::Zero()));
}

} // namespace dynamics
} // namespace dart
