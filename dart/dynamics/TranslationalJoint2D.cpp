/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/dynamics/TranslationalJoint2D.hpp"

#include <string>

#include "dart/common/Console.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/Helpers.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
TranslationalJoint2D::~TranslationalJoint2D()
{
  // Do nothing
}

//==============================================================================
void TranslationalJoint2D::setProperties(const Properties& properties)
{
  Base::setProperties(static_cast<const Base::Properties&>(properties));
  setProperties(static_cast<const UniqueProperties&>(properties));
}

//==============================================================================
void TranslationalJoint2D::setProperties(const UniqueProperties& properties)
{
  setAspectProperties(properties);
}

//==============================================================================
void TranslationalJoint2D::setAspectProperties(
    const AspectProperties& properties)
{
  mAspectProperties = properties;
  Joint::notifyPositionUpdated();
  updateRelativeJacobian(true);
  Joint::incrementVersion();
}

//==============================================================================
TranslationalJoint2D::Properties
TranslationalJoint2D::getTranslationalJoint2DProperties() const
{
  return Properties(getGenericJointProperties(), mAspectProperties);
}

//==============================================================================
void TranslationalJoint2D::copy(const TranslationalJoint2D& otherJoint)
{
  if (this == &otherJoint)
    return;

  setProperties(otherJoint.getTranslationalJoint2DProperties());
}

//==============================================================================
void TranslationalJoint2D::copy(const TranslationalJoint2D* otherJoint)
{
  if (nullptr == otherJoint)
    return;

  copy(*this);
}

//==============================================================================
TranslationalJoint2D& TranslationalJoint2D::operator=(
    const TranslationalJoint2D& otherJoint)
{
  copy(otherJoint);
  return *this;
}

//==============================================================================
const std::string& TranslationalJoint2D::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& TranslationalJoint2D::getStaticType()
{
  static const std::string name = "TranslationalJoint2D";
  return name;
}

//==============================================================================
bool TranslationalJoint2D::isCyclic(std::size_t /*index*/) const
{
  return false;
}

//==============================================================================
void TranslationalJoint2D::setXYPlane(bool renameDofs)
{
  mAspectProperties.setXYPlane();

  if (renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdated();
}

//==============================================================================
void TranslationalJoint2D::setYZPlane(bool renameDofs)
{
  mAspectProperties.setYZPlane();

  if (renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdated();
}

//==============================================================================
void TranslationalJoint2D::setZXPlane(bool renameDofs)
{
  mAspectProperties.setZXPlane();

  if (renameDofs)
    updateDegreeOfFreedomNames();
  notifyPositionUpdated();
}

//==============================================================================
void TranslationalJoint2D::setArbitraryPlane(
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
TranslationalJoint2D::PlaneType TranslationalJoint2D::getPlaneType() const
{
  return mAspectProperties.getPlaneType();
}

//==============================================================================
Eigen::Vector3d TranslationalJoint2D::getTranslationalAxis1() const
{
  return mAspectProperties.getTranslationalAxis1();
}

//==============================================================================
Eigen::Vector3d TranslationalJoint2D::getTranslationalAxis2() const
{
  return mAspectProperties.getTranslationalAxis2();
}

//==============================================================================
Eigen::Matrix<double, 6, 2> TranslationalJoint2D::getRelativeJacobianStatic(
    const Eigen::Vector2d& /*positions*/) const
{
  // The Jacobian is always constant w.r.t. the generalized coordinates.
  return getRelativeJacobianStatic();
}

//==============================================================================
TranslationalJoint2D::TranslationalJoint2D(const Properties& properties)
  : detail::TranslationalJoint2DBase(properties)
{
  // Inherited Aspects must be created in the final joint class in reverse order
  // or else we get pure virtual function calls
  createTranslationalJoint2DAspect(properties);
  createGenericJointAspect(properties);
  createJointAspect(properties);
}

//==============================================================================
Joint* TranslationalJoint2D::clone() const
{
  return new TranslationalJoint2D(getTranslationalJoint2DProperties());
}

//==============================================================================
void TranslationalJoint2D::updateDegreeOfFreedomNames()
{
  if (!mDofs[0]->isNamePreserved())
    mDofs[0]->setName(Joint::mAspectProperties.mName + "_1", false);
  if (!mDofs[1]->isNamePreserved())
    mDofs[1]->setName(Joint::mAspectProperties.mName + "_2", false);
}

//==============================================================================
void TranslationalJoint2D::updateRelativeTransform() const
{
  const Eigen::Vector2d& positions = getPositionsStatic();
  mT = Joint::mAspectProperties.mT_ParentBodyToJoint
       * Eigen::Translation3d(
           mAspectProperties.getTranslationalAxes() * positions)
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();

  // Verification
  assert(math::verifyTransform(mT));
}

//==============================================================================
void TranslationalJoint2D::updateRelativeJacobian(bool mandatory) const
{
  if (mandatory)
  {
    mJacobian.bottomRows<3>()
        = Joint::mAspectProperties.mT_ChildBodyToJoint.linear()
          * mAspectProperties.getTranslationalAxes();

    // Verification
    assert(mJacobian.topRows<3>() == (Eigen::Matrix<double, 3, 2>::Zero()));
    assert(!math::isNan(mJacobian.bottomRows<3>()));
  }
}

//==============================================================================
void TranslationalJoint2D::updateRelativeJacobianTimeDeriv() const
{
  // Time derivative of translational joint is always zero
  assert(mJacobianDeriv == (Eigen::Matrix<double, 6, 2>::Zero()));
}

} // namespace dynamics
} // namespace dart
