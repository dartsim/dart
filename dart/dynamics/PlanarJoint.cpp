/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
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

#include "dart/dynamics/PlanarJoint.h"

#include <string>

#include "dart/common/Console.h"
#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"

namespace dart {
namespace dynamics {

//==============================================================================
PlanarJoint::PlanarJoint(const std::string& _name)
  : Joint(_name)
{
  mGenCoords.push_back(&mCoordinate[0]);
  mGenCoords.push_back(&mCoordinate[1]);
  mGenCoords.push_back(&mCoordinate[2]);

  mS = Eigen::Matrix<double, 6, 3>::Zero();
  mdS = Eigen::Matrix<double, 6, 3>::Zero();

  mSpringStiffness.resize(3, 0.0);
  mDampingCoefficient.resize(3, 0.0);
  mRestPosition.resize(3, 0.0);

  setXYPlane();
}

//==============================================================================
PlanarJoint::~PlanarJoint()
{
}

//==============================================================================
void PlanarJoint::setXYPlane()
{
  mPlaneType = PT_XY;
  mRotAxis   = Eigen::Vector3d::UnitZ();
  mTransAxis1 = Eigen::Vector3d::UnitX();
  mTransAxis2 = Eigen::Vector3d::UnitY();
}

//==============================================================================
void PlanarJoint::setYZPlane()
{
  mPlaneType = PT_YZ;
  mRotAxis   = Eigen::Vector3d::UnitX();
  mTransAxis1 = Eigen::Vector3d::UnitY();
  mTransAxis2 = Eigen::Vector3d::UnitZ();
}

//==============================================================================
void PlanarJoint::setZXPlane()
{
  mPlaneType = PT_ZX;
  mRotAxis   = Eigen::Vector3d::UnitY();
  mTransAxis1 = Eigen::Vector3d::UnitZ();
  mTransAxis2 = Eigen::Vector3d::UnitX();
}

//==============================================================================
void PlanarJoint::setArbitraryPlane(const Eigen::Vector3d& _transAxis1,
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
  assert(std::fabs(dotProduct) < 1.0 - 1e-6);
  if (std::fabs(dotProduct) > 1e-6)
    mTransAxis2 = (mTransAxis2 - dotProduct * mTransAxis1).normalized();

  // Rotational axis
  mRotAxis = (mTransAxis1.cross(mTransAxis2)).normalized();
}

//==============================================================================
void PlanarJoint::updateTransform()
{
  mT = mT_ParentBodyToJoint
       * Eigen::Translation3d(mTransAxis1 * mCoordinate[0].getPos())
       * Eigen::Translation3d(mTransAxis2 * mCoordinate[1].getPos())
       * math::expAngular    (mRotAxis    * mCoordinate[2].getPos())
       * mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

//==============================================================================
void PlanarJoint::updateJacobian()
{
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 3);
  J.block<3, 1>(3, 0) = mTransAxis1;
  J.block<3, 1>(3, 1) = mTransAxis2;
  J.block<3, 1>(0, 2) = mRotAxis;

  mS.leftCols<2>()
      = math::AdTJac(mT_ChildBodyToJoint
                     * math::expAngular(mRotAxis * -mCoordinate[2].getPos()),
                     J.leftCols<2>());
  mS.col(2)     = math::AdTJac(mT_ChildBodyToJoint, J.col(2));

  assert(!math::isNan(mS));
}

//==============================================================================
void PlanarJoint::updateJacobianTimeDeriv()
{
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 3);
  J.block<3, 1>(3, 0) = mTransAxis1;
  J.block<3, 1>(3, 1) = mTransAxis2;
  J.block<3, 1>(0, 2) = mRotAxis;

  mdS.col(0)
      = -math::ad(mS.col(2)*mCoordinate[2].getVel(),
                  math::AdT(mT_ChildBodyToJoint
                            * math::expAngular(mRotAxis
                                               * -mCoordinate[2].getPos()),
                            J.col(0)));

  mdS.col(1)
      = -math::ad(mS.col(2)*mCoordinate[2].getVel(),
                  math::AdT(mT_ChildBodyToJoint
                            * math::expAngular(mRotAxis
                                               * -mCoordinate[2].getPos()),
                            J.col(1)));

  assert(mdS.col(2) == Eigen::Vector6d::Zero());
  assert(!math::isNan(mdS.col(0)));
  assert(!math::isNan(mdS.col(1)));
}

//==============================================================================
PlaneType PlanarJoint::getPlaneType() const
{
  return mPlaneType;
}

//==============================================================================
const Eigen::Vector3d& PlanarJoint::getRotationalAxis() const
{
  return mRotAxis;
}

//==============================================================================
const Eigen::Vector3d& PlanarJoint::getTranslationalAxis1() const
{
  return mTransAxis1;
}

//==============================================================================
const Eigen::Vector3d& PlanarJoint::getTranslationalAxis2() const
{
  return mTransAxis2;
}

}  // namespace dynamics
}  // namespace dart
