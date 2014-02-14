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

namespace dart {
namespace dynamics {

PlanarJoint::PlanarJoint(const std::string& _name)
  : Joint(PLANAR, _name)
{
  mGenCoords.push_back(&mCoordinate[0]);
  mGenCoords.push_back(&mCoordinate[1]);
  mGenCoords.push_back(&mCoordinate[2]);

  mS = Eigen::Matrix<double, 6, 3>::Zero();
  mdS = Eigen::Matrix<double, 6, 3>::Zero();

  mSpringStiffness.resize(3, 0.0);
  mDampingCoefficient.resize(3, 0.0);
  mRestPosition.resize(3, 0.0);

  setPlaneType(PT_XY);
}

PlanarJoint::~PlanarJoint()
{
}

void PlanarJoint::updateTransform()
{
  mT = mT_ParentBodyToJoint
       * Eigen::Translation3d(mTranAxis1 * mCoordinate[1].get_q())
       * Eigen::Translation3d(mTranAxis2 * mCoordinate[2].get_q())
       * math::expAngular    (mRotAxis   * mCoordinate[0].get_q())
       * mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

void PlanarJoint::updateJacobian()
{
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 3);
  J.block<3, 1>(0, 0) = mRotAxis;
  J.block<3, 1>(3, 1) = mTranAxis1;
  J.block<3, 1>(3, 2) = mTranAxis2;

  mS.col(0)     = math::AdTJac(mT_ChildBodyToJoint, J.col(0));
  mS.rightCols<2>()
      = math::AdTJac(mT_ChildBodyToJoint
                     * math::expAngular(mRotAxis * -mCoordinate[0].get_q()),
                     J.rightCols<2>());

  assert(!math::isNan(mS));
}

void PlanarJoint::updateJacobianTimeDeriv()
{
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 3);
  J.block<3, 1>(0, 0) = mRotAxis;
  J.block<3, 1>(3, 1) = mTranAxis1;
  J.block<3, 1>(3, 2) = mTranAxis2;

  mdS.col(1)
      = -math::ad(mS.col(0)*mCoordinate[0].get_dq(),
                  math::AdT(mT_ChildBodyToJoint
                            * math::expAngular(mRotAxis
                                               * -mCoordinate[0].get_q()),
                            J.col(1)));

  mdS.col(2)
      = -math::ad(mS.col(0)*mCoordinate[0].get_dq(),
                  math::AdT(mT_ChildBodyToJoint
                            * math::expAngular(mRotAxis
                                               * -mCoordinate[0].get_q()),
                            J.col(2)));

  assert(mdS.col(0) == Eigen::Vector6d::Zero());
  assert(!math::isNan(mdS.col(1)));
  assert(!math::isNan(mdS.col(2)));
}

void PlanarJoint::setPlaneType(PlaneType _planeType)
{
  switch (_planeType)
  {
    case PT_XY:
    {
      mPlaneType = _planeType;
      mRotAxis   = Eigen::Vector3d::UnitZ();
      mTranAxis1 = Eigen::Vector3d::UnitX();
      mTranAxis2 = Eigen::Vector3d::UnitY();
      break;
    }
    case PT_YZ:
    {
      mPlaneType = _planeType;
      mRotAxis   = Eigen::Vector3d::UnitX();
      mTranAxis1 = Eigen::Vector3d::UnitY();
      mTranAxis2 = Eigen::Vector3d::UnitZ();
      break;
    }
    case PT_ZX:
    {
      mPlaneType = _planeType;
      mRotAxis   = Eigen::Vector3d::UnitY();
      mTranAxis1 = Eigen::Vector3d::UnitZ();
      mTranAxis2 = Eigen::Vector3d::UnitX();
      break;
    }
    case PT_ARBITRARY:
    {
      dterr << "PlaneJoint::setPlaneType(): PT_ARBITRARY can be set here."
            << "Instead please use use setPlane() for arbitrary plane."
            << std::endl;
      break;
    }
    default:
      dterr << "PlaneJoint::setPlaneType(): Unsupported plane type."
            << std::endl;
      break;
  }
}

PlanarJoint::PlaneType PlanarJoint::getPlaneType() const
{
  return mPlaneType;
}

void PlanarJoint::setPlane(const Eigen::Vector3d& _rotAxis,
                            const Eigen::Vector3d& _tranAxis1)
{
  mPlaneType = PT_ARBITRARY;

  // Rotational axis
  mRotAxis = _rotAxis.normalized();

  // Translational axes
  assert(_rotAxis == _tranAxis1);
  Eigen::Vector3d unitTA1 = _tranAxis1.normalized();
  mTranAxis1 = (unitTA1 - unitTA1.dot(mRotAxis) * mRotAxis).normalized();

  mTranAxis2 = (mRotAxis.cross(mTranAxis1)).normalized();
}

const Eigen::Vector3d&PlanarJoint::getRotationalAxis() const
{
  return mRotAxis;
}

const Eigen::Vector3d&PlanarJoint::getTranslationalAxis1() const
{
  return mTranAxis1;
}

const Eigen::Vector3d&PlanarJoint::getTranslationalAxis2() const
{
  return mTranAxis2;
}


}  // namespace dynamics
}  // namespace dart
