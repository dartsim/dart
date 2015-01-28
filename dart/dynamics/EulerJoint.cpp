/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
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

#include "dart/dynamics/EulerJoint.h"

#include <string>

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/math/Geometry.h"
#include "dart/dynamics/DegreeOfFreedom.h"

namespace dart {
namespace dynamics {

//==============================================================================
EulerJoint::EulerJoint(const std::string& _name)
  : MultiDofJoint(_name),
    mAxisOrder(AO_XYZ)
{
  updateDegreeOfFreedomNames();
}

//==============================================================================
EulerJoint::~EulerJoint()
{
}

//==============================================================================
void EulerJoint::setAxisOrder(EulerJoint::AxisOrder _order, bool _renameDofs)
{
  mAxisOrder = _order;
  if (_renameDofs)
    updateDegreeOfFreedomNames();
}

//==============================================================================
EulerJoint::AxisOrder EulerJoint::getAxisOrder() const
{
  return mAxisOrder;
}

//==============================================================================
void EulerJoint::updateDegreeOfFreedomNames()
{
  std::vector<std::string> affixes;
  switch (mAxisOrder)
  {
    case AO_ZYX:
      affixes.push_back("_z");
      affixes.push_back("_y");
      affixes.push_back("_x");
      break;
    case AO_XYZ:
      affixes.push_back("_x");
      affixes.push_back("_y");
      affixes.push_back("_z");
      break;
    default:
      dterr << "Unsupported axis order in EulerJoint named '" << mName
            << "' (" << mAxisOrder << ")\n";
  }

  if (affixes.size() == 3)
  {
    for (size_t i = 0; i < 3; ++i)
    {
      if(!mDofs[i]->isNamePreserved())
        mDofs[i]->setName(mName + affixes[i], false);
    }
  }
}

//==============================================================================
void EulerJoint::updateLocalTransform()
{
  switch (mAxisOrder)
  {
    case AO_XYZ:
    {
      mT = mT_ParentBodyToJoint
           * Eigen::Isometry3d(math::eulerXYZToMatrix(mPositions))
           * mT_ChildBodyToJoint.inverse();
      break;
    }
    case AO_ZYX:
    {
      mT = mT_ParentBodyToJoint
           * Eigen::Isometry3d(math::eulerZYXToMatrix(mPositions))
           * mT_ChildBodyToJoint.inverse();
      break;
    }
    default:
    {
      dterr << "Undefined Euler axis order\n";
      break;
    }
  }

  assert(math::verifyTransform(mT));
}

//==============================================================================
void EulerJoint::updateLocalJacobian()
{
  // double q0 = mPositions[0];
  double q1 = mPositions[1];
  double q2 = mPositions[2];

  // double c0 = cos(q0);
  double c1 = cos(q1);
  double c2 = cos(q2);

  // double s0 = sin(q0);
  double s1 = sin(q1);
  double s2 = sin(q2);

  Eigen::Vector6d J0 = Eigen::Vector6d::Zero();
  Eigen::Vector6d J1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d J2 = Eigen::Vector6d::Zero();

  switch (mAxisOrder)
  {
    case AO_XYZ:
    {
      //------------------------------------------------------------------------
      // S = [    c1*c2, s2,  0
      //       -(c1*s2), c2,  0
      //             s1,  0,  1
      //              0,  0,  0
      //              0,  0,  0
      //              0,  0,  0 ];
      //------------------------------------------------------------------------
      J0 << c1*c2, -(c1*s2),  s1, 0.0, 0.0, 0.0;
      J1 <<    s2,       c2, 0.0, 0.0, 0.0, 0.0;
      J2 <<   0.0,      0.0, 1.0, 0.0, 0.0, 0.0;

#ifndef NDEBUG
      if (fabs(mPositions[1]) == DART_PI * 0.5)
        std::cout << "Singular configuration in ZYX-euler joint ["
                  << mName << "]. ("
                  << mPositions[0] << ", "
                  << mPositions[1] << ", "
                  << mPositions[2] << ")"
                  << std::endl;
#endif

      break;
    }
    case AO_ZYX:
    {
      //------------------------------------------------------------------------
      // S = [   -s1,    0,   1
      //       s2*c1,   c2,   0
      //       c1*c2,  -s2,   0
      //           0,    0,   0
      //           0,    0,   0
      //           0,    0,   0 ];
      //------------------------------------------------------------------------
      J0 << -s1, s2*c1, c1*c2, 0.0, 0.0, 0.0;
      J1 << 0.0,    c2,   -s2, 0.0, 0.0, 0.0;
      J2 << 1.0,   0.0,   0.0, 0.0, 0.0, 0.0;

#ifndef NDEBUG
      if (fabs(mPositions[1]) == DART_PI * 0.5)
        std::cout << "Singular configuration in ZYX-euler joint ["
                  << mName << "]. ("
                  << mPositions[0] << ", "
                  << mPositions[1] << ", "
                  << mPositions[2] << ")"
                  << std::endl;
#endif

      break;
    }
    default:
    {
      dterr << "Undefined Euler axis order\n";
      break;
    }
  }

  mJacobian.col(0) = math::AdT(mT_ChildBodyToJoint, J0);
  mJacobian.col(1) = math::AdT(mT_ChildBodyToJoint, J1);
  mJacobian.col(2) = math::AdT(mT_ChildBodyToJoint, J2);

  assert(!math::isNan(mJacobian));

#ifndef NDEBUG
  Eigen::MatrixXd JTJ = mJacobian.transpose() * mJacobian;
  Eigen::FullPivLU<Eigen::MatrixXd> luJTJ(JTJ);
  //    Eigen::FullPivLU<Eigen::MatrixXd> luS(mS);
  double det = luJTJ.determinant();
  if (det < 1e-5)
  {
    std::cout << "ill-conditioned Jacobian in joint [" << mName << "]."
              << " The determinant of the Jacobian is (" << det << ")."
              << std::endl;
    std::cout << "rank is (" << luJTJ.rank() << ")." << std::endl;
    std::cout << "det is (" << luJTJ.determinant() << ")." << std::endl;
    //        std::cout << "mS: \n" << mS << std::endl;
  }
#endif
}

//==============================================================================
void EulerJoint::updateLocalJacobianTimeDeriv()
{
  // double q0 = mPositions[0];
  double q1 = mPositions[1];
  double q2 = mPositions[2];

  // double dq0 = mVelocities[0];
  double dq1 = mVelocities[1];
  double dq2 = mVelocities[2];

  // double c0 = cos(q0);
  double c1 = cos(q1);
  double c2 = cos(q2);

  // double s0 = sin(q0);
  double s1 = sin(q1);
  double s2 = sin(q2);

  Eigen::Vector6d dJ0 = Eigen::Vector6d::Zero();
  Eigen::Vector6d dJ1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d dJ2 = Eigen::Vector6d::Zero();

  switch (mAxisOrder)
  {
    case AO_XYZ:
    {
      //------------------------------------------------------------------------
      // dS = [  -(dq1*c2*s1) - dq2*c1*s2,    dq2*c2,  0
      //         -(dq2*c1*c2) + dq1*s1*s2, -(dq2*s2),  0
      //                           dq1*c1,         0,  0
      //                                0,         0,  0
      //                                0,         0,  0
      //                                0,         0,  0 ];
      //------------------------------------------------------------------------
      dJ0 << -(dq1*c2*s1) - dq2*c1*s2, -(dq2*c1*c2) + dq1*s1*s2, dq1*c1,
             0, 0, 0;
      dJ1 << dq2*c2,                -(dq2*s2),    0.0, 0.0, 0.0, 0.0;
      dJ2.setConstant(0.0);

      break;
    }
    case AO_ZYX:
    {
      //------------------------------------------------------------------------
      // dS = [               -c1*dq1,        0,   0
      //          c2*c1*dq2-s2*s1*dq1,  -s2*dq2,   0
      //         -s1*c2*dq1-c1*s2*dq2,  -c2*dq2,   0
      //                            0,        0,   0
      //                            0,        0,   0
      //                            0,        0,   0 ];
      //------------------------------------------------------------------------
      dJ0 << -c1*dq1, c2*c1*dq2 - s2*s1*dq1, -s1*c2*dq1 - c1*s2*dq2,
             0.0, 0.0, 0.0;
      dJ1 <<     0.0,               -s2*dq2,                -c2*dq2,
             0.0, 0.0, 0.0;
      dJ2.setConstant(0.0);
      break;
    }
    default:
    {
      dterr << "Undefined Euler axis order\n";
      break;
    }
  }

  mJacobianDeriv.col(0) = math::AdT(mT_ChildBodyToJoint, dJ0);
  mJacobianDeriv.col(1) = math::AdT(mT_ChildBodyToJoint, dJ1);
  mJacobianDeriv.col(2) = math::AdT(mT_ChildBodyToJoint, dJ2);

  assert(!math::isNan(mJacobianDeriv));
}

}  // namespace dynamics
}  // namespace dart
