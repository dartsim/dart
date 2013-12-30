/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include <algorithm>
#include <iomanip>

#include "dart/math/Geometry.h"

namespace dart {
namespace math {

Eigen::Quaterniond expToQuat(const Eigen::Vector3d& _v) {
  double mag = _v.norm();

  if (mag > 1e-10) {
    Eigen::Quaterniond q(Eigen::AngleAxisd(mag, _v / mag));
    return q;
  } else {
    Eigen::Quaterniond q(1, 0, 0, 0);
    return q;
  }
}

Eigen::Vector3d quatToExp(const Eigen::Quaterniond& _q) {
  Eigen::AngleAxisd aa(_q);
  Eigen::Vector3d v = aa.axis();
  return v*aa.angle();
}

// Reference:
// http://www.geometrictools.com/LibMathematics/Algebra/Wm5Matrix3.inl
Eigen::Vector3d matrixToEulerXYX(const Eigen::Matrix3d& _R) {
  // +-           -+   +-                                                -+
  // | r00 r01 r02 |   |  cy      sy*sx1               sy*cx1             |
  // | r10 r11 r12 | = |  sy*sx0  cx0*cx1-cy*sx0*sx1  -cy*cx1*sx0-cx0*sx1 |
  // | r20 r21 r22 |   | -sy*cx0  cx1*sx0+cy*cx0*sx1   cy*cx0*cx1-sx0*sx1 |
  // +-           -+   +-                                                -+

  if (_R(0, 0) < 1.0) {
    if (_R(0, 0) > -1.0) {
      // y_angle  = acos(r00)
      // x0_angle = atan2(r10,-r20)
      // x1_angle = atan2(r01,r02)
      double y  = acos(_R(0, 0));
      double x0 = atan2(_R(1, 0), -_R(2, 0));
      double x1 = atan2(_R(0, 1), _R(0, 2));
      // return EA_UNIQUE;
      return Eigen::Vector3d(x0, y, x1);
    } else {
      // Not a unique solution:  x1_angle - x0_angle = atan2(-r12,r11)
      double y  = DART_PI;
      double x0 = -atan2(-_R(1, 2), _R(1, 1));
      double x1 = 0.0;
      // return EA_NOT_UNIQUE_DIF;
      return Eigen::Vector3d(x0, y, x1);
    }
  } else {
    // Not a unique solution:  x1_angle + x0_angle = atan2(-r12,r11)
    double y  = 0.0;
    double x0 = -atan2(-_R(1, 2), _R(1, 1));
    double x1 = 0.0;
    // return EA_NOT_UNIQUE_SUM;
    return Eigen::Vector3d(x0, y, x1);
  }
}

Eigen::Vector3d matrixToEulerXYZ(const Eigen::Matrix3d& _R) {
  // +-           -+   +-                                        -+
  // | r00 r01 r02 |   |  cy*cz           -cy*sz            sy    |
  // | r10 r11 r12 | = |  cz*sx*sy+cx*sz   cx*cz-sx*sy*sz  -cy*sx |
  // | r20 r21 r22 |   | -cx*cz*sy+sx*sz   cz*sx+cx*sy*sz   cx*cy |
  // +-           -+   +-                                        -+

  double x, y, z;

  if (_R(0, 2) > (1.0-DART_EPSILON)) {
    std::cout << "North Pole" << std::endl;
    z = atan2(_R(1, 0), _R(1, 1));
    y = DART_PI_HALF;
    x = 0.0;
    return Eigen::Vector3d(x, y, z);
  }

  if (_R(0, 2) < -(1.0-DART_EPSILON)) {
    std::cout << "South Pole" << std::endl;
    z = atan2(_R(1, 0), _R(1, 1));
    y = -DART_PI_HALF;
    x = 0.0;
    return Eigen::Vector3d(x, y, z);
  }

  z = -atan2(_R(0, 1), _R(0, 0));
  y = asin(_R(0, 2));
  x = -atan2(_R(1, 2), _R(2, 2));

  // order of return is the order of input
  return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d matrixToEulerZYX(const Eigen::Matrix3d& _R) {
  double x, y, z;

  if (_R(2, 0) > (1.0-DART_EPSILON)) {
    std::cout << "North Pole" << std::endl;
    x = atan2(_R(0, 1), _R(0, 2));
    y = -DART_PI_HALF;
    z = 0.0;
    return Eigen::Vector3d(z, y, x);
  }

  if (_R(2, 0) < -(1.0-DART_EPSILON)) {
    std::cout << "South Pole" << std::endl;
    x = atan2(_R(0, 1), _R(0, 2));
    y = DART_PI_HALF;
    z = 0.0;
    return Eigen::Vector3d(z, y, x);
  }

  x = atan2(_R(2, 1), _R(2, 2));
  y = -asin(_R(2, 0));
  z = atan2(_R(1, 0), _R(0, 0));

  // order of return is the order of input
  return Eigen::Vector3d(z, y, x);
}

Eigen::Vector3d matrixToEulerXZY(const Eigen::Matrix3d& _R) {
  double x, y, z;

  if (_R(0, 1) > (1.0-DART_EPSILON)) {
    std::cout << "North Pole" << std::endl;
    y = atan2(_R(1, 2), _R(1, 0));
    z = -DART_PI_HALF;
    x = 0.0;
    return Eigen::Vector3d(x, z, y);
  }

  if (_R(0, 1) < -(1.0-DART_EPSILON)) {
    std::cout << "South Pole" << std::endl;
    y = atan2(_R(1, 2), _R(1, 0));
    z = DART_PI_HALF;
    x = 0.0;
    return Eigen::Vector3d(x, z, y);
  }

  y = atan2(_R(0, 2), _R(0, 0));
  z = -asin(_R(0, 1));
  x = atan2(_R(2, 1), _R(1, 1));

  // order of return is the order of input
  return Eigen::Vector3d(x, z, y);
}

Eigen::Vector3d matrixToEulerYZX(const Eigen::Matrix3d& _R) {
  double x, y, z;

  if (_R(1, 0) > (1.0 - DART_EPSILON)) {
    std::cout << "North Pole" << std::endl;
    x = -atan2(_R(0, 2), _R(0, 1));
    z = DART_PI_HALF;
    y = 0.0;
    return Eigen::Vector3d(y, z, x);
  }

  if (_R(1, 0) < -(1.0 - DART_EPSILON)) {
    std::cout << "South Pole" << std::endl;
    x = -atan2(_R(0, 2), _R(0, 1));
    z = -DART_PI_HALF;
    y = 0.0;
    return Eigen::Vector3d(y, z, x);
  }

  x = -atan2(_R(1, 2), _R(1, 1));
  z = asin(_R(1, 0));
  y = -atan2(_R(2, 0), _R(0, 0));

  // order of return is the order of input
  return Eigen::Vector3d(y, z, x);
}

Eigen::Vector3d matrixToEulerZXY(const Eigen::Matrix3d& _R) {
  double x, y, z;

  if (_R(2, 1) > (1.0-DART_EPSILON)) {
    std::cout << "North Pole" << std::endl;
    y = atan2(_R(0, 2), _R(0, 0));
    x = DART_PI_HALF;
    z = 0.0;
    return Eigen::Vector3d(z, x, y);
  }

  if (_R(2, 1) < -(1.0-DART_EPSILON)) {
    std::cout << "South Pole" << std::endl;
    y = atan2(_R(0, 2), _R(0, 0));
    x = -DART_PI_HALF;
    z = 0.0;
    return Eigen::Vector3d(z, x, y);
  }

  y = -atan2(_R(2, 0), _R(2, 2));
  x = asin(_R(2, 1));
  z = -atan2(_R(0, 1), _R(1, 1));

  // order of return is the order of input
  return Eigen::Vector3d(z, x, y);
}

Eigen::Vector3d matrixToEulerYXZ(const Eigen::Matrix3d& _R) {
  double x, y, z;

  if (_R(1, 2) > (1.0-DART_EPSILON)) {
    std::cout << "North Pole" << std::endl;
    z = -atan2(_R(0, 1), _R(0, 0));
    x = -DART_PI_HALF;
    y = 0.0;
    return Eigen::Vector3d(y, x, z);
  }

  if (_R(1, 2) < -(1.0-DART_EPSILON)) {
    std::cout << "South Pole" << std::endl;
    z = -atan2(_R(0, 1), _R(0, 0));
    x = DART_PI_HALF;
    y = 0.0;
    return Eigen::Vector3d(y, x, z);
  }

  z = atan2(_R(1, 0), _R(1, 1));
  x = -asin(_R(1, 2));
  y = atan2(_R(0, 2), _R(2, 2));

  // order of return is the order of input
  return Eigen::Vector3d(y, x, z);
}

// get the derivative of rotation matrix wrt el no.
Eigen::Matrix3d quatDeriv(const Eigen::Quaterniond& _q, int _el) {
  Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();

  switch (_el) {
    case 0:  // wrt w
      mat(0, 0) = _q.w();
      mat(1, 1) = _q.w();
      mat(2, 2) = _q.w();
      mat(0, 1) = -_q.z();
      mat(1, 0) = _q.z();
      mat(0, 2) = _q.y();
      mat(2, 0) = -_q.y();
      mat(1, 2) = -_q.x();
      mat(2, 1) = _q.x();
      break;
    case 1:  // wrt x
      mat(0, 0) = _q.x();
      mat(1, 1) = -_q.x();
      mat(2, 2) = -_q.x();
      mat(0, 1) = _q.y();
      mat(1, 0) = _q.y();
      mat(0, 2) = _q.z();
      mat(2, 0) = _q.z();
      mat(1, 2) = -_q.w();
      mat(2, 1) = _q.w();
      break;
    case 2:  // wrt y
      mat(0, 0) = -_q.y();
      mat(1, 1) = _q.y();
      mat(2, 2) = -_q.y();
      mat(0, 1) = _q.x();
      mat(1, 0) = _q.x();
      mat(0, 2) = _q.w();
      mat(2, 0) = -_q.w();
      mat(1, 2) = _q.z();
      mat(2, 1) = _q.z();
      break;
    case 3:  // wrt z
      mat(0, 0) = -_q.z();
      mat(1, 1) = -_q.z();
      mat(2, 2) = _q.z();
      mat(0, 1) = -_q.w();
      mat(1, 0) = _q.w();
      mat(0, 2) = _q.x();
      mat(2, 0) = _q.x();
      mat(1, 2) = _q.y();
      mat(2, 1) = _q.y();
      break;
    default:
      break;
  }

  return 2*mat;
}

Eigen::Matrix3d quatSecondDeriv(const Eigen::Quaterniond& _q,
                                int _el1, int _el2) {
  Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();

  if (_el1 == _el2) {  // wrt same dof
    switch (_el1) {
      case 0:  // wrt w
        mat(0, 0) = 1;
        mat(1, 1) = 1;
        mat(2, 2) = 1;
        break;
      case 1:  // wrt x
        mat(0, 0) = 1;
        mat(1, 1) = -1;
        mat(2, 2) = -1;
        break;
      case 2:  // wrt y
        mat(0, 0) = -1;
        mat(1, 1) = 1;
        mat(2, 2) =-1;
        break;
      case 3:  // wrt z
        mat(0, 0) = -1;
        mat(1, 1) = -1;
        mat(2, 2) = 1;
        break;
    }
  } else {  // wrt different dofs
    // arrange in increasing order
    if (_el1 > _el2) {
      int temp = _el2;
      _el2 = _el1;
      _el1 = temp;
    }

    switch (_el1) {
      case 0:  // wrt w
        switch (_el2) {
          case 1:  // wrt x
            mat(1, 2) = -1;
            mat(2, 1) = 1;
            break;
          case 2:  // wrt y
            mat(0, 2) = 1;
            mat(2, 0) = -1;
            break;
          case 3:  // wrt z
            mat(0, 1) = -1;
            mat(1, 0) = 1;
            break;
        }
        break;
      case 1:  // wrt x
        switch (_el2) {
          case 2:  // wrt y
            mat(0, 1) = 1;
            mat(1, 0) = 1;
            break;
          case 3:  // wrt z
            mat(0, 2) = 1;
            mat(2, 0) = 1;
            break;
        }
        break;
      case 2:  // wrt y
        switch (_el2) {
          case 3:  // wrt z
            mat(1, 2) = 1;
            mat(2, 1) = 1;
            break;
        }
        break;
    }
  }

  return 2*mat;
}

Eigen::Vector3d rotatePoint(const Eigen::Quaterniond& _q,
                            const Eigen::Vector3d& _pt) {
  Eigen::Quaterniond quat_pt(0, _pt[0], _pt[1], _pt[2]);
  Eigen::Quaterniond qinv = _q.inverse();

  Eigen::Quaterniond rot = _q*quat_pt*qinv;

  // check below - assuming same format of point achieved
  Eigen::Vector3d temp;
//  VLOG(1) << "Point before: " << 0 << " "
//          << pt.x << " " << pt.y << " " << pt.z << "\n";
//  VLOG(1) << "Point after:  "
//          << rot.x << " " << rot.y << " " << rot.z << " " << rot.w << "\n";
  temp[0] = rot.x();
  temp[1] = rot.y();
  temp[2] = rot.z();

  //  VLOG(1) << "Point after rotation: "
  //          << temp[0] << " " << temp[1] << " " << temp[2] << endl;
  return temp;
}

Eigen::Vector3d rotatePoint(const Eigen::Quaterniond& _q,
                            double _x, double _y, double _z) {
  Eigen::Vector3d pt(_x, _y, _z);
  return rotatePoint(_q, pt);
}

// ----------- expmap computations -------------

#define EPSILON_EXPMAP_THETA 1.0e-3

Eigen::Matrix3d expMapRot(const Eigen::Vector3d &_q) {
  double theta = _q.norm();

  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d qss =  math::makeSkewSymmetric(_q);
  Eigen::Matrix3d qss2 =  qss*qss;

  if (theta < EPSILON_EXPMAP_THETA)
    R = Eigen::Matrix3d::Identity() + qss + 0.5*qss2;
  else
    R = Eigen::Matrix3d::Identity()
        + (sin(theta)/theta)*qss
        + ((1-cos(theta))/(theta*theta))*qss2;

  return R;
}

Eigen::Matrix3d expMapJac(const Eigen::Vector3d &_q) {
  double theta = _q.norm();

  Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d qss =  math::makeSkewSymmetric(_q);
  Eigen::Matrix3d qss2 =  qss*qss;

  if (theta < EPSILON_EXPMAP_THETA)
    J = Eigen::Matrix3d::Identity() + 0.5*qss +  (1.0/6.0)*qss2;
  else
    J = Eigen::Matrix3d::Identity()
        + ((1-cos(theta))/(theta*theta))*qss
        + ((theta-sin(theta))/(theta*theta*theta))*qss2;

  return J;
}

Eigen::Matrix3d expMapJacDot(const Eigen::Vector3d& _q,
                             const Eigen::Vector3d& _qdot) {
  double theta = _q.norm();

  Eigen::Matrix3d Jdot = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d qss  = math::makeSkewSymmetric(_q);
  Eigen::Matrix3d qss2 = qss*qss;
  Eigen::Matrix3d qdss = math::makeSkewSymmetric(_qdot);
  double ttdot = _q.dot(_qdot);   // theta*thetaDot
  double st = sin(theta);
  double ct = cos(theta);
  double t2 = theta*theta;
  double t3 = t2*theta;
  double t4 = t3*theta;
  double t5 = t4*theta;

  if (theta < EPSILON_EXPMAP_THETA) {
    Jdot = 0.5*qdss + (1.0/6.0)*(qss*qdss + qdss*qss);
    Jdot += (-1.0/12)*ttdot*qss + (-1.0/60)*ttdot*qss2;
  } else {
    Jdot = ((1-ct)/t2)*qdss + ((theta-st)/t3)*(qss*qdss + qdss*qss);
    Jdot += ((theta*st + 2*ct - 2)/t4)*ttdot*qss
            + ((3*st - theta*ct - 2*theta)/t5)*ttdot*qss2;
  }

  return Jdot;
}

Eigen::Matrix3d expMapJacDeriv(const Eigen::Vector3d& _q, int _qi ) {
  assert(_qi >= 0 && _qi <= 2);

  Eigen::Vector3d qdot = Eigen::Vector3d::Zero();
  qdot[_qi] = 1.0;
  return expMapJacDot(_q, qdot);
}

// Vec3 AdInvTLinear(const SE3& T, const Vec3& v)
// {
//     return Vec3(T(0,0)*v[0] + T(1,0)*v[1] + T(2,0)*v[2],
//                 T(0,1)*v[0] + T(1,1)*v[1] + T(2,1)*v[2],
//                 T(0,2)*v[0] + T(1,2)*v[1] + T(2,2)*v[2]);
// }

// Vec3 ad_Vec3_se3(const Vec3& s1, const se3& s2)
// {
//     Vec3 ret;

//     ret << s2[2]*s1[1] - s2[1]*s1[2],
//            s2[0]*s1[2] - s2[2]*s1[0],
//            s2[1]*s1[0] - s2[0]*s1[1];

//     return ret;
// }

Eigen::Vector3d logMap(const Eigen::Matrix3d& _R) {
  //--------------------------------------------------------------------------
  // T = (R, p) = exp([w, v]), t = ||w||
  // v = beta*p + gamma*w + 1 / 2*cross(p, w)
  //    , beta = t*(1 + cos(t)) / (2*sin(t)), gamma = <w, p>*(1 - beta) / t^2
  //--------------------------------------------------------------------------
  double theta =
      std::acos(
        std::max(
          std::min(0.5 * (_R(0, 0) + _R(1, 1) + _R(2, 2) - 1.0), 1.0), -1.0));

  if (theta > DART_PI - DART_EPSILON) {
    double delta = 0.5 + 0.125*(DART_PI - theta)*(DART_PI - theta);

    return Eigen::Vector3d(
          _R(2, 1) > _R(1, 2) ? theta*sqrt(1.0 + (_R(0, 0) - 1.0)*delta) :
                             -theta*sqrt(1.0 + (_R(0, 0) - 1.0)*delta),
          _R(0, 2) > _R(2, 0) ? theta*sqrt(1.0 + (_R(1, 1) - 1.0)*delta) :
                             -theta*sqrt(1.0 + (_R(1, 1) - 1.0)*delta),
          _R(1, 0) > _R(0, 1) ? theta*sqrt(1.0 + (_R(2, 2) - 1.0)*delta) :
                             -theta*sqrt(1.0 + (_R(2, 2) - 1.0)*delta));
  } else {
    double alpha = 0.0;

    if (theta > DART_EPSILON)
      alpha = 0.5*theta / sin(theta);
    else
      alpha = 0.5 + DART_1_12*theta*theta;

    return Eigen::Vector3d(alpha*(_R(2, 1) - _R(1, 2)),
                           alpha*(_R(0, 2) - _R(2, 0)),
                           alpha*(_R(1, 0) - _R(0, 1)));
  }
}

Eigen::Vector6d logMap(const Eigen::Isometry3d& _T) {
  //--------------------------------------------------------------------------
  // T = (R, p) = exp([w, v]), t = ||w||
  // v = beta*p + gamma*w + 1 / 2*cross(p, w)
  //    , beta = t*(1 + cos(t)) / (2*sin(t)), gamma = <w, p>*(1 - beta) / t^2
  //--------------------------------------------------------------------------
  double theta =
      std::acos(
        std::max(
          std::min(0.5 * (_T(0, 0) + _T(1, 1) + _T(2, 2) - 1.0), 1.0), -1.0));
  double alpha;
  double beta;
  double gamma;
  Eigen::Vector6d ret;

  if (theta > DART_PI - DART_EPSILON) {
    const double c1 = 0.10132118364234;  // 1 / pi^2
    const double c2 = 0.01507440267955;  // 1 / 4 / pi - 2 / pi^3
    const double c3 = 0.00546765085347;  // 3 / pi^4 - 1 / 4 / pi^2

    double phi = DART_PI - theta;
    double delta = 0.5 + 0.125*phi*phi;

    double w[] = { _T(2, 1) > _T(1, 2)
                   ? theta*sqrt(1.0 + (_T(0, 0) - 1.0)*delta)
                   : -theta*sqrt(1.0 + (_T(0, 0) - 1.0)*delta),
                   _T(0, 2) > _T(2, 0)
                   ? theta*sqrt(1.0 + (_T(1, 1) - 1.0)*delta)
                   : -theta*sqrt(1.0 + (_T(1, 1) - 1.0)*delta),
                   _T(1, 0) > _T(0, 1)
                   ? theta*sqrt(1.0 + (_T(2, 2) - 1.0)*delta)
                   : -theta*sqrt(1.0 + (_T(2, 2) - 1.0)*delta) };

    beta = 0.25*theta*(DART_PI - theta);
    gamma = (w[0]*_T(0, 3) + w[1]*_T(1, 3) + w[2]*_T(2, 3))
            * (c1 -  c2*phi + c3*phi*phi);

    ret << w[0],
        w[1],
        w[2],
        beta*_T(0, 3) - 0.5*(w[1]*_T(2, 3) - w[2]*_T(1, 3)) + gamma*w[0],
        beta*_T(1, 3) - 0.5*(w[2]*_T(0, 3) - w[0]*_T(2, 3)) + gamma*w[1],
        beta*_T(2, 3) - 0.5*(w[0]*_T(1, 3) - w[1]*_T(0, 3)) + gamma*w[2];
  } else {
    if (theta > DART_EPSILON) {
      alpha = 0.5*theta / sin(theta);
      beta = (1.0 + cos(theta))*alpha;
      gamma = (1.0 - beta) / theta / theta;
    } else {
      alpha = 0.5 + DART_1_12*theta*theta;
      beta = 1.0 - DART_1_12*theta*theta;
      gamma = DART_1_12 + DART_1_720*theta*theta;
    }

    double w[] = { alpha*(_T(2, 1) - _T(1, 2)),
                   alpha*(_T(0, 2) - _T(2, 0)),
                   alpha*(_T(1, 0) - _T(0, 1)) };
    gamma *= w[0]*_T(0, 3) + w[1]*_T(1, 3) + w[2]*_T(2, 3);

    ret << w[0],
        w[1],
        w[2],
        beta*_T(0, 3) + 0.5*(w[2]*_T(1, 3) - w[1]*_T(2, 3)) + gamma*w[0],
        beta*_T(1, 3) + 0.5*(w[0]*_T(2, 3) - w[2]*_T(0, 3)) + gamma*w[1],
        beta*_T(2, 3) + 0.5*(w[1]*_T(0, 3) - w[0]*_T(1, 3)) + gamma*w[2];
  }

  return ret;
}

// res = T * s * Inv(T)
Eigen::Vector6d AdT(const Eigen::Isometry3d& _T, const Eigen::Vector6d& _V) {
  //--------------------------------------------------------------------------
  // w' = R*w
  // v' = p x R*w + R*v
  //--------------------------------------------------------------------------
  Eigen::Vector6d res;
  res.head<3>().noalias() = _T.linear() * _V.head<3>();
  res.tail<3>().noalias() = _T.linear() * _V.tail<3>() +
                            _T.translation().cross(res.head<3>());
  return res;
}

Eigen::Vector6d AdR(const Eigen::Isometry3d& _T, const Eigen::Vector6d& _V) {
  //--------------------------------------------------------------------------
  // w' = R*w
  // v' = R*v
  //--------------------------------------------------------------------------
  Eigen::Vector6d res;
  res.head<3>().noalias() = _T.linear() * _V.head<3>();
  res.tail<3>().noalias() = _T.linear() * _V.tail<3>();
  return res;
}

Eigen::Vector6d AdTAngular(const Eigen::Isometry3d& _T,
                           const Eigen::Vector3d& _w) {
  //--------------------------------------------------------------------------
  // w' = R*w
  // v' = p x R*w
  //--------------------------------------------------------------------------
  Eigen::Vector6d res;
  res.head<3>().noalias() = _T.linear() * _w;
  res.tail<3>() = _T.translation().cross(res.head<3>());
  return res;
}

Eigen::Vector6d AdTLinear(const Eigen::Isometry3d& _T,
                          const Eigen::Vector3d& _v) {
  //--------------------------------------------------------------------------
  // w' = 0
  // v' = R*v
  //--------------------------------------------------------------------------
  Eigen::Vector6d res = Eigen::Vector6d::Zero();
  res.tail<3>().noalias() = _T.linear() * _v;
  return res;
}

Jacobian AdTJac(const Eigen::Isometry3d& _T, const Jacobian& _J) {
  Jacobian res = Jacobian::Zero(6, _J.cols());
//  res.topRows<3>().noalias() = T.linear() * J.topRows<3>();
//  res.bottomRows<3>().noalias()
//      = -res.topRows<3>().colwise().cross(T.translation())
//        + T.linear() * J.bottomRows<3>();
  for (int i = 0; i < _J.cols(); ++i)
    res.col(i) = AdT(_T, _J.col(i));
  return res;
}

// se3 AdP(const Vec3& p, const se3& s)
// {
//  //--------------------------------------------------------------------------
//  // w' = w
//  // v' = p x w + v
//  //--------------------------------------------------------------------------
//  se3 ret;
//  ret << s[0],
//      s[1],
//      s[2],
//      p[1]*s[2] - p[2]*s[1] + s[3],
//      p[2]*s[0] - p[0]*s[2] + s[4],
//      p[0]*s[1] - p[1]*s[0] + s[5];

//  return ret;
// }

// Jacobian AdRJac(const SE3& T, const Jacobian& J)
// {
//     Jacobian AdTJ(6,J.cols());

//     for (int i = 0; i < J.cols(); ++i)
//     {
//         AdTJ.col(i) = math::AdR(T, J.col(i));
//     }

//     return AdTJ;
// }

// re = Inv(T)*s*T
Eigen::Vector6d AdInvT(const Eigen::Isometry3d& _T, const Eigen::Vector6d& _V) {
  Eigen::Vector6d res;
  res.head<3>().noalias() = _T.linear().transpose() * _V.head<3>();
  res.tail<3>().noalias() =
      _T.linear().transpose()
      * (_V.tail<3>() + _V.head<3>().cross(_T.translation()));
  return res;
}

Jacobian AdInvTJac(const Eigen::Isometry3d& _T, const Jacobian& _J) {
  Jacobian res = Jacobian::Zero(6, _J.cols());
//  res.topRows<3>().noalias()    = T.linear().transpose() * J.topRows<3>();
//  res.bottomRows<3>().noalias()
//      = T.linear().transpose()
//      * (J.bottomRows<3>() + J.topRows<3>().colwise().cross(T.translation()));
  for (int i = 0; i < _J.cols(); ++i)
    res.col(i) = AdInvT(_T, _J.col(i));
  return res;
}

// se3 AdInvR(const SE3& T, const se3& s)
// {
//     se3 ret;

//     ret << T(0,0)*s[0] + T(1,0)*s[1] + T(2,0)*s[2],
//                 T(0,1)*s[0] + T(1,1)*s[1] + T(2,1)*s[2],
//                 T(0,2)*s[0] + T(1,2)*s[1] + T(2,2)*s[2],
//                 T(0,0)*s[3] + T(1,0)*s[4] + T(2,0)*s[5],
//                 T(0,1)*s[3] + T(1,1)*s[4] + T(2,1)*s[5],
//                 T(0,2)*s[3] + T(1,2)*s[4] + T(2,2)*s[5];

//     return ret;
// }

Eigen::Vector6d AdInvRLinear(const Eigen::Isometry3d& _T,
                             const Eigen::Vector3d& _v) {
  Eigen::Vector6d res = Eigen::Vector6d::Zero();
  res.tail<3>().noalias() = _T.linear().transpose() * _v;
  return res;
}

Eigen::Vector6d ad(const Eigen::Vector6d& _X, const Eigen::Vector6d& _Y) {
  //--------------------------------------------------------------------------
  // ad(s1, s2) = | [w1]    0 | | w2 |
  //              | [v1] [w1] | | v2 |
  //
  //            = |          [w1]w2 |
  //              | [v1]w2 + [w1]v2 |
  //--------------------------------------------------------------------------
  Eigen::Vector6d res;
  res.head<3>() = _X.head<3>().cross(_Y.head<3>());
  res.tail<3>() = _X.head<3>().cross(_Y.tail<3>()) +
                  _X.tail<3>().cross(_Y.head<3>());
  return res;
}

Eigen::Vector6d dAdT(const Eigen::Isometry3d& _T, const Eigen::Vector6d& _F) {
  Eigen::Vector6d res;
  res.head<3>().noalias() =
      _T.linear().transpose()
      * (_F.head<3>() + _F.tail<3>().cross(_T.translation()));
  res.tail<3>().noalias() = _T.linear().transpose() * _F.tail<3>();
  return res;
}

// dse3 dAdTLinear(const SE3& T, const Vec3& v)
// {
//     dse3 ret;
//     double tmp[3] = { - T(1,3)*v[2] + T(2,3)*v[1],
//                       - T(2,3)*v[0] + T(0,3)*v[2],
//                       - T(0,3)*v[1] + T(1,3)*v[0] };
//     ret << T(0,0)*tmp[0] + T(1,0)*tmp[1] + T(2,0)*tmp[2],
//                 T(0,1)*tmp[0] + T(1,1)*tmp[1] + T(2,1)*tmp[2],
//                 T(0,2)*tmp[0] + T(1,2)*tmp[1] + T(2,2)*tmp[2],
//                 T(0,0)*v[0] + T(1,0)*v[1] + T(2,0)*v[2],
//                 T(0,1)*v[0] + T(1,1)*v[1] + T(2,1)*v[2],
//                 T(0,2)*v[0] + T(1,2)*v[1] + T(2,2)*v[2];

//     return ret;
// }

Eigen::Vector6d dAdInvT(const Eigen::Isometry3d& _T,
                        const Eigen::Vector6d& _F) {
  Eigen::Vector6d res;
  res.tail<3>().noalias() = _T.linear() * _F.tail<3>();
  res.head<3>().noalias() = _T.linear() * _F.head<3>();
  res.head<3>() += _T.translation().cross(res.tail<3>());
  return res;
}

Jacobian dAdInvTJac(const Eigen::Isometry3d& _T, const Jacobian& _J) {
  math::Jacobian res = math::Jacobian::Zero(6, _J.cols());
//  res.bottomRows<3>().noalias() = T.linear() * J.bottomRows<3>();
//  res.topRows<3>().noalias()    = T.linear() * J.topRows<3>();
//  res.topRows<3>() -= res.bottomRows<3>().colwise().cross(T.translation());

  for (int i = 0; i < _J.cols(); ++i)
    res.col(i) = dAdInvT(_T, _J.col(i));

  return res;
}

Eigen::Vector6d dAdInvR(const Eigen::Isometry3d& _T,
                        const Eigen::Vector6d& _F) {
  Eigen::Vector6d res;
  res.head<3>().noalias() = _T.linear() * _F.head<3>();
  res.tail<3>().noalias() = _T.linear() * _F.tail<3>();
  return res;
}

// dse3 dAdInvPLinear(const Vec3& p, const Vec3& f)
// {
//     dse3 ret;

//     ret << p[1]*f[2] - p[2]*f[1],
//                 p[2]*f[0] - p[0]*f[2],
//                 p[0]*f[1] - p[1]*f[0],
//                 f[0],
//                 f[1],
//                 f[2];

//     return ret;
// }

// Reference:
// http://www.geometrictools.com/LibMathematics/Algebra/Wm5Matrix3.inl
Eigen::Matrix3d eulerXYXToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                                -+
  // | r00 r01 r02 |   |  cy      sy*sx1               sy*cx1             |
  // | r10 r11 r12 | = |  sy*sx0  cx0*cx1-cy*sx0*sx1  -cy*cx1*sx0-cx0*sx1 |
  // | r20 r21 r22 |   | -sy*cx0  cx1*sx0+cy*cx0*sx1   cy*cx0*cx1-sx0*sx1 |
  // +-           -+   +-                                                -+

  Eigen::Matrix3d ret;

  double cx0 = cos(_angle(0));
  double sx0 = sin(_angle(0));
  double cy  = cos(_angle(1));
  double sy  = sin(_angle(1));
  double cx1 = cos(_angle(2));
  double sx1 = sin(_angle(2));

  ret(0, 0) =  cy;
  ret(1, 0) =  sy*sx0;
  ret(2, 0) = -sy*cx0;

  ret(0, 1) = sy*sx1;
  ret(1, 1) = cx0*cx1-cy*sx0*sx1;
  ret(2, 1) = cx1*sx0+cy*cx0*sx1;

  ret(0, 2) =  sy*cx1;
  ret(1, 2) = -cy*cx1*sx0-cx0*sx1;
  ret(2, 2) =  cy*cx0*cx1-sx0*sx1;

  return ret;
}

Eigen::Matrix3d eulerXYZToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                        -+
  // | r00 r01 r02 |   |  cy*cz           -cy*sz            sy    |
  // | r10 r11 r12 | = |  cz*sx*sy+cx*sz   cx*cz-sx*sy*sz  -cy*sx |
  // | r20 r21 r22 |   | -cx*cz*sy+sx*sz   cz*sx+cx*sy*sz   cx*cy |
  // +-           -+   +-                                        -+

  Eigen::Matrix3d ret;

  double cx = cos(_angle[0]);
  double sx = sin(_angle[0]);
  double cy = cos(_angle[1]);
  double sy = sin(_angle[1]);
  double cz = cos(_angle[2]);
  double sz = sin(_angle[2]);

  ret(0, 0) = cy*cz;
  ret(1, 0) = cx*sz + cz*sx*sy;
  ret(2, 0) = sx*sz - cx*cz*sy;

  ret(0, 1) = -cy*sz;
  ret(1, 1) =  cx*cz - sx*sy*sz;
  ret(2, 1) =  cz*sx + cx*sy*sz;

  ret(0, 2) =  sy;
  ret(1, 2) = -cy*sx;
  ret(2, 2) =  cx*cy;

  return ret;
}

Eigen::Matrix3d eulerXZXToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                                -+
  // | r00 r01 r02 |   | cz      -sz*cx1               sz*sx1             |
  // | r10 r11 r12 | = | sz*cx0   cz*cx0*cx1-sx0*sx1  -cx1*sx0-cz*cx0*sx1 |
  // | r20 r21 r22 |   | sz*sx0   cz*cx1*sx0+cx0*sx1   cx0*cx1-cz*sx0*sx1 |
  // +-           -+   +-                                                -+

  Eigen::Matrix3d ret;

  double cx0 = cos(_angle(0));
  double sx0 = sin(_angle(0));
  double cz  = cos(_angle(1));
  double sz  = sin(_angle(1));
  double cx1 = cos(_angle(2));
  double sx1 = sin(_angle(2));

  ret(0, 0) = cz;
  ret(1, 0) = sz*cx0;
  ret(2, 0) = sz*sx0;

  ret(0, 1) = -sz*cx1;
  ret(1, 1) =  cz*cx0*cx1-sx0*sx1;
  ret(2, 1) =  cz*cx1*sx0+cx0*sx1;

  ret(0, 2) =  sz*sx1;
  ret(1, 2) = -cx1*sx0-cz*cx0*sx1;
  ret(2, 2) =  cx0*cx1-cz*sx0*sx1;

  return ret;
}

Eigen::Matrix3d eulerXZYToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                        -+
  // | r00 r01 r02 |   |  cy*cz           -sz      cz*sy          |
  // | r10 r11 r12 | = |  sx*sy+cx*cy*sz   cx*cz  -cy*sx+cx*sy*sz |
  // | r20 r21 r22 |   | -cx*sy+cy*sx*sz   cz*sx   cx*cy+sx*sy*sz |
  // +-           -+   +-                                        -+

  Eigen::Matrix3d ret;

  double cx = cos(_angle(0));
  double sx = sin(_angle(0));
  double cz = cos(_angle(1));
  double sz = sin(_angle(1));
  double cy = cos(_angle(2));
  double sy = sin(_angle(2));

  ret(0, 0) =  cy*cz;
  ret(1, 0) =  sx*sy+cx*cy*sz;
  ret(2, 0) = -cx*sy+cy*sx*sz;

  ret(0, 1) = -sz;
  ret(1, 1) =  cx*cz;
  ret(2, 1) =  cz*sx;

  ret(0, 2) =  cz*sy;
  ret(1, 2) = -cy*sx+cx*sy*sz;
  ret(2, 2) =  cx*cy+sx*sy*sz;

  return ret;
}

Eigen::Matrix3d eulerYXYToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                                -+
  // | r00 r01 r02 |   |  cy0*cy1-cx*sy0*sy1  sx*sy0   cx*cy1*sy0+cy0*sy1 |
  // | r10 r11 r12 | = |  sx*sy1              cx      -sx*cy1             |
  // | r20 r21 r22 |   | -cy1*sy0-cx*cy0*sy1  sx*cy0   cx*cy0*cy1-sy0*sy1 |
  // +-           -+   +-                                                -+

  Eigen::Matrix3d ret;

  double cy0 = cos(_angle(0));
  double sy0 = sin(_angle(0));
  double cx  = cos(_angle(1));
  double sx  = sin(_angle(1));
  double cy1 = cos(_angle(2));
  double sy1 = sin(_angle(2));

  ret(0, 0) =  cy0*cy1-cx*sy0*sy1;
  ret(1, 0) =  sx*sy1;
  ret(2, 0) = -cy1*sy0-cx*cy0*sy1;

  ret(0, 1) = sx*sy0;
  ret(1, 1) = cx;
  ret(2, 1) = sx*cy0;

  ret(0, 2) =  cx*cy1*sy0+cy0*sy1;
  ret(1, 2) = -sx*cy1;
  ret(2, 2) =  cx*cy0*cy1-sy0*sy1;

  return ret;
}

Eigen::Matrix3d eulerYXZToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                       -+
  // | r00 r01 r02 |   |  cy*cz+sx*sy*sz  cz*sx*sy-cy*sz   cx*sy |
  // | r10 r11 r12 | = |  cx*sz           cx*cz           -sx    |
  // | r20 r21 r22 |   | -cz*sy+cy*sx*sz  cy*cz*sx+sy*sz   cx*cy |
  // +-           -+   +-                                       -+

  Eigen::Matrix3d ret;

  double cy = cos(_angle(0));
  double sy = sin(_angle(0));
  double cx = cos(_angle(1));
  double sx = sin(_angle(1));
  double cz = cos(_angle(2));
  double sz = sin(_angle(2));

  ret(0, 0) =  cy*cz+sx*sy*sz;  ret(0, 1) = cz*sx*sy-cy*sz;  ret(0, 2) =  cx*sy;
  ret(1, 0) =  cx*sz;           ret(1, 1) = cx*cz;           ret(1, 2) = -sx;
  ret(2, 0) = -cz*sy+cy*sx*sz;  ret(2, 1) = cy*cz*sx+sy*sz;  ret(2, 2) =  cx*cy;

  return ret;
}

Eigen::Matrix3d eulerYZXToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                       -+
  // | r00 r01 r02 |   |  cy*cz  sx*sy-cx*cy*sz   cx*sy+cy*sx*sz |
  // | r10 r11 r12 | = |  sz     cx*cz           -cz*sx          |
  // | r20 r21 r22 |   | -cz*sy  cy*sx+cx*sy*sz   cx*cy-sx*sy*sz |
  // +-           -+   +-                                       -+

  Eigen::Matrix3d ret;

  double cy = cos(_angle(0));
  double sy = sin(_angle(0));
  double cz = cos(_angle(1));
  double sz = sin(_angle(1));
  double cx = cos(_angle(2));
  double sx = sin(_angle(2));

  ret(0, 0) =  cy*cz;  ret(0, 1) = sx*sy-cx*cy*sz;  ret(0, 2) =  cx*sy+cy*sx*sz;
  ret(1, 0) =  sz;     ret(1, 1) = cx*cz;           ret(1, 2) = -cz*sx;
  ret(2, 0) = -cz*sy;  ret(2, 1) = cy*sx+cx*sy*sz;  ret(2, 2) =  cx*cy-sx*sy*sz;

  return ret;
}

Eigen::Matrix3d eulerYZYToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                                -+
  // | r00 r01 r02 |   |  cz*cy0*cy1-sy0*sy1  -sz*cy0  cy1*sy0+cz*cy0*sy1 |
  // | r10 r11 r12 | = |  sz*cy1               cz      sz*sy1             |
  // | r20 r21 r22 |   | -cz*cy1*sy0-cy0*sy1   sz*sy0  cy0*cy1-cz*sy0*sy1 |
  // +-           -+   +-                                                -+

  Eigen::Matrix3d ret;

  double cy0 = cos(_angle(0));
  double sy0 = sin(_angle(0));
  double cz  = cos(_angle(1));
  double sz  = sin(_angle(1));
  double cy1 = cos(_angle(2));
  double sy1 = sin(_angle(2));

  ret(0, 0) =  cz*cy0*cy1-sy0*sy1;
  ret(1, 0) =  sz*cy1;
  ret(2, 0) = -cz*cy1*sy0-cy0*sy1;

  ret(0, 1) = -sz*cy0;
  ret(1, 1) =  cz;
  ret(2, 1) =  sz*sy0;

  ret(0, 2) = cy1*sy0+cz*cy0*sy1;
  ret(1, 2) = sz*sy1;
  ret(2, 2) = cy0*cy1-cz*sy0*sy1;

  return ret;
}

Eigen::Matrix3d eulerZXYToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                        -+
  // | r00 r01 r02 |   |  cy*cz-sx*sy*sz  -cx*sz   cz*sy+cy*sx*sz |
  // | r10 r11 r12 | = |  cz*sx*sy+cy*sz   cx*cz  -cy*cz*sx+sy*sz |
  // | r20 r21 r22 |   | -cx*sy            sx      cx*cy          |
  // +-           -+   +-                                        -+

  Eigen::Matrix3d ret;

  double cz = cos(_angle(0));
  double sz = sin(_angle(0));
  double cx = cos(_angle(1));
  double sx = sin(_angle(1));
  double cy = cos(_angle(2));
  double sy = sin(_angle(2));

  ret(0, 0) =  cy*cz-sx*sy*sz;
  ret(1, 0) =  cz*sx*sy+cy*sz;
  ret(2, 0) = -cx*sy;

  ret(0, 1) = -cx*sz;
  ret(1, 1) =  cx*cz;
  ret(2, 1) =  sx;

  ret(0, 2) =  cz*sy+cy*sx*sz;
  ret(1, 2) = -cy*cz*sx+sy*sz;
  ret(2, 2) =  cx*cy;

  return ret;
}

Eigen::Matrix3d eulerZYXToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                      -+
  // | r00 r01 r02 |   |  cy*cz  cz*sx*sy-cx*sz  cx*cz*sy+sx*sz |
  // | r10 r11 r12 | = |  cy*sz  cx*cz+sx*sy*sz -cz*sx+cx*sy*sz |
  // | r20 r21 r22 |   | -sy     cy*sx           cx*cy          |
  // +-           -+   +-                                      -+

  Eigen::Matrix3d ret;

  double cz = cos(_angle[0]);
  double sz = sin(_angle[0]);
  double cy = cos(_angle[1]);
  double sy = sin(_angle[1]);
  double cx = cos(_angle[2]);
  double sx = sin(_angle[2]);

  ret(0, 0) =  cz*cy;
  ret(1, 0) =  sz*cy;
  ret(2, 0) = -sy;

  ret(0, 1) = cz*sy*sx - sz*cx;
  ret(1, 1) = sz*sy*sx + cz*cx;
  ret(2, 1) = cy*sx;

  ret(0, 2) = cz*sy*cx + sz*sx;
  ret(1, 2) = sz*sy*cx - cz*sx;
  ret(2, 2) = cy*cx;

  return ret;
}

Eigen::Matrix3d eulerZXZToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                                -+
  // | r00 r01 r02 |   | cz0*cz1-cx*sz0*sz1  -cx*cz1*sz0-cz0*sz1   sx*sz0 |
  // | r10 r11 r12 | = | cz1*sz0+cx*cz0*sz1   cx*cz0*cz1-sz0*sz1  -sz*cz0 |
  // | r20 r21 r22 |   | sx*sz1               sx*cz1               cx     |
  // +-           -+   +-                                                -+

  Eigen::Matrix3d ret;

  double cz0 = cos(_angle(0));
  double sz0 = sin(_angle(0));
  double cx  = cos(_angle(1));
  double sx  = sin(_angle(1));
  double cz1 = cos(_angle(2));
  double sz1 = sin(_angle(2));

  ret(0, 0) = cz0*cz1-cx*sz0*sz1;
  ret(1, 0) = cz1*sz0+cx*cz0*sz1;
  ret(2, 0) = sx*sz1;

  ret(0, 1) = -cx*cz1*sz0-cz0*sz1;
  ret(1, 1) =  cx*cz0*cz1-sz0*sz1;
  ret(2, 1) =  sx*cz1;

  ret(0, 2) =  sx*sz0;
  ret(1, 2) = -sx*cz0;
  ret(2, 2) =  cx;

  return ret;
}

Eigen::Matrix3d eulerZYZToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                                -+
  // | r00 r01 r02 |   |  cy*cz0*cz1-sz0*sz1  -cz1*sz0-cy*cz0*sz1  sy*cz0 |
  // | r10 r11 r12 | = |  cy*cz1*sz0+cz0*sz1   cz0*cz1-cy*sz0*sz1  sy*sz0 |
  // | r20 r21 r22 |   | -sy*cz1               sy*sz1              cy     |
  // +-           -+   +-                                                -+

  Eigen::Matrix3d ret = Eigen::Matrix3d::Identity();

  double cz0 = cos(_angle[0]);
  double sz0 = sin(_angle[0]);
  double cy = cos(_angle[1]);
  double sy = sin(_angle[1]);
  double cz1 = cos(_angle[2]);
  double sz1 = sin(_angle[2]);

  ret(0, 0) =  cz0*cy*cz1 - sz0*sz1;
  ret(1, 0) =  sz0*cy*cz1 + cz0*sz1;
  ret(2, 0) = -sy*cz1;

  ret(0, 1) = -cz0*cy*sz1 - sz0*cz1;
  ret(1, 1) = cz0*cz1 - sz0*cy*sz1;
  ret(2, 1) = sy*sz1;

  ret(0, 2) = cz0*sy;
  ret(1, 2) = sz0*sy;
  ret(2, 2) = cy;

  return ret;
}

// R = Exp(w)
// p = sin(t) / t*v + (t - sin(t)) / t^3*<w, v>*w + (1 - cos(t)) / t^2*(w X v)
// , when S = (w, v), t = |w|
Eigen::Isometry3d expMap(const Eigen::Vector6d& _S) {
  Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
  double s2[] = { _S[0]*_S[0], _S[1]*_S[1], _S[2]*_S[2] };
  double s3[] = { _S[0]*_S[1], _S[1]*_S[2], _S[2]*_S[0] };
  double theta = sqrt(s2[0] + s2[1] + s2[2]);
  double cos_t = cos(theta), alpha, beta, gamma;

  if (theta > DART_EPSILON) {
    double sin_t = sin(theta);
    alpha = sin_t / theta;
    beta = (1.0 - cos_t) / theta / theta;
    gamma = (_S[0]*_S[3] + _S[1]*_S[4] + _S[2]*_S[5])
            * (theta - sin_t) / theta / theta / theta;
  } else {
    alpha = 1.0 - theta*theta/6.0;
    beta = 0.5 - theta*theta/24.0;
    gamma = (_S[0]*_S[3] + _S[1]*_S[4] + _S[2]*_S[5])/6.0 - theta*theta/120.0;
  }

  ret(0, 0) = beta*s2[0] + cos_t;
  ret(1, 0) = beta*s3[0] + alpha*_S[2];
  ret(2, 0) = beta*s3[2] - alpha*_S[1];

  ret(0, 1) = beta*s3[0] - alpha*_S[2];
  ret(1, 1) = beta*s2[1] + cos_t;
  ret(2, 1) = beta*s3[1] + alpha*_S[0];

  ret(0, 2) = beta*s3[2] + alpha*_S[1];
  ret(1, 2) = beta*s3[1] - alpha*_S[0];
  ret(2, 2) = beta*s2[2] + cos_t;

  ret(0, 3) = alpha*_S[3] + beta*(_S[1]*_S[5] - _S[2]*_S[4]) + gamma*_S[0];
  ret(1, 3) = alpha*_S[4] + beta*(_S[2]*_S[3] - _S[0]*_S[5]) + gamma*_S[1];
  ret(2, 3) = alpha*_S[5] + beta*(_S[0]*_S[4] - _S[1]*_S[3]) + gamma*_S[2];

  return ret;
}

// I + sin(t) / t*[S] + (1 - cos(t)) / t^2*[S]^2, where t = |S|
Eigen::Isometry3d expAngular(const Eigen::Vector3d& _s) {
  Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
  double s2[] = { _s[0]*_s[0], _s[1]*_s[1], _s[2]*_s[2] };
  double s3[] = { _s[0]*_s[1], _s[1]*_s[2], _s[2]*_s[0] };
  double theta = sqrt(s2[0] + s2[1] + s2[2]);
  double cos_t = cos(theta);
  double alpha = 0.0;
  double beta = 0.0;

  if (theta > DART_EPSILON) {
    alpha = sin(theta) / theta;
    beta = (1.0 - cos_t) / theta / theta;
  } else {
    alpha = 1.0 - theta*theta/6.0;
    beta = 0.5 - theta*theta/24.0;
  }

  ret(0, 0) = beta*s2[0] + cos_t;
  ret(1, 0) = beta*s3[0] + alpha*_s[2];
  ret(2, 0) = beta*s3[2] - alpha*_s[1];

  ret(0, 1) = beta*s3[0] - alpha*_s[2];
  ret(1, 1) = beta*s2[1] + cos_t;
  ret(2, 1) = beta*s3[1] + alpha*_s[0];

  ret(0, 2) = beta*s3[2] + alpha*_s[1];
  ret(1, 2) = beta*s3[1] - alpha*_s[0];
  ret(2, 2) = beta*s2[2] + cos_t;

  return ret;
}

// SE3 Normalize(const SE3& T)
// {
//    SE3 ret = SE3::Identity();
//    double idet = 1.0 / (T(0,0)*(T(1,1)*T(2,2) - T(2,1)*T(1,2)) +
//                              T(0,1)*(T(2,0)*T(1,2) - T(1,0)*T(2,2)) +
//                              T(0,2)*(T(1,0)*T(2,1) - T(2,0)*T(1,1)));

//    ret(0,0) = 1.0_2*(T(0,0) + idet*(T(1,1)*T(2,2) - T(2,1)*T(1,2)));
//    ret(0,1) = 1.0_2*(T(0,1) + idet*(T(2,0)*T(1,2) - T(1,0)*T(2,2)));
//    ret(0,2) = 1.0_2*(T(0,2) + idet*(T(1,0)*T(2,1) - T(2,0)*T(1,1)));
//    ret(0,3) = T(0,3);

//    ret(1,0) = 1.0_2*(T(1,0) + idet*(T(2,1)*T(0,2) - T(0,1)*T(2,2)));
//    ret(1,1) = 1.0_2*(T(1,1) + idet*(T(0,0)*T(2,2) - T(2,0)*T(0,2)));
//    ret(1,2) = 1.0_2*(T(1,2) + idet*(T(2,0)*T(0,1) - T(0,0)*T(2,1)));
//    ret(1,3) = T(1,3);

//    ret(2,0) = 1.0_2*(T(2,0) + idet*(T(0,1)*T(1,2) - T(1,1)*T(0,2)));
//    ret(2,1) = 1.0_2*(T(2,1) + idet*(T(1,0)*T(0,2) - T(0,0)*T(1,2)));
//    ret(2,2) = 1.0_2*(T(2,2) + idet*(T(0,0)*T(1,1) - T(1,0)*T(0,1)));
//    ret(2,3) = T(2,3);

//    return ret;
// }

// Axis Reparameterize(const Axis& s)
// {
//  double theta = std::sqrt(s[0]*s[0] + s[1]*s[1] + s[2]*s[2]);
//  double eta = theta < LIE_EPS
//               ? 1.0
//               : 1.0 - (double)((int)(theta / M_PI + 1.0) / 2)*M_2PI / theta;

//  return eta*s;
// }

// Axis AdInvTAngular(const SE3& T, const Axis& v)
// {
//    return Axis(T(0,0)*v[0] + T(1,0)*v[1] + T(2,0)*v[2],
//                T(0,1)*v[0] + T(1,1)*v[1] + T(2,1)*v[2],
//                T(0,2)*v[0] + T(1,2)*v[1] + T(2,2)*v[2]);
// }

// Axis ad(const Axis& s1, const se3& s2)
// {
//    return Axis(s2[2]*s1[1] - s2[1]*s1[2],
//                s2[0]*s1[2] - s2[2]*s1[0],
//                s2[1]*s1[0] - s2[0]*s1[1]);
// }

// Axis ad_Axis_Axis(const Axis& s1, const Axis& s2)
// {
//    return Axis(s2[2]*s1[1] - s2[1]*s1[2],
//                s2[0]*s1[2] - s2[2]*s1[0],
//                s2[1]*s1[0] - s2[0]*s1[1]);
// }

Eigen::Vector6d dad(const Eigen::Vector6d& _s, const Eigen::Vector6d& _t) {
  Eigen::Vector6d res;
  res.head<3>() = _t.head<3>().cross(_s.head<3>())
                  + _t.tail<3>().cross(_s.tail<3>());
  res.tail<3>() = _t.tail<3>().cross(_s.head<3>());
  return res;
}

Inertia transformInertia(const Eigen::Isometry3d& _T, const Inertia& _I) {
  // operation count: multiplication = 186, addition = 117, subtract = 21

  Inertia ret = Inertia::Identity();

  double d0 = _I(0, 3) + _T(2, 3) * _I(3, 4) - _T(1, 3) * _I(3, 5);
  double d1 = _I(1, 3) - _T(2, 3) * _I(3, 3) + _T(0, 3) * _I(3, 5);
  double d2 = _I(2, 3) + _T(1, 3) * _I(3, 3) - _T(0, 3) * _I(3, 4);
  double d3 = _I(0, 4) + _T(2, 3) * _I(4, 4) - _T(1, 3) * _I(4, 5);
  double d4 = _I(1, 4) - _T(2, 3) * _I(3, 4) + _T(0, 3) * _I(4, 5);
  double d5 = _I(2, 4) + _T(1, 3) * _I(3, 4) - _T(0, 3) * _I(4, 4);
  double d6 = _I(0, 5) + _T(2, 3) * _I(4, 5) - _T(1, 3) * _I(5, 5);
  double d7 = _I(1, 5) - _T(2, 3) * _I(3, 5) + _T(0, 3) * _I(5, 5);
  double d8 = _I(2, 5) + _T(1, 3) * _I(3, 5) - _T(0, 3) * _I(4, 5);
  double e0 = _I(0, 0) + _T(2, 3) * _I(0, 4) - _T(1, 3) * _I(0, 5)
              + d3 * _T(2, 3) - d6 * _T(1, 3);
  double e3 = _I(0, 1) + _T(2, 3) * _I(1, 4) - _T(1, 3) * _I(1, 5)
              - d0 * _T(2, 3) + d6 * _T(0, 3);
  double e4 = _I(1, 1) - _T(2, 3) * _I(1, 3) + _T(0, 3) * _I(1, 5)
              - d1 * _T(2, 3) + d7 * _T(0, 3);
  double e6 = _I(0, 2) + _T(2, 3) * _I(2, 4) - _T(1, 3) * _I(2, 5)
              + d0 * _T(1, 3) - d3 * _T(0, 3);
  double e7 = _I(1, 2) - _T(2, 3) * _I(2, 3) + _T(0, 3) * _I(2, 5)
              + d1 * _T(1, 3) - d4 * _T(0, 3);
  double e8 = _I(2, 2) + _T(1, 3) * _I(2, 3) - _T(0, 3) * _I(2, 4)
              + d2 * _T(1, 3) - d5 * _T(0, 3);
  double f0 = _T(0, 0) * e0 + _T(1, 0) * e3 + _T(2, 0) * e6;
  double f1 = _T(0, 0) * e3 + _T(1, 0) * e4 + _T(2, 0) * e7;
  double f2 = _T(0, 0) * e6 + _T(1, 0) * e7 + _T(2, 0) * e8;
  double f3 = _T(0, 0) * d0 + _T(1, 0) * d1 + _T(2, 0) * d2;
  double f4 = _T(0, 0) * d3 + _T(1, 0) * d4 + _T(2, 0) * d5;
  double f5 = _T(0, 0) * d6 + _T(1, 0) * d7 + _T(2, 0) * d8;
  double f6 = _T(0, 1) * e0 + _T(1, 1) * e3 + _T(2, 1) * e6;
  double f7 = _T(0, 1) * e3 + _T(1, 1) * e4 + _T(2, 1) * e7;
  double f8 = _T(0, 1) * e6 + _T(1, 1) * e7 + _T(2, 1) * e8;
  double g0 = _T(0, 1) * d0 + _T(1, 1) * d1 + _T(2, 1) * d2;
  double g1 = _T(0, 1) * d3 + _T(1, 1) * d4 + _T(2, 1) * d5;
  double g2 = _T(0, 1) * d6 + _T(1, 1) * d7 + _T(2, 1) * d8;
  double g3 = _T(0, 2) * d0 + _T(1, 2) * d1 + _T(2, 2) * d2;
  double g4 = _T(0, 2) * d3 + _T(1, 2) * d4 + _T(2, 2) * d5;
  double g5 = _T(0, 2) * d6 + _T(1, 2) * d7 + _T(2, 2) * d8;
  double h0 = _T(0, 0) * _I(3, 3) + _T(1, 0) * _I(3, 4) + _T(2, 0) * _I(3, 5);
  double h1 = _T(0, 0) * _I(3, 4) + _T(1, 0) * _I(4, 4) + _T(2, 0) * _I(4, 5);
  double h2 = _T(0, 0) * _I(3, 5) + _T(1, 0) * _I(4, 5) + _T(2, 0) * _I(5, 5);
  double h3 = _T(0, 1) * _I(3, 3) + _T(1, 1) * _I(3, 4) + _T(2, 1) * _I(3, 5);
  double h4 = _T(0, 1) * _I(3, 4) + _T(1, 1) * _I(4, 4) + _T(2, 1) * _I(4, 5);
  double h5 = _T(0, 1) * _I(3, 5) + _T(1, 1) * _I(4, 5) + _T(2, 1) * _I(5, 5);

  ret(0, 0) = f0 * _T(0, 0) + f1 * _T(1, 0) + f2 * _T(2, 0);
  ret(0, 1) = f0 * _T(0, 1) + f1 * _T(1, 1) + f2 * _T(2, 1);
  ret(0, 2) = f0 * _T(0, 2) + f1 * _T(1, 2) + f2 * _T(2, 2);
  ret(0, 3) = f3 * _T(0, 0) + f4 * _T(1, 0) + f5 * _T(2, 0);
  ret(0, 4) = f3 * _T(0, 1) + f4 * _T(1, 1) + f5 * _T(2, 1);
  ret(0, 5) = f3 * _T(0, 2) + f4 * _T(1, 2) + f5 * _T(2, 2);
  ret(1, 1) = f6 * _T(0, 1) + f7 * _T(1, 1) + f8 * _T(2, 1);
  ret(1, 2) = f6 * _T(0, 2) + f7 * _T(1, 2) + f8 * _T(2, 2);
  ret(1, 3) = g0 * _T(0, 0) + g1 * _T(1, 0) + g2 * _T(2, 0);
  ret(1, 4) = g0 * _T(0, 1) + g1 * _T(1, 1) + g2 * _T(2, 1);
  ret(1, 5) = g0 * _T(0, 2) + g1 * _T(1, 2) + g2 * _T(2, 2);
  ret(2, 2) = (_T(0, 2) * e0 + _T(1, 2) * e3 + _T(2, 2) * e6) * _T(0, 2)
              + (_T(0, 2) * e3 + _T(1, 2) * e4 + _T(2, 2) * e7) * _T(1, 2)
              + (_T(0, 2) * e6 + _T(1, 2) * e7 + _T(2, 2) * e8) * _T(2, 2);
  ret(2, 3) = g3 * _T(0, 0) + g4 * _T(1, 0) + g5 * _T(2, 0);
  ret(2, 4) = g3 * _T(0, 1) + g4 * _T(1, 1) + g5 * _T(2, 1);
  ret(2, 5) = g3 * _T(0, 2) + g4 * _T(1, 2) + g5 * _T(2, 2);
  ret(3, 3) = h0 * _T(0, 0) + h1 * _T(1, 0) + h2 * _T(2, 0);
  ret(3, 4) = h0 * _T(0, 1) + h1 * _T(1, 1) + h2 * _T(2, 1);
  ret(3, 5) = h0 * _T(0, 2) + h1 * _T(1, 2) + h2 * _T(2, 2);
  ret(4, 4) = h3 * _T(0, 1) + h4 * _T(1, 1) + h5 * _T(2, 1);
  ret(4, 5) = h3 * _T(0, 2) + h4 * _T(1, 2) + h5 * _T(2, 2);
  ret(5, 5) =
      (_T(0, 2) * _I(3, 3) + _T(1, 2) * _I(3, 4) + _T(2, 2) * _I(3, 5))
      * _T(0, 2)
      + (_T(0, 2) * _I(3, 4) + _T(1, 2) * _I(4, 4) + _T(2, 2) * _I(4, 5))
      * _T(1, 2)
      + (_T(0, 2) * _I(3, 5) + _T(1, 2) * _I(4, 5) + _T(2, 2) * _I(5, 5))
      * _T(2, 2);

  ret.triangularView<Eigen::StrictlyLower>() = ret.transpose();

  return ret;
}

bool verifyRotation(const Eigen::Matrix3d& _T) {
  return !isNan(_T)
      && fabs(_T.determinant() - 1.0) <= DART_EPSILON;
}

bool verifyTransform(const Eigen::Isometry3d& _T) {
  return !isNan(_T.matrix().topRows<3>())
      && fabs(_T.linear().determinant() - 1.0) <= DART_EPSILON;
}

bool isNan(const Eigen::MatrixXd& _m) {
  for (int i = 0; i < _m.rows(); ++i)
    for (int j = 0; j < _m.cols(); ++j)
      if (_m(i, j) != _m(i, j))
        return true;

  return false;
}

Eigen::Vector3d fromSkewSymmetric(const Eigen::Matrix3d& _m) {
#ifndef NDEBUG
  if (fabs(_m(0, 0)) > DART_EPSILON
      || fabs(_m(1, 1)) > DART_EPSILON
      || fabs(_m(2, 2)) > DART_EPSILON) {
    std::cout << "Not skew symmetric matrix" << std::endl;
    std::cerr << _m << std::endl;
    return Eigen::Vector3d::Zero();
  }
#endif
  Eigen::Vector3d ret;
  ret << _m(2, 1), _m(0, 2), _m(1, 0);
  return ret;
}

Eigen::Matrix3d makeSkewSymmetric(const Eigen::Vector3d& _v) {
  Eigen::Matrix3d result = Eigen::Matrix3d::Zero();

  result(0, 1) = -_v(2);
  result(1, 0) =  _v(2);
  result(0, 2) =  _v(1);
  result(2, 0) = -_v(1);
  result(1, 2) = -_v(0);
  result(2, 1) =  _v(0);

  return result;
}

}  // namespace math
}  // namespace dart
