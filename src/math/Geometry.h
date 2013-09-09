/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date: 06/12/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#ifndef DART_MATH_GEOMETRY_H
#define DART_MATH_GEOMETRY_H

#include <vector>
#include <cassert>
#include <iostream>
#include <cmath>
#include <cfloat>

#include <Eigen/Dense>

#include "common/UtilsCode.h"
#include "math/MathTypes.h"

namespace dart {
namespace math {

/// @brief Compute geometric distance on SE(3) manifold.
/// Norm(Log(Inv(T1) * T2)).
//double Distance(const SE3& T1, const SE3& T2);

//SE3 operator/(const SE3& T1, const SE3& T2);

//------------------------------------------------------------------------------
/// @brief homogeneous transformation of the vector _v with the last value
/// treated a 1
inline Eigen::Vector3d xformHom(const Eigen::Matrix4d& _W,
                                const Eigen::Vector3d& _x)
{
    return _W.topLeftCorner<3,3>() * _x + _W.topRightCorner<3,1>();
}

/// @brief homogeneous transformation of the vector _v with the last value
///treated a 1
inline Eigen::Vector3d xformHom(const Eigen::Isometry3d& _W,
                                const Eigen::Vector3d& _x)
{
    return _W.rotation() * _x + _W.translation();
}

/// @brief homogeneous transformation of the vector _v treated as a direction:
/// last value 0
inline Eigen::Vector3d xformHomDir(const Eigen::Matrix4d& _W,
                                   const Eigen::Vector3d& _v)
{
    return _W.topLeftCorner<3,3>() * _v;
}

/// @brief homogeneous transformation of the vector _v treated as a direction:
/// last value 0
inline Eigen::Vector3d xformHomDir(const Eigen::Isometry3d& _W,
                                   const Eigen::Vector3d& _v)
{
    return _W.rotation() * _v;
}

inline Eigen::Matrix3d makeSkewSymmetric(const Eigen::Vector3d& v){
    Eigen::Matrix3d result = Eigen::Matrix3d::Zero();

    result(0, 1) = -v(2);
    result(1, 0) =  v(2);
    result(0, 2) =  v(1);
    result(2, 0) = -v(1);
    result(1, 2) = -v(0);
    result(2, 1) =  v(0);

    return result;
}

inline Eigen::Vector3d fromSkewSymmetric(const Eigen::Matrix3d& m) {
#ifndef NDEBUG
    if (fabs(m(0, 0)) > M_EPSILON || fabs(m(1, 1)) > M_EPSILON || fabs(m(2, 2)) > M_EPSILON) {
        std::cout << "Not skew symmetric matrix" << std::endl;
        std::cerr << m << std::endl;
        return Eigen::Vector3d::Zero();
    }
#endif
    Eigen::Vector3d ret;
    ret << m(2,1), m(0,2), m(1,0);
    return ret;
}

//------------------------------------------------------------------------------
enum RotationOrder {UNKNOWN, XYZ, XZY, YZX, YXZ, ZXY, ZYX};

Eigen::Quaterniond matrixToQuat(Eigen::Matrix3d& m);	// forms the Quaterniond from a rotation matrix
Eigen::Matrix3d quatToMatrix(Eigen::Quaterniond& q);

Eigen::Quaterniond expToQuat(Eigen::Vector3d& v);
Eigen::Vector3d quatToExp(Eigen::Quaterniond& q);


// Note: xyz order means matrix is Rz*Ry*Rx i.e a point as transformed as Rz*Ry*Rx(p)
// coord sys transformation as in GL will be written as glRotate(z); glRotate(y); glRotate(x)
Eigen::Vector3d matrixToEuler(Eigen::Matrix3d& m, RotationOrder _order);
Eigen::Matrix3d eulerToMatrix(Eigen::Vector3d& v, RotationOrder _order);

Eigen::Matrix3d eulerToMatrixX(double x);
Eigen::Matrix3d eulerToMatrixY(double y);
Eigen::Matrix3d eulerToMatrixZ(double z);

Eigen::Vector3d rotatePoint(const Eigen::Quaterniond& q, const Eigen::Vector3d& pt);
Eigen::Vector3d rotatePoint(const Eigen::Quaterniond& q, double x, double y, double z);

// quaternion stuff
Eigen::Matrix3d quatDeriv(const Eigen::Quaterniond& q, int el);
Eigen::Matrix3d quatSecondDeriv(const Eigen::Quaterniond& q, int el1, int el2);

// compute expmap stuff
Eigen::Matrix3d expMapRot(const Eigen::Vector3d &_expmap); ///< computes the Rotation matrix from a given expmap vector
Eigen::Matrix3d expMapJac(const Eigen::Vector3d &_expmap);  ///< computes the Jacobian of the expmap
Eigen::Matrix3d expMapJacDot(const Eigen::Vector3d &_expmap, const Eigen::Vector3d &_qdot); ///< computes the time derivative of the expmap Jacobian
Eigen::Matrix3d expMapJacDeriv(const Eigen::Vector3d &_expmap, int _qi);    ///< computes the derivative of the Jacobian of the expmap wrt to _qi indexed dof; _qi \in {0,1,2}

//------------------------------------------------------------------------------

/// @brief Get a transformation matrix given by the Euler XYZ angle,
/// where the positional part is set to be zero.
Eigen::Isometry3d EulerXYZ(const Eigen::Vector3d& angle);

/// @brief get a transformation matrix given by the Euler XYZ angle and
/// position.
Eigen::Isometry3d EulerXYZ(const Eigen::Vector3d& angle, const Eigen::Vector3d& position);

/// @brief get a transformation matrix given by the Euler ZYX angle,
/// where the positional part is set to be zero.
/// singularity : x[1] = -+ 0.5*PI
/// @sa SE3::iEulerZYX
Eigen::Isometry3d EulerZYX(const Eigen::Vector3d& angle);

///// @brief get a transformation matrix given by the Euler ZYX angle and
///// position.
///// singularity : x[1] = -+ 0.5*PI
//SE3 EulerZYX(const Vec3& angle, const Vec3& position);

///// @brief Get a transformation matrix given by the Euler ZYZ angle,
///// where the positional part is set to be zero.
///// singularity : x[1] = 0, PI
///// @sa SE3::iEulerZYZ
//SE3 EulerZYZ(const Vec3& angle);

///// @brief get a transformation matrix given by the Euler ZYZ angle and
///// position.
///// singularity : x[1] = 0, PI
//SE3 EulerZYZ(const Vec3& angle, const Vec3& position);

/// @brief get the Euler ZYX angle from T
////// @sa Eigen_Vec3::EulerXYZ
Eigen::Vector3d iEulerXYZ(const Eigen::Isometry3d& T);

///// @brief get the Euler ZYX angle from T
///// @sa Eigen_Vec3::EulerZYX
//Vec3 iEulerZYX(const SE3& T);

///// @brief get the Euler ZYZ angle from T
///// @sa Eigen_Vec3::EulerZYZ
//Vec3 iEulerZYZ(const SE3& T);

///// @brief get the Euler ZYZ angle from T
///// @sa Eigen_Vec3::EulerZXY
//Vec3 iEulerZXY(const SE3 &T);

//------------------------------------------------------------------------------
/// @brief rotate q by T.
/// @return @f$R q@f$, where @f$T=(R,p)@f$.
Eigen::Vector3d Rotate(const Eigen::Isometry3d& T, const Eigen::Vector3d& q);

///// @brief rotate q by Inv(T).
//Vec3 RotateInv(const SE3& T, const Vec3& q);

//------------------------------------------------------------------------------

///// @brief reparameterize such as ||s'|| < M_PI and Exp(s) == Epx(s')
//Axis Reparameterize(const Axis& s);

/// @brief Exponential mapping
Eigen::Isometry3d Exp(const Eigen::Vector6d& s);

/// @brief fast version of Exp(se3(s, 0))
Eigen::Isometry3d ExpAngular(const Eigen::Vector3d& s);

///// @brief fast version of Exp(t * s), when |s| = 1
//SE3 ExpAngular(const Axis& s, double t);

/// @brief fast version of Exp(se3(s, 0))
Eigen::Isometry3d ExpLinear(const Eigen::Vector3d& s);

///// @brief Log mapping
//se3 Log(const SE3& );

///// @brief Log mapping of rotation part only
///// @note When @f$|LogR(T)| = @pi@f$, Exp(LogR(T) = Exp(-LogR(T)).
///// The implementation returns only the positive one.
//Axis LogR(const SE3& T);

//------------------------------------------------------------------------------

/// @brief get inversion of T
/// @note @f$T^{-1} = (R^T, -R^T p), where T=(R,p)@in SE(3)@f$.
Eigen::Isometry3d Inv(const Eigen::Isometry3d& T);

//------------------------------------------------------------------------------

/// @brief get rotation matrix rotated along x-Eigen_Axis by theta angle.
/// @note theta is represented in radian.
Eigen::Isometry3d RotX(double angle);

/// @brief get rotation matrix rotated along y-Eigen_Axis by theta angle.
Eigen::Isometry3d RotY(double angle);

/// @brief get rotation matrix rotated along z-Eigen_Axis by theta angle.
Eigen::Isometry3d RotZ(double angle);

//------------------------------------------------------------------------------
///// @brief Rectify the rotation part so as that it satifies the orthogonality
///// condition.
/////
///// It is one step of @f$R_{i_1}=1/2(R_i + R_i^{-T})@f$.
///// Hence by calling this function iterativley, you can make the rotation part
///// closer to SO(3).
//SE3 Normalize(const SE3& T);

//------------------------------------------------------------------------------

/// @brief adjoint mapping
/// @note @f$Ad_TV = ( Rw@,, ~p @times Rw + Rv)@f$,
/// where @f$T=(R,p)@in SE(3), @quad V=(w,v)@in se(3) @f$.
Eigen::Vector6d AdT(const Eigen::Isometry3d& T, const Eigen::Vector6d& V);

/// @brief Fast version of Ad([R 0; 0 1], V)
Eigen::Vector6d AdR(const Eigen::Isometry3d& T, const Eigen::Vector6d& V);

/// @brief fast version of Ad(T, se3(w, 0))
Eigen::Vector6d AdTAngular(const Eigen::Isometry3d& T, const Eigen::Vector3d& w);

/// @brief fast version of Ad(T, se3(0, v))
Eigen::Vector6d AdTLinear(const Eigen::Isometry3d& T, const Eigen::Vector3d& v);

///// @brief fast version of Ad([I p; 0 1], V)
//se3 AdP(const Vec3& p, const se3& s);

/// @brief
Jacobian AdTJac(const Eigen::Isometry3d& T, const Jacobian& J);

///// @brief fast version of Ad([R 0; 0 1], J)
//Jacobian AdRJac(const SE3& T, const Jacobian& J);

/// @brief fast version of Ad(Inv(T), V)
Eigen::Vector6d AdInvT(const Eigen::Isometry3d& T, const Eigen::Vector6d& V);

///// @brief fast version of Ad(Inv(T), se3(Eigen_Vec3(0), v))
//Eigen::Vector3d AdInvTLinear(const Eigen::Isometry3d& T, const Eigen::Vector3d& v);

///// @brief fast version of Ad(Inv(T), se3(w, Eigen_Vec3(0)))
//Axis AdInvTAngular(const SE3& T, const Axis& w);

///// @brief Fast version of Ad(Inv([R 0; 0 1]), V)
//se3 AdInvR(const SE3& T, const se3& V);

/// @brief Fast version of Ad(Inv([R 0; 0 1]), se3(0, v))
Eigen::Vector6d AdInvRLinear(const Eigen::Isometry3d& T, const Eigen::Vector3d& V);

/// @brief dual adjoint mapping
/// @note @f$Ad^{@,*}_TF = ( R^T (m - p@times f)@,,~ R^T f)@f$, where @f$T=(R,p)@in SE(3), F=(m,f)@in se(3)^*@f$.
Eigen::Vector6d dAdT(const Eigen::Isometry3d& T, const Eigen::Vector6d& F);

///// @brief fast version of Ad(Inv(T), dse3(Eigen_Vec3(0), F))
//dse3 dAdTLinear(const SE3& T, const Vec3& F);

/// @brief fast version of dAd(Inv(T), F)
Eigen::Vector6d dAdInvT(const Eigen::Isometry3d& T, const Eigen::Vector6d& F);

/// @brief fast version of dAd(Inv([R 0; 0 1]), F)
Eigen::Vector6d dAdInvR(const Eigen::Isometry3d& T, const Eigen::Vector6d& F);

///// @brief fast version of dAd(Inv(SE3(p)), dse3(Eigen_Vec3(0), F))
//dse3 dAdInvPLinear(const Vec3& p, const Vec3& F);

/// @brief adjoint mapping
/// @note @f$ad_X Y = ( w_X @times w_Y@,,~w_X @times v_Y - w_Y @times v_X),@f$,
/// where @f$X=(w_X,v_X)@in se(3), @quad Y=(w_Y,v_Y)@in se(3) @f$.
Eigen::Vector6d ad(const Eigen::Vector6d& X, const Eigen::Vector6d& Y);

/// @brief fast version of ad(se3(Eigen_Vec3(0), v), S)
//Vec3 ad_Vec3_se3(const Vec3& v, const se3& S);

/// @brief fast version of ad(se3(w, 0), se3(v, 0))	-> check
//Axis ad_Axis_Axis(const Axis& w, const Axis& v);

/// @brief dual adjoint mapping
/// @note @f$ad^{@,*}_V F = (m @times w + f @times v@,,~ f @times w),@f$
/// , where @f$F=(m,f)@in se^{@,*}(3), @quad V=(w,v)@in se(3) @f$.
Eigen::Vector6d dad(const Eigen::Vector6d& V, const Eigen::Vector6d& F);

/// @brief
Inertia Transform(const Eigen::Isometry3d& T, const Inertia& AI);

/// @brief
bool VerifySE3(const Eigen::Isometry3d& _T);

/// @brief
bool Verifyse3(const Eigen::Vector6d& _V);

} // namespace math
} // namespace dart

#endif // #ifndef DART_MATH_GEOMETRY_H

