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

#include <iomanip>

#include "math/Geometry.h"

namespace dart {
namespace math {

Eigen::Quaterniond matrixToQuat(Eigen::Matrix3d& mat)
{
    return Eigen::Quaterniond(mat);
}

Eigen::Matrix3d quatToMatrix(Eigen::Quaterniond& q)
{
    return Eigen::Matrix3d(q);
}

Eigen::Quaterniond expToQuat(Eigen::Vector3d& v)
{
    double mag = v.norm();

    if(mag > 1e-10)
    {
        Eigen::Quaterniond q(Eigen::AngleAxisd(mag, v / mag));
        return q;
    }
    else
    {
        Eigen::Quaterniond q(1, 0, 0, 0);
        return q;
    }
}

Eigen::Vector3d quatToExp(Eigen::Quaterniond& q)
{
    Eigen::AngleAxisd aa(q);
    Eigen::Vector3d v = aa.axis();
    return v*aa.angle();
}

// Note: xyz order means matrix is Rz*Ry*Rx i.e a point as transformed as Rz*Ry*Rx(p)
// coordinate system transformation as in GL will be written as glRotate(z); glRotate(y); glRotate(x)
Eigen::Vector3d matrixToEuler(Eigen::Matrix3d& m, RotationOrder order)
{
    double x, y, z;

    if(order == XYZ)
    {
        if (m(2, 0) > (1.0-DART_EPSILON))
        {
            std::cout << "North Pole" << std::endl;
            x = atan2(m(0, 1), m(0, 2));
            y = -DART_PI / 2.0;
            z = 0.0;
        }

        if (m(2, 0) < -(1.0-DART_EPSILON))
        {
            std::cout << "South Pole" << std::endl;
            x = atan2(m(0, 1), m(0, 2));
            y = DART_PI / 2.0;
            z = 0.0;
        }

        x = atan2(m(2, 1), m(2, 2));
        y = -asin(m(2, 0));
        z = atan2(m(1, 0), m(0, 0));

        // order of return is the order of input
        return Eigen::Vector3d(x,y,z);
    }

    if (order == ZYX)
    {
        if (m(0, 2) > (1.0-DART_EPSILON))
        {
            std::cout << "North Pole" << std::endl;
            z = atan2(m(1, 0), m(1, 1));
            y = DART_PI / 2.0;
            x = 0.0;
        }

        if (m(0, 2) < -(1.0-DART_EPSILON))
        {
            std::cout << "South Pole" << std::endl;
            z = atan2(m(1, 0), m(1, 1));
            y = -DART_PI / 2.0;
            x = 0.0;
        }

        z = -atan2(m(0, 1), m(0, 0));
        y = asin(m(0, 2));
        x = -atan2(m(1, 2), m(2, 2));

        // order of return is the order of input
        return Eigen::Vector3d(z,y,x);
    }

    if (order == YZX)
    {
        if (m(0, 1) > (1.0-DART_EPSILON))
        {
            std::cout << "North Pole" << std::endl;
            y = atan2(m(1, 2), m(1, 0));
            z = -DART_PI / 2.0;
            x = 0.0;
        }

        if (m(0, 1) < -(1.0-DART_EPSILON))
        {
            std::cout << "South Pole" << std::endl;
            y = atan2(m(1, 2), m(1, 0));
            z = DART_PI / 2.0;
            x = 0.0;
        }

        y = atan2(m(0, 2), m(0, 0));
        z = -asin(m(0, 1));
        x = atan2(m(2, 1), m(1, 1));

        // order of return is the order of input
        return Eigen::Vector3d(y,z,x);
    }

    if (order == XZY)
    {
        if(m(1, 0) > (1.0-DART_EPSILON))
        {
            std::cout << "North Pole" << std::endl;
            x = -atan2(m(0, 2), m(0, 1));
            z = DART_PI / 2.0;
            y = 0.0;
        }

        if(m(1, 0) < -(1.0-DART_EPSILON))
        {
            std::cout << "South Pole" << std::endl;
            x = -atan2(m(0, 2), m(0, 1));
            z = -DART_PI / 2.0;
            y = 0.0;
        }

        x = -atan2(m(1, 2), m(1, 1));
        z = asin(m(1, 0));
        y = -atan2(m(2, 0), m(0, 0));

        // order of return is the order of input
        return Eigen::Vector3d(x,z,y);
    }

    if (order == YXZ)
    {
        if (m(2, 1) > (1.0-DART_EPSILON))
        {
            std::cout << "North Pole" << std::endl;
            y = atan2(m(0, 2), m(0, 0));
            x = DART_PI / 2.0;
            z = 0.0;
        }

        if (m(2, 1) < -(1.0-DART_EPSILON))
        {
            std::cout << "South Pole" << std::endl;
            y = atan2(m(0, 2), m(0, 0));
            x = -DART_PI / 2.0;
            z = 0.0;
        }

        y = -atan2(m(2, 0), m(2, 2));
        x = asin(m(2, 1));
        z = -atan2(m(0, 1), m(1, 1));

        // order of return is the order of input
        return Eigen::Vector3d(y, x, z);
    }

    if (order==ZXY)
    {
        if (m(1, 2) > (1.0-DART_EPSILON))
        {
            std::cout << "North Pole" << std::endl;
            z = -atan2(m(0, 1), m(0, 0));
            x = -DART_PI / 2.0;
            y = 0.0;
        }

        if(m(1, 2) < -(1.0-DART_EPSILON))
        {
            std::cout << "South Pole" << std::endl;
            z = -atan2(m(0, 1), m(0, 0));
            x = DART_PI / 2.0;
            y = 0.0;
        }

        z = atan2(m(1, 0), m(1, 1));
        x = -asin(m(1, 2));
        y = atan2(m(0, 2), m(2, 2));

        // order of return is the order of input
        return Eigen::Vector3d(z,x,y);
    }

    printf("Matrix_to_Euler - Do not support rotation order %d. Make sure letters are in lowercase\n", order);

    return Eigen::Vector3d(0.0, 0.0, 0.0);
}

Eigen::Matrix3d eulerToMatrix(Eigen::Vector3d& v, RotationOrder order)
{
    if(order==XYZ)
        return eulerToMatrixZ(v[2])*eulerToMatrixY(v[1])*eulerToMatrixX(v[0]);

    if(order==ZYX)
        return eulerToMatrixX(v[0])*eulerToMatrixY(v[1])*eulerToMatrixZ(v[2]);

    if(order==YZX)
        return eulerToMatrixX(v[0])*eulerToMatrixZ(v[2])*eulerToMatrixY(v[1]);

    if(order==XZY)
        return eulerToMatrixY(v[1])*eulerToMatrixZ(v[2])*eulerToMatrixX(v[0]);

    if(order==YXZ)
        return eulerToMatrixZ(v[2])*eulerToMatrixX(v[0])*eulerToMatrixY(v[1]);

    if(order==ZXY)
        return eulerToMatrixY(v[1])*eulerToMatrixX(v[0])*eulerToMatrixZ(v[2]);

    printf("euler_to_matrix - Do not support rotation order %d. Make sure letters are in lowercase\n", order);

    return Eigen::Matrix3d::Zero();
}

Eigen::Matrix3d eulerToMatrixX(double x)
{
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
    double cosangle = cos(x);
    double sinangle = sin(x);
    mat(0, 0) = 1.0;
    mat(1, 1) = cosangle;
    mat(1, 2) = -sinangle;
    mat(2, 1) = sinangle;
    mat(2, 2) = cosangle;
    return mat;
}

Eigen::Matrix3d eulerToMatrixY(double y)
{
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
    double cosangle = cos(y);
    double sinangle = sin(y);
    mat(1, 1) = 1.0;
    mat(2, 2) = cosangle;
    mat(2, 0) = -sinangle;
    mat(0, 2) = sinangle;
    mat(0, 0) = cosangle;
    return mat;
}

Eigen::Matrix3d eulerToMatrixZ(double z)
{
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
    double cosangle = cos(z);
    double sinangle = sin(z);
    mat(2, 2) = 1.0;
    mat(0, 0) = cosangle;
    mat(0, 1) = -sinangle;
    mat(1, 0) = sinangle;
    mat(1, 1) = cosangle;
    return mat;
}

// get the derivative of rotation matrix wrt el no.
Eigen::Matrix3d quatDeriv(const Eigen::Quaterniond& q, int el)
{
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();

    switch(el)
    {
        case 0:	// wrt w
            mat(0, 0) = q.w();
            mat(1, 1) = q.w();
            mat(2, 2) = q.w();
            mat(0, 1) = -q.z();
            mat(1, 0) = q.z();
            mat(0, 2) = q.y();
            mat(2, 0) = -q.y();
            mat(1, 2) = -q.x();
            mat(2, 1) = q.x();
            break;
        case 1:	// wrt x
            mat(0, 0) = q.x();
            mat(1, 1) = -q.x();
            mat(2, 2) = -q.x();
            mat(0, 1) = q.y();
            mat(1, 0) = q.y();
            mat(0, 2) = q.z();
            mat(2, 0) = q.z();
            mat(1, 2) = -q.w();
            mat(2, 1) = q.w();
            break;
        case 2:	// wrt y
            mat(0, 0) = -q.y();
            mat(1, 1) = q.y();
            mat(2, 2) = -q.y();
            mat(0, 1) = q.x();
            mat(1, 0) = q.x();
            mat(0, 2) = q.w();
            mat(2, 0) = -q.w();
            mat(1, 2) = q.z();
            mat(2, 1) = q.z();
            break;
        case 3:	// wrt z
            mat(0, 0) = -q.z();
            mat(1, 1) = -q.z();
            mat(2, 2) = q.z();
            mat(0, 1) = -q.w();
            mat(1, 0) = q.w();
            mat(0, 2) = q.x();
            mat(2, 0) = q.x();
            mat(1, 2) = q.y();
            mat(2, 1) = q.y();
            break;
        default:
            break;
    }

    return 2*mat;
}

Eigen::Matrix3d quatSecondDeriv(const Eigen::Quaterniond& q, int el1, int el2)
{
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();

    // wrt same dof
    if (el1 == el2)
    {
        switch(el1)
        {
            case 0:	// wrt w
                mat(0, 0) = 1;
                mat(1, 1) = 1;
                mat(2, 2) = 1;
                break;
            case 1:	// wrt x
                mat(0, 0) = 1;
                mat(1, 1) = -1;
                mat(2, 2) = -1;
                break;
            case 2:	// wrt y
                mat(0, 0) = -1;
                mat(1, 1) = 1;
                mat(2, 2) =-1;
                break;
            case 3:	// wrt z
                mat(0, 0) = -1;
                mat(1, 1) = -1;
                mat(2, 2) = 1;
                break;
        }
    }
    // wrt different dofs
    else
    {
        // arrange in increasing order
        if (el1 > el2)
        {
            int temp = el2;
            el2 = el1;
            el1 = temp;
        }

        switch(el1)
        {
            case 0:	// wrt w
                switch(el2)
                {
                    case 1:	// wrt x
                        mat(1, 2) = -1;
                        mat(2, 1) = 1;
                        break;
                    case 2:	// wrt y
                        mat(0, 2) = 1;
                        mat(2, 0) = -1;
                        break;
                    case 3:	// wrt z
                        mat(0, 1) = -1;
                        mat(1, 0) = 1;
                        break;
                }
                break;
            case 1:	// wrt x
                switch(el2)
                {
                    case 2:	// wrt y
                        mat(0, 1) = 1;
                        mat(1, 0) = 1;
                        break;
                    case 3:	// wrt z
                        mat(0, 2) = 1;
                        mat(2, 0) = 1;
                        break;
                }
                break;
            case 2:	// wrt y
                switch(el2)
                {
                    case 3:	// wrt z
                        mat(1, 2) = 1;
                        mat(2, 1) = 1;
                        break;
                }
                break;
        }
    }

    return 2*mat;
}

Eigen::Vector3d rotatePoint(const Eigen::Quaterniond& q, const Eigen::Vector3d& pt)
{
    Eigen::Quaterniond quat_pt(0, pt[0], pt[1], pt[2]);
    Eigen::Quaterniond qinv = q.inverse();

    Eigen::Quaterniond rot = q*quat_pt*qinv;

    // check below - assuming same format of point achieved
    Eigen::Vector3d temp;
    //VLOG(1)<<"Point before: "<<0<<" "<<pt.x<<" "<<pt.y<<" "<<pt.z<<"\n";
    //VLOG(1)<<"Point after:  "<<rot.x<<" "<<rot.y<<" "<<rot.z<<" "<<rot.w<<"\n";
    temp[0]=rot.x();
    temp[1]=rot.y();
    temp[2]=rot.z();

    //VLOG(1)<<"Point after rotation: "<<temp[0]<<" "<<temp[1]<<" "<<temp[2]<<endl;
    return temp;
}

Eigen::Vector3d rotatePoint(const Eigen::Quaterniond& q, double x, double y, double z)
{
    Eigen::Vector3d pt;
    pt[0]=x;
    pt[1]=y;
    pt[2]=z;

    return rotatePoint(q, pt);
}

// ----------- expmap computations -------------

#define EPSILON_EXPMAP_THETA 1.0e-3

Eigen::Matrix3d expMapRot(const Eigen::Vector3d &_q)
{
    double theta = _q.norm();

    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d qss =  math::makeSkewSymmetric(_q);
    Eigen::Matrix3d qss2 =  qss*qss;

    if (theta < EPSILON_EXPMAP_THETA)
        R = Eigen::Matrix3d::Identity() + qss + 0.5*qss2;
    else
        R = Eigen::Matrix3d::Identity() + (sin(theta)/theta)*qss + ((1-cos(theta))/(theta*theta))*qss2;

    return R;
}

Eigen::Matrix3d expMapJac(const Eigen::Vector3d &_q)
{
    double theta = _q.norm();

    Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d qss =  math::makeSkewSymmetric(_q);
    Eigen::Matrix3d qss2 =  qss*qss;

    if(theta<EPSILON_EXPMAP_THETA)
        J = Eigen::Matrix3d::Identity() + 0.5*qss +  (1.0/6.0)*qss2;
    else
        J = Eigen::Matrix3d::Identity() + ((1-cos(theta))/(theta*theta))*qss + ((theta-sin(theta))/(theta*theta*theta))*qss2;

    return J;
}

Eigen::Matrix3d expMapJacDot(const Eigen::Vector3d& _q, const Eigen::Vector3d& _qdot)
{
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

    if(theta<EPSILON_EXPMAP_THETA)
    {
        Jdot = 0.5*qdss + (1.0/6.0)*(qss*qdss + qdss*qss);
        Jdot += (-1.0/12)*ttdot*qss + (-1.0/60)*ttdot*qss2;
    }
    else
    {
        Jdot = ((1-ct)/t2)*qdss + ((theta-st)/t3)*(qss*qdss + qdss*qss);
        Jdot += ((theta*st + 2*ct - 2)/t4)*ttdot*qss + ((3*st - theta*ct - 2*theta)/t5)*ttdot*qss2;
    }

    return Jdot;
}

Eigen::Matrix3d expMapJacDeriv(const Eigen::Vector3d& _q, int _qi )
{
    assert(_qi>=0 && _qi<=2);

    Eigen::Vector3d qdot = Eigen::Vector3d::Zero();
    qdot[_qi] = 1.0;
    return expMapJacDot(_q, qdot);
}

//SE3 operator/(const SE3& T1, const SE3& T2)
//{
//    SE3 ret = SE3::Identity();

//    double tmp[] = {T1(0,0)*T2(0,0) + T1(0,1)*T2(0,1) + T1(0,2)*T2(0,2),
//                    T1(1,0)*T2(0,0) + T1(1,1)*T2(0,1) + T1(1,2)*T2(0,2),
//                    T1(2,0)*T2(0,0) + T1(2,1)*T2(0,1) + T1(2,2)*T2(0,2),
//                    T1(0,0)*T2(1,0) + T1(0,1)*T2(1,1) + T1(0,2)*T2(1,2),
//                    T1(1,0)*T2(1,0) + T1(1,1)*T2(1,1) + T1(1,2)*T2(1,2),
//                    T1(2,0)*T2(1,0) + T1(2,1)*T2(1,1) + T1(2,2)*T2(1,2),
//                    T1(0,0)*T2(2,0) + T1(0,1)*T2(2,1) + T1(0,2)*T2(2,2),
//                    T1(1,0)*T2(2,0) + T1(1,1)*T2(2,1) + T1(1,2)*T2(2,2),
//                    T1(2,0)*T2(2,0) + T1(2,1)*T2(2,1) + T1(2,2)*T2(2,2) };

//    ret(0,0) = tmp[0];  ret(0,1) = tmp[3];  ret(0,2) = tmp[6];  ret(0,3) = T1(0,3) - tmp[0]*T2(0,3) - tmp[3]*T2(1,3) - tmp[6]*T2(2,3);
//    ret(1,0) = tmp[1];  ret(1,1) = tmp[4];  ret(1,2) = tmp[7];  ret(1,3) = T1(1,3) - tmp[1]*T2(0,3) - tmp[4]*T2(1,3) - tmp[7]*T2(2,3);
//    ret(2,0) = tmp[2];  ret(2,1) = tmp[5];  ret(2,2) = tmp[8];  ret(2,3) = T1(2,3) - tmp[2]*T2(0,3) - tmp[5]*T2(1,3) - tmp[8]*T2(2,3);

//    return ret;
//}

//Vec3 AdInvTLinear(const SE3& T, const Vec3& v)
//{
//    return Vec3(T(0,0)*v[0] + T(1,0)*v[1] + T(2,0)*v[2],
//                T(0,1)*v[0] + T(1,1)*v[1] + T(2,1)*v[2],
//                T(0,2)*v[0] + T(1,2)*v[1] + T(2,2)*v[2]);
//}

Eigen::Vector3d iEulerXYZ(const Eigen::Isometry3d& T)
{
    return Eigen::Vector3d(atan2(-T(1,2), T(2,2)),
                           atan2( T(0,2), sqrt(T(1,2)*T(1,2) + T(2,2)*T(2,2))),
                           atan2(-T(0,1), T(0,0)));
}

//Vec3 iEulerZYX(const SE3& T)
//{
//    return Vec3(atan2( T(1,0), T(0,0)),
//                atan2(-T(2,0), sqrt(T(0,0)*T(0,0) + T(1,0)*T(1,0))),
//                atan2( T(2,1), T(2,2)));
//}

//Vec3 iEulerZYZ(const SE3& T)
//{
//    return Vec3(atan2(T(1,2), T(0,2)),
//                atan2(sqrt(T(2,0)*T(2,0) + T(2,1)*T(2,1)), T(2,2)),
//                atan2(T(2,1), -T(2,0)));
//}

//Vec3 iEulerZXY(const SE3& T)
//{
//    return Vec3(atan2(-T(0,1), T(1,1)),
//                atan2( T(2,1), sqrt(T(0,1)*T(0,1)+T(1,1)*T(1,1))),
//                atan2(-T(2,0), T(2,2)));
//}

//Vec3 ad_Vec3_se3(const Vec3& s1, const se3& s2)
//{
//    Vec3 ret;

//    ret << s2[2]*s1[1] - s2[1]*s1[2],
//           s2[0]*s1[2] - s2[2]*s1[0],
//           s2[1]*s1[0] - s2[0]*s1[1];

//    return ret;
//}

Eigen::Vector3d Log(const Eigen::Matrix3d& R)
{
    //--------------------------------------------------------------------------
    // T = (R, p) = exp([w, v]), t = ||w||
    // v = beta*p + gamma*w + 1 / 2*cross(p, w)
    //    , beta = t*(1 + cos(t)) / (2*sin(t)), gamma = <w, p>*(1 - beta) / t^2
    //--------------------------------------------------------------------------
    double theta = std::acos(std::max(std::min(0.5 * (R(0,0) + R(1,1) + R(2,2) - 1.0), 1.0), -1.0));

    if (theta > DART_PI - DART_EPSILON)
    {
        double delta = 0.5 + 0.125*(DART_PI - theta)*(DART_PI - theta);

        return Eigen::Vector3d(
                    R(2,1) > R(1,2) ? theta*sqrt(1.0 + (R(0,0) - 1.0)*delta) :
                                      -theta*sqrt(1.0 + (R(0,0) - 1.0)*delta),
                    R(0,2) > R(2,0) ? theta*sqrt(1.0 + (R(1,1) - 1.0)*delta) :
                                      -theta*sqrt(1.0 + (R(1,1) - 1.0)*delta),
                    R(1,0) > R(0,1) ? theta*sqrt(1.0 + (R(2,2) - 1.0)*delta) :
                                      -theta*sqrt(1.0 + (R(2,2) - 1.0)*delta));
    }
    else
    {
        double alpha = 0.0;

        if (theta > DART_EPSILON)
            alpha = 0.5*theta / sin(theta);
        else
            alpha = 0.5 + DART_1_12*theta*theta;

        return Eigen::Vector3d(alpha*(R(2,1) - R(1,2)),
                               alpha*(R(0,2) - R(2,0)),
                               alpha*(R(1,0) - R(0,1)));
    }
}

Eigen::Vector6d Log(const Eigen::Isometry3d& T)
{
    //--------------------------------------------------------------------------
    // T = (R, p) = exp([w, v]), t = ||w||
    // v = beta*p + gamma*w + 1 / 2*cross(p, w)
    //    , beta = t*(1 + cos(t)) / (2*sin(t)), gamma = <w, p>*(1 - beta) / t^2
    //--------------------------------------------------------------------------
    double theta = std::acos(std::max(std::min(0.5 * (T(0,0) + T(1,1) + T(2,2) - 1.0), 1.0), -1.0));
    double alpha;
    double beta;
    double gamma;
    Eigen::Vector6d ret;

    if (theta > DART_PI - DART_EPSILON)
    {
        const double c1 = 0.10132118364234;		// 1 / pi^2
        const double c2 = 0.01507440267955;		// 1 / 4 / pi - 2 / pi^3
        const double c3 = 0.00546765085347;		// 3 / pi^4 - 1 / 4 / pi^2

        double phi = DART_PI - theta;
        double delta = 0.5 + 0.125*phi*phi;

        double w[] = {	T(2,1) > T(1,2) ? theta*sqrt(1.0 + (T(0,0) - 1.0)*delta) : -theta*sqrt(1.0 + (T(0,0) - 1.0)*delta),
                        T(0,2) > T(2,0) ? theta*sqrt(1.0 + (T(1,1) - 1.0)*delta) : -theta*sqrt(1.0 + (T(1,1) - 1.0)*delta),
                        T(1,0) > T(0,1) ? theta*sqrt(1.0 + (T(2,2) - 1.0)*delta) : -theta*sqrt(1.0 + (T(2,2) - 1.0)*delta) };

        beta = 0.25*theta*(DART_PI - theta);
        gamma = (w[0]*T(0,3) + w[1]*T(1,3) + w[2]*T(2,3))*(c1 -  c2*phi + c3*phi*phi);

        ret << w[0],
               w[1],
               w[2],
               beta*T(0,3) - 0.5*(w[1]*T(2,3) - w[2]*T(1,3)) + gamma*w[0],
               beta*T(1,3) - 0.5*(w[2]*T(0,3) - w[0]*T(2,3)) + gamma*w[1],
               beta*T(2,3) - 0.5*(w[0]*T(1,3) - w[1]*T(0,3)) + gamma*w[2];
    }
    else
    {
        if (theta > DART_EPSILON)
        {
            alpha = 0.5*theta / sin(theta);
            beta = (1.0 + cos(theta))*alpha;
            gamma = (1.0 - beta) / theta / theta;
        }
        else
        {
            alpha = 0.5 + DART_1_12*theta*theta;
            beta = 1.0 - DART_1_12*theta*theta;
            gamma = DART_1_12 + DART_1_720*theta*theta;
        }

        double w[] = { alpha*(T(2,1) - T(1,2)), alpha*(T(0,2) - T(2,0)), alpha*(T(1,0) - T(0,1)) };
        gamma *= w[0]*T(0,3) + w[1]*T(1,3) + w[2]*T(2,3);

        ret << w[0],
               w[1],
               w[2],
               beta*T(0,3) + 0.5*(w[2]*T(1,3) - w[1]*T(2,3)) + gamma*w[0],
               beta*T(1,3) + 0.5*(w[0]*T(2,3) - w[2]*T(0,3)) + gamma*w[1],
               beta*T(2,3) + 0.5*(w[1]*T(0,3) - w[0]*T(1,3)) + gamma*w[2];
    }

    return ret;
}

// re = T*s*Inv(T)
Eigen::Vector6d AdT(const Eigen::Isometry3d& T, const Eigen::Vector6d& s)
{
    //--------------------------------------------------------------------------
    // w' = R*w
    // v' = p x R*w + R*v
    //--------------------------------------------------------------------------
    Eigen::Vector6d ret = Eigen::Vector6d::Zero();
    double Rw[3] = { T(0,0)*s[0] + T(0,1)*s[1] + T(0,2)*s[2],
                     T(1,0)*s[0] + T(1,1)*s[1] + T(1,2)*s[2],
                     T(2,0)*s[0] + T(2,1)*s[1] + T(2,2)*s[2] };
    ret << Rw[0],
           Rw[1],
           Rw[2],
           T(1,3)*Rw[2] - T(2,3)*Rw[1] + T(0,0)*s[3] + T(0,1)*s[4] + T(0,2)*s[5],
           T(2,3)*Rw[0] - T(0,3)*Rw[2] + T(1,0)*s[3] + T(1,1)*s[4] + T(1,2)*s[5],
           T(0,3)*Rw[1] - T(1,3)*Rw[0] + T(2,0)*s[3] + T(2,1)*s[4] + T(2,2)*s[5];

    return ret;
}

Eigen::Vector6d AdR(const Eigen::Isometry3d& T, const Eigen::Vector6d& s)
{
    //--------------------------------------------------------------------------
    // w' = R*w
    // v' = R*v
    //--------------------------------------------------------------------------
    Eigen::Vector6d ret = Eigen::Vector6d::Zero();

    ret << T(0,0)*s[0] + T(0,1)*s[1] + T(0,2)*s[2],
           T(1,0)*s[0] + T(1,1)*s[1] + T(1,2)*s[2],
           T(2,0)*s[0] + T(2,1)*s[1] + T(2,2)*s[2],
           T(0,0)*s[3] + T(0,1)*s[4] + T(0,2)*s[5],
           T(1,0)*s[3] + T(1,1)*s[4] + T(1,2)*s[5],
           T(2,0)*s[3] + T(2,1)*s[4] + T(2,2)*s[5];

    return ret;
}

Eigen::Vector6d AdTAngular(const Eigen::Isometry3d& T, const Eigen::Vector3d& s)
{
    //--------------------------------------------------------------------------
    // w' = R*w
    // v' = p x R*w
    //--------------------------------------------------------------------------
    Eigen::Vector6d ret = Eigen::Vector6d::Zero();
    double Rw[3] = { T(0,0)*s[0] + T(0,1)*s[1] + T(0,2)*s[2],
                     T(1,0)*s[0] + T(1,1)*s[1] + T(1,2)*s[2],
                     T(2,0)*s[0] + T(2,1)*s[1] + T(2,2)*s[2] };
    ret << Rw[0],
           Rw[1],
           Rw[2],
           T(1,3)*Rw[2] - T(2,3)*Rw[1],
           T(2,3)*Rw[0] - T(0,3)*Rw[2],
           T(0,3)*Rw[1] - T(1,3)*Rw[0];

    return ret;
}

Eigen::Vector6d AdTLinear(const Eigen::Isometry3d& T, const Eigen::Vector3d& v)
{
    //--------------------------------------------------------------------------
    // w' = 0
    // v' = R*v
    //--------------------------------------------------------------------------
    Eigen::Vector6d ret = Eigen::Vector6d::Zero();

    ret << 0.0, 0.0, 0.0,
           T(0,0)*v[0] + T(0,1)*v[1] + T(0,2)*v[2],
           T(1,0)*v[0] + T(1,1)*v[1] + T(1,2)*v[2],
           T(2,0)*v[0] + T(2,1)*v[1] + T(2,2)*v[2];

    return ret;
}

Jacobian AdTJac(const Eigen::Isometry3d& T, const Jacobian& J)
{
    Jacobian AdTJ = Jacobian::Zero(6,J.cols());

    for (int i = 0; i < J.cols(); ++i)
    {
        AdTJ.col(i) = AdT(T, J.col(i));
    }

    return AdTJ;
}

//se3 AdP(const Vec3& p, const se3& s)
//{
//    //--------------------------------------------------------------------------
//    // w' = w
//    // v' = p x w + v
//    //--------------------------------------------------------------------------
//    se3 ret;
//    ret << s[0],
//           s[1],
//           s[2],
//           p[1]*s[2] - p[2]*s[1] + s[3],
//           p[2]*s[0] - p[0]*s[2] + s[4],
//           p[0]*s[1] - p[1]*s[0] + s[5];

//    return ret;
//}

//Jacobian AdRJac(const SE3& T, const Jacobian& J)
//{
//    Jacobian AdTJ(6,J.cols());

//    for (int i = 0; i < J.cols(); ++i)
//    {
//        AdTJ.col(i) = math::AdR(T, J.col(i));
//    }

//    return AdTJ;
//}

// re = Inv(T)*s*T
Eigen::Vector6d AdInvT(const Eigen::Isometry3d& T, const Eigen::Vector6d& s)
{
    Eigen::Vector6d ret = Eigen::Vector6d::Zero();
    double tmp[3] = {	s[3] + s[1]*T(2,3) - s[2]*T(1,3),
                        s[4] + s[2]*T(0,3) - s[0]*T(2,3),
                        s[5] + s[0]*T(1,3) - s[1]*T(0,3) };

    ret << T(0,0)*s[0] + T(1,0)*s[1] + T(2,0)*s[2],
           T(0,1)*s[0] + T(1,1)*s[1] + T(2,1)*s[2],
           T(0,2)*s[0] + T(1,2)*s[1] + T(2,2)*s[2],
           T(0,0)*tmp[0] + T(1,0)*tmp[1] + T(2,0)*tmp[2],
           T(0,1)*tmp[0] + T(1,1)*tmp[1] + T(2,1)*tmp[2],
           T(0,2)*tmp[0] + T(1,2)*tmp[1] + T(2,2)*tmp[2];

    return ret;
}

//se3 AdInvR(const SE3& T, const se3& s)
//{
//    se3 ret;

//    ret << T(0,0)*s[0] + T(1,0)*s[1] + T(2,0)*s[2],
//                T(0,1)*s[0] + T(1,1)*s[1] + T(2,1)*s[2],
//                T(0,2)*s[0] + T(1,2)*s[1] + T(2,2)*s[2],
//                T(0,0)*s[3] + T(1,0)*s[4] + T(2,0)*s[5],
//                T(0,1)*s[3] + T(1,1)*s[4] + T(2,1)*s[5],
//                T(0,2)*s[3] + T(1,2)*s[4] + T(2,2)*s[5];

//    return ret;
//}

Eigen::Vector6d AdInvRLinear(const Eigen::Isometry3d& T, const Eigen::Vector3d& v)
{
    Eigen::Vector6d ret = Eigen::Vector6d::Zero();

    ret << 0.0,
           0.0,
           0.0,
           T(0,0)*v[0] + T(1,0)*v[1] + T(2,0)*v[2],
           T(0,1)*v[0] + T(1,1)*v[1] + T(2,1)*v[2],
           T(0,2)*v[0] + T(1,2)*v[1] + T(2,2)*v[2];

    return ret;
}

Eigen::Vector6d ad(const Eigen::Vector6d& s1, const Eigen::Vector6d& s2)
{
    //--------------------------------------------------------------------------
    // ad(s1, s2) = | [w1]    0 | | w2 |
    //              | [v1] [w1] | | v2 |
    //
    //            = |          [w1]w2 |
    //              | [v1]w2 + [w1]v2 |
    //--------------------------------------------------------------------------
    Eigen::Vector6d ret = Eigen::Vector6d::Zero();

    ret << s1[1]*s2[2] - s1[2]*s2[1],
           s1[2]*s2[0] - s1[0]*s2[2],
           s1[0]*s2[1] - s1[1]*s2[0],
           s1[1]*s2[5] - s1[2]*s2[4] - s2[1]*s1[5] + s2[2]*s1[4],
           s1[2]*s2[3] - s1[0]*s2[5] - s2[2]*s1[3] + s2[0]*s1[5],
           s1[0]*s2[4] - s1[1]*s2[3] - s2[0]*s1[4] + s2[1]*s1[3];

    return ret;
}

Eigen::Vector6d dAdT(const Eigen::Isometry3d& T, const Eigen::Vector6d& t)
{
    Eigen::Vector6d ret = Eigen::Vector6d::Zero();

    double tmp[3] = {	t[0] - T(1,3)*t[5] + T(2,3)*t[4],
                        t[1] - T(2,3)*t[3] + T(0,3)*t[5],
                        t[2] - T(0,3)*t[4] + T(1,3)*t[3] };
    ret << T(0,0)*tmp[0] + T(1,0)*tmp[1] + T(2,0)*tmp[2],
           T(0,1)*tmp[0] + T(1,1)*tmp[1] + T(2,1)*tmp[2],
           T(0,2)*tmp[0] + T(1,2)*tmp[1] + T(2,2)*tmp[2],
           T(0,0)*t[3] + T(1,0)*t[4] + T(2,0)*t[5],
           T(0,1)*t[3] + T(1,1)*t[4] + T(2,1)*t[5],
           T(0,2)*t[3] + T(1,2)*t[4] + T(2,2)*t[5];

    return ret;
}

//dse3 dAdTLinear(const SE3& T, const Vec3& v)
//{
//    dse3 ret;
//    double tmp[3] = {	- T(1,3)*v[2] + T(2,3)*v[1],
//                        - T(2,3)*v[0] + T(0,3)*v[2],
//                        - T(0,3)*v[1] + T(1,3)*v[0] };
//    ret << T(0,0)*tmp[0] + T(1,0)*tmp[1] + T(2,0)*tmp[2],
//                T(0,1)*tmp[0] + T(1,1)*tmp[1] + T(2,1)*tmp[2],
//                T(0,2)*tmp[0] + T(1,2)*tmp[1] + T(2,2)*tmp[2],
//                T(0,0)*v[0] + T(1,0)*v[1] + T(2,0)*v[2],
//                T(0,1)*v[0] + T(1,1)*v[1] + T(2,1)*v[2],
//                T(0,2)*v[0] + T(1,2)*v[1] + T(2,2)*v[2];

//    return ret;
//}

Eigen::Vector6d dAdInvT(const Eigen::Isometry3d& T, const Eigen::Vector6d& t)
{
    Eigen::Vector6d ret = Eigen::Vector6d::Zero();

    double tmp[3] = { T(0,0)*t[3] + T(0,1)*t[4] + T(0,2)*t[5],
                      T(1,0)*t[3] + T(1,1)*t[4] + T(1,2)*t[5],
                      T(2,0)*t[3] + T(2,1)*t[4] + T(2,2)*t[5] };

    ret << T(1,3)*tmp[2] - T(2,3)*tmp[1] + T(0,0)*t[0] + T(0,1)*t[1] + T(0,2)*t[2],
           T(2,3)*tmp[0] - T(0,3)*tmp[2] + T(1,0)*t[0] + T(1,1)*t[1] + T(1,2)*t[2],
           T(0,3)*tmp[1] - T(1,3)*tmp[0] + T(2,0)*t[0] + T(2,1)*t[1] + T(2,2)*t[2],
           tmp[0], tmp[1], tmp[2];

    return ret;
}

Eigen::Vector6d dAdInvR(const Eigen::Isometry3d& T, const Eigen::Vector6d& t)
{
    Eigen::Vector6d ret = Eigen::Vector6d::Zero();

    ret << T(0,0)*t[0] + T(0,1)*t[1] + T(0,2)*t[2],
           T(1,0)*t[0] + T(1,1)*t[1] + T(1,2)*t[2],
           T(2,0)*t[0] + T(2,1)*t[1] + T(2,2)*t[2],
           T(0,0)*t[3] + T(0,1)*t[4] + T(0,2)*t[5],
           T(1,0)*t[3] + T(1,1)*t[4] + T(1,2)*t[5],
           T(2,0)*t[3] + T(2,1)*t[4] + T(2,2)*t[5];

    return ret;
}

//dse3 dAdInvPLinear(const Vec3& p, const Vec3& f)
//{
//    dse3 ret;

//    ret << p[1]*f[2] - p[2]*f[1],
//                p[2]*f[0] - p[0]*f[2],
//                p[0]*f[1] - p[1]*f[0],
//                f[0],
//                f[1],
//                f[2];

//    return ret;
//}

Eigen::Isometry3d EulerXYZ(const Eigen::Vector3d& angle)
{
    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();

    double c0 = cos(angle[0]);
    double s0 = sin(angle[0]);
    double c1 = cos(angle[1]);
    double s1 = sin(angle[1]);
    double c2 = cos(angle[2]);
    double s2 = sin(angle[2]);

    ret(0,0) = c1*c2;             ret(0,1) = -c1*s2;            ret(0,2) = s1;
    ret(1,0) = c0*s2 + c2*s0*s1;  ret(1,1) = c0*c2 - s0*s1*s2;  ret(1,2) = -c1*s0;
    ret(2,0) = s0*s2 - c0*c2*s1;  ret(2,1) = c2*s0 + c0*s1*s2;  ret(2,2) = c0*c1;

    return ret;
}

Eigen::Isometry3d EulerZYX(const Eigen::Vector3d& angle)
{
    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();

    double ca = cos(angle[0]);
    double sa = sin(angle[0]);
    double cb = cos(angle[1]);
    double sb = sin(angle[1]);
    double cg = cos(angle[2]);
    double sg = sin(angle[2]);

    ret(0,0) = ca*cb;  ret(0,1) = ca*sb*sg - sa*cg;  ret(0,2) = ca*sb*cg + sa*sg;
    ret(1,0) = sa*cb;  ret(1,1) = sa*sb*sg + ca*cg;  ret(1,2) = sa*sb*cg - ca*sg;
    ret(2,0) = -sb;    ret(2,1) = cb*sg;             ret(2,2) = cb*cg;

    return ret;
}

//SE3 EulerZYZ(const Vec3& angle)
//{
//    SE3 ret = SE3::Identity();

//    double ca = cos(angle[0]);
//    double sa = sin(angle[0]);
//    double cb = cos(angle[1]);
//    double sb = sin(angle[1]);
//    double cg = cos(angle[2]);
//    double sg = sin(angle[2]);

//    ret(0,0) = ca*cb*cg - sa*sg;  ret(0,1) = -ca*cb*sg - sa*cg;  ret(0,2) = ca*sb;
//    ret(1,0) = sa*cb*cg + ca*sg;  ret(1,1) = ca*cg - sa*cb*sg;   ret(1,2) = sa*sb;
//    ret(2,0) = -sb*cg;            ret(2,1) = sb*sg;              ret(2,2) = cb;

//    return ret;
//}

Eigen::Isometry3d EulerXYZ(const Eigen::Vector3d& angle, const Eigen::Vector3d& pos)
{
    Eigen::Isometry3d T = RotX(angle[0])*RotY(angle[1])*RotZ(angle[2]);

    T(0,3) = pos[0];
    T(1,3) = pos[1];
    T(2,3) = pos[2];

    return T;
}

//SE3 EulerZYX(const Vec3& angle, const Vec3& pos)
//{
//    SE3 ret;
//    double ca = cos(angle[0]);
//    double sa = sin(angle[0]);
//    double cb = cos(angle[1]);
//    double sb = sin(angle[1]);
//    double cg = cos(angle[2]);
//    double sg = sin(angle[2]);

//    ret(0,0) = ca*cb;  ret(0,1) = ca*sb*sg - sa*cg;  ret(0,2) = ca*sb*cg + sa*sg;  ret(0,3) = pos[0];
//    ret(1,0) = sa*cb;  ret(1,1) = sa*sb*sg + ca*cg;  ret(1,2) = sa*sb*cg - ca*sg;  ret(1,3) = pos[1];
//    ret(2,0) = -sb;    ret(2,1) = cb*sg;             ret(2,2) = cb*cg;             ret(2,3) = pos[2];

//    return ret;
//}

//SE3 EulerZYZ(const Vec3& angle, const Vec3& pos)
//{
//    SE3 ret;
//    double ca = cos(angle[0]);
//    double sa = sin(angle[0]);
//    double cb = cos(angle[1]);
//    double sb = sin(angle[1]);
//    double cg = cos(angle[2]);
//    double sg = sin(angle[2]);

//    ret(0,0) = ca*cb*cg - sa*sg;  ret(0,1) = -ca*cb*sg - sa*cg;  ret(0,2) = ca*sb;  ret(0,3) = pos[0];
//    ret(1,0) = sa*cb*cg + ca*sg;  ret(1,1) = ca*cg - sa*cb*sg;   ret(1,2) = sa*sb;  ret(1,3) = pos[1];
//    ret(2,0) = -sb*cg;            ret(2,1) = sb*sg;              ret(2,2) = cb;     ret(2,3) = pos[2];

//    return ret;
//}

// R = Exp(w)
// p = sin(t) / t*v + (t - sin(t)) / t^3*<w, v>*w + (1 - cos(t)) / t^2*(w X v)
// , when S = (w, v), t = |w|
Eigen::Isometry3d Exp(const Eigen::Vector6d& s)
{
    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
    double s2[] = { s[0]*s[0], s[1]*s[1], s[2]*s[2] };
    double s3[] = { s[0]*s[1], s[1]*s[2], s[2]*s[0] };
    double theta = sqrt(s2[0] + s2[1] + s2[2]), cos_t = cos(theta), alpha, beta, gamma;

    if ( theta > DART_EPSILON )
    {
        double sin_t = sin(theta);
        alpha = sin_t / theta;
        beta = (1.0 - cos_t) / theta / theta;
        gamma = (s[0]*s[3] + s[1]*s[4] + s[2]*s[5])*(theta - sin_t) / theta / theta / theta;
    }
    else
    {
        alpha = 1.0 - theta*theta/6.0;
        beta = 0.5 - theta*theta/24.0;
        gamma = (s[0]*s[3] + s[1]*s[4] + s[2]*s[5])/6.0 - theta*theta/120.0;
    }

    ret(0,0) = beta*s2[0] + cos_t;       ret(0,1) = beta*s3[0] - alpha*s[2];  ret(0,2) = beta*s3[2] + alpha*s[1];  ret(0,3) = alpha*s[3] + beta*(s[1]*s[5] - s[2]*s[4]) + gamma*s[0];
    ret(1,0) = beta*s3[0] + alpha*s[2];  ret(1,1) = beta*s2[1] + cos_t;       ret(1,2) = beta*s3[1] - alpha*s[0];  ret(1,3) = alpha*s[4] + beta*(s[2]*s[3] - s[0]*s[5]) + gamma*s[1];
    ret(2,0) = beta*s3[2] - alpha*s[1];  ret(2,1) = beta*s3[1] + alpha*s[0];  ret(2,2) = beta*s2[2] + cos_t;       ret(2,3) = alpha*s[5] + beta*(s[0]*s[4] - s[1]*s[3]) + gamma*s[2];

    return ret;
}

// I + sin(t) / t*[S] + (1 - cos(t)) / t^2*[S]^2, where t = |S|
Eigen::Isometry3d ExpAngular(const Eigen::Vector3d& S)
{
    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
    double s2[] = { S[0]*S[0], S[1]*S[1], S[2]*S[2] };
    double s3[] = { S[0]*S[1], S[1]*S[2], S[2]*S[0] };
    double theta = sqrt(s2[0] + s2[1] + s2[2]);
    double cos_t = cos(theta);
    double alpha = 0.0;
    double beta = 0.0;

    if (theta > DART_EPSILON)
    {
        alpha = sin(theta) / theta;
        beta = (1.0 - cos_t) / theta / theta;
    }
    else
    {
        alpha = 1.0 - theta*theta/6.0;
        beta = 0.5 - theta*theta/24.0;
    }

    ret(0,0) = beta*s2[0] + cos_t;       ret(0,1) = beta*s3[0] - alpha*S[2];  ret(0,2) = beta*s3[2] + alpha*S[1];
    ret(1,0) = beta*s3[0] + alpha*S[2];  ret(1,1) = beta*s2[1] + cos_t;       ret(1,2) = beta*s3[1] - alpha*S[0];
    ret(2,0) = beta*s3[2] - alpha*S[1];  ret(2,1) = beta*s3[1] + alpha*S[0];  ret(2,2) = beta*s2[2] + cos_t;

    return ret;
}

//// I + sin(t)*[S] + (1 - cos(t))*[S]^2,, where |S| = 1
//SE3 ExpAngular(const Axis& S, double theta)
//{
//    SE3 ret = SE3::Identity();
//    double s2[] = { S[0]*S[0], S[1]*S[1], S[2]*S[2] };

//    if ( abs(s2[0] + s2[1] + s2[2] - 1.0) > LIE_EPS ) return ExpAngular(theta*S);

//    double s3[] = { S[0]*S[1], S[1]*S[2], S[2]*S[0] };
//    double alpha = sin(theta), cos_t = cos(theta), beta = 1.0 - cos_t;

//    ret(0,0) = beta*s2[0] + cos_t;       ret(0,1) = beta*s3[0] - alpha*S[2];  ret(0,2) = beta*s3[2] + alpha*S[1];
//    ret(1,0) = beta*s3[0] + alpha*S[2];  ret(1,1) = beta*s2[1] + cos_t;       ret(1,2) = beta*s3[1] - alpha*S[0];
//    ret(2,0) = beta*s3[2] - alpha*S[1];  ret(2,1) = beta*s3[1] + alpha*S[0];  ret(2,2) = beta*s2[2] + cos_t;

//    return ret;
//}

Eigen::Isometry3d ExpLinear(const Eigen::Vector3d& s)
{
    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();

    ret(0,3) = s[0];
    ret(1,3) = s[1];
    ret(2,3) = s[2];

    return ret;
}

Eigen::Isometry3d RotX(double t)
{
    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
    double c = cos(t);
    double s = sin(t);

    ret(1,1) = c;  ret(1,2) = -s;
    ret(2,1) = s;  ret(2,2) = c;

    return ret;
}

Eigen::Isometry3d RotY(double t)
{
    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
    double c = cos(t);
    double s = sin(t);

    ret(0,0) = c;   ret(0,2) = s;
    ret(2,0) = -s;  ret(2,2) = c;

    return ret;
}

Eigen::Isometry3d RotZ(double t)
{
    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
    double c = cos(t);
    double s = sin(t);

    ret(0,0) = c;  ret(0,1) = -s;
    ret(1,0) = s;  ret(1,1) = c;

    return ret;
}

//SE3 Normalize(const SE3& T)
//{
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
//}

//Axis Reparameterize(const Axis& s)
//{
//    double theta = sqrt(s[0]*s[0] + s[1]*s[1] + s[2]*s[2]);
//    double eta = theta < LIE_EPS ? 1.0 : 1.0 - (double)((int)(theta / M_PI + 1.0) / 2)*M_2PI / theta;

//    return eta*s;
//}

//Axis AdInvTAngular(const SE3& T, const Axis& v)
//{
//    return Axis(T(0,0)*v[0] + T(1,0)*v[1] + T(2,0)*v[2],
//                T(0,1)*v[0] + T(1,1)*v[1] + T(2,1)*v[2],
//                T(0,2)*v[0] + T(1,2)*v[1] + T(2,2)*v[2]);
//}

//Axis ad(const Axis& s1, const se3& s2)
//{
//    return Axis(s2[2]*s1[1] - s2[1]*s1[2],
//                s2[0]*s1[2] - s2[2]*s1[0],
//                s2[1]*s1[0] - s2[0]*s1[1]);
//}

//Axis ad_Axis_Axis(const Axis& s1, const Axis& s2)
//{
//    return Axis(s2[2]*s1[1] - s2[1]*s1[2],
//                s2[0]*s1[2] - s2[2]*s1[0],
//                s2[1]*s1[0] - s2[0]*s1[1]);
//}

Eigen::Vector6d dad(const Eigen::Vector6d& s, const Eigen::Vector6d& t)
{
    Eigen::Vector6d ret = Eigen::Vector6d::Zero();

    ret << t[1] * s[2] - t[2] * s[1] + t[4] * s[5] - t[5] * s[4],
           t[2] * s[0] - t[0] * s[2] + t[5] * s[3] - t[3] * s[5],
           t[0] * s[1] - t[1] * s[0] + t[3] * s[4] - t[4] * s[3],
           t[4] * s[2] - t[5] * s[1],
           t[5] * s[0] - t[3] * s[2],
           t[3] * s[1] - t[4] * s[0];

    return ret;
}

Inertia Transform(const Eigen::Isometry3d& T, const Inertia& AI)
{
    // operation count: multiplication = 186, addition = 117, subtract = 21

    Inertia ret = Inertia::Identity();

    double d0 = AI(0,3) + T(2,3) * AI(3,4) - T(1,3) * AI(3,5);
    double d1 = AI(1,3) - T(2,3) * AI(3,3) + T(0,3) * AI(3,5);
    double d2 = AI(2,3) + T(1,3) * AI(3,3) - T(0,3) * AI(3,4);
    double d3 = AI(0,4) + T(2,3) * AI(4,4) - T(1,3) * AI(4,5);
    double d4 = AI(1,4) - T(2,3) * AI(3,4) + T(0,3) * AI(4,5);
    double d5 = AI(2,4) + T(1,3) * AI(3,4) - T(0,3) * AI(4,4);
    double d6 = AI(0,5) + T(2,3) * AI(4,5) - T(1,3) * AI(5,5);
    double d7 = AI(1,5) - T(2,3) * AI(3,5) + T(0,3) * AI(5,5);
    double d8 = AI(2,5) + T(1,3) * AI(3,5) - T(0,3) * AI(4,5);
    double e0 = AI(0,0) + T(2,3) * AI(0,4) - T(1,3) * AI(0,5) + d3 * T(2,3) - d6 * T(1,3);
    double e3 = AI(0,1) + T(2,3) * AI(1,4) - T(1,3) * AI(1,5) - d0 * T(2,3) + d6 * T(0,3);
    double e4 = AI(1,1) - T(2,3) * AI(1,3) + T(0,3) * AI(1,5) - d1 * T(2,3) + d7 * T(0,3);
    double e6 = AI(0,2) + T(2,3) * AI(2,4) - T(1,3) * AI(2,5) + d0 * T(1,3) - d3 * T(0,3);
    double e7 = AI(1,2) - T(2,3) * AI(2,3) + T(0,3) * AI(2,5) + d1 * T(1,3) - d4 * T(0,3);
    double e8 = AI(2,2) + T(1,3) * AI(2,3) - T(0,3) * AI(2,4) + d2 * T(1,3) - d5 * T(0,3);
    double f0 = T(0,0) * e0 + T(1,0) * e3 + T(2,0) * e6;
    double f1 = T(0,0) * e3 + T(1,0) * e4 + T(2,0) * e7;
    double f2 = T(0,0) * e6 + T(1,0) * e7 + T(2,0) * e8;
    double f3 = T(0,0) * d0 + T(1,0) * d1 + T(2,0) * d2;
    double f4 = T(0,0) * d3 + T(1,0) * d4 + T(2,0) * d5;
    double f5 = T(0,0) * d6 + T(1,0) * d7 + T(2,0) * d8;
    double f6 = T(0,1) * e0 + T(1,1) * e3 + T(2,1) * e6;
    double f7 = T(0,1) * e3 + T(1,1) * e4 + T(2,1) * e7;
    double f8 = T(0,1) * e6 + T(1,1) * e7 + T(2,1) * e8;
    double g0 = T(0,1) * d0 + T(1,1) * d1 + T(2,1) * d2;
    double g1 = T(0,1) * d3 + T(1,1) * d4 + T(2,1) * d5;
    double g2 = T(0,1) * d6 + T(1,1) * d7 + T(2,1) * d8;
    double g3 = T(0,2) * d0 + T(1,2) * d1 + T(2,2) * d2;
    double g4 = T(0,2) * d3 + T(1,2) * d4 + T(2,2) * d5;
    double g5 = T(0,2) * d6 + T(1,2) * d7 + T(2,2) * d8;
    double h0 = T(0,0) * AI(3,3) + T(1,0) * AI(3,4) + T(2,0) * AI(3,5);
    double h1 = T(0,0) * AI(3,4) + T(1,0) * AI(4,4) + T(2,0) * AI(4,5);
    double h2 = T(0,0) * AI(3,5) + T(1,0) * AI(4,5) + T(2,0) * AI(5,5);
    double h3 = T(0,1) * AI(3,3) + T(1,1) * AI(3,4) + T(2,1) * AI(3,5);
    double h4 = T(0,1) * AI(3,4) + T(1,1) * AI(4,4) + T(2,1) * AI(4,5);
    double h5 = T(0,1) * AI(3,5) + T(1,1) * AI(4,5) + T(2,1) * AI(5,5);

    ret(0,0) = f0 * T(0,0) + f1 * T(1,0) + f2 * T(2,0);
    ret(0,1) = f0 * T(0,1) + f1 * T(1,1) + f2 * T(2,1);
    ret(0,2) = f0 * T(0,2) + f1 * T(1,2) + f2 * T(2,2);
    ret(0,3) = f3 * T(0,0) + f4 * T(1,0) + f5 * T(2,0);
    ret(0,4) = f3 * T(0,1) + f4 * T(1,1) + f5 * T(2,1);
    ret(0,5) = f3 * T(0,2) + f4 * T(1,2) + f5 * T(2,2);
    ret(1,1) = f6 * T(0,1) + f7 * T(1,1) + f8 * T(2,1);
    ret(1,2) = f6 * T(0,2) + f7 * T(1,2) + f8 * T(2,2);
    ret(1,3) = g0 * T(0,0) + g1 * T(1,0) + g2 * T(2,0);
    ret(1,4) = g0 * T(0,1) + g1 * T(1,1) + g2 * T(2,1);
    ret(1,5) = g0 * T(0,2) + g1 * T(1,2) + g2 * T(2,2);
    ret(2,2) = (T(0,2) * e0 + T(1,2) * e3 + T(2,2) * e6) * T(0,2) + (T(0,2) * e3 + T(1,2) * e4 + T(2,2) * e7) * T(1,2) + (T(0,2) * e6 + T(1,2) * e7 + T(2,2) * e8) * T(2,2);
    ret(2,3) = g3 * T(0,0) + g4 * T(1,0) + g5 * T(2,0);
    ret(2,4) = g3 * T(0,1) + g4 * T(1,1) + g5 * T(2,1);
    ret(2,5) = g3 * T(0,2) + g4 * T(1,2) + g5 * T(2,2);
    ret(3,3) = h0 * T(0,0) + h1 * T(1,0) + h2 * T(2,0);
    ret(3,4) = h0 * T(0,1) + h1 * T(1,1) + h2 * T(2,1);
    ret(3,5) = h0 * T(0,2) + h1 * T(1,2) + h2 * T(2,2);
    ret(4,4) = h3 * T(0,1) + h4 * T(1,1) + h5 * T(2,1);
    ret(4,5) = h3 * T(0,2) + h4 * T(1,2) + h5 * T(2,2);
    ret(5,5) = (T(0,2) * AI(3,3) + T(1,2) * AI(3,4) + T(2,2) * AI(3,5)) * T(0,2) + (T(0,2) * AI(3,4) + T(1,2) * AI(4,4) + T(2,2) * AI(4,5)) * T(1,2) + (T(0,2) * AI(3,5) + T(1,2) * AI(4,5) + T(2,2) * AI(5,5)) * T(2,2);

    ret.triangularView<Eigen::StrictlyLower>() = ret.transpose();

    return ret;
}

bool VerifySE3(const Eigen::Isometry3d& _T)
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; j++)
            if (_T(i,j) != _T(i,j))
                return false;

    if (_T(3,0) != 0.0)
        return false;

    if (_T(3,1) != 0.0)
        return false;

    if (_T(3,2) != 0.0)
        return false;

    if (_T(3,3) != 1.0)
        return false;

    if (fabs(fabs(_T.rotation().determinant()) - 1.0) > 0.001)
        return false;

    return true;
}

bool Verifyse3(const Eigen::Vector6d& _V)
{
    for (int i = 0; i < 6; ++i)
        if (_V(i) != _V(i))
            return false;

//    for (int i = 0; i < 6; ++i)
//        if (_V(i) > 1000.0)
//            return false;

    return true;
}

} // namespace math
} // namespace dart
