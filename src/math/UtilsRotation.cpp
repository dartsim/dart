/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
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

#include "math/UtilsRotation.h"
#include "math/UtilsMath.h"

// Standard Libraries
#include <cstdio>
#include <iostream>
using namespace std;
using namespace Eigen;

namespace dart_math {
    Quaterniond matrixToQuat(Matrix3d& mat) {
        return Quaterniond(mat);
    }

    Matrix3d quatToMatrix(Quaterniond& q) {
        return Matrix3d(q);
    }

    Quaterniond expToQuat(Vector3d& v) {
        double mag = v.norm();
        if(mag > 1e-10){
            Quaterniond q(AngleAxisd(mag, v / mag));
            return q;
        }
        else{
            Quaterniond q(1, 0, 0, 0);
            return q;
        }
    }

    Vector3d quatToExp(Quaterniond& q) {
        AngleAxisd aa(q);
        Vector3d v = aa.axis();
        return v*aa.angle();
    }

// Note: xyz order means matrix is Rz*Ry*Rx i.e a point as transformed as Rz*Ry*Rx(p)
// coordinate system transformation as in GL will be written as glRotate(z); glRotate(y); glRotate(x)
    Vector3d matrixToEuler(Matrix3d& m, RotationOrder order) {
        double x, y, z;

        if(order==XYZ) {
            if(m(2, 0) > (1.0-M_EPSILON)) {
                cout << "North Pole" << endl;
                x = atan2(m(0, 1), m(0, 2));
                y = -M_PI / 2.0;
                z = 0.0;
            }
            if(m(2, 0) < -(1.0-M_EPSILON)) {
                cout << "South Pole" << endl;
                x = atan2(m(0, 1), m(0, 2));
                y = M_PI / 2.0;
                z = 0.0;
            }
            x = atan2(m(2, 1), m(2, 2));
            y = -asin(m(2, 0));
            z = atan2(m(1, 0), m(0, 0));
            return Vector3d(x,y,z);	// order of return is the order of input
        }

        if(order==ZYX) {
            if(m(0, 2) > (1.0-M_EPSILON)) {
                cout << "North Pole" << endl;
                z = atan2(m(1, 0), m(1, 1));
                y = M_PI / 2.0;
                x = 0.0;
            }
            if(m(0, 2) < -(1.0-M_EPSILON)) {
                cout << "South Pole" << endl;
                z = atan2(m(1, 0), m(1, 1));
                y = -M_PI / 2.0;
                x = 0.0;
            }
            z = -atan2(m(0, 1), m(0, 0));
            y = asin(m(0, 2));
            x = -atan2(m(1, 2), m(2, 2));
            return Vector3d(z,y,x);	// order of return is the order of input
        }

        if(order==YZX) {
            if(m(0, 1) > (1.0-M_EPSILON)) {
                cout << "North Pole" << endl;
                y = atan2(m(1, 2), m(1, 0));
                z = -M_PI / 2.0;
                x = 0.0;
            }
            if(m(0, 1) < -(1.0-M_EPSILON)) {
                cout << "South Pole" << endl;
                y = atan2(m(1, 2), m(1, 0));
                z = M_PI / 2.0;
                x = 0.0;
            }
            y = atan2(m(0, 2), m(0, 0));
            z = -asin(m(0, 1));
            x = atan2(m(2, 1), m(1, 1));
            return Vector3d(y,z,x);	// order of return is the order of input
        }

        if(order==XZY) {
            if(m(1, 0) > (1.0-M_EPSILON)) {
                cout << "North Pole" << endl;
                x = -atan2(m(0, 2), m(0, 1));
                z = M_PI / 2.0;
                y = 0.0;
            }
            if(m(1, 0) < -(1.0-M_EPSILON)) {
                cout << "South Pole" << endl;
                x = -atan2(m(0, 2), m(0, 1));
                z = -M_PI / 2.0;
                y = 0.0;
            }
            x = -atan2(m(1, 2), m(1, 1));
            z = asin(m(1, 0));
            y = -atan2(m(2, 0), m(0, 0));
            return Vector3d(x,z,y);	// order of return is the order of input
        }

        if(order==YXZ){
            if(m(2, 1) > (1.0-M_EPSILON)) {
                cout << "North Pole" << endl;
                y = atan2(m(0, 2), m(0, 0));
                x = M_PI / 2.0;
                z = 0.0;
            }
            if(m(2, 1) < -(1.0-M_EPSILON)) {
                cout << "South Pole" << endl;
                y = atan2(m(0, 2), m(0, 0));
                x = -M_PI / 2.0;
                z = 0.0;
            }
            y = -atan2(m(2, 0), m(2, 2));
            x = asin(m(2, 1));
            z = -atan2(m(0, 1), m(1, 1));
            return Vector3d(y, x, z);	// order of return is the order of input
        }

        if(order==ZXY){
            if(m(1, 2) > (1.0-M_EPSILON)) {
                cout << "North Pole" << endl;
                z = -atan2(m(0, 1), m(0, 0));
                x = -M_PI / 2.0;
                y = 0.0;
            }
            if(m(1, 2) < -(1.0-M_EPSILON)) {
                cout << "South Pole" << endl;
                z = -atan2(m(0, 1), m(0, 0));
                x = M_PI / 2.0;
                y = 0.0;
            }
            z = atan2(m(1, 0), m(1, 1));
            x = -asin(m(1, 2));
            y = atan2(m(0, 2), m(2, 2));
            return Vector3d(z,x,y);	// order of return is the order of input
        }
        printf("Matrix_to_Euler - Do not support rotation order %d. Make sure letters are in lowercase\n", order);
        return Vector3d(0.0, 0.0, 0.0);
    }

    Matrix3d eulerToMatrix(Vector3d& v, RotationOrder order) {
        if(order==XYZ){
            return eulerToMatrixZ(v[2])*eulerToMatrixY(v[1])*eulerToMatrixX(v[0]);
        }
        if(order==ZYX){
            return eulerToMatrixX(v[0])*eulerToMatrixY(v[1])*eulerToMatrixZ(v[2]);
        }
        if(order==YZX){
            return eulerToMatrixX(v[0])*eulerToMatrixZ(v[2])*eulerToMatrixY(v[1]);
        }
        if(order==XZY){
            return eulerToMatrixY(v[1])*eulerToMatrixZ(v[2])*eulerToMatrixX(v[0]);
        }
        if(order==YXZ){
            return eulerToMatrixZ(v[2])*eulerToMatrixX(v[0])*eulerToMatrixY(v[1]);
        }
        if(order==ZXY){
            return eulerToMatrixY(v[1])*eulerToMatrixX(v[0])*eulerToMatrixZ(v[2]);
        }

        printf("euler_to_matrix - Do not support rotation order %d. Make sure letters are in lowercase\n", order);
        return Matrix3d::Zero();
    }

    Matrix3d eulerToMatrixX(double x) {
        Matrix3d mat = Matrix3d::Zero();
        double cosangle = cos(x);
        double sinangle = sin(x);
        mat(0, 0) = 1.0;
        mat(1, 1) = cosangle;
        mat(1, 2) = -sinangle;
        mat(2, 1) = sinangle;
        mat(2, 2) = cosangle;
        return mat;
    }

    Matrix3d eulerToMatrixY(double y) {
        Matrix3d mat = Matrix3d::Zero();
        double cosangle = cos(y);
        double sinangle = sin(y);
        mat(1, 1) = 1.0;
        mat(2, 2) = cosangle;
        mat(2, 0) = -sinangle;
        mat(0, 2) = sinangle;
        mat(0, 0) = cosangle;
        return mat;
    }

    Matrix3d eulerToMatrixZ(double z) {
        Matrix3d mat = Matrix3d::Zero();
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
    Matrix3d quatDeriv(const Quaterniond& q, int el){
        Matrix3d mat = Matrix3d::Zero();
        switch(el){
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

    Matrix3d quatSecondDeriv(const Quaterniond& q, int el1, int el2){
        Matrix3d mat = Matrix3d::Zero();

        // wrt same dof
        if(el1==el2){
            switch(el1){
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
        else {
            // arrange in increasing order
            if(el1>el2){
                int temp=el2;
                el2=el1;
                el1=temp;
            }
            switch(el1){
            case 0:	// wrt w
                switch(el2){
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
                switch(el2){
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
                switch(el2){
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

    Vector3d rotatePoint(const Quaterniond& q, const Vector3d& pt) {
        Quaterniond quat_pt(0, pt[0], pt[1], pt[2]);
        Quaterniond qinv = q.inverse();

        Quaterniond rot = q*quat_pt*qinv;

        // check below - assuming same format of point achieved
        Vector3d temp;
        //VLOG(1)<<"Point before: "<<0<<" "<<pt.x<<" "<<pt.y<<" "<<pt.z<<"\n";
        //VLOG(1)<<"Point after:  "<<rot.x<<" "<<rot.y<<" "<<rot.z<<" "<<rot.w<<"\n";
        temp[0]=rot.x();
        temp[1]=rot.y();
        temp[2]=rot.z();

        //VLOG(1)<<"Point after rotation: "<<temp[0]<<" "<<temp[1]<<" "<<temp[2]<<endl;
        return temp;
    }

    Vector3d rotatePoint(const Quaterniond& q, double x, double y, double z){
        Vector3d pt;
        pt[0]=x;
        pt[1]=y;
        pt[2]=z;

        return rotatePoint(q, pt);
    }

    // ----------- expmap computations -------------

#define EPSILON_EXPMAP_THETA 1.0e-3

    Matrix3d expMapRot(const Vector3d &_q){
        double theta = _q.norm();

        Matrix3d R = Matrix3d::Zero();
        Matrix3d qss =  dart_math::makeSkewSymmetric(_q);
        Matrix3d qss2 =  qss*qss;

        if(theta<EPSILON_EXPMAP_THETA){
            R = Matrix3d::Identity() + qss + 0.5*qss2;
        }
        else {
            R = Matrix3d::Identity() + (sin(theta)/theta)*qss + ((1-cos(theta))/(theta*theta))*qss2;
        }
        return R;
    }

    Matrix3d expMapJac(const Vector3d &_q){
        double theta = _q.norm();

        Matrix3d J = Matrix3d::Zero();
        Matrix3d qss =  dart_math::makeSkewSymmetric(_q);
        Matrix3d qss2 =  qss*qss;

        if(theta<EPSILON_EXPMAP_THETA){
            J = Matrix3d::Identity() + 0.5*qss +  (1.0/6.0)*qss2;
        }
        else {
            J = Matrix3d::Identity() + ((1-cos(theta))/(theta*theta))*qss + ((theta-sin(theta))/(theta*theta*theta))*qss2;
        }
        return J;
    }

    Matrix3d expMapJacDot(const Vector3d &_q, const Vector3d &_qdot){
        double theta = _q.norm();

        Matrix3d Jdot = Matrix3d::Zero();
        Matrix3d qss =  dart_math::makeSkewSymmetric(_q);
        Matrix3d qss2 =  qss*qss;
        Matrix3d qdss = dart_math::makeSkewSymmetric(_qdot);
        double ttdot = _q.dot(_qdot);   // theta*thetaDot
        double st = sin(theta);
        double ct = cos(theta);
        double t2 = theta*theta;
        double t3 = t2*theta;
        double t4 = t3*theta;
        double t5 = t4*theta;

        if(theta<EPSILON_EXPMAP_THETA){
            Jdot = 0.5*qdss + (1.0/6.0)*(qss*qdss + qdss*qss);
            Jdot += (-1.0/12)*ttdot*qss + (-1.0/60)*ttdot*qss2;
        }
        else {
            Jdot = ((1-ct)/t2)*qdss + ((theta-st)/t3)*(qss*qdss + qdss*qss);
            Jdot += ((theta*st + 2*ct - 2)/t4)*ttdot*qss + ((3*st - theta*ct - 2*theta)/t5)*ttdot*qss2;
        }
        return Jdot;
    }

    Matrix3d expMapJacDeriv( const Vector3d &_q, int _qi ) {
        assert(_qi>=0 && _qi<=2);
        Vector3d qdot = Vector3d::Zero();
        qdot[_qi] = 1.0;
        return expMapJacDot(_q, qdot);
    }

} // namespace utils
