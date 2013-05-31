/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
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

#include "TrfmRotateQuat.h"
#include "math/UtilsRotation.h"
#include "renderer/RenderInterface.h"

#include <cassert>
using namespace std;
using namespace Eigen;

#include "Dof.h"

namespace kinematics {

TrfmRotateQuat::TrfmRotateQuat(Dof *w, Dof *x, Dof *y, Dof *z, const char *_name){
    mDofs.clear();
    mDofs.resize(4);
    mDofs[0]=w;
    mDofs[1]=x;
    mDofs[2]=y;
    mDofs[3]=z;
    mType = Transformation::T_ROTATEQUAT;
    if(_name != NULL)
        strcpy(mName, _name);
    else
        strcpy(mName, "QUAT");
}

void TrfmRotateQuat::computeTransform(){
    // Quaternion constructor takes (w, x, y, z)
    Quaterniond q(mDofs[0]->getValue(), mDofs[1]->getValue(),mDofs[2]->getValue(),mDofs[3]->getValue());
    q.normalize();

    mTransform.setZero();
    Matrix3d rot = q.matrix();
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            mTransform(i, j) = rot(i, j);
        }
    }
    mTransform(3, 3) = 1.0;
}

Matrix4d TrfmRotateQuat::getDeriv(const Dof *d) const{
    Quaterniond q(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue(), mDofs[3]->getValue());
    q.normalize();

    Matrix3d mat = Matrix3d::Zero();
    Matrix4d ret = Matrix4d::Zero();

    int el=-1;
    for(int i=0; i<4; i++) if(d==mDofs[i]) el=i;
    assert(el!=-1);
    mat = dart_math::quatDeriv(q, el);

    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++)
            ret(i, j) = mat(i, j);
    }

    return ret;
}

Matrix4d TrfmRotateQuat::getSecondDeriv(const Dof *d1, const Dof *d2) const{
    Quaterniond q(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue(), mDofs[3]->getValue());
    q.normalize();

    Matrix3d mat = Matrix3d::Zero();
    Matrix4d ret = Matrix4d::Identity();

    int el1=-1, el2=-1;
    for(int i=0; i<4; i++) {
        if(d1==mDofs[i]) el1=i;
        if(d2==mDofs[i]) el2=i;
    }
    assert(el1!=-1);
    assert(el2!=-1);
    mat = dart_math::quatSecondDeriv(q, el1, el2);

    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++)
            ret(i, j) = mat(i, j);
    }
    return ret;
}

Eigen::MatrixXd TrfmRotateQuat::getJacobian() const
{
    assert(0);
    assert(getNumDofs() == 4);

    Eigen::MatrixXd J = Eigen::Matrix<double,6,4>::Zero();

    // TODO: NOT IMPLEMENTED !
    J.topLeftCorner<3,3>() = Eigen::Matrix3d::Identity();

    return J;
}

void TrfmRotateQuat::applyGLTransform(renderer::RenderInterface* _ri) const {
    Quaterniond q(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue(), mDofs[3]->getValue());
    q.normalize();

    AngleAxisd angleAxis(q);
    Vector3d a = angleAxis.axis();
    double theta = angleAxis.angle();
    if(a.dot(a)>M_EPSILON*M_EPSILON && theta>M_EPSILON) {
        if (_ri) _ri->rotate(a, theta*180/M_PI);
    }
}
} // namespace kinematics
