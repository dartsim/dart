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

#include "TrfmRotateExpmap.h"
#include "Dof.h"
#include "math/UtilsRotation.h"
#include "math/UtilsMath.h"
#include "renderer/RenderInterface.h"


using namespace std;
using namespace Eigen;

namespace kinematics {

TrfmRotateExpMap::TrfmRotateExpMap(Dof *x, Dof *y, Dof *z, const char* _name){
    mDofs.resize(3);
    mDofs[0]=x;
    mDofs[1]=y;
    mDofs[2]=z;
    x->setTrans(this);
    y->setTrans(this);
    z->setTrans(this);
    mType = Transformation::T_ROTATEEXPMAP;
    if(_name!=NULL)
        strcpy(mName, _name);
    else
        strcpy(mName, "EXPMAP");
}

void TrfmRotateExpMap::computeTransform() {
    Vector3d q(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());

    Matrix3d rot = dart_math::expMapRot(q);
    mTransform.setZero();
    mTransform.topLeftCorner<3,3>() = rot;
    mTransform(3, 3) = 1.0;
}

Matrix4d TrfmRotateExpMap::getInvTransform(){
    if(mDirty){
        computeTransform();
        mDirty=false;
    }
    return mTransform.transpose();
}


Matrix4d TrfmRotateExpMap::getDeriv(const Dof *d) const
{
    Vector3d q(mDofs[0]->getValue(),
            mDofs[1]->getValue(),
            mDofs[2]->getValue());

    // derivative wrt which dof
    int j = -1;
    for (unsigned int i=0; i<mDofs.size(); i++)
        if (d==mDofs[i]) j=i;
    assert(j!=-1);
    assert(j>=0 && j<=2);

    Matrix3d R = dart_math::expMapRot(q);
    Matrix3d J = dart_math::expMapJac(q);
    Matrix3d dRdj = dart_math::makeSkewSymmetric(J.col(j))*R;

    Matrix4d dRdj4d = Matrix4d::Zero();
    dRdj4d.topLeftCorner<3,3>() = dRdj;

    return dRdj4d;
}

Matrix4d TrfmRotateExpMap::getSecondDeriv(const Dof *q1, const Dof *q2) const
{
    Vector3d q(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());

    // derivative wrt which mDofs
    int j=-1, k=-1;
    for(unsigned int i=0; i<mDofs.size(); i++) {
        if(q1==mDofs[i]) j=i;
        if(q2==mDofs[i]) k=i;
    }
    assert(j!=-1);
    assert(k!=-1);
    assert(j>=0 && j<=2);
    assert(k>=0 && k<=2);

    Matrix3d R = dart_math::expMapRot(q);
    Matrix3d J = dart_math::expMapJac(q);
    Matrix3d Jjss = dart_math::makeSkewSymmetric(J.col(j));
    Matrix3d Jkss = dart_math::makeSkewSymmetric(J.col(k));
    Matrix3d dJjdkss = dart_math::makeSkewSymmetric(dart_math::expMapJacDeriv(q, k).col(j));

    Matrix3d d2Rdidj = (Jjss*Jkss + dJjdkss)*R;

    Matrix4d d2Rdidj4 = Matrix4d::Zero();
    d2Rdidj4.topLeftCorner<3,3>() = d2Rdidj;

    return d2Rdidj4;
}

Eigen::MatrixXd TrfmRotateExpMap::getJacobian() const {
    // Assume the number of dofs is 3.
    assert(getNumDofs() == 3);

    Eigen::MatrixXd J = Eigen::Matrix<double,6,3>::Zero();

    J.topLeftCorner<3,3>() = Eigen::Matrix3d::Identity();

    Matrix4d X = getDeriv(mDofs[0]);
    Matrix4d Y = getDeriv(mDofs[1]);
    Matrix4d Z = getDeriv(mDofs[2]);

    Matrix4d X2 = mTransform.inverse() * X;
    Matrix4d Y2 = mTransform.inverse() * Y;
    Matrix4d Z2 = mTransform.inverse() * Z;

    Vector3d x = dart_math::fromSkewSymmetric(X2.topLeftCorner<3,3>());
    Vector3d y = dart_math::fromSkewSymmetric(Y2.topLeftCorner<3,3>());
    Vector3d z = dart_math::fromSkewSymmetric(Z2.topLeftCorner<3,3>());

    return J;
}

void TrfmRotateExpMap::applyGLTransform(renderer::RenderInterface* _ri) const{
    Vector3d v(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());
    double theta = v.norm();
    Vector3d vhat = Vector3d::Zero();
    if(!dart_math::isZero(theta)) {
        vhat= v/theta;
        _ri->rotate(vhat, theta * 180 / M_PI);
    }

}
} // namespace kinematics
