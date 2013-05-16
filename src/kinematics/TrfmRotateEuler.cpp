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

#include "TrfmRotateEuler.h"
#include "Dof.h"
#include "renderer/RenderInterface.h"

using namespace Eigen;

namespace kinematics {
////////////////////////////////////////////////////////////////////////////////
// rotation about X axis
TrfmRotateEulerX::TrfmRotateEulerX(Dof *q, const char *_name){
    mDofs.push_back(q);
    q->setTrans(this);
    mType = Transformation::T_ROTATEX;
    if(_name!=NULL)
        strcpy(mName, _name);
    else
        strcpy(mName, "EulerX");
}

void TrfmRotateEulerX::applyGLTransform(renderer::RenderInterface* _ri) const{
	if (_ri)
		_ri->rotate(Vector3d(1.0, 0.0, 0.0), mDofs[0]->getValue()*180/M_PI);
}

void TrfmRotateEulerX::computeTransform(){
    mTransform.setZero();
    double cosq = cos(mDofs[0]->getValue());
    double sinq = sin(mDofs[0]->getValue());
    mTransform(A_X, A_X) = 1.0;
    mTransform(A_Y, A_Y) = cosq;
    mTransform(A_Y, A_Z) = -sinq;
    mTransform(A_Z, A_Y) = sinq;
    mTransform(A_Z, A_Z) = cosq;
    mTransform(3, 3) = 1.0;
}

Matrix4d TrfmRotateEulerX::getDeriv(const Dof *q) const{
    Matrix4d ret = Matrix4d::Zero();
    if(q==mDofs[0]){
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        ret(A_Y, A_Y) = -sinq;
        ret(A_Y, A_Z) = -cosq;
        ret(A_Z, A_Y) = cosq;
        ret(A_Z, A_Z) = -sinq;
    }
    return ret;
}

Matrix4d TrfmRotateEulerX::getSecondDeriv(const Dof *q1, const Dof *q2) const{
    Matrix4d ret = Matrix4d::Zero();
    if(mDofs[0]==q1 && mDofs[0]==q2){
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        ret(A_Y, A_Y) = -cosq;
        ret(A_Y, A_Z) = sinq;
        ret(A_Z, A_Y) = -sinq;
        ret(A_Z, A_Z) = -cosq;
    }
    return ret;
}

Eigen::MatrixXd TrfmRotateEulerX::getJacobian() const {
    Eigen::Matrix<double,6,1> J = Eigen::Matrix<double,6,1>::Zero();
    J(0) = 1.0;
    return J;
    Vector3d a;
}

Matrix4d TrfmRotateEulerX::getInvTransform(){
    Matrix4d ret = Matrix4d::Identity();
    double cosq = cos(mDofs[0]->getValue());
    double sinq = sin(mDofs[0]->getValue());
    ret(A_Y, A_Y) = cosq;
    ret(A_Y, A_Z) = sinq;
    ret(A_Z, A_Y) = -sinq;
    ret(A_Z, A_Z) = cosq;
    return ret;
}

////////////////////////////////////////////////////////////////////////////////
// rotation about Y axis
TrfmRotateEulerY::TrfmRotateEulerY(Dof *q, const char *_name){
    mDofs.push_back(q);
    q->setTrans(this);
    mType = Transformation::T_ROTATEY;
    if(_name!=NULL)
        strcpy(mName, _name);
    else
        strcpy(mName, "EulerY");
}

void TrfmRotateEulerY::applyGLTransform(renderer::RenderInterface* _ri) const{
	if (_ri)
		_ri->rotate(Vector3d(0.0, 1.0, 0.0), mDofs[0]->getValue()*180/M_PI);

}

void TrfmRotateEulerY::computeTransform(){
    mTransform.setZero();
    double cosq = cos(mDofs[0]->getValue());
    double sinq = sin(mDofs[0]->getValue());
    mTransform(A_Y, A_Y) = 1.0;
    mTransform(A_X, A_X) = cosq;
    mTransform(A_X, A_Z) = sinq;
    mTransform(A_Z, A_X) = -sinq;
    mTransform(A_Z, A_Z) = cosq;
    mTransform(3, 3) = 1.0;
}

Matrix4d TrfmRotateEulerY::getDeriv(const Dof *q) const{
    Matrix4d ret = Matrix4d::Zero();
    if(q==mDofs[0]){
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        ret(A_X, A_X) = -sinq;
        ret(A_X, A_Z) = cosq;
        ret(A_Z, A_X) = -cosq;
        ret(A_Z, A_Z) = -sinq;
    }
    return ret;
}

Matrix4d TrfmRotateEulerY::getSecondDeriv(const Dof *q1, const Dof *q2) const{
    Matrix4d ret = Matrix4d::Zero();
    if(mDofs[0]==q1 && mDofs[0]==q2){
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        ret(A_X, A_X) = -cosq;
        ret(A_X, A_Z) = -sinq;
        ret(A_Z, A_X) = sinq;
        ret(A_Z, A_Z) = -cosq;
    }
    return ret;
}

Eigen::MatrixXd TrfmRotateEulerY::getJacobian() const {
    Eigen::Matrix<double,6,1> J = Eigen::Matrix<double,6,1>::Zero();
    J(1) = 1.0;
    return J;
}

Matrix4d TrfmRotateEulerY::getInvTransform(){
    Matrix4d ret = Matrix4d::Ones();
    double cosq = cos(mDofs[0]->getValue());
    double sinq = sin(mDofs[0]->getValue());
    ret(A_X, A_X) = cosq;
    ret(A_X, A_Z) = -sinq;
    ret(A_Z, A_X) = sinq;
    ret(A_Z, A_Z) = cosq;
    return ret;
}

////////////////////////////////////////////////////////////////////////////////
// rotation about Z axis
TrfmRotateEulerZ::TrfmRotateEulerZ(Dof *q, const char* _name){
    mDofs.push_back(q);
    q->setTrans(this);
    mType = Transformation::T_ROTATEZ;
    if(_name!=NULL)
        strcpy(mName, _name);
    else
        strcpy(mName, "EulerZ");
}

void TrfmRotateEulerZ::applyGLTransform(renderer::RenderInterface* _ri) const{
	if (_ri)
		_ri->rotate(Vector3d(0.0, 0.0, 1.0), mDofs[0]->getValue()*180/M_PI);
}

void TrfmRotateEulerZ::computeTransform(){
    mTransform.setZero();
    double cosq = cos(mDofs[0]->getValue());
    double sinq = sin(mDofs[0]->getValue());
    mTransform(A_Z, A_Z) = 1.0;
    mTransform(A_X, A_X) = cosq;
    mTransform(A_X, A_Y) = -sinq;
    mTransform(A_Y, A_X) = sinq;
    mTransform(A_Y, A_Y) = cosq;
    mTransform(3, 3) = 1.0;
}

Matrix4d TrfmRotateEulerZ::getDeriv(const Dof *q) const{
    Matrix4d ret = Matrix4d::Zero();
    if(q==mDofs[0]){
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        ret(A_X, A_X) = -sinq;
        ret(A_X, A_Y) = -cosq;
        ret(A_Y, A_X) = cosq;
        ret(A_Y, A_Y) = -sinq;
    }
    return ret;
}

Matrix4d TrfmRotateEulerZ::getSecondDeriv(const Dof *q1, const Dof *q2) const{
    Matrix4d ret = Matrix4d::Zero();
    if(mDofs[0]==q1 && mDofs[0]==q2){
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        ret(A_X, A_X) = -cosq;
        ret(A_X, A_Y) = sinq;
        ret(A_Y, A_X) = -sinq;
        ret(A_Y, A_Y) = -cosq;
    }
    return ret;
}

Eigen::MatrixXd TrfmRotateEulerZ::getJacobian() const {
    Eigen::MatrixXd J = Eigen::Matrix<double,6,1>::Zero();
    J(2) = 1.0;
    return J;
}

Matrix4d TrfmRotateEulerZ::getInvTransform(){
    Matrix4d ret = Matrix4d::Ones();
    double cosq = cos(mDofs[0]->getValue());
    double sinq = sin(mDofs[0]->getValue());
    ret(A_X, A_X) = cosq;
    ret(A_X, A_Y) = sinq;
    ret(A_Y, A_X) = -sinq;
    ret(A_Y, A_Y) = cosq;
    return ret;
}
//

} // namespace kinematics
