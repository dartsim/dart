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

#include <iostream>
#include <map>

#include "math/UtilsMath.h"
#include "kinematics/Dof.h"
#include "kinematics/Transformation.h"
#include "kinematics/TrfmRotateAxis.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Joint.h"

using namespace Eigen;
using namespace std;

namespace kinematics {
Joint::Joint(BodyNode *_bIn, BodyNode *_bOut, const char *_name)
	: System() {
	mType = J_UNKNOWN;
	mSkelIndex=-1;
	mNodeParent=_bIn;
	mNodeChild=_bOut;
	if(mNodeParent != NULL) mNodeParent->addChildJoint(this);
	if(mNodeChild != NULL) mNodeChild->setParentJoint(this);
	mStaticTransform = Matrix4d::Identity();
	mFirstVariableTransformIndex = 0;
	mNumDofsRot=0;
	mNumDofsTrans=0;
	mRotTransformIndex.clear();
	if(_name) setName(_name);
}

Joint::~Joint(){
    for(unsigned int i=0; i<mTransforms.size(); i++){
        delete mTransforms[i];
    }
    mTransforms.clear();
    mDofs.clear();
    mDampingCoefficient.clear();
    mFrictionCoefficient.clear();
}

bool Joint::isPresent (const Dof* _q) const{
    for(unsigned int i=0; i<mDofs.size(); i++)
        if(_q==mDofs[i]) return true;
    return false;
}

int Joint::getDofLocalIndex (int _dofSkelIndex) const{
    for(unsigned int i=0; i<mDofs.size(); i++)
        if(mDofs[i]->getSkelIndex()==_dofSkelIndex) return i;
    return -1;
}

int Joint::getFirstDofIndex() const {
    return mDofs[0]->getSkelIndex();
}

int Joint::getFirstRotDofIndex() const {
    return mTransforms[mRotTransformIndex[0]]->getDof(0)->getSkelIndex();
    //return mDofs[0]->getSkelIndex() + mNumDofsTrans;
}

int Joint::getFirstTransDofIndex() const {  // ASSUME: trans dofs are in the beginning
    return mDofs[0]->getSkelIndex();
}

Matrix4d Joint::getLocalTransform() {
    Matrix4d m = mStaticTransform;
    for(int i = mFirstVariableTransformIndex; i < mTransforms.size(); i++) {
        m *= mTransforms[i]->getTransform();
    }
    return m;
}

void Joint::applyTransform(Vector3d& _v){
    for(int i=mTransforms.size()-1; i>=0; i--){
        mTransforms[i]->applyTransform(_v);
    }
}

void Joint::applyTransform(Matrix4d& _m){
    for(int i=mTransforms.size()-1; i>=0; i--){
        mTransforms[i]->applyTransform(_m);
    }
}

void Joint::computeRotationJac(MatrixXd *_J, MatrixXd *_Jdot, const VectorXd *_qdot){
    assert(_J);
    assert(mType!=J_UNKNOWN);
    _J->resize(3, mNumDofsRot);
    _J->setZero();
    if(_Jdot) {
        _Jdot->resize(3, mNumDofsRot);
        _Jdot->setZero();
    }

    map<Transformation::TransFormType, int> rotEulerMap;
    rotEulerMap[Transformation::T_ROTATEX]=0;
    rotEulerMap[Transformation::T_ROTATEY]=1;
    rotEulerMap[Transformation::T_ROTATEZ]=2;

    if(mType==J_HINGE){
        assert(mNumDofsRot==1);
        assert(mRotTransformIndex.size()==1);
        if(TrfmRotateAxis* rotateAxisTransform = dynamic_cast<TrfmRotateAxis*>(mTransforms[mRotTransformIndex[0]]))
            _J->topLeftCorner<3, 1>() = rotateAxisTransform->getAxis();
        else
            (*_J)(rotEulerMap[mTransforms[mRotTransformIndex[0]]->getType()], 0) = 1.0;
        // _Jdot is zero
    }
    else if(mType==J_UNIVERSAL){
        assert(mNumDofsRot==2);
        assert(mRotTransformIndex.size()==2);

        Matrix4d R0 = mTransforms[mRotTransformIndex[0]]->getTransform();
        // first col
        (*_J)(rotEulerMap[mTransforms[mRotTransformIndex[0]]->getType()], 0) = 1.0;
        // second col
        for(int r=0; r<3; r++) (*_J)(r, 1) = R0(r, rotEulerMap[mTransforms[mRotTransformIndex[1]]->getType()]);
        if(_Jdot){
            // first col is zero
            // second col w_0 x(R_0*e_1), (w_0 = e_0*qd_0) == qd_0 * J_0 x J_1
            Vector3d J_0((*_J)(0,0), (*_J)(1,0), (*_J)(2,0));
            Vector3d J_1((*_J)(0,1), (*_J)(1,1), (*_J)(2,1));
            Vector3d Jdot_1 = J_0.cross(J_1)*(*_qdot)[0];
            for(int r=0; r<3; r++) (*_Jdot)(r, 1) = Jdot_1[r];
        }

    }
    else if(mType==J_BALLEULER || mType==J_FREEEULER){
        assert(mNumDofsRot==3);
        assert(mRotTransformIndex.size()==3);

        Matrix4d R0 = mTransforms[mRotTransformIndex[0]]->getTransform();
        Matrix4d R1 = mTransforms[mRotTransformIndex[1]]->getTransform();
        // first col
        (*_J)(rotEulerMap[mTransforms[mRotTransformIndex[0]]->getType()], 0) = 1.0;
        // second col
        for(int r=0; r<3; r++) (*_J)(r, 1) = R0(r, rotEulerMap[mTransforms[mRotTransformIndex[1]]->getType()]);
        // third col
        VectorXd R1e2 = R1.col(rotEulerMap[mTransforms[mRotTransformIndex[2]]->getType()]);
        VectorXd J_2 = R0*R1e2;
        for(int r=0; r<3; r++) (*_J)(r, 2) = J_2[r];
        if(_Jdot){
            // first col is zero
            // second col w_0 x(R_0*e_1), (w_0 = e_0*qd_0) == qd_0 * J_0 x J_1
            Vector3d J_0((*_J)(0, 0), (*_J)(1, 0), (*_J)(2, 0));
            Vector3d J_1((*_J)(0, 1), (*_J)(1, 1), (*_J)(2, 1));
            Vector3d J_2((*_J)(0, 2), (*_J)(1, 2), (*_J)(2, 2));
            Vector3d Jdot_1 = J_0.cross(J_1)*(*_qdot)[0];
            Vector3d J0qd0pJ1qd1 = J_0*(*_qdot)[0] + J_1*(*_qdot)[1];
            Vector3d Jdot_2 = J0qd0pJ1qd1.cross(J_2);
            for(int r=0; r<3; r++) {
                (*_Jdot)(r, 1) = Jdot_1[r];
                (*_Jdot)(r, 2) = Jdot_2[r];
            }
        }
    }
    else if(mType==J_BALLEXPMAP || mType==J_FREEEXPMAP){
        assert(mNumDofsRot==3);
        assert(mRotTransformIndex.size()==1);
        Transformation *em = mTransforms[mRotTransformIndex[0]];
        Vector3d q(em->getDof(0)->getValue(), em->getDof(1)->getValue(), em->getDof(2)->getValue());
        *_J = dart_math::expMapJac(q);
        if(_Jdot){
            *_Jdot = dart_math::expMapJacDot(q, *_qdot);
        }
    }
    else {
        cout<<"computeRotationJac not implemented yet for this joint type\n";
    }

    // adjust for the static rotation transformations: ASSUME that they are only positioned before variable transforms
    (*_J) = mStaticTransform.topLeftCorner<3,3>() * (*_J);
    if(_Jdot) (*_Jdot) = mStaticTransform.topLeftCorner<3,3>() * (*_Jdot);
}

dart_math::RotationOrder Joint::getEulerOrder(){
    if(mType == J_BALLEXPMAP || mType == J_FREEEXPMAP) return dart_math::UNKNOWN;

    assert(mNumDofsRot==mRotTransformIndex.size());
    string rot="";
    for(int i=mNumDofsRot-1; i>=0; i--){
        if(mTransforms[mRotTransformIndex[i]]->getType()==Transformation::T_ROTATEX) rot+="x";
        else if(mTransforms[mRotTransformIndex[i]]->getType()==Transformation::T_ROTATEY) rot+="y";
        else if(mTransforms[mRotTransformIndex[i]]->getType()==Transformation::T_ROTATEZ) rot+="z";
    }

    if(rot.compare("xyz")==0 || rot.compare("xy")==0 || rot.compare("yz")==0 || rot.compare("xz")==0 || rot.compare("x")==0 || rot.compare("y")==0 || rot.compare("z")==0) return dart_math::XYZ;
    if(rot.compare("yzx")==0 ) return dart_math::YZX;
    if(rot.compare("zxy")==0 ) return dart_math::ZXY;
    if(rot.compare("xzy")==0 ) return dart_math::XZY;
    if(rot.compare("yxz")==0 ) return dart_math::YXZ;
    if(rot.compare("zyx")==0 || rot.compare("zy")==0|| rot.compare("yx")==0|| rot.compare("zx")==0 ) return dart_math::ZYX;

    return dart_math::UNKNOWN;
}

Vector3d Joint::getAxis( unsigned int _i ){
    assert(_i>=0 && _i<=2);
    if(mTransforms.size()<=_i) return Vector3d::Zero();
    Transformation *rot = mTransforms[mTransforms.size()-_i - 1];
    if(rot->getType()==Transformation::T_ROTATEX) return Vector3d::UnitX();
    if(rot->getType()==Transformation::T_ROTATEY) return Vector3d::UnitY();
    if(rot->getType()==Transformation::T_ROTATEZ) return Vector3d::UnitZ();
    if(rot->getType()==Transformation::T_ROTATEAXIS) return ((TrfmRotateAxis*)rot)->getAxis();
    return Vector3d::Zero();
}

Matrix4d Joint::getDeriv(const Dof* _q) {
    Matrix4d m = mStaticTransform;
    for(int i = mFirstVariableTransformIndex; i < mTransforms.size(); i++) {
        if(mTransforms[i]->isPresent(_q))
            m *= mTransforms[i]->getDeriv(_q);
        else
            m *= mTransforms[i]->getTransform();
    }
    return m;
}

void Joint::applyDeriv(const Dof* _q, Vector3d& _v){
    for(int i=mTransforms.size()-1; i>=0; i--){
        if(mTransforms[i]->isPresent(_q))
            mTransforms[i]->applyDeriv(_q, _v);
        else
            mTransforms[i]->applyTransform(_v);
    }
}

void Joint::applyDeriv(const Dof* _q, Matrix4d& _m){
    for(int i=mTransforms.size()-1; i>=0; i--){
        if(mTransforms[i]->isPresent(_q))
            mTransforms[i]->applyDeriv(_q, _m);
        else
            mTransforms[i]->applyTransform(_m);
    }
}

Matrix4d Joint::getSecondDeriv(const Dof* _q1, const Dof* _q2) {
    Matrix4d m = mStaticTransform;
    for(int i = mFirstVariableTransformIndex; i < mTransforms.size(); i++) {
        Transformation* transf = mTransforms[i];
        if(transf->isPresent(_q1) && transf->isPresent(_q2))
            m *= transf->getSecondDeriv(_q1, _q2);
        else if(transf->isPresent(_q1))
            m *= transf->getDeriv(_q1);
        else if(transf->isPresent(_q2))
            m *= transf->getDeriv(_q2);
        else
            m *= transf->getTransform();
    }
    return m;
}

void Joint::applySecondDeriv(const Dof* _q1, const Dof* _q2, Vector3d& _v){
    for(int i=mTransforms.size()-1; i>=0; i--){
        Transformation* transf = mTransforms[i];
        if(transf->isPresent(_q1) && transf->isPresent(_q2)){
            transf->applySecondDeriv(_q1, _q2, _v);
        }else if(transf->isPresent(_q1)){
            transf->applyDeriv(_q1, _v);
        }else if(transf->isPresent(_q2)){
            transf->applyDeriv(_q2, _v);
        }else{
            transf->applyTransform(_v);
        }
    }
}

void Joint::applySecondDeriv(const Dof* _q1, const Dof* _q2, Matrix4d& _m){
    for(int i=mTransforms.size()-1; i>=0; i--){
        Transformation* transf = mTransforms[i];
        if(transf->isPresent(_q1) && transf->isPresent(_q2)){
            transf->applySecondDeriv(_q1, _q2, _m);
        }else if(transf->isPresent(_q1)){
            transf->applyDeriv(_q1, _m);
        }else if(transf->isPresent(_q2)){
            transf->applyDeriv(_q2, _m);
        }else{
            transf->applyTransform(_m);
        }
    }
}

void Joint::addTransform(Transformation *_t, bool _isVariable){
    _t->setJoint(this);
    _t->setVariable(_isVariable);
    mTransforms.push_back(_t);
    if(_isVariable) {	// add dofs of variable transform only
        for(int i=0; i<_t->getNumDofs(); i++){
            addDof(_t->getDof(i));
        }
        switch(_t->getType()){
            case Transformation::T_ROTATEX:
            case Transformation::T_ROTATEY:
            case Transformation::T_ROTATEZ:
            case Transformation::T_ROTATEEXPMAP:
            case Transformation::T_ROTATEAXIS:
                mNumDofsRot+=_t->getNumDofs();
                mRotTransformIndex.push_back(mTransforms.size()-1);
                break;
            default:	// translation dofs
                mNumDofsTrans+=_t->getNumDofs();
                break;
        }
        if(_t->getType()==Transformation::T_ROTATEEXPMAP && mNumDofsRot==3 && mNumDofsTrans==3) mType=J_FREEEXPMAP;
        else if(_t->getType()==Transformation::T_ROTATEEXPMAP && mNumDofsRot==3 && mNumDofsTrans==0) mType=J_BALLEXPMAP;
        else if(mNumDofsRot==3 && mNumDofsTrans==3) mType=J_FREEEULER;
        else if(mNumDofsRot==3 && mNumDofsTrans==0) mType=J_BALLEULER;
        else if(mNumDofsRot==2 && mNumDofsTrans==0) mType=J_UNIVERSAL;
        else if(mNumDofsRot==1 && mNumDofsTrans==0) mType=J_HINGE;
        else if(mNumDofsRot==0 && mNumDofsTrans>0) mType=J_TRANS;
        else {
            mType=J_UNKNOWN;
            //cout<<"Type of joint not recognized\n";
        }
    }
    else if(mFirstVariableTransformIndex == mTransforms.size() - 1) {
        mFirstVariableTransformIndex++;
        mStaticTransform *= _t->getTransform();
    }
}

void Joint::addDof(Dof *_d) {
    mDofs.push_back(_d);
    _d->setJoint(this);

    // TODO: We assume that the default values are zero.
    mFrictionCoefficient.push_back(0);
    mDampingCoefficient.push_back(0);

    // TODO:
    //mLocalJacobian = Eigen::MatrixXd(6,getNumDofs());
}


void Joint::updateStaticTransform() {
    mStaticTransform = Matrix4d::Identity();
    for(int i = 0; i < mFirstVariableTransformIndex; i++) {
        mStaticTransform *= mTransforms[i]->getTransform();
    }
}

//void Joint::updateForwardKinematics()
//{
//    mLocalJacobian = __recursive_evalLocalJacobian();
//}

Eigen::MatrixXd Joint::getLocalJacobian()
{
    //return mLocalJacobian;
    return __recursive_evalLocalJacobian();
}

void Joint::setDampingCoefficient(int _idx, double _d)
{
    assert(0 <= _idx && _idx < getNumDofs());
    assert(_d >= 0.0);

    mDampingCoefficient[_idx] = _d;
}

void Joint::setFrictionCoefficient(int _idx, double _f)
{
    assert(0 <= _idx && _idx < getNumDofs());
    assert(_f >= 0.0);

    mFrictionCoefficient[_idx] = _f;
}

double Joint::getDampingCoefficient(int _idx) const
{
    assert(0 <= _idx && _idx < getNumDofs());

    return mDampingCoefficient[_idx];
}

double Joint::getFrictionCoefficient(int _idx) const
{
    assert(0 <= _idx && _idx < getNumDofs());

    return mFrictionCoefficient[_idx];
}

Eigen::VectorXd Joint::getDampingForce() const
{
    int numDofs = getNumDofs();
    Eigen::VectorXd dampingForce(numDofs);

    for (int i = 0; i < numDofs; ++i)
        dampingForce(i) = -mDampingCoefficient[i] * getDof(i)->dq;

    return dampingForce;
}

Eigen::VectorXd Joint::getFrictionForce() const
{
    int numDofs = getNumDofs();
    Eigen::VectorXd frictionForce(numDofs);

    // TODO: NOT IMPLEMENTED



    return frictionForce;
}

//dart_math::Vector6d Joint::evalLocalVelocity() const
//{

//}

Eigen::MatrixXd Joint::__recursive_evalLocalJacobian()
{
    // TODO: Below looks messy but, once we have concrete joint classes,
    // then these will be more compact.
    int numTransforms = getNumTransforms();
    int numDofs = getNumDofs();
    Eigen::MatrixXd localJacobian = Eigen::MatrixXd::Zero(6,numDofs);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    int k = numDofs;
    for (int i = numTransforms - 1; i > -1; --i)
    {
        int numTransformDofs = mTransforms[i]->getNumDofs();

        if (mTransforms[i]->isVariable())
        {
            Eigen::MatrixXd jointJ = mTransforms[i]->getJacobian();
            assert(jointJ.cols() == numTransformDofs);

            for (int j = 0; j < jointJ.cols(); ++j)
                jointJ.col(j) = dart_math::InvAd(T, jointJ.col(j));

            k -= numTransformDofs;
            assert(0 <= k && k < numDofs);
            localJacobian.block(0,k,6,numTransformDofs) = jointJ;
        }

        T = mTransforms[i]->getTransform() * T;
    }

    return localJacobian;
}

} // namespace kinematics

