/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#include "BodyNode.h"
#include <algorithm>
#include "Primitive.h"
#include "Joint.h"
#include "Marker.h"
#include "Dof.h"
#include "Skeleton.h"
#include "Transformation.h"
#include "utils/Misc.h"
#include "utils/UtilsMath.h"

using namespace std;
using namespace Eigen;

namespace model3d {
    int BodyNode::msBodyNodeCount = 0;
  
    BodyNode::BodyNode(const char *_name) 
        : mSkelIndex(-1), mPrimitive(NULL), mJointParent(NULL), mNodeParent(NULL), mMass(0), mCOMLocal(0,0,0)
    {
        mJointsChild.clear();
        mMarkers.clear();
        mDependentDofs.clear();

        mID = BodyNode::msBodyNodeCount++;

        if (_name == NULL) {
            strcpy(mName, "BodyNode");
        } else {
            strcpy(mName, _name);
        }
    }

    BodyNode::~BodyNode() {
        for (unsigned int i = 0; i < mMarkers.size(); ++i){
            delete mMarkers[i];
        }
        mMarkers.clear();

        if (mPrimitive != NULL) {
            delete mPrimitive;
            mPrimitive = NULL;
        }
        mJointsChild.clear();
    }

    void BodyNode::init() {
        if (mPrimitive != NULL) {
            mMass = mPrimitive->getMass();
        } else {
            mMass = 0;
        }

        mT = Matrix4d::Identity();
        mW = Matrix4d::Identity();
        mIc = Matrix3d::Zero();

        const int numLocalDofs = getNumLocalDofs();
        mTq.resize(numLocalDofs, Matrix4d::Zero());
        const int numDepDofs = getNumDependentDofs();
        mWq.resize(numDepDofs, Matrix4d::Zero());
        
        mJv = MatrixXd::Zero(3, numDepDofs);
        mJw = MatrixXd::Zero(3, numDepDofs);

        mNumRootTrans = 0;
        for(int i=0; i<mSkel->getNumDofs(); i++){
            if(!(mSkel->getDof(i)->getTrans()->getType()==Transformation::T_TRANSLATE 
                || mSkel->getDof(i)->getTrans()->getType()==Transformation::T_TRANSLATEX  
                || mSkel->getDof(i)->getTrans()->getType()==Transformation::T_TRANSLATEY 
                || mSkel->getDof(i)->getTrans()->getType()==Transformation::T_TRANSLATEZ)) {
                    mNumRootTrans = i;
                    break;
            }
        }
    }

    void BodyNode::updateTransform() {
        mT = mJointParent->getLocalTransform();
        if (mNodeParent) mW = mNodeParent->mW * mT;
        else mW = mT;

        // update the inertia matrix 
        Matrix3d R = mW.topLeftCorner(3,3);
        if(mPrimitive!=NULL)
            mIc = R*mPrimitive->getInertia()*R.transpose();
    }

    void BodyNode::updateFirstDerivatives() {
        const int numLocalDofs = getNumLocalDofs();
        const int numParentDofs = getNumDependentDofs()-numLocalDofs;

        // Update Local Derivatives
        for(int i = 0; i < numLocalDofs; i++) {
            mTq.at(i) = getLocalDeriv(getDof(i));
        }

        // Update World Derivatives
        // parent dofs
        for (int i = 0; i < numParentDofs; i++) {
            assert(mNodeParent);    // should always have a parent if enters this for loop
            if(i<mNumRootTrans) 
                mWq.at(i) = mNodeParent->mWq.at(i); // in turn its equal to dT/dqi where T is the translation 4x4 matrix for the first 3 dofs
            else 
                mWq.at(i) = mNodeParent->mWq.at(i) * mT;
        }
        // local dofs
        for(int i = 0; i < numLocalDofs; i++){
            if(mNodeParent) 
                mWq.at(numParentDofs+i) = mNodeParent->mW * mTq.at(i);
            else 
                mWq.at(i) = mTq.at(i);
        }

        evalJacLin();
        evalJacAng();
    }

    void BodyNode::evalJacLin() {
        mJv.setZero();
        for (unsigned int i=0; i<mDependentDofs.size(); i++) {
            VectorXd Ji = utils::xformHom(mWq.at(i), mCOMLocal);
            mJv(0, i) = Ji(0);
            mJv(1, i) = Ji(1);
            mJv(2, i) = Ji(2);
        }
    }

    void BodyNode::evalJacAng() {
        mJw.setZero();
        for (unsigned int i=mNumRootTrans; i<mDependentDofs.size(); i++) {
            MatrixXd omegaSkewSymmetric = mWq.at(i).topLeftCorner(3, 3) * mW.topLeftCorner(3,3).transpose();
            VectorXd omegai = utils::fromSkewSymmetric(omegaSkewSymmetric);

            mJw(0, i) = omegai(0);
            mJw(1, i) = omegai(1);
            mJw(2, i) = omegai(2);
        }
    }

    Vector3d BodyNode::evalWorldPos(const Vector3d& _lp) {
        Vector3d result = utils::xformHom(mW, _lp);
        return result;
    }

    Matrix4d BodyNode::getLocalDeriv(Dof* _q) const {
        return mJointParent->getDeriv(_q);
    }
    
    void BodyNode::setDependDofList() {
        mDependentDofs.clear();
        if (mNodeParent != NULL) {
            mDependentDofs.insert(mDependentDofs.end(), mNodeParent->mDependentDofs.begin(), mNodeParent->mDependentDofs.end());
        }

        for (int i = 0; i < getNumLocalDofs(); i++) {
            int dofID = getDof(i)->getSkelIndex();
            mDependentDofs.push_back(dofID);
        }

#if _DEBUG
        for (unsigned int i = 0; i < mDependentDofs.size() - 1; i++) {
            int now = mDependentDofs[i];
            int next = mDependentDofs[i + 1];
            if (now > next) {
                cerr << "Array not sorted!!!" << endl;
                exit(0);
            }
        }
#endif
    }

    bool BodyNode::dependsOn(int _dofIndex) const {
        return binary_search(mDependentDofs.begin(), mDependentDofs.end(), _dofIndex);
    }
        
    void BodyNode::draw(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor, int _depth) const {
        if (!_ri) return;
        _ri->pushMatrix();
        // render the self geometry
        for (int i = 0; i < mJointParent->getNumTransforms(); i++) {
            mJointParent->getTransform(i)->applyGLTransform(_ri);
        }
        if (mPrimitive != NULL) {
            _ri->pushName((unsigned)mID);
            _ri->pushMatrix();
            _ri->translate(mCOMLocal);
            mPrimitive->draw(_ri, _color, _useDefaultColor);
            _ri->popMatrix();
            _ri->popName();
        }

        // render the subtree
        for (unsigned int i = 0; i < mJointsChild.size(); i++){
            mJointsChild[i]->getChildNode()->draw(_ri, _color, _useDefaultColor);
        }
        _ri->popMatrix();

    }

    void BodyNode::drawMarkers(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
        if (!_ri) return;
        _ri->pushMatrix();
        for (int i = 0; i < mJointParent->getNumTransforms(); i++) {
            mJointParent->getTransform(i)->applyGLTransform(_ri);
        }

        // render the corresponding mMarkerss
        for (unsigned int i = 0; i < mMarkers.size(); i++) {
            mMarkers[i]->draw(_ri, true, _color, _useDefaultColor);
        }
        for (unsigned int i = 0; i < mJointsChild.size(); i++) {
            mJointsChild[i]->getChildNode()->drawMarkers(_ri,_color, _useDefaultColor);
        }
        _ri->popMatrix();

    }

    void BodyNode::setParentJoint(Joint *_p) {
        mJointParent = _p; 
        mNodeParent = _p->getParentNode();
    }

    BodyNode* BodyNode::getChildNode(int _idx) const {
        return mJointsChild[_idx]->getChildNode();
    }

    int BodyNode::getNumLocalDofs() const {
        return mJointParent->getNumDofs();
    }

    Dof* BodyNode::getDof(int _idx) const {
        return mJointParent->getDof(_idx);
    }

    bool BodyNode::isPresent(Dof* _q) {
        return mJointParent->isPresent(_q);
    }

    Eigen::Matrix4d BodyNode::getDerivLocalTransform(int index) const {
        return mTq[index];
    }
    
    Eigen::Matrix4d BodyNode::getDerivWorldTransform(int index) const {
        return mWq[index];
    }
    
    Eigen::MatrixXd BodyNode::getJacobianLinear() const {
        return mJv;
    }
    
    Eigen::MatrixXd BodyNode::getJacobianAngular() const {
        return mJw;
    }
    
} // namespace model3d

