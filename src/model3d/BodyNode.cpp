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
        : mSkelIndex(-1), mPrimitive(NULL), mJointParent(NULL), mNodeParent(NULL), mMass(0), mOffset(0,0,0)
    {
        mJointsChild.clear();
        mHandles.clear();
        mDependantDofs.clear();

        mID = BodyNode::msBodyNodeCount++;

        if (_name == NULL) {
            strcpy(mName, "BodyNode");
        } else {
            strcpy(mName, _name);
        }
    }

    BodyNode::~BodyNode() {
        for (unsigned int i = 0; i < mHandles.size(); ++i){
            delete mHandles[i];
        }
        mHandles.clear();

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

        const int numLocalDofs = getNumLocalDofs();
        mTq.resize(numLocalDofs, Matrix4d::Zero());
        //const int numDepDofs = getSkel()->getNumLocalDofs();
        const int numDepDofs = getNumDependantDofs();
        mWq.resize(numDepDofs, Matrix4d::Zero());
        
        mJc = MatrixXd::Zero(3, numDepDofs);
        mJw = MatrixXd::Zero(3, numDepDofs);
    }

    void BodyNode::updateTransform() {
        mT = mJointParent->getLocalTransform();
        if (mNodeParent) {
            mW = mNodeParent->mW * mT;
        } 
        else {
            mW = mT;
        }
    }

    void BodyNode::updateFirstDerivatives() {
        const int numLocalDofs = getNumLocalDofs();
        const int numParentDofs = getNumDependantDofs()-numLocalDofs;

        // Update Local Derivatives
        for(int i = 0; i < numLocalDofs; i++) {
            mTq.at(i) = getLocalDeriv(getDof(i));
        }

        //// Update World Derivatives
        //if (mNodeParent) {
        //    for (int i = 0; i < getNumDependantDofs(); i++) {
        //        int index = getDependantDof(i);
        //        if (mNodeParent->dependsOn(index)) {
        //            mWq.at(index) = mNodeParent->mWq.at(index) * mT;
        //        } 
        //        else {
        //            mWq.at(index) = mNodeParent->mW * mTq.at( mJointParent->getLocalIndex(index) );
        //        }
        //    }
        //} 
        //else {
        //    for(int i = 0; i < numLocalDofs; ++i) {
        //        int index1 = mJointParent->getDof(i)->getSkelIndex();
        //        mWq.at(index1) = mTq.at(i);
        //    }
        //}

        // Update World Derivatives
        // parent dofs
        for (int i = 0; i < numParentDofs; i++) {
            assert(mNodeParent);    // should always have a parent if enters this for loop
            mWq.at(i) = mNodeParent->mWq.at(i) * mT;
        }
        // local dofs
        for(int i = 0; i < numLocalDofs; i++){
            if(mNodeParent) {
                mWq.at(numParentDofs+i) = mNodeParent->mW * mTq.at(i);
            }
            else {
                mWq.at(numParentDofs+i) = mTq.at(i);
            }
        }

        evalJacLin();
        evalJacAng();
    }

    Vector3d BodyNode::evalWorldPos(const Vector3d& _lp) {
        Vector3d result = utils::xformHom(mW, _lp);
        return result;
    }
    

    void BodyNode::setDependDofList() {
        mDependantDofs.clear();
        if (mNodeParent != NULL) {
            mDependantDofs.insert(mDependantDofs.end(), mNodeParent->mDependantDofs.begin(), mNodeParent->mDependantDofs.end());
        }

        for (int i = 0; i < getNumLocalDofs(); i++) {
            int dofID = getDof(i)->getSkelIndex();
            mDependantDofs.push_back(dofID);
        }

#if _DEBUG
        for (unsigned int i = 0; i < mDependantDofs.size() - 1; i++) {
            int now = mDependantDofs[i];
            int next = mDependantDofs[i + 1];
            if (now > next) {
                cerr << "Array not sorted!!!" << endl;
                exit(0);
            }
        }
#endif
    }

    bool BodyNode::dependsOn(int _dofIndex) const {
        return binary_search(mDependantDofs.begin(), mDependantDofs.end(), _dofIndex);
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
            _ri->translate(mOffset);
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

    void BodyNode::drawHandles(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
        if (!_ri) return;
        _ri->pushMatrix();
        for (int i = 0; i < mJointParent->getNumTransforms(); i++) {
            mJointParent->getTransform(i)->applyGLTransform(_ri);
        }

        // render the corresponding mHandless
        for (unsigned int i = 0; i < mHandles.size(); i++) {
            mHandles[i]->draw(_ri, true, _color, _useDefaultColor);
        }
        for (unsigned int i = 0; i < mJointsChild.size(); i++) {
            mJointsChild[i]->getChildNode()->drawHandles(_ri,_color, _useDefaultColor);
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

    Matrix4d BodyNode::getLocalDeriv(Dof* _q) const {
        return mJointParent->getDeriv(_q);
    }
    
    void BodyNode::evalJacLin() {
        mJc.setZero();
        //for (vector<int>::iterator i_iter = mDependantDofs.begin();
        //     i_iter != mDependantDofs.end(); i_iter++) {
        //    int i = (*i_iter);
        //    VectorXd J = utils::xformHom(mWq.at(i), mOffset);
        //    mJc(0, i) = J(0);
        //    mJc(1, i) = J(1);
        //    mJc(2, i) = J(2);
        //}
        for (unsigned int i=0; i<mDependantDofs.size(); i++) {
            VectorXd J = utils::xformHom(mWq.at(i), mOffset);
            mJc(0, i) = J(0);
            mJc(1, i) = J(1);
            mJc(2, i) = J(2);
        }
    }

    void BodyNode::evalJacAng() {
        mJw.setZero();
        //for (vector<int>::iterator i_iter = mDependantDofs.begin();
        //     i_iter != mDependantDofs.end(); i_iter++) {
        //    int i = (*i_iter);

        //    MatrixXd transR = mW.topLeftCorner(3,3).transpose();
        //    MatrixXd dRdq = mWq.at(i).topLeftCorner(3, 3);
        //    MatrixXd omegaSkewSymmetric = dRdq * transR;
        //    VectorXd omega = utils::fromSkewSymmetric(omegaSkewSymmetric);
        //    omega = transR * omega;
        //    
        //    mJw(0, i) = omega(0);
        //    mJw(1, i) = omega(1);
        //    mJw(2, i) = omega(2);
        //}
        for (unsigned int i=0; i<mDependantDofs.size(); i++) {
            MatrixXd transR = mW.topLeftCorner(3,3).transpose();
            MatrixXd dRdq = mWq.at(i).topLeftCorner(3, 3);
            MatrixXd omegaSkewSymmetric = dRdq * transR;
            VectorXd omega = utils::fromSkewSymmetric(omegaSkewSymmetric);
            omega = transR * omega;

            mJw(0, i) = omega(0);
            mJw(1, i) = omega(1);
            mJw(2, i) = omega(2);
        }
    }
    
} // namespace model3d

