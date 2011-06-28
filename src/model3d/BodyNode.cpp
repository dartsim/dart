/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#include "BodyNode.h"

#include <algorithm>

using namespace std;
using namespace Eigen;

#include "Primitive.h"
#include "Joint.h"
#include "Marker.h"
#include "Dof.h"
#include "Skeleton.h"
#include "Transformation.h"

#include "utils/Misc.h"
#include "utils/Utils.h"
#include "utils/EigenHelper.h"
#include "utils/LoadOpengl.h"

namespace model3d {
    int BodyNode::msBodyNodeCount = 0;
  
    BodyNode::BodyNode(char *_name) 
        : mModelIndex(-1), mPrimitive(NULL), mJointIn(NULL), mNodeIn(NULL), mMass(0), mOffset(0,0,0)
    {
        mJointOut.clear();
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
        mJointOut.clear();
    }

    void BodyNode::init() {
        if (mPrimitive != NULL) {
            mMass = mPrimitive->getMass();
        } else {
            mMass = 0;
        }

        mTransLocal = Matrix4d::Identity();
        mTransWorld = Matrix4d::Identity();

        const int localDofs = getNumDofs();
        Tq.resize(localDofs, MatrixXd::Identity(4, 4));
        const int nDofs = getModel()->getNumDofs();
        Wq.resize(nDofs, MatrixXd::Identity(4, 4));

        mJC = MatrixXd::Zero(3, nDofs);
        mJW = MatrixXd::Zero(3, nDofs);
    }

    void BodyNode::updateTransform() {
        mTransLocal = mJointIn->getLocalTransform();
        if (mNodeIn) {
            mTransWorld = mNodeIn->mTransWorld * mTransLocal;
        } else {
            mTransWorld = mTransLocal;
        }
    }

    void BodyNode::updateDerivatives() {
        const int localDofs = getNumDofs();

        // Update Local Derivatives
        for(int i = 0; i < localDofs; i++) {
            Tq.at(i) = getLocalDeriv(getDof(i));
        }

        // Update World Derivates
        if (mNodeIn) {
            for (int i = 0; i < getNumDependantDofs(); i++) {
                int index = getDependantDof(i);
                if (mNodeIn->dependsOn(index)) {
                    Wq.at(index) = mNodeIn->Wq.at(index) * mTransLocal;
                } else {
                    Wq.at(index) = mNodeIn->mTransWorld * Tq.at( mJointIn->getLocalIndex(index) );
                }
            }
        } else {
            for(int i = 0; i < localDofs; ++i) {
                int index1 = mJointIn->getDof(i)->getModelIndex();
                Wq.at(index1) = Tq.at(i);
            }
        }

        evalJC();
        evalJW();

    }

    Vector3d BodyNode::evalWorldPos(const Vector3d& _lp) {
        Vector3d result = utils::transform(mTransWorld, _lp);
        return result;
    }
    

    void BodyNode::setDependDofList() {
        mDependantDofs.clear();
        if (mNodeIn != NULL) {
            mDependantDofs.insert(mDependantDofs.end(),
                                  mNodeIn->mDependantDofs.begin(),
                                  mNodeIn->mDependantDofs.end());
        }

        for (int i = 0; i < getNumDofs(); i++) {
            int dofID = getDof(i)->getModelIndex();
            mDependantDofs.push_back(dofID);
        }

        for (unsigned int i = 0; i < mDependantDofs.size() - 1; i++) {
            int now = mDependantDofs[i];
            int next = mDependantDofs[i + 1];
            if (now > next) {
                cerr << "Array not sorted!!!" << endl;
                exit(0);
            }
        }
    }

    bool BodyNode::dependsOn(int _dofIndex) const {
        return binary_search(mDependantDofs.begin(), mDependantDofs.end(), _dofIndex);
    }
        
    void BodyNode::draw(const Vector4d& _color, bool _useDefaultColor, int _depth) const {
        glPushMatrix();
        // render the self geometry
        for (int i = 0; i < mJointIn->getNumTransforms(); i++) {
            mJointIn->getTransform(i)->applyGLTransform();
        }
        if (mPrimitive != NULL) {
            glPushName((unsigned)mID);
            glPushMatrix();
            glTranslatef(mOffset[0],mOffset[1],mOffset[2]);
            mPrimitive->draw(_color, _useDefaultColor);
            glPopMatrix();
            glPopName();
        }

        // render the subtree
        for (unsigned int i = 0; i < mJointOut.size(); i++) {
            mJointOut[i]->getNodeOut()->draw(_color, _useDefaultColor);
        }
        glPopMatrix();
    }

    void BodyNode::drawHandles(const Vector4d& _color, bool _useDefaultColor) const {
        glPushMatrix();
        for (int i = 0; i < mJointIn->getNumTransforms(); i++) {
            mJointIn->getTransform(i)->applyGLTransform();
        }

        // render the corresponding mHandless
        for (unsigned int i = 0; i < mHandles.size(); i++) {
            mHandles[i]->draw(true, _color, _useDefaultColor);
        }
        for (unsigned int i = 0; i < mJointOut.size(); i++) {
            mJointOut[i]->getNodeOut()->drawHandles(_color, _useDefaultColor);
        }
        glPopMatrix();
    }

    void BodyNode::setJointIn(Joint *_p) {
        mJointIn = _p; 
        mNodeIn = _p->getNodeIn();
    }

    BodyNode* BodyNode::getNodeOut(int _idx) const {
        return mJointOut[_idx]->getNodeOut();
    }

    int BodyNode::getNumDofs() const {
        return mJointIn->getNumDofs();
    }

    Dof* BodyNode::getDof(int _idx) const {
        return mJointIn->getDof(_idx);
    }

    bool BodyNode::isPresent(Dof* _q) {
        return mJointIn->isPresent(_q);
    }

    Matrix4d BodyNode::getLocalDeriv(Dof* q) const {
        return mJointIn->getDeriv(q);
    }

    void BodyNode::evalJC() {
        mJC.setZero();

        for (vector<int>::iterator i_iter = mDependantDofs.begin();
             i_iter != mDependantDofs.end(); i_iter++) {
            int i = (*i_iter);
            VectorXd J = utils::transform(Wq.at(i), mOffset);
            mJC(0, i) = J(0);
            mJC(1, i) = J(1);
            mJC(2, i) = J(2);
        }
    }

    void BodyNode::evalJW() {
        using eigenhelper::sub;
        using eigenhelper::trans;
        
        mJW.setZero();
        for (vector<int>::iterator i_iter = mDependantDofs.begin();
             i_iter != mDependantDofs.end(); i_iter++) {
            int i = (*i_iter);

            MatrixXd transR = trans(sub(mTransWorld, 3, 3));
            MatrixXd dRdq = sub(Wq.at(i), 3, 3);
            MatrixXd omegaSkewSymmetric = dRdq * transR;
            VectorXd omega = utils::fromSkewSymmetric(omegaSkewSymmetric);
            omega = transR * omega;
            
            mJW(0, i) = omega(0);
            mJW(1, i) = omega(1);
            mJW(2, i) = omega(2);
        }
    }
    
} // namespace model3d

