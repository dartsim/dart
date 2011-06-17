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
        for (int i = 0; i < mHandles.size(); ++i){
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
        if (mPrimitive != NULL)
            mMass = mPrimitive->getMass();
        else
            mMass = 0;
        mTransLocal = Matrix4d::Identity();
        mTransWorld = Matrix4d::Identity();
    }

    void BodyNode::updateTransform() {
        mTransLocal = mJointIn->getLocalTransform();
        if (mNodeIn) {
            mTransWorld = mNodeIn->mTransWorld * mTransLocal;
        } else {
            mTransWorld = mTransLocal;
        }
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

        for (int i = 0; i < mDependantDofs.size() - 1; i++) {
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
        for (int i = 0; i < mJointOut.size(); i++) {
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
        for (int i = 0; i < mHandles.size(); i++) {
            mHandles[i]->draw(true, _color, _useDefaultColor);
        }
        for (int i = 0; i < mJointOut.size(); i++) {
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

} // namespace model3d

