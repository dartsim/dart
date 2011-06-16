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

#ifdef _RENDERER_TEST
#include "renderer/OpenGLRenderInterface.h"
#else
#include "utils/LoadOpengl.h"
#endif

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
        mTransLocal = mJointIn->getTransform();
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
    
    // void BodyNode::setDependDofMap(int _numDofs){
    //     // initialize the map
    //     mDependsOnDof = new bool[_numDofs];
    //     memset(mDependsOnDof, false, _numDofs*sizeof(bool));
    
    //     // if this is not the root node, copy its parent's map first
    //     if(mNodeIn!=NULL){
    //         memcpy(mDependsOnDof, mNodeIn->mDependsOnDof, _numDofs*sizeof(bool));
    //     }

    //     // set dependence for itself
    //     for( int i=0; i<getNumDofs(); i++){
    //         int dofID = getDof(i)->getModelIndex();
    //         mDependsOnDof[dofID] = true;
    //     }
    // }
    
    void BodyNode::draw(Renderer::OpenGLRenderInterface* _RI, const Vector4d& _color, bool _default, int _depth)
    {
#ifdef _RENDERER_TEST
        if (!_RI) return;
        RI->PushMatrix();
        // render the self geometry
        for (int i = 0; i < mJointIn->getNumTransforms(); i++) {
            mJointIn->getTransform(i)->applyGLTransform(_RI);
        }
        if(mPrimitive != NULL) {
            _RI->PushName((unsigned)mID);
            _RI->PushMatrix();
            _RI->Translate(mOffset);
            mPrimitive->draw(_RI, _color, _default);
            _RI->PopMatrix();
            _RI->PopName();
        }

        // render the subtree
        for (int i = 0; i < mJointOut.size(); i++) {
            mJointOut[i]->getNodeOut()->draw(_RI, _color, _default);
        }
        _RI->PopMatrix();
#else
        glPushMatrix();
        // render the self geometry
        for (int i = 0; i < mJointIn->getNumTransforms(); i++) {
            mJointIn->getTransform(i)->applyGLTransform(_RI);
        }
        if (mPrimitive != NULL) {
            glPushName((unsigned)mID);
            glPushMatrix();
            glTranslatef(mOffset[0],mOffset[1],mOffset[2]);
            mPrimitive->draw(_RI, _color, _default);
            glPopMatrix();
            glPopName();
        }

        // render the subtree
        for (int i = 0; i < mJointOut.size(); i++) {
            mJointOut[i]->getNodeOut()->draw(_RI, _color, _default);
        }
        glPopMatrix();
#endif
    }

    void BodyNode::drawHandles(Renderer::OpenGLRenderInterface* _RI, const Vector4d& _color, bool _default)
    {
#ifdef _RENDERER_TEST
        if (!_RI) return;
        _RI->PushMatrix();
        for (int i = 0; i < mJointIn->getNumTransforms(); i++) {
            mJointIn->getTransform(i)->applyGLTransform(_RI);
        }

        // render the corresponding mHandless
        for (int i = 0; i < mHandles.size(); i++) {
            mHandles[i]->draw(_RI, true, _color, _default);
        }
        for (int i = 0; i < mJointOut.size(); i++) {
            mJointOut[i]->getNodeOut()->drawHandles(_RI, _color, _default);
        }
        _RI->PopMatrix();
#else
        glPushMatrix();
        for (int i = 0; i < mJointIn->getNumTransforms(); i++) {
            mJointIn->getTransform(i)->applyGLTransform(_RI);
        }

        // render the corresponding mHandless
        for (int i = 0; i < mHandles.size(); i++) {
            mHandles[i]->draw(_RI, true, _color, _default);
        }
        for (int i = 0; i < mJointOut.size(); i++) {
            mJointOut[i]->getNodeOut()->drawHandles(_RI,_color, _default);
        }
        glPopMatrix();
#endif
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

