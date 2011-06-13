/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#include "BodyNode.h"

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
            : mModelIndex(-1), mPrimitive(NULL), mJointIn(NULL), mNodeIn(NULL), dependsOnDof(NULL), mMass(0), mOffset(0,0,0)
    {
        mJointOut.clear();
        mHandles.clear();

        mID = BodyNode::msBodyNodeCount++;

        if(_name == NULL) {
            strcpy(mName, "BodyNode");
        } else {
            strcpy(mName, _name);
        }
    }

    BodyNode::~BodyNode() {
        for(int i = 0; i < mHandles.size(); ++i){
            delete mHandles[i];
        }
        mHandles.clear();

        if(mPrimitive != NULL) {
            delete mPrimitive;
            mPrimitive = NULL;
        }
        mJointOut.clear();
        delete[] dependsOnDof;
        dependsOnDof = NULL;
    }

    void BodyNode::init() {
        if(mPrimitive!=NULL)
            mMass = mPrimitive->getMass();
        else
            mMass = 0;

        T = Matrix4d::Identity();
        W = Matrix4d::Identity();
    }

    void BodyNode::updateTransform() {
        T = mJointIn->getTransform();
        if (mNodeIn) {
            W = mNodeIn->W*T;
        } else {
            W = T;
        }
    }

    Vector3d BodyNode::evalWorldPos(Vector3d& lp) {
        Vector3d result = utils::transform(W,lp);
        return result;
    }
    
    void BodyNode::setDependDofMap(int _numDofs){
        // initialize the map
        dependsOnDof = new bool[_numDofs];
        memset(dependsOnDof, false, _numDofs*sizeof(bool));
    
        // if this is not the root node, copy its parent's map first
        if(mNodeIn!=NULL){
            memcpy(dependsOnDof, mNodeIn->dependsOnDof, _numDofs*sizeof(bool));
        }

        // set dependence for itself
        for( int i=0; i<getNumDofs(); i++){
            int dofID = getDof(i)->getModelIndex();
            dependsOnDof[dofID] = true;
        }
    }
    
    void BodyNode::draw(Renderer::OpenGLRenderInterface* RI, const Vector4d& _color, bool _default, int depth)
    {
#ifdef _RENDERER_TEST
        if (!RI) return;
        RI->PushMatrix();
        // render the self geometry
        for(int i=0; i<mJointIn->getNumTransforms(); i++){
            mJointIn->getTransform(i)->applyGLTransform(RI);
        }
        if(mPrimitive != NULL) {
            RI->PushName((unsigned)mID);
            RI->PushMatrix();
            RI->Translate(mOffset);
            mPrimitive->draw(RI, _color, _default);
            RI->PopMatrix();
            RI->PopName();
        }

        // render the subtree
        for(int i=0; i<mJointOut.size(); i++){
            mJointOut[i]->getNodeOut()->draw(RI, _color, _default);
        }
        RI->PopMatrix();
#else
        glPushMatrix();
        // render the self geometry
        for(int i=0; i<mJointIn->getNumTransforms(); i++){
            mJointIn->getTransform(i)->applyGLTransform(RI);
        }
        if(mPrimitive != NULL) {
            glPushName((unsigned)mID);
            glPushMatrix();
            glTranslatef(mOffset[0],mOffset[1],mOffset[2]);
            mPrimitive->draw(RI, _color, _default);
            glPopMatrix();
            glPopName();
        }

        // render the subtree
        for(int i=0; i<mJointOut.size(); i++){
            mJointOut[i]->getNodeOut()->draw(RI, _color, _default);
        }
        glPopMatrix();
#endif
    }

    void BodyNode::drawHandles(Renderer::OpenGLRenderInterface* RI, const Vector4d& _color, bool _default)
    {
#ifdef _RENDERER_TEST
        if (!RI) return;
        RI->PushMatrix();
        for(int i=0; i<mJointIn->getNumTransforms(); i++){
            mJointIn->getTransform(i)->applyGLTransform(RI);
        }

        // render the corresponding mHandless
        for(int i=0; i<mHandles.size(); i++){
            mHandles[i]->draw(RI, true, _color, _default);
        }
        for(int i=0; i<mJointOut.size(); i++){
            mJointOut[i]->getNodeOut()->drawHandles(RI,_color, _default);
        }
        RI->PopMatrix();
#else
        glPushMatrix();
        for(int i=0; i<mJointIn->getNumTransforms(); i++){
            mJointIn->getTransform(i)->applyGLTransform(RI);
        }

        // render the corresponding mHandless
        for(int i=0; i<mHandles.size(); i++){
            mHandles[i]->draw(RI, true, _color, _default);
        }
        for(int i=0; i<mJointOut.size(); i++){
            mJointOut[i]->getNodeOut()->drawHandles(RI,_color, _default);
        }
        glPopMatrix();
#endif
    }

    void BodyNode::setJointIn(Joint *_p) {
        mJointIn=_p; mNodeIn = _p->getNodeIn();
    }

    BodyNode* BodyNode::getNodeOut(int i) const {
        return mJointOut[i]->getNodeOut();
    }

    int BodyNode::getNumDofs() const {
        return mJointIn->getNumDofs();
    }

    Dof* BodyNode::getDof(int i) {
        return mJointIn->getDof(i);
    }

    bool BodyNode::isPresent(Dof* q) {
        return mJointIn->isPresent(q);
    }

} // namespace model3d

