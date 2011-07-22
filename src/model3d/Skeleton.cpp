/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "Skeleton.h"

#include <cassert>
using namespace std;
using namespace Eigen;

#include "Dof.h"
#include "Joint.h"
#include "BodyNode.h"
#include "Marker.h"
#include "Transformation.h"

#include "renderer/RenderInterface.h"


namespace model3d {

    Skeleton::Skeleton() {
        mMass = 0;
    }

    Skeleton::~Skeleton(){
        for(unsigned int i = 0; i < mJoints.size(); i++) delete mJoints[i];
        mJoints.clear();
        mDofs.clear();
        mTransforms.clear();
        for(unsigned int i=0; i<mNodes.size(); i++) delete mNodes[i];
        mNodes.clear();
        mHandles.clear();
    }

    BodyNode* Skeleton::createBodyNode(const char* const _name) {
        return new BodyNode(_name);
    }

    void Skeleton::addHandle(Marker *_h) {
        mHandles.push_back(_h);
        _h->setSkelIndex(mHandles.size()-1);
        BodyNode *body = _h->getNode();
        body->addHandle(_h);
    }

    void Skeleton::addNode(BodyNode *_b) {
        mNodes.push_back(_b);
        _b->setSkelIndex(mNodes.size()-1);
        addJoint(_b->getJointIn());
    }

    void Skeleton::addJoint(Joint *_j) {
        mJoints.push_back(_j);
        _j->setSkelIndex(mJoints.size()-1);
    }

    void Skeleton::addDof(Dof *_q) {
        mDofs.push_back(_q);
        _q->setSkelIndex(mDofs.size()-1);
        _q->setVariable();
    }

    void Skeleton::addTransform(Transformation *_t) {
        mTransforms.push_back(_t);
        _t->setVariable(true);
        _t->setSkelIndex(mTransforms.size()-1);
        for(int i=0; i<_t->getNumDofs(); i++) {
            addDof(_t->getDof(i));
        }
    }
  
    void Skeleton::initSkel() {
        mRoot = mNodes[0];
        mNumDofs = mDofs.size();
        mNumNodes = mNodes.size();
        mNumHandles = mHandles.size();

        // calculate mass
        // init the dependsOnDof stucture for each bodylink
        for(int i=0; i<mNumNodes; i++) {
            mNodes[i]->setSkel(this);
            // mNodes[i]->setDependDofMap(mNumDofs);
            mNodes[i]->setDependDofList();
            mNodes.at(i)->init();
            mMass += mNodes[i]->getMass();
        }

        mCurrState = VectorXd::Zero(mNumDofs);

        for(int i=0; i<mNumDofs; i++)
            mCurrState[i] = mDofs.at(i)->getValue();
        for(int i=0; i<mNumNodes; i++)
            mNodes.at(i)->updateTransform();
    }

    BodyNode* Skeleton::getNode(const char* const name) {
        const int nNodes = getNumNodes();
        for(int i = 0; i < nNodes; i++){
            BodyNode* node = getNode(i);
            if (strcmp(name, node->getName()) == 0) {
                return node;
            }
        }
        return NULL;
    }

    int Skeleton::getNodeIndex(const char* const name) {
        const int nNodes = getNumNodes();
        for(int i = 0; i < nNodes; i++){
            BodyNode* node = getNode(i);
            if (strcmp(name, node->getName()) == 0) {
                return i;
            }
        }
        return -1;
    }
  
    void Skeleton::setState(const VectorXd& state, bool bCalcTrans, bool bCalcDeriv) {
        assert(state.size() == mNumDofs);
        int k=0;
        for(k=0; k<mNumDofs; k++)
            if(mCurrState[k]!=state[k]) break;
        if(k==mNumDofs) return;

        mCurrState = state;
        for(int i=0; i<mNumDofs; i++) {
            mDofs.at(i)->setValue(state[i]);
        }

        if (bCalcTrans) {
            for(int i = 0; i < mNumNodes; i++) {
                mNodes.at(i)->updateTransform();
            }
        }

        if (bCalcDeriv) {
            for(int i = 0; i < mNumNodes; i++) {
                mNodes.at(i)->updateFirstDerivatives();
            }
        }
    }

    void Skeleton::setState(const vector<double>& state, bool bCalcTrans, bool bCalcDeriv) {
        VectorXd x(state.size());
        for (unsigned int i = 0; i < state.size(); i++) {
            x(i) = state[i];
        }
        setState(x, bCalcTrans, bCalcDeriv);
    }

    void Skeleton::setPose(const VectorXd& _pose){
        assert(_pose.size() == mNumDofs);
        for(int i=0; i<mNumDofs; i++)
            mDofs[i]->setValue(_pose(i));
    }

    void Skeleton::setPose(const vector<double>& _pose){
        assert(_pose.size() == mNumDofs);
        for(int i=0; i<mNumDofs; i++)
            mDofs[i]->setValue(_pose[i]);
    }

    void Skeleton::getPose(Eigen::VectorXd& _pose) {
        _pose.resize(mNumDofs);
        for (int i = 0; i < mNumDofs; i++) {
            _pose(i) = mDofs[i]->getValue();
        }
    }
    void Skeleton::getPose(std::vector<double>& _pose) {
        _pose.resize(mNumDofs);
        for (int i = 0; i < mNumDofs; i++) {
            _pose[i] = mDofs[i]->getValue();
        }
    }

	void Skeleton::draw(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
        mRoot->draw(_ri, _color, _useDefaultColor);
    }
    void Skeleton::drawHandles(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
        mRoot->drawHandles(_ri, _color, _useDefaultColor);
    }


} // namespace model3d
